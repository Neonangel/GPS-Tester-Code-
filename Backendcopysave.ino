/////////////////////////////////////////////////////////////////
//
//  GPS Tester - BACKEND ONLY
//
//  Target module : GY-NEO6MV2  (u-blox NEO-6M)
//
//  This file contains ONLY the detection / diagnostic logic:
//    - read the GPS over SoftwareSerial
//    - parse NMEA (checksum, fields, fix, satellites, signal)
//    - measure time-to-first-fix (TTFF)
//    - UBX: probe config, count stored almanac / ephemeris, reset
//    - derive stage / start-type / battery / acquisition verdicts
//
//  There is NO Serial-print and NO display code here on purpose.
//  Results are exposed as global variables and query functions so a
//  separate front-end (Serial, LCD, OLED, web, ...) can read them.
//
//  How a front-end uses this:
//    - call pump() often (loop already does) to ingest GPS bytes
//    - read state:  haveFix, ttff, fixType, satsInView, maxSNR,
//                   satsUsed, bytesRx, sawValidSentence, ubxResponsive,
//                   almCount, ephCount, assistChecked, earlyTimeValid
//    - or call the query helpers: computeStage(), startType(),
//                   batteryVerdict(), acquisitionState()
//    - actions:     assistCheck(), ubxReset(mask), resetDiagnostics()
//
/////////////////////////////////////////////////////////////////

#include <SoftwareSerial.h>

// ---------------- CONFIGURATION ----------------------------------
// GPS wiring (TX/RX crossed): GPS TX -> D4 (RX_PIN), GPS RX -> D3 (TX_PIN)
const uint8_t RX_PIN = 4;        // Arduino pin that listens to GPS TX
const uint8_t TX_PIN = 3;        // Arduino pin that talks to GPS RX
const long    GPS_BAUD = 9600;   // NEO-6M default

// Timing thresholds (milliseconds)
const unsigned long HOT_MS     = 5000UL;    // fix faster than this = hot start
const unsigned long WARM_MS    = 40000UL;   // fix faster than this = warm start
const unsigned long ALM_MS     = 780000UL;  // 13 min: almanac-download horizon
const unsigned long BYTECHK_MS = 3000UL;    // initial byte-flow test length

// Typical time to (re)acquire assist data from live satellites:
const unsigned long EPH_ACQUIRE_S = 30UL;    // ephemeris: broadcast ~every 30 s
const unsigned long ALM_ACQUIRE_S = 750UL;   // almanac: ~12.5 min for a full set
const unsigned long EPH_VALID_S   = 14400UL;   // ephemeris good ~4 h before a new download is needed
const unsigned long ALM_VALID_S   = 7776000UL; // almanac good ~90 days before it should be refreshed

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

// ---------------- DIAGNOSTIC STATE (read by the front-end) -------
unsigned long bootMs;
unsigned long ttff = 0;
bool  haveFix = false;

int   fixQuality = -1;    // GGA: 0 none, 1 GPS, 2 DGPS
int   fixType    = -1;    // GSA: 1 none, 2 = 2D, 3 = 3D
int   satsUsed   = 0;     // satellites used in the solution
int   satsInView = 0;     // satellites the antenna can see
int   maxSNR     = 0;     // strongest satellite signal (dB-Hz)
float latDeg = 0, lonDeg = 0, altM = 0;   // position (decimal deg / metres) - valid at fix
int   snrArr[32]; int snrArrN = 0;        // SNRs from the last complete GSV cycle (for median)
int   snrTmp[32]; int snrTmpN = 0;        // SNRs accumulating in the current GSV cycle

unsigned long bytesRx = 0;
bool  sawValidSentence = false;
bool  sawGGA=false, sawRMC=false, sawGSA=false, sawGSV=false, sawTXT=false;
bool  ubxResponsive = false;
bool  earlyTimeValid = false;   // RTC time seen at boot => backup battery hint

// almanac / ephemeris presence (from UBX-AID poll). -1 = not checked yet.
int   almCount = -1;            // # of satellites with stored almanac
int   ephCount = -1;            // # of satellites with stored ephemeris
bool  assistChecked = false;
unsigned long ephFreshMs = 0;   // millis() when ephemeris was last confirmed present
unsigned long almFreshMs = 0;   // millis() when almanac   was last confirmed present

// line buffer for one NMEA sentence
char line[100];
uint8_t lineLen = 0;

// ---------------- NMEA HELPERS -----------------------------------
byte hexVal(char c){
  if(c>='0' && c<='9') return c-'0';
  if(c>='A' && c<='F') return c-'A'+10;
  if(c>='a' && c<='f') return c-'a'+10;
  return 0;
}

bool checksumOK(const char* s){
  if(s[0] != '$') return false;
  byte sum = 0; int i = 1;
  while(s[i] && s[i] != '*'){ sum ^= (byte)s[i]; i++; }
  if(s[i] != '*') return false;
  byte given = hexVal(s[i+1])*16 + hexVal(s[i+2]);
  return given == sum;
}

// Copy the idx-th comma-separated field (field 0 = "$GPGGA") into out.
bool getField(const char* s, int idx, char* out, int outsz){
  int field = 0, oi = 0;
  for(const char* p = s;; p++){
    char c = *p;
    if(c == 0 || c == '*' || c == ','){
      if(field == idx){ out[oi] = 0; return true; }
      if(c == 0 || c == '*'){ out[0] = 0; return false; }
      field++;
      continue;
    }
    if(field == idx && oi < outsz-1) out[oi++] = c;
  }
}

// message type match, e.g. isType(line,"GGA") ignores talker (GP/GN/GL)
bool isType(const char* s, const char* t){
  return s[0]=='$' && strncmp(s+3, t, 3) == 0;
}

// Convert an NMEA ddmm.mmmm / dddmm.mmmm field to signed decimal degrees.
float nmeaToDeg(const char* s, char hemi){
  double v = atof(s);
  int deg = (int)(v / 100.0);
  double minutes = v - deg * 100.0;
  double d = deg + minutes / 60.0;
  if(hemi == 'S' || hemi == 'W') d = -d;
  return (float)d;
}

// ---------------- SENTENCE PROCESSING ----------------------------
void processLine(){
  if(lineLen < 6) return;
  line[lineLen] = 0;
  if(!checksumOK(line)) return;      // ignore garbled lines
  sawValidSentence = true;

  char f[12];

  if(isType(line,"GGA")){
    sawGGA = true;
    if(getField(line,6,f,sizeof(f)) && f[0]) fixQuality = atoi(f);
    if(getField(line,7,f,sizeof(f)) && f[0]) satsUsed   = atoi(f);
    // position: lat (f2)+N/S (f3), lon (f4)+E/W (f5), altitude metres (f9)
    char raw[16], hemi[3];
    if(getField(line,2,raw,sizeof(raw)) && raw[0] &&
       getField(line,3,hemi,sizeof(hemi)) && hemi[0]) latDeg = nmeaToDeg(raw, hemi[0]);
    if(getField(line,4,raw,sizeof(raw)) && raw[0] &&
       getField(line,5,hemi,sizeof(hemi)) && hemi[0]) lonDeg = nmeaToDeg(raw, hemi[0]);
    if(getField(line,9,raw,sizeof(raw)) && raw[0]) altM = atof(raw);
  }
  else if(isType(line,"GSA")){
    sawGSA = true;
    if(getField(line,2,f,sizeof(f)) && f[0]) fixType = atoi(f);
  }
  else if(isType(line,"GSV")){
    sawGSV = true;
    int total = 0, msgNum = 0;
    if(getField(line,1,f,sizeof(f)) && f[0]) total  = atoi(f);
    if(getField(line,2,f,sizeof(f)) && f[0]) msgNum = atoi(f);
    if(getField(line,3,f,sizeof(f)) && f[0]) satsInView = atoi(f);
    if(msgNum == 1) snrTmpN = 0;         // first message of a GSV cycle -> restart collection
    // SNR of up to 4 satellites in this message: fields 7, 11, 15, 19
    for(int fi = 7; fi <= 19; fi += 4){
      if(getField(line,fi,f,sizeof(f)) && f[0]){
        int snr = atoi(f);
        if(snr > maxSNR) maxSNR = snr;
        if(snr > 0 && snrTmpN < 32) snrTmp[snrTmpN++] = snr;   // only sats we actually receive
      }
    }
    if(total > 0 && msgNum == total){    // last message: publish this cycle's set
      snrArrN = snrTmpN;
      for(int i = 0; i < snrArrN; i++) snrArr[i] = snrTmp[i];
    }
  }
  else if(isType(line,"RMC")){
    sawRMC = true;
    // field 9 = date ddmmyy. A real date arriving early (before a fix)
    // means the RTC kept time => backup battery is holding charge.
    if(getField(line,9,f,sizeof(f)) && strlen(f) == 6 && strncmp(f,"000000",6) != 0){
      if(!haveFix && (millis() - bootMs) < 6000UL) earlyTimeValid = true;
    }
  }
  else if(isType(line,"TXT")){
    sawTXT = true;                   // module boot/antenna message (not decoded here)
  }

  // Fix detection + TTFF
  bool nowFix = (fixQuality >= 1) || (fixType >= 2);
  if(nowFix && !haveFix){
    haveFix = true;
    ttff = millis() - bootMs;
  }
}

void feed(char c){
  if(c == '$'){ lineLen = 0; line[lineLen++] = c; return; }
  if(c == '\r' || c == '\n'){
    if(lineLen > 0){ processLine(); lineLen = 0; }
    return;
  }
  if(lineLen < sizeof(line)-1) line[lineLen++] = c;
}

// Ingest whatever the GPS has sent. Call this often.
void pump(){
  while(gpsSerial.available()){ char c = gpsSerial.read(); bytesRx++; feed(c); }
}

// Reset all live diagnostics (e.g. after a restart) so TTFF is fresh.
void resetDiagnostics(){
  bootMs = millis(); ttff = 0; haveFix = false;
  fixQuality = -1; fixType = -1; satsUsed = 0; satsInView = 0; maxSNR = 0;
  snrArrN = 0; snrTmpN = 0;
  latDeg = 0; lonDeg = 0; altM = 0;
  bytesRx = 0; sawValidSentence = false;
  sawGGA = sawRMC = sawGSA = sawGSV = sawTXT = false; earlyTimeValid = false;
  lineLen = 0;
}

// ---------------- UBX FRAME I/O ----------------------------------
// Build+send a UBX frame with checksum. NMEA is pure ASCII (<0x80),
// so it can never be mistaken for the 0xB5 0x62 UBX sync bytes.
void ubxSend(byte cls, byte id, const byte* pl, uint16_t len){
  byte cka = 0, ckb = 0;
  byte hdr[6] = {0xB5, 0x62, cls, id, (byte)(len & 0xFF), (byte)(len >> 8)};
  for(int i = 2; i < 6; i++){ cka += hdr[i]; ckb += cka; }
  for(uint16_t i = 0; i < len; i++){ cka += pl[i]; ckb += cka; }
  gpsSerial.write(hdr, 6);
  if(len) gpsSerial.write(pl, len);
  gpsSerial.write(cka); gpsSerial.write(ckb);
}

// Poll UBX-CFG-PRT. A UBX reply proves the module honours binary config
// AND that the Arduino-TX -> GPS-RX wire works. Sets ubxResponsive.
void ubxProbe(){
  while(gpsSerial.available()) gpsSerial.read();     // flush
  ubxSend(0x06, 0x00, NULL, 0);
  unsigned long t = millis(); int st = 0;
  while(millis() - t < 1500UL){
    if(!gpsSerial.available()) continue;
    byte b = gpsSerial.read();
    if(st == 0 && b == 0xB5) st = 1;
    else if(st == 1 && b == 0x62){ ubxResponsive = true; break; }
    else st = 0;
  }
}

// Poll UBX-AID-ALM (id 0x30) or UBX-AID-EPH (id 0x31) for ALL satellites
// and count how many carry real data. Reply payload is 8 bytes when a
// satellite has NO data, and longer (40 almanac / 104 ephemeris) when it
// DOES. So len > 8 == "this satellite has data".
int ubxCountAid(byte id, unsigned long window){
  while(gpsSerial.available()) gpsSerial.read();     // flush
  ubxSend(0x0B, id, NULL, 0);                        // poll all SVs
  int withData = 0, frames = 0;
  int st = 0; uint16_t len = 0, cnt = 0;
  unsigned long t = millis();
  while(millis() - t < window && frames < 33){
    if(!gpsSerial.available()) continue;
    byte b = gpsSerial.read();
    switch(st){
      case 0: st = (b == 0xB5) ? 1 : 0; break;
      case 1: st = (b == 0x62) ? 2 : 0; break;
      case 2: st = (b == 0x0B) ? 3 : 0; break;       // AID class
      case 3: st = (b == id)   ? 4 : 0; break;       // ALM or EPH
      case 4: len = b; st = 5; break;                // length lo
      case 5: len |= ((uint16_t)b << 8);             // length hi
              if(len > 8) withData++;
              ubxResponsive = true;                  // module answered a UBX poll
              frames++; cnt = 0; st = 6; break;
      case 6: cnt++; if(cnt >= len) st = 7; break;   // skip payload
      case 7: st = 8; break;                         // ck_a
      case 8: st = 0; break;                         // ck_b
    }
  }
  return withData;
}

// UBX-CFG-RST: restart the receiver.
//   bbrMask 0x0000 = hot start (keep almanac/ephemeris/time)
//           0xFFFF = cold start (wipe everything)
//   resetMode 0x00 = immediate hardware (watchdog) reset
// Not acknowledged (the module resets).
void ubxReset(uint16_t bbrMask){
  byte pl[4] = {(byte)(bbrMask & 0xFF), (byte)(bbrMask >> 8), 0x00, 0x00};
  ubxSend(0x06, 0x04, pl, 4);
}

// Poll and store how much almanac / ephemeris the module currently holds.
void assistCheck(unsigned long window = 2000){
  ubxResponsive = false;                  // re-evaluated from the poll replies below
  almCount = ubxCountAid(0x30, window);   // UBX-AID-ALM
  ephCount = ubxCountAid(0x31, window);   // UBX-AID-EPH
  if(ephCount > 0) ephFreshMs = millis(); // ephemeris present -> reset its freshness clock
  if(almCount > 0) almFreshMs = millis(); // almanac present   -> reset its freshness clock
  assistChecked = true;                   // ubxResponsive now reflects THIS poll
}

// ---------------- DERIVED VERDICTS (pure logic) ------------------
// 0 = no data, 1 = data but not valid NMEA, 2 = no sats in view,
// 3 = tracking/acquiring, 4 = 2D fix, 5 = 3D fix
int computeStage(){
  if(bytesRx == 0)        return 0;
  if(!sawValidSentence)   return 1;
  if(satsInView == 0)     return 2;
  if(!haveFix)            return 3;
  if(fixType == 2)        return 4;
  return 5;
}

// 0 = no fix yet, 1 = hot, 2 = warm, 3 = cold
int startType(){
  if(!haveFix)            return 0;
  if(ttff < HOT_MS)       return 1;
  if(ttff < WARM_MS)      return 2;
  return 3;
}

// 0 = unknown, 1 = OK, 2 = suspect (possibly dead)
int batteryVerdict(){
  if(assistChecked && (almCount > 0 || ephCount > 0)) return 1;
  if(haveFix && ttff < HOT_MS)                        return 1;
  if(earlyTimeValid)                                  return 1;
  if(haveFix && ttff < WARM_MS)                       return 1;
  if(assistChecked)                                   return 2;
  return 0;
}

// 0 = have fix, 1 = downloading ephemeris (normal), 2 = slow acquisition,
// 3 = suspect (no almanac after a long time), 4 = no satellites tracked yet
int acquisitionState(){
  if(haveFix) return 0;
  unsigned long el = millis() - bootMs;
  if(satsInView >= 1 && maxSNR > 0){
    if(el < WARM_MS) return 1;
    if(el < ALM_MS)  return 2;
    return 3;
  }
  return 4;
}

// Rough seconds until ephemeris / almanac would be freshly available.
// 0 means the module already has it. Counts down from the typical acquire
// time using elapsed-since-boot; floors to a small number once overdue.
unsigned long ephUpdateETA(){
  if(ephCount > 0) return 0;
  unsigned long el = (millis() - bootMs) / 1000UL;
  return (el < EPH_ACQUIRE_S) ? (EPH_ACQUIRE_S - el) : 5;
}
unsigned long almUpdateETA(){
  if(almCount > 0) return 0;
  unsigned long el = (millis() - bootMs) / 1000UL;
  return (el < ALM_ACQUIRE_S) ? (ALM_ACQUIRE_S - el) : 30;
}

// Download progress (0-100 %) based on how many satellites have data stored.
//   Almanac complete  = a full constellation (~32 satellites).
//   Ephemeris complete = ephemeris for every satellite currently in view.
int almProgress(){
  if(almCount <= 0) return 0;
  long p = (long)almCount * 100 / 32;
  return p > 100 ? 100 : (int)p;
}
// Percent of validity LEFT before a new download is needed: 100 % right after
// the data is present, decaying to 0 % over `validS`. Time-based (seconds, to
// avoid overflow on the long almanac window), so no satellites-in-view artifact.
int freshPct(unsigned long freshMs, unsigned long validS){
  if(freshMs == 0) return 0;                    // never present
  unsigned long ageS = (millis() - freshMs) / 1000UL;
  if(ageS >= validS) return 0;
  return (int)(100UL - (ageS * 100UL) / validS);
}
int ephFreshPct(){ return freshPct(ephFreshMs, EPH_VALID_S); }
int almFreshPct(){ return freshPct(almFreshMs, ALM_VALID_S); }

// Median of the SNRs from the last complete GSV cycle (0 if none received).
int medianSNR(){
  int n = snrArrN;
  if(n <= 0) return 0;
  int a[32];
  for(int i = 0; i < n; i++) a[i] = snrArr[i];
  for(int i = 1; i < n; i++){                 // insertion sort (n is small)
    int key = a[i], j = i - 1;
    while(j >= 0 && a[j] > key){ a[j+1] = a[j]; j--; }
    a[j+1] = key;
  }
  return (n & 1) ? a[n/2] : (a[n/2 - 1] + a[n/2]) / 2;
}

// Overall readiness toward full functionality, 0-100 %. Weighted so a fully
// working unit (valid data, config link, sats, assist data, 3D fix) = 100 %.
int functionalityPct(){
  int p = 0;
  if(bytesRx > 0)      p += 10;   // module is sending data
  if(sawValidSentence) p += 10;   // data is valid NMEA
  if(ubxResponsive)    p += 10;   // config/UBX link works
  if(satsInView > 0)   p += 10;   // seeing satellites
  p += almProgress() * 10 / 100;  // almanac downloaded (up to 10)
  if(ephCount > 0)     p += 15;   // ephemeris present
  if(haveFix) p += (fixType == 3) ? 35 : 20;   // fix achieved (3D is best)
  return p > 100 ? 100 : p;
}

// ---------------- ARDUINO ENTRY POINTS ---------------------------
void setup(){
  gpsSerial.begin(GPS_BAUD);
  bootMs = millis();

  // initial byte-flow test: gather bytes for a fixed window
  unsigned long t = millis();
  while(millis() - t < BYTECHK_MS) pump();

  // probe the UBX/config subsystem, then read stored almanac/ephemeris
  ubxProbe();
  assistCheck();

  displayBegin();          // front-end init (defined in the DISPLAY section below)
}

void loop(){
  // Ingest GPS data; all state/verdicts update in place for a front-end.
  pump();

  // The almanac/ephemeris re-poll is driven by the display pager (once per
  // 25 s cycle, at the Phase A page) - see displayUpdate() below.
  displayUpdate();
}

