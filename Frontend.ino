/////////////////////////////////////////////////////////////////
//
//  GPS Tester - FRONT END (display)
//
//  Second tab of the Backendcopysave sketch. Arduino compiles every
//  .ino in the folder together, so this reads the backend globals and
//  query functions directly (no extern needed).
//
/////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/////////////////////////////////////////////////////////////////
//
//  DISPLAY (front-end)  -  add the display code below.
//
//  displayBegin()  runs once at the end of setup().
//  displayUpdate() runs every loop() after new GPS data is ingested;
//                  throttle it in here if you don't want it every pass.
//
//  Read these state variables (already populated by the backend):
//    haveFix, ttff, fixType, satsUsed, satsInView, maxSNR,
//    bytesRx, sawValidSentence, ubxResponsive,
//    almCount, ephCount, assistChecked, earlyTimeValid
//
//  Or call these query helpers (no output, pure logic):
//    computeStage()      0 no data,1 invalid,2 no sats,3 acquiring,4 2D,5 3D
//    startType()         0 no fix,1 hot,2 warm,3 cold
//    batteryVerdict()    0 unknown,1 OK,2 suspect
//    acquisitionState()  0 fixed,1 downloading,2 slow,3 suspect,4 no sats
//
//  Actions you can trigger:  assistCheck(), ubxReset(0x0000/0xFFFF),
//                            resetDiagnostics()
//
/////////////////////////////////////////////////////////////////

// ---- Display hardware (20x4 I2C LCD) ----
const uint8_t LCD_COLS = 20;
const uint8_t LCD_ROWS = 4;
LiquidCrystal_I2C* lcd = nullptr;
bool lcdPresent = false;

// ---- Low-level LCD helpers ----

// print s onto row `row`, padded/truncated to `width` columns
void lcdRowN(uint8_t row, const char* s, uint8_t width){
  char buf[LCD_COLS + 1]; uint8_t i = 0;
  for(; s[i] && i < width; i++) buf[i] = s[i];
  for(; i < width; i++) buf[i] = ' ';
  buf[width] = 0;
  lcd->setCursor(0, row);
  lcd->print(buf);
}

// stamp " Phase X" at the far right of the bottom (4th) row
void lcdPhaseLabel(char phase){
  char t[9] = {' ', 'P', 'h', 'a', 's', 'e', ' ', phase, 0};   // 8 chars
  lcd->setCursor(LCD_COLS - 8, LCD_ROWS - 1);
  lcd->print(t);
}

// Draw a full screen: rows 0-2 are free content (20 cols), row 3 holds
// short content on the left (12 cols) and the "Phase X" counter on the right.
void showScreen(const char* l0, const char* l1, const char* l2,
                const char* l3, char phase){
  if(!lcdPresent || !lcd) return;
  lcdRowN(0, l0, LCD_COLS);
  lcdRowN(1, l1, LCD_COLS);
  lcdRowN(2, l2, LCD_COLS);
  lcdRowN(3, l3, LCD_COLS - 8);     // leave the right 8 cols for " Phase X"
  lcdPhaseLabel(phase);
}

/////////////////////////////////////////////////////////////////
//  PER-DISPLAY SCREENS
//  Each screen reports on one function's task. Fill in the content
//  (and the check) once described. Keep the phase letter argument so
//  the "Phase X" counter shows in the bottom-right.
//  Read backend state directly or via computeStage()/startType()/
//  batteryVerdict()/acquisitionState().
/////////////////////////////////////////////////////////////////

// format an ETA in seconds as "HAVE", "~30s" or "~13m"
void fmtETA(char* out, unsigned long s){
  if(s == 0)        strcpy(out, "HAVE");
  else if(s < 100)  snprintf(out, 8, "~%lus", s);
  else              snprintf(out, 8, "~%lum", (s + 59) / 60);
}

// Phase A - byte-flow signal, wiring verdict, and the UBX test.
//   GPS data  : did any bytes arrive from GPS TX (-> Arduino D4)?
//   NMEA valid: are they proper sentences (right baud)?
//   UBX reply : did the module answer a command (Arduino D3 -> GPS RX)?
//   Wires     : one-line verdict on what's likely miswired.
void screenPhaseA(){
  char l0[21], l1[21], l2[21];
  snprintf(l0, sizeof(l0), "Byte Flow Test: %s", bytesRx ? "YES" : "NONE");
  snprintf(l1, sizeof(l1), "NMEA valid: %s", sawValidSentence ? "YES" : "NO");
  snprintf(l2, sizeof(l2), "UBX reply:  %s", ubxResponsive ? "YES" : "NO");
  const char* w;
  if(bytesRx == 0)            w = "Chk TX->D4";     // GPS TX must reach D4
  else if(!sawValidSentence)  w = "Baud 9600?";     // bytes but garbled
  else if(!ubxResponsive)     w = "Chk RX->D3";     // can't command the module
  else                        w = "Wires OK";
  showScreen(l0, l1, l2, w, 'A');
}

// Phase B - start type, stored ephemeris/almanac, and time-to-update.
void screenPhaseB(){
  char l0[21], l1[21], l2[21];
  if(haveFix){
    const char* st = (startType() == 1) ? "HOT" : (startType() == 2) ? "WARM" : "COLD";
    snprintf(l0, sizeof(l0), "Start: %s (%lus)", st, ttff / 1000UL);
  } else {
    const char* pst = (ephCount > 0) ? "HOT?" : (almCount > 0) ? "WARM?" : "COLD?";
    snprintf(l0, sizeof(l0), "Start(pred): %s", pst);
  }
  // Each line reports 2 things: HAVE/NONE (stored in the module's memory)
  // and the % of validity left before it needs to be re-downloaded.
  snprintf(l1, sizeof(l1), "Ephemeris: %s %d%%", ephCount > 0 ? "HAVE" : "NONE", ephFreshPct());
  snprintf(l2, sizeof(l2), "Almanac: %s %d%%",   almCount > 0 ? "HAVE" : "NONE", almFreshPct());
  showScreen(l0, l1, l2, "", 'B');
}

// Phase C - live satellite feed + live fix (can it see anything right now?).
void screenPhaseC(){
  char l0[21], l1[21], l3[13];
  snprintf(l0, sizeof(l0), "Sats view:%d used:%d", satsInView, satsUsed);
  snprintf(l1, sizeof(l1), "Median sig: %d dB", medianSNR());
  const char* fx;
  if(haveFix)              fx = (fixType == 3) ? "Fix: 3D FIX now" : "Fix: 2D fix now";
  else if(satsInView > 0)  fx = "Fix: searching..";
  else                     fx = "Fix: no sats yet";
  snprintf(l3, sizeof(l3), "Alm:%s Eph:%s",
           almCount > 0 ? "Y" : "N", ephCount > 0 ? "Y" : "N");
  showScreen(l0, l1, fx, l3, 'C');
}

// Phase D - position (lat / lon / altitude) + overall readiness percent.
void screenPhaseD(){
  char l0[21], l1[21], l2[21], l3[13];
  if(haveFix){
    char latS[14], lonS[14], altS[10];
    dtostrf(latDeg, 0, 6, latS);   // AVR: use dtostrf (snprintf has no %f)
    dtostrf(lonDeg, 0, 6, lonS);   // 6 decimals ~= 0.1 m
    dtostrf(altM,   0, 1, altS);
    snprintf(l0, sizeof(l0), "Lat: %s", latS);
    snprintf(l1, sizeof(l1), "Lon: %s", lonS);
    snprintf(l2, sizeof(l2), "Alt: %s m", altS);
  } else {
    snprintf(l0, sizeof(l0), "Lat: --");
    snprintf(l1, sizeof(l1), "Lon: --");
    snprintf(l2, sizeof(l2), "Alt: --");
  }
  snprintf(l3, sizeof(l3), "Ready: %d%%", functionalityPct());
  showScreen(l0, l1, l2, l3, 'D');
}

// Add more screens here (Phase E, F, ...) and bump NUM_PAGES below.
void renderPage(uint8_t p){
  switch(p){
    case 0: screenPhaseA(); break;
    case 1: screenPhaseB(); break;
    case 2: screenPhaseC(); break;
    case 3: screenPhaseD(); break;
  }
}
const uint8_t NUM_PAGES = 4;

// ---- Pager: cycle the screens, refresh the current one for live data ----
// How long each screen is shown, indexed by page (Phase A, B, C).
const unsigned long PAGE_MS[NUM_PAGES] = { 7000UL, 7000UL, 7000UL, 7000UL };
const unsigned long REFRESH_MS = 400;    // how often the current screen redraws
uint8_t curPage = 0;
unsigned long lastPage = 0, lastDraw = 0;

// Phase A "fails" when its wiring verdict is not OK: no data, bad baud, or no
// UBX reply. While it fails, Phases B (assist data) and D (position) are
// meaningless, so we only rotate Phases A and C.
bool phaseAfail(){
  return (bytesRx == 0) || !sawValidSentence || !ubxResponsive;
}

// Next page in the rotation; skips Phases B and D while Phase A is failing.
uint8_t nextPage(uint8_t cur){
  for(uint8_t step = 1; step <= NUM_PAGES; step++){
    uint8_t p = (cur + step) % NUM_PAGES;
    if(phaseAfail() && p != 0 && p != 2) continue;   // only A (0) and C (2) on failure
    return p;
  }
  return 0;
}

// ---- Front-end hooks called by setup()/loop() ----
void displayBegin(){
  Wire.begin();
  // auto-detect the LCD address (0x27 or 0x3F are typical)
  uint8_t addr = 0, first = 0;
  for(uint8_t a = 1; a < 127; a++){
    Wire.beginTransmission(a);
    if(Wire.endTransmission() == 0){
      if(first == 0) first = a;
      if(a == 0x27 || a == 0x3F) addr = a;
    }
  }
  if(addr == 0) addr = first;
  if(addr){
    lcd = new LiquidCrystal_I2C(addr, LCD_COLS, LCD_ROWS);
    lcd->init(); lcd->backlight();
    lcdPresent = true;
    showScreen("GPS Tester", "starting up...", "", "", 'A');
  }
}

void displayUpdate(){
  if(!lcdPresent) return;
  unsigned long now = millis();
  if(now - lastPage >= PAGE_MS[curPage]){
    lastPage = now;
    curPage = nextPage(curPage);     // skips B & D while Phase A is failing
    // re-poll almanac/ephemeris each time we return to the Phase A page
    if(curPage == 0) assistCheck(1200);
  }
  if(now - lastDraw >= REFRESH_MS){ lastDraw = now; renderPage(curPage); }
}
