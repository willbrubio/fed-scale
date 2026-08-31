/*
  Mouse Scale - logging + SCD-30 + paged display + grams calibration
  Feather M0 Adalogger (microSD CS=4) + NAU7802 + PCF8523 RTC + SH1107 OLED + SCD-30

  Buttons:  A = cycle display page (Weight -> Env -> IR)
            B = run 2-step grams calibration (saved to SD, survives reboot)
            C = tare (zero the platform)
  Logging runs regardless of the page. Weight shows grams once calibrated, raw counts
  until then. A "*" by the page tag means logging is active.

  Set KNOWN_MASS_G below to the calibration weight you actually own before calibrating.

  Libraries: Adafruit NAU7802, Adafruit SH110X, Adafruit GFX, RTClib, Adafruit SCD30,
             Adafruit Unified Sensor. (SD is built in.)
  Board: "Adafruit Feather M0".
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NAU7802.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_AMG88xx.h>
#include "RTClib.h"

// >>> Set this to the mass of your calibration weight, in grams. <<<
#define KNOWN_MASS_G 20.0

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
#define OLED_ADDR 0x3C

Adafruit_NAU7802 nau;
RTC_PCF8523 rtc;
Adafruit_SCD30 scd30;
Adafruit_AMG88xx amg;          // 8x8 thermal cam at 0x69

#define SD_CS 4                // onboard microSD CS on the M0 Adalogger (NOT D10)
#define MODE_BTN 9             // wing button A (also VBAT sense - fine as a button)
#define CAL_BTN  6             // wing button B
#define TARE_BTN 5             // wing button C
#define CAL_FILE "CALIB.TXT"

enum { PAGE_WEIGHT, PAGE_ENV, PAGE_IR, PAGE_INFO, PAGE_COUNT };
uint8_t page = PAGE_WEIGHT;
bool lastModeBtn = HIGH;
bool lastCalBtn = HIGH;
bool lastTareBtn = HIGH;
uint32_t lastModePress = 0;

// Screen sleep: blank the OLED after this long with no button press (prevents burn-in
// on a long unattended run), and wake on the next press. Logging is unaffected.
const uint32_t SLEEP_MS = 60000;   // 60 s; raise/lower to taste
uint32_t lastInteraction = 0;
bool screenOn = true;

const uint8_t AVG_N = 8;       // display smoothing; Serial still gets every raw sample
int32_t ring[AVG_N];
uint8_t ringIdx = 0;
bool ringFull = false;

int32_t tareOffset = 0;
float countsPerGram = 0;       // 0 = uncalibrated; set by calibration or loaded from SD

// Cached SCD-30 values held between its ~2 s updates; NAN until the first real reading.
float co2 = NAN, tempC = NAN, rh = NAN;
bool haveSCD = false;
bool envValid = false;

// AMG8833 thermal: 8x8 grid + per-frame reductions, cached like the env values.
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];   // 64 pixel temps (C)
float irMin = NAN, irMax = NAN, irMean = NAN;
uint8_t irHot = 0;             // index 0-63 of the hottest pixel (row=idx/8, col=idx%8)
bool haveAMG = false;
bool irValid = false;

const uint16_t LOG_MS = 100;    uint32_t lastLog = 0;
const uint16_t DRAW_MS = 200;   uint32_t lastDraw = 0;
const uint16_t FLUSH_MS = 1000; uint32_t lastFlush = 0;
const uint16_t ENV_MS = 1000;   uint32_t lastEnv = 0;
const uint16_t IR_MS = 250;     uint32_t lastIR = 0;   // poll the thermal cam ~4 Hz
const uint16_t THERMAL_MS = 1000; uint32_t lastThermal = 0;  // full 8x8 frame to SD ~1 Hz

File logFile;
char logName[13] = "DATA0000.CSV";
char thermalName[13] = "THRM0000.CSV";   // paired full-frame file (same run number)
bool logging = false;
uint32_t rowCount = 0;
uint32_t logStartMillis = 0;   // when logging began, for the recording-duration display

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);       // 100 kHz for the SCD-30 - set before its begin()

  display.begin(OLED_ADDR, true);
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  if (!nau.begin()) { showError("NAU7802 not found"); while (1) delay(10); }
  nau.setLDO(NAU7802_3V0);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_80SPS);
  for (uint8_t i = 0; i < 10; i++) { while (!nau.available()) delay(1); nau.read(); }
  nau.calibrate(NAU7802_CALMOD_INTERNAL);
  nau.calibrate(NAU7802_CALMOD_OFFSET);

  if (!rtc.begin()) { showError("RTC not found"); while (1) delay(10); }
  if (!rtc.initialized() || rtc.lostPower())
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  rtc.start();

  if (scd30.begin()) {
    haveSCD = true;
    // scd30.setAltitudeOffset(140);        // ~St. Louis elevation (m): better CO2 accuracy
    // scd30.selfCalibrationEnabled(false); // disable ASC if it won't see fresh air daily
  } else {
    showWarn("SCD30 not found");
  }

  if (amg.begin()) haveAMG = true;          // AMG8833 thermal cam (default 0x69)
  else showWarn("AMG8833 not found");

  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(CAL_BTN,  INPUT_PULLUP);
  pinMode(TARE_BTN, INPUT_PULLUP);

  for (uint8_t i = 0; i < AVG_N; i++) { while (!nau.available()) delay(1); pushSample(nau.read()); }
  tareOffset = readAveraged();

  if (SD.begin(SD_CS)) {
    loadCalibration();                        // restore counts-per-gram if a cal exists
    uint16_t idx = 0;
    for (; idx < 10000; idx++) {              // first free run number
      sprintf(logName, "DATA%04u.CSV", idx);
      if (!SD.exists(logName)) break;
    }
    sprintf(thermalName, "THRM%04u.CSV", idx); // pair the thermal file to the same number
    logFile = SD.open(logName, FILE_WRITE);
    if (logFile) {
      logFile.println("iso,unixtime,millis,raw,tared,grams,co2_ppm,temp_c,rh_pct,ir_min,ir_max,ir_mean,ir_hot");
      logFile.flush();
      logging = true;
      logStartMillis = millis();              // mark recording start
    }
    // Create the full-frame file with a px0..px63 header (only if the cam is present).
    if (logging && haveAMG) {
      File tf = SD.open(thermalName, FILE_WRITE);
      if (tf) {
        tf.print("iso,unixtime,millis");
        for (uint8_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) { tf.print(",px"); tf.print(i); }
        tf.println();
        tf.close();
      }
    }
  }
  if (!logging) showWarn("SD failed - display only");

  Wire.setClock(100000);       // reassert in case a begin() bumped it to 400
  lastInteraction = millis();  // start the screen-sleep timer from boot
}

void loop() {
  if (nau.available()) {
    int32_t raw = nau.read();
    pushSample(raw);
    Serial.println(raw - tareOffset);
  }

  // --- Buttons (all edge-detected) + screen wake ---
  bool aNow = digitalRead(MODE_BTN);
  bool bNow = digitalRead(CAL_BTN);
  bool cNow = digitalRead(TARE_BTN);
  bool aEdge = (lastModeBtn == HIGH && aNow == LOW);   // falling edge = fresh press
  bool bEdge = (lastCalBtn  == HIGH && bNow == LOW);
  bool cEdge = (lastTareBtn == HIGH && cNow == LOW);

  // Any button down resets the sleep timer.
  if (aNow == LOW || bNow == LOW || cNow == LOW) lastInteraction = millis();

  // If the screen is asleep, the first press only wakes it - don't also run its action.
  bool wokePress = false;
  if (!screenOn && (aEdge || bEdge || cEdge)) { screenOn = true; drawScreen(); wokePress = true; }

  if (screenOn && !wokePress) {
    if (aEdge && millis() - lastModePress > 200) {     // A: cycle page
      page = (page + 1) % PAGE_COUNT;
      lastModePress = millis();
      drawScreen();
    }
    if (bEdge) {                                        // B: calibrate (blocking)
      runCalibration();
      lastInteraction = millis();                       // don't sleep right after cal
      bNow = digitalRead(CAL_BTN);                       // buttons released during cal
    }
    if (cEdge) tareOffset = readAveraged();             // C: tare
  }

  lastModeBtn = aNow;
  lastCalBtn = bNow;
  lastTareBtn = cNow;

  if (haveSCD && millis() - lastEnv >= ENV_MS) {
    lastEnv = millis();
    if (scd30.dataReady() && scd30.read() && scd30.CO2 > 0) {
      co2 = scd30.CO2; tempC = scd30.temperature; rh = scd30.relative_humidity;
      envValid = true;
    }
  }

  // Poll the thermal cam: read the 8x8 frame and reduce to min/max/mean/hotspot.
  if (haveAMG && millis() - lastIR >= IR_MS) {
    lastIR = millis();
    amg.readPixels(pixels);
    float mn = pixels[0], mx = pixels[0], sum = 0;
    uint8_t hot = 0;
    for (uint8_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      float v = pixels[i];
      sum += v;
      if (v < mn) mn = v;
      if (v > mx) { mx = v; hot = i; }   // track the hottest pixel index
    }
    irMin = mn; irMax = mx; irMean = sum / AMG88xx_PIXEL_ARRAY_SIZE; irHot = hot;
    irValid = true;
  }

  if (logging && millis() - lastLog >= LOG_MS)     { lastLog = millis(); logRow(); }
  if (logging && millis() - lastFlush >= FLUSH_MS) { lastFlush = millis(); logFile.flush(); }

  // Full 8x8 thermal frame to its own file at ~1 Hz (uses the latest cached frame).
  if (logging && haveAMG && irValid && millis() - lastThermal >= THERMAL_MS) {
    lastThermal = millis();
    writeThermalFrame();
  }

  // Sleep the screen after inactivity: blank framebuffer = all pixels off (no burn-in).
  if (screenOn && millis() - lastInteraction > SLEEP_MS) {
    screenOn = false;
    display.clearDisplay();
    display.display();
  }

  // Redraw only while awake.
  if (screenOn && millis() - lastDraw >= DRAW_MS) { lastDraw = millis(); drawScreen(); }
}

void pushSample(int32_t v) {
  ring[ringIdx] = v;
  ringIdx = (ringIdx + 1) % AVG_N;
  if (ringIdx == 0) ringFull = true;
}

int32_t readAveraged() {
  uint8_t n = ringFull ? AVG_N : ringIdx;
  if (n == 0) return 0;
  int64_t sum = 0;
  for (uint8_t i = 0; i < n; i++) sum += ring[i];
  return (int32_t)(sum / n);
}

// Blocking average of n fresh conversions - used during calibration for a stable reading.
int32_t sampleFresh(uint8_t n) {
  int64_t sum = 0;
  for (uint8_t i = 0; i < n; i++) { while (!nau.available()) delay(1); sum += nau.read(); }
  return (int32_t)(sum / n);
}

void logRow() {
  DateTime now = rtc.now();
  char iso[] = "YYYY-MM-DD hh:mm:ss";
  now.toString(iso);
  int32_t avg = readAveraged();
  int32_t tared = avg - tareOffset;
  // grams only if calibrated; otherwise NAN -> "nan" (pandas NaN).
  float grams = (countsPerGram > 0) ? tared / countsPerGram : NAN;

  logFile.print(iso);              logFile.print(',');
  logFile.print(now.unixtime());   logFile.print(',');
  logFile.print(millis());         logFile.print(',');
  logFile.print(avg);              logFile.print(',');
  logFile.print(tared);            logFile.print(',');
  logFile.print(grams, 2);         logFile.print(',');
  logFile.print(co2, 1);           logFile.print(',');
  logFile.print(tempC, 2);         logFile.print(',');
  logFile.print(rh, 1);            logFile.print(',');
  logFile.print(irMin, 1);         logFile.print(',');
  logFile.print(irMax, 1);         logFile.print(',');
  logFile.print(irMean, 1);        logFile.print(',');
  logFile.println(irHot);          // hottest-pixel index 0-63
  rowCount++;
}

// Append one full 8x8 frame (64 pixel temps + timestamp) to the paired thermal file.
// Opened and closed per write so each row is committed immediately (power-cut safe) and
// no second file is held open alongside the main log.
void writeThermalFrame() {
  File tf = SD.open(thermalName, FILE_WRITE);   // append
  if (!tf) return;
  DateTime now = rtc.now();
  char iso[] = "YYYY-MM-DD hh:mm:ss";
  now.toString(iso);
  tf.print(iso);            tf.print(',');
  tf.print(now.unixtime()); tf.print(',');
  tf.print(millis());
  for (uint8_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) { tf.print(','); tf.print(pixels[i], 1); }
  tf.println();
  tf.close();                                   // close flushes to card
}

// ---- Calibration ---------------------------------------------------------------

// Two-step guided cal on button B: capture empty-platform zero, then a known mass,
// and solve counts-per-gram. Saves to SD so it persists across reboots.
void runCalibration() {
  calScreen("CAL 1/2", "Empty platform", "then press B");
  waitCalButton();
  int32_t zero = sampleFresh(32);

  // Prompt for the known mass, printing the float directly (dtostrf isn't on SAMD).
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("CAL 2/2");
  display.setCursor(0, 22); display.print("Place "); display.print(KNOWN_MASS_G, 1); display.print("g");
  display.setCursor(0, 44); display.print("then press B");
  display.display();
  waitCalButton();
  int32_t loaded = sampleFresh(32);

  float cpg = (float)(loaded - zero) / KNOWN_MASS_G;
  if (cpg <= 0) {                                  // wrong load direction or no change
    calScreen("CAL FAILED", "reading <= 0", "check direction");
    delay(2500);
    return;
  }

  countsPerGram = cpg;
  tareOffset = zero;                               // adopt the cal zero as current tare
  saveCalibration();

  // Show the result, printing the float directly.
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("CAL DONE");
  display.setCursor(0, 22); display.print(cpg, 2); display.print(" cts/g");
  display.setCursor(0, 44); display.print("saved to SD");
  display.display();
  delay(2500);
}

// Wait for one full press+release of button B (blocking - cal is a bench step).
void waitCalButton() {
  while (digitalRead(CAL_BTN) == HIGH) delay(5);   // wait for press
  delay(40);
  while (digitalRead(CAL_BTN) == LOW) delay(5);    // wait for release
  delay(40);
}

void saveCalibration() {
  SD.remove(CAL_FILE);                             // FILE_WRITE appends, so clear first
  File f = SD.open(CAL_FILE, FILE_WRITE);
  if (f) { f.println(countsPerGram, 6); f.close(); }
}

void loadCalibration() {
  if (!SD.exists(CAL_FILE)) return;
  File f = SD.open(CAL_FILE);
  if (f) { countsPerGram = f.parseFloat(); f.close(); }
}

void calScreen(const char *l1, const char *l2, const char *l3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print(l1);
  display.setCursor(0, 22); display.print(l2);
  display.setCursor(0, 44); display.print(l3);
  display.display();
}

// ---- Display -------------------------------------------------------------------

void drawHeader() {
  DateTime now = rtc.now();
  char hhmmss[] = "hh:mm:ss";
  now.toString(hhmmss);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(hhmmss);
  const char *tag = (page == PAGE_WEIGHT) ? "WT"
                  : (page == PAGE_ENV)    ? "ENV"
                  : (page == PAGE_IR)     ? "IR"
                  : "INFO";
  int tagW = (int)strlen(tag) * 6;
  display.setCursor(128 - tagW, 0);
  display.print(tag);
  if (logging) { display.setCursor(128 - tagW - 12, 0); display.print("*"); }  // REC
}

void drawWeightPage() {
  int32_t tared = readAveraged() - tareOffset;
  display.setTextSize(3);
  display.setCursor(0, 20);
  if (countsPerGram > 0) display.print(tared / countsPerGram, 1);   // grams
  else                   display.print(tared);                      // raw counts
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print(countsPerGram > 0 ? "grams" : "counts (uncal)");
  display.setCursor(0, 54);
  display.print("C=tare  B=cal");
}

void drawEnvPage() {
  display.setTextSize(1);
  if (!haveSCD)  { display.setCursor(0, 28); display.print("SCD30 not found"); return; }
  if (!envValid) { display.setCursor(0, 28); display.print("CO2 warming up..."); return; }
  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print((int)co2);
  display.setTextSize(1);
  display.setCursor(0, 38);
  display.print("ppm CO2");
  display.setCursor(0, 48);
  display.print("T "); display.print(tempC, 1);
  display.print("C RH "); display.print((int)rh);
}

void drawIRPage() {
  display.setTextSize(1);
  if (!haveAMG) { display.setCursor(0, 28); display.print("AMG8833 not found"); return; }
  if (!irValid) { display.setCursor(0, 28); display.print("IR reading...");    return; }

  // 8x8 thermal "image": fill each cell whose temp is above the frame midpoint, so
  // the warm blob (a mouse) shows up regardless of absolute room temperature.
  // If the image looks mirrored vs reality, flip r or c in the index below.
  float thr = (irMin + irMax) * 0.5f;
  const int cell = 5, x0 = 0, y0 = 14;         // 8*5 = 40 px grid, y 14..54
  for (uint8_t r = 0; r < 8; r++)
    for (uint8_t c = 0; c < 8; c++)
      if (pixels[r * 8 + c] >= thr)
        display.fillRect(x0 + c * cell, y0 + r * cell, cell, cell, SH110X_WHITE);

  // Stats to the right of the grid.
  display.setCursor(48, 16); display.print("Mx"); display.print(irMax, 1);
  display.setCursor(48, 30); display.print("Mn"); display.print(irMin, 1);
  display.setCursor(48, 44); display.print("Av"); display.print(irMean, 1);
}

// Print a value as two digits with a leading zero (for HH:MM:SS).
void print2(uint32_t v) { if (v < 10) display.print('0'); display.print(v); }

// Print an elapsed-ms span as [Dd ]HH:MM:SS.
void printDuration(uint32_t ms) {
  uint32_t s = ms / 1000;
  uint32_t d = s / 86400; s %= 86400;
  uint32_t h = s / 3600;  s %= 3600;
  uint32_t m = s / 60;    s %= 60;
  if (d > 0) { display.print(d); display.print("d "); }
  print2(h); display.print(':'); print2(m); display.print(':'); print2(s);
}

void drawInfoPage() {
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("File ");
  display.print(logging ? logName : "none");
  display.setCursor(0, 30);
  display.print("Rows ");
  display.print(rowCount);
  display.setCursor(0, 44);
  display.print("Rec  ");
  if (logging) printDuration(millis() - logStartMillis);   // recording duration
  else         display.print("not logging");
  display.setCursor(0, 54);
  if (logging && haveAMG) { display.print("IR   "); display.print(thermalName); }
}

void drawScreen() {
  display.clearDisplay();
  drawHeader();
  switch (page) {
    case PAGE_WEIGHT: drawWeightPage(); break;
    case PAGE_ENV:    drawEnvPage();    break;
    case PAGE_IR:     drawIRPage();     break;
    case PAGE_INFO:   drawInfoPage();   break;
  }
  display.display();
}

void showError(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("ERROR:");
  display.setCursor(0, 12); display.print(msg);
  display.display();
  Serial.println(msg);
}

void showWarn(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("WARN:");
  display.setCursor(0, 12); display.print(msg);
  display.display();
  Serial.println(msg);
  delay(1500);
}
