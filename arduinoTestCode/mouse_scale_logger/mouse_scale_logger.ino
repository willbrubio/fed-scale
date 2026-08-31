/*
  Mouse Scale - logging build (PCF8523 RTC)
  Feather M0 Adalogger (onboard microSD, CS=4) + NAU7802 + PCF8523 RTC + SH1107 OLED wing

  Reads the load cell, shows it on the OLED, streams raw samples to Serial (Serial
  Plotter), and writes timestamped CSV rows to a fresh SD file each run.

  Scope: load cell + OLED + RTC + SD. The SCD-30 and AMG8833 slot onto the same I2C
  chain later - the 100 kHz bus set here is already SCD-30-safe.

  Libraries: Adafruit NAU7802, Adafruit SH110X, Adafruit GFX, RTClib. (SD is built in.)
  Board: "Adafruit Feather M0".
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NAU7802.h>
#include "RTClib.h"

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
#define OLED_ADDR 0x3C

Adafruit_NAU7802 nau;
RTC_PCF8523 rtc;               // PCF8523 (NOT DS3231): shares 0x68 but different registers

// Onboard microSD chip-select on the M0 Adalogger is D4 (NOT D10 - that's the
// datalogger *wing*'s CS). A wrong CS here is the usual "SD failed" cause.
#define SD_CS 4

#define TARE_BTN 5             // wing button C; A (D9) doubles as VBAT on the Adalogger

// Display smoothing only; Serial still gets every raw sample for the plotter.
const uint8_t AVG_N = 8;
int32_t ring[AVG_N];
uint8_t ringIdx = 0;
bool ringFull = false;

int32_t tareOffset = 0;

// Log one averaged row every LOG_MS. 10 Hz keeps files small and is plenty for a
// weight/feeder trace; drop this number for finer time resolution later.
const uint16_t LOG_MS = 100;
uint32_t lastLog = 0;

const uint16_t DRAW_MS = 200;  // OLED refresh ~5 Hz
uint32_t lastDraw = 0;

// Flush to card once a second so a hard power cut loses <1 s of data.
const uint16_t FLUSH_MS = 1000;
uint32_t lastFlush = 0;

File logFile;
char logName[13] = "DATA0000.CSV";  // 8.3 name; bumped until an unused one is found
bool logging = false;               // false if SD init fails -> run display-only
uint32_t rowCount = 0;

void setup() {
  Serial.begin(115200);            // no while(!Serial): must run headless in the cage
  Wire.begin();

  // Display first so any init error has somewhere to show.
  display.begin(OLED_ADDR, true);
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  // Load-cell ADC.
  if (!nau.begin()) { showError("NAU7802 not found"); while (1) delay(10); }
  nau.setLDO(NAU7802_3V0);         // 3.0V excitation, safe below the 3.3V rail
  nau.setGain(NAU7802_GAIN_128);   // max gain - load-cell output is only a few mV
  nau.setRate(NAU7802_RATE_80SPS);
  for (uint8_t i = 0; i < 10; i++) { while (!nau.available()) delay(1); nau.read(); }
  nau.calibrate(NAU7802_CALMOD_INTERNAL);
  nau.calibrate(NAU7802_CALMOD_OFFSET);   // zero the offset - leave platform empty!

  // RTC (PCF8523). Only set the time if it was never set or lost power; then start()
  // it, since the PCF8523 can power up with its oscillator halted.
  if (!rtc.begin()) { showError("RTC not found"); while (1) delay(10); }
  if (!rtc.initialized() || rtc.lostPower())
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  rtc.start();

  pinMode(TARE_BTN, INPUT_PULLUP);

  // Prime the ring buffer and tare from the empty platform.
  for (uint8_t i = 0; i < AVG_N; i++) { while (!nau.available()) delay(1); pushSample(nau.read()); }
  tareOffset = readAveraged();

  // SD: pick the next unused DATA####.CSV so runs never overwrite each other.
  if (SD.begin(SD_CS)) {
    for (uint16_t i = 0; i < 10000; i++) {
      sprintf(logName, "DATA%04u.CSV", i);
      if (!SD.exists(logName)) break;       // first free name wins
    }
    logFile = SD.open(logName, FILE_WRITE);
    if (logFile) {
      logFile.println("iso,unixtime,millis,raw,tared");   // CSV header
      logFile.flush();
      logging = true;
    }
  }
  if (!logging) showWarn("SD failed - display only");

  // Set the bus speed LAST so no library's begin() can override it back to 400 kHz.
  // 100 kHz is the ceiling the SCD-30 will need when it joins the chain.
  Wire.setClock(100000);
}

void loop() {
  // Grab a sample whenever the ADC has one ready.
  if (nau.available()) {
    int32_t raw = nau.read();
    pushSample(raw);
    Serial.println(raw - tareOffset);       // every sample -> Serial Plotter
  }

  // Tare on button C: current load becomes zero.
  if (digitalRead(TARE_BTN) == LOW) { tareOffset = readAveraged(); delay(250); }

  // Timestamped log row at LOG_MS cadence.
  if (logging && millis() - lastLog >= LOG_MS) { lastLog = millis(); logRow(); }

  // Periodic flush guards against data loss on a sudden power cut.
  if (logging && millis() - lastFlush >= FLUSH_MS) { lastFlush = millis(); logFile.flush(); }

  // Redraw the OLED on its own cadence.
  if (millis() - lastDraw >= DRAW_MS) { lastDraw = millis(); drawScreen(); }
}

void pushSample(int32_t v) {
  ring[ringIdx] = v;
  ringIdx = (ringIdx + 1) % AVG_N;
  if (ringIdx == 0) ringFull = true;
}

int32_t readAveraged() {
  uint8_t n = ringFull ? AVG_N : ringIdx;
  if (n == 0) return 0;
  int64_t sum = 0;                          // 64-bit: 8 x 24-bit can't overflow
  for (uint8_t i = 0; i < n; i++) sum += ring[i];
  return (int32_t)(sum / n);
}

void logRow() {
  DateTime now = rtc.now();
  char iso[] = "YYYY-MM-DD hh:mm:ss";       // toString() rewrites this buffer in place
  now.toString(iso);
  int32_t avg = readAveraged();

  // Columns: human time, unix time, board millis (orders sub-second rows), raw, tared.
  // millis matters because the RTC only resolves to 1 s; for tight cross-device timing
  // you'd lean on the FED BNC fiducial sync, not this timestamp.
  logFile.print(iso);            logFile.print(',');
  logFile.print(now.unixtime()); logFile.print(',');
  logFile.print(millis());       logFile.print(',');
  logFile.print(avg);            logFile.print(',');
  logFile.println(avg - tareOffset);
  rowCount++;
}

void drawScreen() {
  int32_t avg = readAveraged();
  int32_t tared = avg - tareOffset;

  DateTime now = rtc.now();
  char hhmmss[] = "hh:mm:ss";
  now.toString(hhmmss);

  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(hhmmss);                    // clock, top-left

  // Big tared value - still raw ADC counts, not grams (calibration is the next step).
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(tared);

  display.setTextSize(1);
  display.setCursor(0, 36);
  if (logging) { display.print("LOG "); display.print(logName); }
  else         { display.print("NO SD"); }

  display.setCursor(0, 46);
  display.print("rows:");
  display.print(rowCount);

  display.setCursor(0, 56);
  display.print("C=zero");

  display.display();
}

void showError(const char *msg) {          // fatal: show and halt
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("ERROR:");
  display.setCursor(0, 12); display.print(msg);
  display.display();
  Serial.println(msg);
}

void showWarn(const char *msg) {           // non-fatal: flash and continue
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("WARN:");
  display.setCursor(0, 12); display.print(msg);
  display.display();
  Serial.println(msg);
  delay(1500);
}
