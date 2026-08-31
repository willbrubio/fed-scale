/*
  Mouse Scale - bring-up sketch
  Feather M0 Adalogger + NAU7802 (load-cell ADC) + 128x64 SH1107 OLED FeatherWing

  Purpose: confirm the load cell reads cleanly BEFORE worrying about grams or the
  moving-mouse algorithm. Shows raw + tared ADC counts on the OLED and streams the
  tared value to Serial so you can open the Serial Plotter and watch the platform's
  mechanical response (press a corner vs. the center, look for drift/noise).

  Libraries (Library Manager): Adafruit NAU7802, Adafruit SH110X, Adafruit GFX.
  Board: "Adafruit Feather M0" (same target as FED).
*/

// Get Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>   // SH1107 driver for the STEMMA QT wing (NOT SSD1306)
#include <Adafruit_NAU7802.h>

// SH1107 panel is physically 64x128; we rotate it to a 128x64 landscape view.
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
#define OLED_ADDR 0x3C         // fixed I2C address of the OLED wing

Adafruit_NAU7802 nau;

// Tare button = wing button C (D5). Button A (D9) is avoided: on the Adalogger
// D9 doubles as the VBAT analog divider, so it's a poor choice for a clean input.
#define TARE_BTN 5

// Display smoothing: show the mean of the last AVG_N samples so the on-screen
// number doesn't jitter. Serial still gets every sample, so the plotter stays honest.
const uint8_t AVG_N = 8;
int32_t ring[AVG_N];           // circular buffer of recent raw readings
uint8_t ringIdx = 0;
bool ringFull = false;         // true once the buffer has wrapped at least once

int32_t tareOffset = 0;        // subtracted from readings to zero the platform

// Redraw the OLED at ~5 Hz. Redrawing every sample floods the I2C bus and flickers.
uint32_t lastDraw = 0;
const uint16_t DRAW_MS = 200;

void setup() {
  // Don't wait on Serial: this must run headless (in the cage) with no USB host.
  Serial.begin(115200);
  Wire.begin();

  // Bring the display up first so we have somewhere to print init errors.
  display.begin(OLED_ADDR, true);   // second arg performs a hardware reset
  display.setRotation(1);           // landscape; buttons end up on the left edge
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  // NAU7802 init. Failure here is almost always wiring or the load cell, so
  // surface it on-screen instead of hanging silently.
  if (!nau.begin()) {
    showError("NAU7802 not found");
    while (1) delay(10);            // nothing useful to do without the ADC
  }

  nau.setLDO(NAU7802_3V0);          // 3.0V excitation to the cell (safe below the 3.3V rail)
  nau.setGain(NAU7802_GAIN_128);    // max gain: load-cell output is only a few mV
  nau.setRate(NAU7802_RATE_80SPS);  // fast enough to see the platform respond live

  // Discard the first few conversions, then run the startup calibrations the chip needs.
  for (uint8_t i = 0; i < 10; i++) { while (!nau.available()) delay(1); nau.read(); }
  nau.calibrate(NAU7802_CALMOD_INTERNAL);  // internal ADC gain/offset cal
  nau.calibrate(NAU7802_CALMOD_OFFSET);    // zero the system offset (leave platform empty!)

  pinMode(TARE_BTN, INPUT_PULLUP);  // button reads LOW when pressed

  // Prime the ring buffer and seed the tare from the empty platform.
  for (uint8_t i = 0; i < AVG_N; i++) { while (!nau.available()) delay(1); pushSample(nau.read()); }
  tareOffset = readAveraged();
}

void loop() {
  // Take a sample whenever the ADC has one ready.
  if (nau.available()) {
    int32_t raw = nau.read();
    pushSample(raw);
    // Stream the tared sample for the Serial Plotter (true, unsmoothed response).
    Serial.print("Time:");
    Serial.print(millis());
    Serial.print("   Weight:");
    Serial.println(raw - tareOffset);
  }

  // Tare: press C to make the current load read zero (use to zero the platform).
  if (digitalRead(TARE_BTN) == LOW) {
    tareOffset = readAveraged();
    delay(250);                     // crude debounce; fine for a manual press
  }

  // Redraw on a fixed cadence rather than every sample.
  if (millis() - lastDraw >= DRAW_MS) {
    lastDraw = millis();
    drawScreen();
  }
}


/* -------------------------------------------
Define functions for use
*/ -------------------------------------------

// Insert a reading into the circular buffer.
void pushSample(int32_t v) {
  ring[ringIdx] = v;
  ringIdx = (ringIdx + 1) % AVG_N;
  if (ringIdx == 0) ringFull = true;
}


// Mean of the samples actually collected so far.
int32_t readAveraged() {
  uint8_t n = ringFull ? AVG_N : ringIdx;
  if (n == 0) return 0;
  int64_t sum = 0;                  // 64-bit: 8 x 24-bit values can't overflow this
  for (uint8_t i = 0; i < n; i++) sum += ring[i];
  return (int32_t)(sum / n);
}


// ---  SCREEN CONTROL FUNCTION IN OPERATION --- //
// Dimensions: 64 x 128
void drawScreen() {
  int32_t avg = readAveraged();
  int32_t tared = avg - tareOffset;

  display.clearDisplay();

  // Header
  display.setTextSize(1); // size = 1
  display.setCursor(0, 0); // x = 0, y = 0
  display.print("Mouse Scale");

  // Big tared value: the number to watch while pressing the platform.
  // Still raw ADC counts, not grams. To calibrate later: place a known mass,
  // read tared counts, CAL = counts / grams; then grams = tared / CAL.
  display.setTextSize(2); // size = 2
  display.setCursor(0, 20); // x = 0, y = 20
  display.print(tared);

  // Untared (pre-zero) counts, small, for reference.
  display.setTextSize(1); // size = 2
  display.setCursor(0, 48); // x = 0, y = 48
  display.print("raw:");
  display.print(avg);

  // Button instructions
  display.setCursor(0, 56); // x = 0, y = 56
  display.print("C = zero");

  display.display();
}

// ---  SCREEN CONTROL FUNCTION IN ERROR --- //
// Define display for error
void showError(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("ERROR:");
  display.setCursor(0, 12);
  display.print(msg);
  display.display();
  Serial.println(msg);
}
