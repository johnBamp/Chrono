#include "Arduino.h"
#include <SPI.h>
#include "Firmware/Screen/screen.h"

#include "Firmware/Display/display.h"

static uint32_t frames = 0;
static uint32_t lastReportMs = 0;

static inline void reportFPS(const char* label) {
  uint32_t now = millis();
  if (now - lastReportMs >= 1000) {
    float fps = frames * 1000.0f / float(now - lastReportMs);
    Serial.print(label);
    Serial.print(" FPS: ");
    Serial.println(fps, 2);
    frames = 0;
    lastReportMs = now;
  }
}

// Simple fast-ish PRNG (faster than random())
static uint32_t rng = 123456789;
static inline uint32_t xorshift32() {
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return rng;
}

Display tft;

void setup() {
  Serial.begin(115200);
  delay(200);

  SPI.begin();

  if (!tft.begin()) {
    Serial.println("Display framebuffer alloc failed!");
    while (true) delay(1000);
  }

  tft.clear(0x0000);

  Serial.println("Starting TFT benchmark...");
  lastReportMs = millis();
}

void loop() {
  constexpr int16_t W = 240;
  constexpr int16_t H = 320;

  // ---- TEST 1: Full-screen fill (measures raw pixel throughput) ----
  {
    uint16_t c1 = tft.RGB565(0, 0, 0);
    uint16_t c2 = tft.RGB565(0, 80, 255);

    tft.fillRect(0, 0, W, H, c1);
    tft.present();  // push to screen

    frames++;
    reportFPS("FillScreen");

    tft.fillRect(0, 0, W, H, c2);
    tft.present();  // push to screen

    frames++;
    reportFPS("FillScreen");
  }

}
