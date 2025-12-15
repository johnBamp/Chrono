#include "Arduino.h"
#include <SPI.h>
#include "Firmware/Screen/screen.h"

#include "Firmware/Display/display.h"

namespace {
struct FpsCounter {
  const char* label;
  uint32_t frames = 0;
  uint32_t lastReportMs = 0;

  explicit constexpr FpsCounter(const char* name) : label(name) {}

  void tick() {
    const uint32_t now = millis();
    if (lastReportMs == 0) {
      lastReportMs = now;
    }

    frames++;
    const uint32_t elapsed = now - lastReportMs;
    if (elapsed >= 1000) {
      const float fps = frames * 1000.0f / static_cast<float>(elapsed);
      Serial.print(label);
      Serial.print(" FPS: ");
      Serial.println(fps, 2);
      frames = 0;
      lastReportMs = now;
    }
  }
};

constexpr int16_t kScreenWidth  = static_cast<int16_t>(Display::width());
constexpr int16_t kScreenHeight = static_cast<int16_t>(Display::height());

void fillFrame(Display& display, uint16_t color, FpsCounter& counter) {
  display.fillRect(0, 0, kScreenWidth, kScreenHeight, color);
  display.present();
  counter.tick();
}
}  // namespace

Display tft;

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!tft.begin()) {
    Serial.println("Display framebuffer alloc failed!");
    while (true) delay(1000);
  }

  tft.clear(0x0000);

  Serial.println("Starting TFT benchmark...");
}

void loop() {
  static FpsCounter fillFps("FillScreen");
  static const uint16_t colorDark = Display::RGB565(0, 0, 0);
  static const uint16_t colorBlue = Display::RGB565(0, 80, 255);

  // ---- TEST 1: Full-screen fill (measures raw pixel throughput) ----
  fillFrame(tft, colorDark, fillFps);
  fillFrame(tft, colorBlue, fillFps);
  yield();  // keep watchdogs happy when running the tight render loop
}
