#include "display.h"
#include <SPI.h>

bool Display::begin() {
  SPI.begin();
  screen_.init();
  return canvas_.begin(screen_);
}

void Display::clear(uint16_t color) {
  canvas_.clear(color);
}

void Display::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  canvas_.fillRect(x, y, w, h, color);
}

void Display::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  canvas_.drawRect(x, y, w, h, color);
}

void Display::present() {
  canvas_.present();
}

void Display::presentRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  canvas_.presentRect(x, y, w, h);
}
