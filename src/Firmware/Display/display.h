#pragma once
#include <Arduino.h>
#include "Firmware/Screen/screen.h"   // brings in Screen + Canvas565

class Display {
public:
  // Starts SPI, inits screen, allocates framebuffer.
  // Returns false if framebuffer allocation fails.
  bool begin();

  // Common helpers
  static uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) { return Screen::RGB565(r, g, b); }
  static constexpr uint16_t width()  { return Screen::width(); }
  static constexpr uint16_t height() { return Screen::height(); }

  // Draw to the canvas (NOT directly to the screen)
  void clear(uint16_t color = 0x0000);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

  // Push canvas to screen
  void present();
  void presentRect(int16_t x, int16_t y, int16_t w, int16_t h);

private:
  Screen screen_;
  Canvas565 canvas_;
};
