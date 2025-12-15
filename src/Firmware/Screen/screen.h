#pragma once

#include <Arduino.h>
#include "Hardware/hardware.h"   // provides TFT_CS, TFT_DC, TFT_RST, TFT_LITE, etc.

class Screen;
class Canvas565 {
public:
  Canvas565() = default;
  ~Canvas565() { end(); }

  bool begin(Screen& screen);   // allocates framebuffer
  void end();                   // frees framebuffer

  uint16_t width()  const { return w_; }
  uint16_t height() const { return h_; }

  // Drawing into the framebuffer (NOT the screen)
  void clear(uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

  // Push framebuffer to the screen
  void present();                                // full screen
  void presentRect(int16_t x, int16_t y, int16_t w, int16_t h); // partial

  // Debug / advanced use
  const uint16_t* data() const { return buf_; }
  uint16_t* data() { return buf_; }

private:
  void markDirty(int16_t x, int16_t y, int16_t w, int16_t h);
  bool hasDirty() const { return dirtyX0_ <= dirtyX1_ && dirtyY0_ <= dirtyY1_; }
  void resetDirty() { dirtyX0_ = w_; dirtyY0_ = h_; dirtyX1_ = -1; dirtyY1_ = -1; }

  Screen* screen_ = nullptr;
  uint16_t* buf_ = nullptr;  // stored byte-swapped for fast SPI
  uint16_t w_ = 0;
  uint16_t h_ = 0;

  int16_t dirtyX0_ = 0;
  int16_t dirtyY0_ = 0;
  int16_t dirtyX1_ = -1;
  int16_t dirtyY1_ = -1;
};

// Low-level screen driver (ILI9341 over SPI).
// You generally won't call draw functions here; you draw via Canvas565.
class Screen {
public:
  Screen() = default;

  void init();

  static uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b);

  static constexpr uint16_t width()  { return 240; }
  static constexpr uint16_t height() { return 320; }

private:
  friend class Canvas565;

  // Push a rectangle of RGB565 *wire-order* pixels (byte-swapped) to the panel.
  void pushRect565Wire(const uint16_t* wirePixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

  // ---- Low-level I/O ----
  void startWrite();
  void endWrite();
  void dcCommand();
  void dcData();

  void write8(uint8_t v);
  void writeBytes(const uint8_t* data, size_t n);

  void tftCmd(uint8_t c);
  void tftData8(uint8_t d);
  void tftDataN(const uint8_t* data, size_t n);

  // Caller must hold an active SPI transaction via startWrite()/endWrite().
  void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
};
