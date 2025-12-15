#include "screen.h"
#include <SPI.h>
namespace {
    enum class Cmd : uint8_t {
        SWRESET = 0x01,
        SLPOUT  = 0x11,
        DISPON  = 0x29,
        CASET   = 0x2A,
        PASET   = 0x2B,
        RAMWR   = 0x2C,
        MADCTL  = 0x36,
        PIXFMT  = 0x3A,
    };

    constexpr uint32_t TFT_SPI_HZ = 40000000UL; // no digit separators
    constexpr uint16_t TFT_W = 240;
    constexpr uint16_t TFT_H = 320;

    SPISettings tftSPI(TFT_SPI_HZ, MSBFIRST, SPI_MODE0);

    inline void tftSelect()   { digitalWrite(TFT_CS, LOW); }
    inline void tftDeselect() { digitalWrite(TFT_CS, HIGH); }
}

Screen::Screen(){
    return;
}

// Example colors
uint16_t Screen::RGB565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

void Screen::tftCmd(uint8_t c) {
  SPI.beginTransaction(tftSPI);
  tftSelect();
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(c);
  tftDeselect();
  SPI.endTransaction();
}

void Screen::tftData8(uint8_t d) {
  SPI.beginTransaction(tftSPI);
  tftSelect();
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(d);
  tftDeselect();
  SPI.endTransaction();
}

void Screen::tftData16(uint16_t d) {
  SPI.beginTransaction(tftSPI);
  tftSelect();
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer((uint8_t)(d >> 8));
  SPI.transfer((uint8_t)(d & 0xFF));
  tftDeselect();
  SPI.endTransaction();
}

void Screen::tftDataN(const uint8_t* data, size_t n) {
  SPI.beginTransaction(tftSPI);
  tftSelect();
  digitalWrite(TFT_DC, HIGH);
  while (n--) SPI.transfer(*data++);
  tftDeselect();
  SPI.endTransaction();
}

// -------- Address window + drawing --------
void Screen::tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  tftCmd(static_cast<uint8_t>(Cmd::CASET));
  uint8_t caset[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
  tftDataN(caset, 4);

  tftCmd(static_cast<uint8_t>(Cmd::PASET));
  uint8_t paset[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
  tftDataN(paset, 4);

  tftCmd(static_cast<uint8_t>(Cmd::RAMWR));
}

// Fill a rectangle with RGB565 color
void Screen::tftFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w <= 0 || h <= 0) return;

  // Clip
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > (int16_t)TFT_W) w = TFT_W - x;
  if (y + h > (int16_t)TFT_H) h = TFT_H - y;
  if (w <= 0 || h <= 0) return;

  tftSetAddrWindow(x, y, x + w - 1, y + h - 1);

  // Stream pixels (write repeated color efficiently in chunks)
  const uint32_t pixels = (uint32_t)w * (uint32_t)h;
  const uint8_t hi = (uint8_t)(color >> 8);
  const uint8_t lo = (uint8_t)(color & 0xFF);

  // Build a small buffer of repeated color bytes
  uint8_t buf[64]; // 32 pixels worth (2 bytes each)
  for (int i = 0; i < (int)sizeof(buf); i += 2) { buf[i] = hi; buf[i + 1] = lo; }

  SPI.beginTransaction(tftSPI);
  tftSelect();
  digitalWrite(TFT_DC, HIGH);

  uint32_t remaining = pixels;
  while (remaining) {
    uint32_t chunkPixels = remaining;
    if (chunkPixels > (sizeof(buf) / 2)) chunkPixels = (sizeof(buf) / 2);
    // send chunkPixels * 2 bytes
    for (uint32_t i = 0; i < chunkPixels * 2; i++) SPI.transfer(buf[i]);
    remaining -= chunkPixels;
  }

  tftDeselect();
  SPI.endTransaction();
}

// Draw a rectangle outline (1px)
void Screen::tftDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  tftFillRect(x, y, w, 1, color);
  tftFillRect(x, y + h - 1, w, 1, color);
  tftFillRect(x, y, 1, h, color);
  tftFillRect(x + w - 1, y, 1, h, color);
}

// -------- Init sequence --------
void Screen::init() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH); // backlight on (change if active-low)

  if (TFT_RST >= 0) {
    pinMode(TFT_RST, OUTPUT);
    digitalWrite(TFT_RST, HIGH);
    delay(10);
    digitalWrite(TFT_RST, LOW);
    delay(20);
    digitalWrite(TFT_RST, HIGH);
    delay(120);
  } else {
    // Software reset if no hardware reset pin
    tftCmd(static_cast<uint8_t>(Cmd::SWRESET));
    delay(150);
  }

  // ---- Common ILI9341 init (works for most modules) ----
  // (These values are widely used across known-good init sequences.)
  tftCmd(0xCF);  { uint8_t d[] = {0x00, 0xC1, 0x30}; tftDataN(d, 3); }
  tftCmd(0xED);  { uint8_t d[] = {0x64, 0x03, 0x12, 0x81}; tftDataN(d, 4); }
  tftCmd(0xE8);  { uint8_t d[] = {0x85, 0x00, 0x78}; tftDataN(d, 3); }
  tftCmd(0xCB);  { uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02}; tftDataN(d, 5); }
  tftCmd(0xF7);  { uint8_t d[] = {0x20}; tftDataN(d, 1); }
  tftCmd(0xEA);  { uint8_t d[] = {0x00, 0x00}; tftDataN(d, 2); }

  tftCmd(0xC0);  { uint8_t d[] = {0x23}; tftDataN(d, 1); } // Power control VRH
  tftCmd(0xC1);  { uint8_t d[] = {0x10}; tftDataN(d, 1); } // Power control SAP/BT
  tftCmd(0xC5);  { uint8_t d[] = {0x3E, 0x28}; tftDataN(d, 2); } // VCOM
  tftCmd(0xC7);  { uint8_t d[] = {0x86}; tftDataN(d, 1); } // VCOM offset

  // Memory Access Control (rotation + RGB/BGR)
  // 0x48 = MX + BGR (common). If colors are swapped, try toggling BGR bit (0x08).
  tftCmd(static_cast<uint8_t>(Cmd::MADCTL));
  tftData8(0x48);

  // Pixel format: 16-bit
  tftCmd(static_cast<uint8_t>(Cmd::PIXFMT));
  tftData8(0x55);

  tftCmd(0xB1);  { uint8_t d[] = {0x00, 0x18}; tftDataN(d, 2); } // Frame rate
  tftCmd(0xB6);  { uint8_t d[] = {0x08, 0x82, 0x27}; tftDataN(d, 3); } // Display function
  tftCmd(0xF2);  { uint8_t d[] = {0x00}; tftDataN(d, 1); } // 3Gamma off
  tftCmd(0x26);  { uint8_t d[] = {0x01}; tftDataN(d, 1); } // Gamma curve

  // Positive gamma correction
  tftCmd(0xE0); {
    uint8_t d[] = {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00};
    tftDataN(d, sizeof(d));
  }
  // Negative gamma correction
  tftCmd(0xE1); {
    uint8_t d[] = {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F};
    tftDataN(d, sizeof(d));
  }

  tftCmd(static_cast<uint8_t>(Cmd::SLPOUT));
  delay(120);
  tftCmd(static_cast<uint8_t>(Cmd::DISPON));
  delay(20);
}



