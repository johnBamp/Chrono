#include "screen.h"
#include <SPI.h>

#if defined(ESP32)
  #include "esp_heap_caps.h"
  #include "esp_system.h"
#endif

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

  constexpr uint32_t TFT_SPI_HZ = 80000000;
  SPISettings tftSPI(TFT_SPI_HZ, MSBFIRST, SPI_MODE0);

  inline void tftSelect()   { digitalWrite(hw::TFT_CS, LOW); }
  inline void tftDeselect() { digitalWrite(hw::TFT_CS, HIGH); }

  inline uint16_t bswap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }
}

uint16_t Screen::RGB565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// ---- Screen low-level ----
void Screen::startWrite() {
  SPI.beginTransaction(tftSPI);
  tftSelect();
}

void Screen::endWrite() {
  tftDeselect();
  SPI.endTransaction();
}

void Screen::dcCommand() { digitalWrite(hw::TFT_DC, LOW); }
void Screen::dcData()    { digitalWrite(hw::TFT_DC, HIGH); }

void Screen::write8(uint8_t v) { SPI.transfer(v); }

void Screen::writeBytes(const uint8_t* data, size_t n) {
#if defined(ESP32)
  SPI.writeBytes(data, n);
#else
  while (n--) SPI.transfer(*data++);
#endif
}

void Screen::tftCmd(uint8_t c) {
  startWrite();
  dcCommand();
  write8(c);
  endWrite();
}

void Screen::tftData8(uint8_t d) {
  startWrite();
  dcData();
  write8(d);
  endWrite();
}

void Screen::tftDataN(const uint8_t* data, size_t n) {
  startWrite();
  dcData();
  writeBytes(data, n);
  endWrite();
}

void Screen::tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  startWrite();

  dcCommand();
  write8((uint8_t)Cmd::CASET);
  dcData();
  uint8_t caset[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
  writeBytes(caset, 4);

  dcCommand();
  write8((uint8_t)Cmd::PASET);
  dcData();
  uint8_t paset[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
  writeBytes(paset, 4);

  dcCommand();
  write8((uint8_t)Cmd::RAMWR);

  endWrite();
}

void Screen::pushRect565Wire(const uint16_t* wirePixels,
                             uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  if (!wirePixels || w == 0 || h == 0) return;

  tftSetAddrWindow(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));

  // Now stream pixel data
  startWrite();
  dcData();
  writeBytes((const uint8_t*)wirePixels, (size_t)w * (size_t)h * 2);
  endWrite();
}

// ---- Init ----
void Screen::init() {
  pinMode(hw::TFT_CS, OUTPUT);
  pinMode(hw::TFT_DC, OUTPUT);
  digitalWrite(hw::TFT_CS, HIGH);
  digitalWrite(hw::TFT_DC, HIGH);

  pinMode(hw::TFT_LITE, OUTPUT);
  digitalWrite(hw::TFT_LITE, HIGH);

  if (hw::TFT_RST >= 0) {
    pinMode(hw::TFT_RST, OUTPUT);
    digitalWrite(hw::TFT_RST, HIGH);
    delay(10);
    digitalWrite(hw::TFT_RST, LOW);
    delay(20);
    digitalWrite(hw::TFT_RST, HIGH);
    delay(120);
  } else {
    tftCmd((uint8_t)Cmd::SWRESET);
    delay(150);
  }

  // Common ILI9341 init (same as you had)
  tftCmd(0xCF);  { uint8_t d[] = {0x00, 0xC1, 0x30}; tftDataN(d, 3); }
  tftCmd(0xED);  { uint8_t d[] = {0x64, 0x03, 0x12, 0x81}; tftDataN(d, 4); }
  tftCmd(0xE8);  { uint8_t d[] = {0x85, 0x00, 0x78}; tftDataN(d, 3); }
  tftCmd(0xCB);  { uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02}; tftDataN(d, 5); }
  tftCmd(0xF7);  { uint8_t d[] = {0x20}; tftDataN(d, 1); }
  tftCmd(0xEA);  { uint8_t d[] = {0x00, 0x00}; tftDataN(d, 2); }

  tftCmd(0xC0);  { uint8_t d[] = {0x23}; tftDataN(d, 1); }
  tftCmd(0xC1);  { uint8_t d[] = {0x10}; tftDataN(d, 1); }
  tftCmd(0xC5);  { uint8_t d[] = {0x3E, 0x28}; tftDataN(d, 2); }
  tftCmd(0xC7);  { uint8_t d[] = {0x86}; tftDataN(d, 1); }

  tftCmd((uint8_t)Cmd::MADCTL);
  tftData8(0x48);

  tftCmd((uint8_t)Cmd::PIXFMT);
  tftData8(0x55);

  tftCmd(0xB1);  { uint8_t d[] = {0x00, 0x18}; tftDataN(d, 2); }
  tftCmd(0xB6);  { uint8_t d[] = {0x08, 0x82, 0x27}; tftDataN(d, 3); }
  tftCmd(0xF2);  { uint8_t d[] = {0x00}; tftDataN(d, 1); }
  tftCmd(0x26);  { uint8_t d[] = {0x01}; tftDataN(d, 1); }

  tftCmd((uint8_t)Cmd::SLPOUT);
  delay(120);
  tftCmd((uint8_t)Cmd::DISPON);
  delay(20);
}

// ---- Canvas565 implementation ----
static uint16_t* allocFB(size_t bytes) {
#if defined(ESP32)
  // Try DMA-capable internal RAM first (fast), then PSRAM, then heap.
  uint16_t* p = (uint16_t*)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
  if (p) return p;

  if (psramFound()) {
    p = (uint16_t*)ps_malloc(bytes);
    if (p) return p;
  }

  return (uint16_t*)malloc(bytes);
#else
  return (uint16_t*)malloc(bytes);
#endif
}

bool Canvas565::begin(Screen& screen) {
  end();

  screen_ = &screen;
  w_ = Screen::width();
  h_ = Screen::height();

  const size_t bytes = (size_t)w_ * (size_t)h_ * 2;
  buf_ = allocFB(bytes);
  if (!buf_) {
    screen_ = nullptr;
    w_ = h_ = 0;
    return false;
  }

  clear(0x0000);
  return true;
}

void Canvas565::end() {
  if (buf_) {
#if defined(ESP32)
    heap_caps_free(buf_);
#else
    free(buf_);
#endif
  }
  buf_ = nullptr;
  screen_ = nullptr;
  w_ = h_ = 0;
}

void Canvas565::clear(uint16_t color) {
  if (!buf_) return;
  const uint16_t wire = bswap16(color);
  const uint32_t n = (uint32_t)w_ * (uint32_t)h_;
  for (uint32_t i = 0; i < n; i++) buf_[i] = wire;
}

void Canvas565::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (!buf_ || w <= 0 || h <= 0) return;

  // Clip
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > (int16_t)w_) w = (int16_t)w_ - x;
  if (y + h > (int16_t)h_) h = (int16_t)h_ - y;
  if (w <= 0 || h <= 0) return;

  const uint16_t wire = bswap16(color);
  const int16_t stride = (int16_t)w_;

  for (int16_t row = 0; row < h; row++) {
    uint16_t* dst = buf_ + (y + row) * stride + x;
    for (int16_t col = 0; col < w; col++) dst[col] = wire;
  }
}

void Canvas565::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  fillRect(x, y, w, 1, color);
  fillRect(x, y + h - 1, w, 1, color);
  fillRect(x, y, 1, h, color);
  fillRect(x + w - 1, y, 1, h, color);
}

void Canvas565::present() {
  if (!buf_ || !screen_) return;
  screen_->pushRect565Wire(buf_, 0, 0, w_, h_);
}

void Canvas565::presentRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  if (!buf_ || !screen_ || w <= 0 || h <= 0) return;

  // Clip to canvas bounds
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > (int16_t)w_) w = (int16_t)w_ - x;
  if (y + h > (int16_t)h_) h = (int16_t)h_ - y;
  if (w <= 0 || h <= 0) return;

  // Push row-by-row (since rect isn't contiguous as one block in memory unless full-width)
  screen_->tftSetAddrWindow((uint16_t)x, (uint16_t)y,
                            (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));

  screen_->startWrite();
  screen_->dcData();

  const int16_t stride = (int16_t)w_;
  for (int16_t row = 0; row < h; row++) {
    const uint16_t* src = buf_ + (y + row) * stride + x;
    screen_->writeBytes((const uint8_t*)src, (size_t)w * 2);
  }

  screen_->endWrite();
}
