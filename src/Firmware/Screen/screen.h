#pragma once
#include "Hardware/hardware.h"
#include "Arduino.h"
class Screen {
    private:
        void tftCmd(uint8_t c);
        void tftData8(uint8_t d);
        void tftData16(uint16_t d);
        void tftDataN(const uint8_t* data, size_t n);
        void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

    public:
        Screen();

        void init(); 

        static uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b);

        static constexpr uint16_t width()  { return 240; }
        static constexpr uint16_t height() { return 320; }

        void tftFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        void tftDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

};
