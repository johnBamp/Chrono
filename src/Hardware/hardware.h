#pragma once
#include <Arduino.h>
#include <driver/i2s.h>  // for i2s_port_t / I2S_NUM_0 on ESP32 (optional but nice)

namespace hw {
    constexpr int NC = -1;   // Not connected

    // ---- TFT ----
    constexpr int TFT_DC   = 14;
    constexpr int TFT_CS   = 32;
    constexpr int TFT_LITE = 15;
    constexpr int TFT_RST  = NC;

    // ---- UI / misc ----
    constexpr int MENU_PIN = 27;
    constexpr int SD_CS    = 5;

    // ---- Power sense ----
    constexpr int VBATPIN        = A13;
    constexpr int VBUS_SENSE_PIN = NC;

    // ---- Audio ----
    constexpr int SPEAKER_PIN = 33;

    constexpr size_t BUF_LEN      = 2048;
    constexpr uint32_t SAMPLE_RATE = 44100;

    constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

    // I2S pins
    constexpr int DATA_PIN = 26;
    constexpr int BCLK_PIN = 12;
    constexpr int WS_PIN   = 25;

} // namespace hw
