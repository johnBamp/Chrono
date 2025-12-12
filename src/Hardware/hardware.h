#pragma once

#define TFT_DC 14
#define TFT_CS 32
#define TFT_LITE 15

#define MENU_PIN 27
#define SD_CS 5
#define VBATPIN A13
#define VBUS_SENSE_PIN -1
// Route the board's VBUS pad (5V) through a small divider (e.g., 100k/100k) to GPIO x (input-only)

#define SPEAKER_PIN 33 //A9

#define BUF_LEN 2048
#define SAMPLE_RATE 44100
#define I2S_PORT I2S_NUM_0

#define DATA_PIN 26  // A0 on Feather ESP32 V2
#define BCLK_PIN 12  // GPIO12
#define WS_PIN 25    // A1 on Feather ESP32 V2

#define I2S_PORT I2S_NUM_0
