#include "Arduino.h"
#include <SPI.h>

#include "Firmware/Screen/screen.h"

Screen tft;

void setup() {
  Serial.begin(115200);

  SPI.begin();
  tft.init();

  tft.tftFillRect(0, 0, 320, 240, 0x0000);

  tft.tftDrawRect(0, 0, 100, 100, tft.RGB565(255, 0, 0));
  


}

void loop() {

}
