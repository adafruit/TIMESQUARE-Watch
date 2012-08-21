#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

Watch watch(true); // Use double-buffered animation
RTC_DS1307 RTC;

uint8_t mode = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  watch.begin();
}

void loop() {
  switch(mode) {
   case 0:
    break;
  }
  watch.swapBuffers();
}

