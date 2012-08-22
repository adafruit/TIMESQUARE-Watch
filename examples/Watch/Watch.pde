#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

#define MODE_SET  0
#define MODE_TIME 1

Watch      watch(true); // Use double-buffered animation
RTC_DS1307 RTC;
uint8_t    mode = MODE_TIME;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  watch.begin();
}

void loop() {
  switch(mode) {
   case MODE_SET:
    break;
   case MODE_TIME:
    break;
  }
  watch.swapBuffers();
}

