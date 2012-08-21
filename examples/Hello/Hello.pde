#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

Watch watch(true); // Use double-buffered animation
RTC_DS1307 RTC;
int  textX = 8;
byte ramp[64];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  watch.begin();
  watch.setTextWrap(false);

  // Calc gamma-corrected 'gray' ramp:
  for(byte i=0; i<64; i++) {
    ramp[i] = (int)(255.0 * pow((float)i / 63.0, 2.6) + 0.5);
  }
}

void loop() {
  uint8_t x, y, i;
  uint8_t b = watch.buttons();

  if(b == 1) {
    watch.fillScreen(0);
    watch.setCursor(1, 0);
    watch.setTextColor(255);
    watch.print("L");
  } else if(b == 2) {
    watch.fillScreen(0);
    watch.setCursor(1, 0);
    watch.setTextColor(255);
    watch.print("R");
  } else if(b == 3) {
    watch.fillScreen(0);
    watch.setCursor(1, 0);
    watch.setTextColor(255);
    watch.print("B");
  } else { // b == 0

//    for(i=y=0;y<8;y++) {
//      for(x=0;x<8;x++) {
//        watch.drawPixel(x, y, ramp[i++]);
//      }
//    }

    watch.fillScreen(20);

    watch.setCursor(textX + 1, 1);
    watch.setTextColor(0);
    watch.print("Hello world");
  
    watch.setCursor(textX, 0);
    watch.setTextColor(255);
    watch.print("Hello world");
  }

  watch.swapBuffers();
  watch.delay(3); // Pause 3 frames
//  delay(50);
  if(--textX < -80) textX = 8;
}

