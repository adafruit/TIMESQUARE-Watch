#include "Adafruit_GFX.h"
#include "Watch.h"

Watch watch(true); // Use double-buffered animation

int  textX = 8;
byte ramp[64];

void setup() {
  watch.begin();
  watch.setTextWrap(false);

  // Calc gamma-corrected 'gray' ramp:
  for(byte i=0; i<64; i++) {
    ramp[i] = (int)(255.0 * pow((float)i / 63.0, 2.6));
  }
}

void loop() {
  uint8_t x, y, i;
  uint8_t bits = PIND & B00001100; // buttons

  if(bits == 8) {
    watch.fillScreen(0);
    watch.setCursor(1, 0);
    watch.setTextColor(255);
    watch.print("L");
  } else if(bits == 4) {
    watch.fillScreen(0);
    watch.setCursor(1, 0);
    watch.setTextColor(255);
    watch.print("R");
  } else if(bits == 0) {
    watch.fillScreen(0);
    watch.setCursor(1, 0);
    watch.setTextColor(255);
    watch.print("B");
  } else { // bits == 12
    for(i=y=0;y<8;y++) {
      for(x=0;x<8;x++) {
        watch.drawPixel(x, y, ramp[i++]);
      }
    }
  
    watch.setCursor(textX + 1, 1);
    watch.setTextColor(0);
    watch.print("Hello world");
  
    watch.setCursor(textX, 0);
    watch.setTextColor(255);
    watch.print("Hello world");
  }

  watch.swapBuffers();
  delay(50);
  if(--textX < -80) textX = 8;
}

