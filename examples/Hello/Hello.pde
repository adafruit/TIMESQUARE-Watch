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
Serial.println("A");

  if (! RTC.isrunning()) {
Serial.println("B");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
Serial.println("C");

  watch.begin();
  watch.setTextWrap(false);

  // Calc gamma-corrected 'gray' ramp:
  for(byte i=0; i<64; i++) {
    ramp[i] = (int)(255.0 * pow((float)i / 63.0, 2.6) + 0.5);
  }
}

void loop() {



#ifdef SLART

  uint8_t x, y, i;
//  uint8_t bits = watch.buttons();
  uint8_t bits = watch.poop;

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

#endif

  char buffa[20];
// Hanging here?
  DateTime now = RTC.now();

Serial.println("D");

  sprintf(buffa,"%02d:%02d:%02d",
    now.hour(),
    now.minute(),
    now.second());

  watch.fillScreen(0);
  watch.setCursor(textX, 0);
  watch.setTextColor(255);
  watch.print(buffa);

  watch.swapBuffers();
  delay(50);
  if(--textX < -80) textX = 8;
}

