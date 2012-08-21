#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

Watch      watch(true); // Use double-buffered animation
RTC_DS1307 RTC;
int        textX = 8;

void setup() {
  Wire.begin();
  RTC.begin();
  RTC.adjust(DateTime(__DATE__, __TIME__));
  watch.begin();
  watch.setTextWrap(false);
  watch.setTextColor(255);
}

void loop() {
  char     str[9];
  DateTime now = RTC.now();
  sprintf(str,"%02d:%02d:%02d",
    now.hour(),
    now.minute(),
    now.second());
  watch.fillScreen(0);
  watch.setCursor(textX, 0);
  watch.print(str);
  watch.swapBuffers();
  delay(20);
  if(--textX < -47) textX = 8;
}

