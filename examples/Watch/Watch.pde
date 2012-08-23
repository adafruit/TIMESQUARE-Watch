#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

#define MODE_SET  0
#define MODE_TIME 1

void (*modeFunc[])(void) = {
  mode_set,
  mode_time
};

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
  (*modeFunc[mode])();
}

void mode_set() {
  uint8_t n = watch.action();
  if(n == ACTION_HOLD_BOTH) {
    mode = MODE_TIME;
    return;
  }

  watch.fillScreen(0);
  watch.setTextColor(255);
  watch.setCursor(1, 0);
  if     (n == ACTION_TAP_LEFT)   watch.print('l');
  else if(n == ACTION_TAP_RIGHT)  watch.print('r');
  else if(n == ACTION_HOLD_LEFT)  watch.print('L');
  else if(n == ACTION_HOLD_RIGHT) watch.print('R');
  else                            watch.print('_');
  watch.swapBuffers();
  if(n != ACTION_NONE) watch.delay(15);
}

void mode_time() {
  uint8_t n = watch.action();

  if(n == ACTION_HOLD_BOTH) {
    mode = MODE_SET;
    return;
  }

  static int textX = 8;
  char       str[9];
  DateTime   now;

  now = RTC.now();
  sprintf(str,"%02d:%02d:%02d",
    now.hour(),
    now.minute(),
    now.second());
  watch.fillScreen(0);
  watch.setTextWrap(false);
  watch.setTextColor(255);
  watch.setCursor(textX, 0);
  watch.print(str);
  watch.swapBuffers();
  watch.delay(2);
  if(--textX < -52) textX = 8;
}

