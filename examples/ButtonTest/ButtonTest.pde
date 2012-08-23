#include <Adafruit_GFX.h>
#include <Watch.h>

Watch watch(true); // Use double-buffered animation

void setup() {
  Serial.begin(19200);
  watch.begin();
}

void loop() {
  watch.fillScreen(0);
  watch.setTextColor(255);
  watch.setCursor(0, 0);
  int n = watch.action();
  watch.print(n);
  watch.swapBuffers();
  if(n != ACTION_NONE) watch.delay(10);
}

