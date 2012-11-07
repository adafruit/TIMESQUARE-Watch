#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

#define PLANES  8
#define BITMODE WATCH_LEDS_8

// 1, 2 planes does not work.  3+ is OK.
// Low bitplane counts weren't working because LEDMINTIME was not sufficient
// (interrupt was triggering before prior one was finished).  Will need a
// variable MINTIME depending on bit depth.  :/
Watch watch(PLANES, BITMODE, true);

void setup() {

//Serial.begin(9600); // Only works if serial port enabled in watch library
  Wire.begin();

  watch.setTimeout(WATCH_FPS * 4);
  watch.begin();
  watch.setTextWrap(false); // Allow text to run off right edge
  watch.setTextColor(255);
}

int textX = watch.width();
int textMin = 11 * -8;
char str[20];
uint8_t f = 0;

void loop() {

  uint8_t a = watch.action();
  if(a == ACTION_HOLD_BOTH) {
  } else if(a == ACTION_HOLD_RIGHT) {
  } else if(a == ACTION_HOLD_LEFT) {
  }

  watch.fillScreen(0);
  watch.setCursor(textX, 1);
  sprintf(str,"%d LED %d bit", 1 << (3 - BITMODE), PLANES);
  watch.print(str);

  // Every 4th frame, shift text left 1 pixel
  if(++f >= 4) {
    if((--textX) < textMin) textX = watch.width();
    f = 0;
  }

  watch.swapBuffers();
  watch.setTimeout(100);
}

// To do: add some higher-level clipping here
void blit(uint8_t *img, int iw, int ih, int sx, int sy, int dx, int dy, int w, int h, uint8_t b) {
  int      x, y;
  uint16_t b1 = (uint16_t)b + 1; // +1 so that >>8 (rather than /255) can be used

  for(y=0; y<h; y++) {
    for(x=0;x<w;x++) {
      watch.drawPixel(dx + x, dy + y,
        ((uint8_t)pgm_read_byte(&img[(sy + y) * iw + sx + x]) * b1) >> 8);
    }
  }
}

