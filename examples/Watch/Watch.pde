/*
INTERACTING WITH WATCH:
A button 'tap' is a quick press (a second or less).
A button 'hold' is a long press (2 seconds or longer).
Watch will start up in time display mode.

TIME DISPLAY MODE:
Hold left or right button to switch forward/back between clocks.
Some (but not all) clocks may use left or right button tap to switch display formats.
Hold both buttons to switch to time set mode.

TIME SET MODE:
Tap right button to increase value of current digit.
Tap left button to advance to next digit.
Hold both buttons to return to time display mode.
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Watch.h>

#define MODE_SET     0
#define MODE_MARQUEE 1
#define MODE_BINARY  2
#define MODE_PIE     3
// Additional display modes will be listed here

void (*modeFunc[])(uint8_t) = {
  mode_set,
  mode_marquee,
  mode_binary,
  mode_pie
};
#define N_MODES (sizeof(modeFunc) / sizeof(modeFunc[0]))

// Used by various display modes for smooth fade-out before sleep
PROGMEM uint8_t fade[] = {
   0,  1,  1,  2,  4,  5,  8, 10, 13, 17, 22, 27, 32, 39, 46,
  53, 62, 71, 82, 93,105,117,131,146,161,178,196,214,234,255 };

//Watch      watch(8, LED_PLEX_8, true); // Use double-buffered animation
Watch      watch(7, LED_PLEX_4, true); // Use double-buffered animation
RTC_DS1307 RTC;
uint8_t    mode = MODE_MARQUEE, mode_last = MODE_MARQUEE;
boolean    h24  = false; // 24-hour display mode
uint16_t   fps;

void setup() {
  DateTime now;

//Serial.begin(9600); // Only works if serial port enabled in watch library

  Wire.begin();
  RTC.begin();

  now = RTC.now();
  // If clock is unset, set it to compile time and jump to time-setting mode
  if((now.year() == 2000) && (now.month()  == 1) && (now.day()    == 1) &&
     (now.hour() == 0   ) && (now.minute() == 0) && (now.second() <  5)) {
    RTC.adjust(DateTime(__DATE__, __TIME__));
    mode = MODE_SET;
  }
  fps = watch.getFPS();
  watch.setTimeout(fps * 4);
  watch.begin();
}

void loop() {

  uint8_t a = watch.action();
  if(a == ACTION_HOLD_BOTH) {
    if(mode == MODE_SET) {
      // Exit time setting, return to last used display mode
      set();
      mode = mode_last;
    } else {
      // Save current display mode, switch to time setting
      mode_last = mode;
      mode      = MODE_SET;
    }
  } else if(a == ACTION_HOLD_RIGHT) {
    if(mode != MODE_SET) {
      // Switch to next display mode (w/wrap)
      if(++mode >= N_MODES) mode = 1;
    }
  } else if(a == ACTION_HOLD_LEFT) {
    if(mode != MODE_SET) {
      // Switch to prior display mode (w/wrap)
      if(--mode < 1) mode = N_MODES - 1;
    }
  }

  (*modeFunc[mode])(a); // Action is passed to clock-drawing function
  watch.swapBuffers();
}

// To do: add some higher-level clipping here
void blit(uint8_t *img, int iw, int ih, int sx, int sy, int dx, int dy, int w, int h, uint8_t b) {
  int      x, y;
  uint16_t b1 = (uint16_t)b + 1; // +1 so that >>8 (rather than /255) can be used

  for(y=0; y<h; y++) {
    for(x=0;x<w;x++) {
      watch.drawPixel(dx + x, dy + y,
//        ((uint8_t)pgm_read_byte(&img[(sy + y) * iw + sx + x]) * b1) >> 11);
//        ((uint8_t)pgm_read_byte(&img[(sy + y) * iw + sx + x]) * b1) >> 8);
        ((uint8_t)pgm_read_byte(&img[(sy + y) * iw + sx + x]) * b1) >> 9);
    }
  }
}

