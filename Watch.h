#ifndef _WATCH_H_
#define _WATCH_H_

#include "Adafruit_GFX.h"

#define ACTION_NONE       0
#define ACTION_TAP_LEFT   1
#define ACTION_TAP_RIGHT  2
#define ACTION_HOLD_LEFT  3
#define ACTION_HOLD_RIGHT 4
#define ACTION_HOLD_BOTH  5
#define ACTION_WAKE       6

#define WATCH_FPS         65 // Approximate refresh frames/sec

// Number of concurrent 'on' LEDs
#define WATCH_LEDS_1 0
#define WATCH_LEDS_2 1
#define WATCH_LEDS_4 2
#define WATCH_LEDS_8 3

class Watch : public Adafruit_GFX {

 public:

  Watch(uint8_t nPlanes=8, uint8_t nLEDs=WATCH_LEDS_8,
    boolean doubleBuffer=false);

  void
    begin(void),
    drawPixel(int16_t x, int16_t y, uint16_t c),
    swapBuffers(boolean copy=false),
    delay(uint8_t d),
    setTimeout(uint16_t t),
    setDisplayMode(uint8_t nPlanes=8, uint8_t nLEDs=WATCH_LEDS_8,
      boolean doubleBuffer=false);
  uint8_t
    action(void),
    *backBuffer(void);
  uint16_t
    getTimeout(void);
};

#endif // _WATCH_H_
