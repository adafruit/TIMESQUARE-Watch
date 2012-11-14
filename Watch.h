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

// Display multiplexing mode; number of concurrent 'on' LEDs.
// This is inversely proportional to the number of refresh passes.
#define LED_PLEX_8 0
#define LED_PLEX_4 1
#define LED_PLEX_2 2
#define LED_PLEX_1 3

class Watch : public Adafruit_GFX {

 public:

  Watch(uint8_t nPlanes=8, uint8_t nLEDs=LED_PLEX_8,
    boolean doubleBuffer=false);

  void
    begin(void),
    drawPixel(int16_t x, int16_t y, uint16_t c),
    swapBuffers(uint8_t frames=1, boolean copy=false),
    setTimeout(uint16_t t);
  uint8_t
    *backBuffer(void),
    action(void),
    getPlex(void),
    getDepth(void);
  uint16_t
    setDisplayMode(uint8_t nPlanes=8, uint8_t nLEDs=LED_PLEX_8,
      boolean doubleBuffer=false),
    getFPS(void),
    getmV(void),
    getTimeout(void);
  boolean
    getCursorBlink(void);
};

#endif // _WATCH_H_
