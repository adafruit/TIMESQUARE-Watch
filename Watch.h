#ifndef _WATCH_H_
#define _WATCH_H_

#include "Adafruit_GFX.h"

#define ACTION_NONE       0
#define ACTION_TAP_LEFT   1
#define ACTION_TAP_RIGHT  2
#define ACTION_HOLD_LEFT  3
#define ACTION_HOLD_RIGHT 4
#define ACTION_HOLD_BOTH  5

class Watch : public Adafruit_GFX {

 public:

  Watch(boolean doubleBuffer=false);

  void
    begin(void),
    drawPixel(int16_t x, int16_t y, uint16_t c),
    swapBuffers(boolean copy=false),
    delay(uint8_t);
  uint8_t
    action(void),
    *backBuffer(void);
};

#endif // _WATCH_H_
