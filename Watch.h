#ifndef _WATCH_H_
#define _WATCH_H_

#include "Adafruit_GFX.h"

class Watch : public Adafruit_GFX {

 public:

  Watch(boolean doubleBuffer=false);

  void
    begin(void),
    drawPixel(int16_t x, int16_t y, uint16_t c),
    swapBuffers(boolean copy=false),
    delay(int);
  uint8_t
    buttons(void);
};

#endif // _WATCH_H_
