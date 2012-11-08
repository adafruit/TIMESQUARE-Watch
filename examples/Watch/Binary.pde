#define BACKGROUND 0
#define BIT_CLEAR 20
#define BIT_SET  255

void mode_binary(uint8_t action) {
  DateTime now;
  uint8_t  h, mh, ml, sh, sl, x, bit, b_set, b_clear;
  uint16_t t;

  // Reset sleep timeout on any button action, even
  // if it has no consequences in the current mode.
  if(action != ACTION_NONE) watch.setTimeout(fps * 10);

  now = RTC.now();
  h   = now.hour();
  if     (h  > 12) h -= 12;
  else if(h ==  0) h  = 12;
  x   = now.minute();
  mh  = x / 10;
  ml  = x % 10;
  x   = now.second();
  sh  = x / 10;
  sl  = x % 10;

  // Calc set/clear colors based on current fadeout value
  if((t = watch.getTimeout()) < sizeof(fade)) {
    uint16_t b1 = (uint8_t)pgm_read_byte(&fade[t]) + 1;
    b_set       = (BIT_SET   * b1) >> 8;
    b_clear     = (BIT_CLEAR * b1) >> 8;
  } else {
    b_set       =  BIT_SET;
    b_clear     =  BIT_CLEAR;
  }

  watch.fillScreen(BACKGROUND);
  // Draw hour as 2x2 blocks
  for(x = 0, bit = 0x08; bit; bit >>= 1, x += 2) {
    watch.fillRect(x, 1, 2, 2, (h & bit) ? b_set : b_clear);
  }
  // Draw first digit of minutes, seconds as 3-bit values
  for(x = 0, bit = 0x04; bit; bit >>= 1, x++) {
    watch.drawPixel(x, 4, (mh & bit) ? b_set : b_clear);
    watch.drawPixel(x, 6, (sh & bit) ? b_set : b_clear);
  }
  // 2nd digit of minutes, seconds (4-bit values)
  for(x = 4, bit = 0x08; bit; bit >>= 1, x++) {
    watch.drawPixel(x, 4, (ml & bit) ? b_set : b_clear);
    watch.drawPixel(x, 6, (sl & bit) ? b_set : b_clear);
  }
}

