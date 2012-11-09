#define BACKGROUND 0
#define BIT_CLEAR 20
#define BIT_SET  255

void mode_binary(uint8_t action) {
  DateTime now;
  uint8_t  h, x, bit, b_set, b_clear;
  uint16_t t;

  if(action != ACTION_NONE) {
    // If we just arrived here (whether through mode change
    // or wake from sleep), initialize the matrix driver:
    if(action >= ACTION_HOLD_LEFT) {
      watch.setDisplayMode(4, LED_PLEX_2, true);
      fps   = watch.getFPS();
      depth = 4;
    }
    // Reset sleep timeout on ANY button action, even
    // if it has no consequences in the current mode.
    watch.setTimeout(fps * 10);
  }

  now = RTC.now();
  h   = now.hour();
  if     (h  > 12) h -= 12;
  else if(h ==  0) h  = 12;
  loadDigits(now.minute(), DIGIT_MIN0);
  loadDigits(now.second(), DIGIT_SEC0);

  // Calc set/clear colors based on current fadeout value
  if((t = watch.getTimeout()) < sizeof(fade)) {
    uint16_t b1 = (uint8_t)pgm_read_byte(&fade[t]) + 1;
    b_set       = (BIT_SET   * b1) >> (16 - depth);
    b_clear     = (BIT_CLEAR * b1) >> (16 - depth);
  } else {
    b_set       =  BIT_SET   >> (8 - depth);
    b_clear     =  BIT_CLEAR >> (8 - depth);
  }

  watch.fillScreen(BACKGROUND);
  // Draw hour as 2x2 blocks
  for(x = 0, bit = 0x08; bit; bit >>= 1, x += 2) {
    watch.fillRect(x, 1, 2, 2, (h & bit) ? b_set : b_clear);
  }
  // Draw first digit of minutes, seconds as 3-bit values
  for(x = 0, bit = 0x04; bit; bit >>= 1, x++) {
    watch.drawPixel(x, 4, (digit[DIGIT_MIN0] & bit) ? b_set : b_clear);
    watch.drawPixel(x, 6, (digit[DIGIT_SEC0] & bit) ? b_set : b_clear);
  }
  // 2nd digit of minutes, seconds (4-bit values)
  for(x = 4, bit = 0x08; bit; bit >>= 1, x++) {
    watch.drawPixel(x, 4, (digit[DIGIT_MIN1] & bit) ? b_set : b_clear);
    watch.drawPixel(x, 6, (digit[DIGIT_SEC1] & bit) ? b_set : b_clear);
  }
}

