#define BACKGROUND 0
#define BIT_CLEAR 20
#define BIT_SET  255

void mode_binary(uint8_t action) {
  DateTime now;
  uint8_t  h, m, s, x, bit;

  if(action < ACTION_HOLD_LEFT) watch.delay(10);

  now = RTC.now();
  h   = now.hour();
  if     (h  > 12) h -= 12;
  else if(h ==  0) h  = 12;
  m   = now.minute();
  s   = now.second();

  watch.fillScreen(BACKGROUND);
  for(x = 0, bit = 0x08; bit; bit >>= 1, x += 2) {
    watch.fillRect(x, 1, 2, 2, (h & bit) ? BIT_SET : BIT_CLEAR);
  }
  for(x = 1, bit = 0x20; bit; bit >>= 1, x++) {
    watch.drawPixel(x, 4, (m & bit) ? BIT_SET : BIT_CLEAR);
    watch.drawPixel(x, 6, (s & bit) ? BIT_SET : BIT_CLEAR);
  }
}

