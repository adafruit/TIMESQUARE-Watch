#define BACKGROUND 0
#define BIT_CLEAR 20
#define BIT_SET  255

void mode_binary(uint8_t action) {
  DateTime now;
  uint8_t  h, m, s, x, bit;

  // Reset sleep timeout on any button action, even
  // if it has no consequences in the current mode.
  if(action != ACTION_NONE) watch.setTimeout(650); // ~10 sec

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

