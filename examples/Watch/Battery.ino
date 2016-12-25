// Battery 'fuel gauge' display.
// Important thing to know here: this is based on the voltage when
// the watch last awoke from sleep, NOT instantaneous voltage, as the
// former is a better long-term estimate of how much capacity remains.
// Instantaneous voltage is a poor indicator because running the watch
// drags down the voltage, but this recovers with time.  So...the
// battery display might show lots of 'fuel' remaining, but then shut
// down after an extended bout of button-pressing.  This is
// unfortunate but normal, and the watch should be available again
// after the battery's had some time to rest.

static const uint16_t PROGMEM mVtable[] = { 2884, 2886, 2890, 2900, 65535 };

void mode_battery(uint8_t action) {

  uint8_t  border, fill, i;
  uint16_t t;

  if(action != ACTION_NONE) {
    // If we just arrived here (whether through mode change
    // or wake from sleep), initialize the matrix driver:
    if(action >= ACTION_HOLD_LEFT) {
      // Reconfigure display if needed
      if((watch.getDepth() != 2) || (watch.getPlex() != LED_PLEX_1))
        fps = watch.setDisplayMode(2, LED_PLEX_1, true);
    }
    // Reset sleep timeout on ANY button action
    watch.setTimeout(fps * 3);
  }

  if((t = watch.getTimeout()) < sizeof(fade)) {
    uint16_t b1 = (uint8_t)pgm_read_byte(&fade[t]) + 1;
    border = (255 * b1) >> (16 - 2); // 2 = depth
    fill   = ( 65 * b1) >> (16 - 2);
  } else {
    border =  255 >> (8 - 2);
    fill   =   65 >> (8 - 2);
  }

  watch.fillScreen(0);

  // Draw battery outline
  watch.drawRect(2, 1, 4, 7, border);
  watch.drawLine(3, 0, 4, 0, border);

  // Draw battery 'fuel' inside (blink if very low)
  t = watch.getmV();
  for(i=0; (i<5) && (t>pgm_read_word(&mVtable[i])); i++);
  if(i > 0)                       watch.fillRect(3, 6-i, 2, i+1, fill);
  else if(watch.getCursorBlink()) watch.drawLine(3, 6, 4, 6, fill);
}

