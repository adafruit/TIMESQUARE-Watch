PROGMEM uint16_t mVtable[] = { 2650, 2700, 2750, 2800, 65535 };

void mode_battery(uint8_t action) {

  uint8_t  border, fill, i;
  uint16_t t;

  if(action != ACTION_NONE) {
    // If we just arrived here (whether through mode change
    // or wake from sleep), initialize the matrix driver:
    if(action >= ACTION_HOLD_LEFT) {
      depth = 2;
      fps   = watch.setDisplayMode(depth, LED_PLEX_1, true);
    }
    // Reset sleep timeout on ANY button action
    watch.setTimeout(fps * 3);
  }

  if((t = watch.getTimeout()) < sizeof(fade)) {
    uint16_t b1 = (uint8_t)pgm_read_byte(&fade[t]) + 1;
    border = (255 * b1) >> (16 - depth);
    fill   = ( 65 * b1) >> (16 - depth);
  } else {
    border =  255 >> (8 - depth);
    fill   =   65 >> (8 - depth);
  }

  watch.fillScreen(0);
  watch.drawRect(2, 1, 4, 7, border);
  watch.drawLine(3, 0, 4, 0, border);

  t = watch.getmV();
  for(i=0; (i<5) && (t>pgm_read_word(&mVtable[i])); i++);
  if(i > 0)      watch.fillRect(3, 6-i, 2, i+1, fill);
  else if(curOn) watch.drawLine(3, 6, 4, 6, fill);

  if(++curBlnk >= (fps/4)) {
    curBlnk = 0;
    curOn   = !curOn;
  }
}

