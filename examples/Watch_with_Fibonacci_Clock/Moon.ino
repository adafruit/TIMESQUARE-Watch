//field stores brightness and type(min/hr/both) for every field
//fields are: 1 top, 1 bottom, 2 left, 3 left, 5 right
//brightnesses are: red(hr): 80; green(min): 160; both: 240; nothing: 0
int field[5] = {0, 0, 0, 0, 0};
//draw is like field but stores the brightnesses also while fadeout
int draw[5];


void mode_moon(uint8_t action) {
  if (action != ACTION_NONE) {
    // If we just arrived here (whether through mode change
    // or wake from sleep), initialize the matrix driver:
    if (action >= ACTION_HOLD_LEFT) {
      // Reconfigure display if needed
      if ((watch.getDepth() != 2) || (watch.getPlex() != LED_PLEX_1))
        fps = watch.setDisplayMode(2, LED_PLEX_1, true);
    }
    // Reset sleep timeout on ANY button action
    watch.setTimeout(fps * 3);
  }


  watch.fillScreen(0);
  clearField();
  getTime();
  getFadeout();
  showTime();
}
void clearField() {
  for (int i = 0; i < 5; i++) {
    field[i] = 0;
  }
}

  void getFadeout() {
    uint8_t depth;
    uint16_t t;
    depth = watch.getDepth();
    if ((t = watch.getTimeout()) < sizeof(fade)) {
      uint16_t b1 = (uint8_t)pgm_read_byte(&fade[t]) + 1;
      draw[0]       = (field[0]   * b1) >> (16 - depth);
      draw[1]       = (field[1]   * b1) >> (16 - depth);
      draw[2]       = (field[2]   * b1) >> (16 - depth);
      draw[3]       = (field[3]   * b1) >> (16 - depth);
      draw[4]       = (field[4]   * b1) >> (16 - depth);
    } else {
      draw[0] = field[0] >> (8- depth); 
      draw[1] = field[1] >> (8- depth); 
      draw[2] = field[2] >> (8- depth); 
      draw[3] = field[3] >> (8- depth); 
      draw[4] = field[4] >> (8- depth); 
      }


  }

  void getTime() {
    DateTime now = RTC.now();
    int hr = int(now.hour());
    int mn = int(now.minute()) / 5;


    if (hr >= 12) {
      hr -= 12; //max val of hr is 11 not 23
    }
    if (hr >= 5) {
      field[4] = 80;
      hr -= 5;
    }
    if (hr >= 3) {
      field[3] = 80;
      hr -= 3;
    }
    if (hr >= 2) {
      field[2] = 80;
      hr -= 2;
    }
    if (hr >= 1) {
      field[0] = 80;
      hr -= 1;
    }
    if (mn >= 5) {
      if (field[4] == 80) {
        field[4] = 240;
      } else {
        field[4] = 160;
      }
      mn -= 5;
    }
    if (mn >= 3) {
      if (field[3] == 80) {
        field[3] = 240;
      } else {
        field[3] = 160;
      }
      mn -= 3;
    }
    if (mn >= 2) {
      if (field[2] == 80) {
        field[2] = 240;
      } else {
        field[2] = 160;
      }
      mn -= 2;
    }
    if (mn >= 1) {
      field[1] = 160;
    }

  }


  void showTime() {
    // 1 fields
    watch.drawPixel(2, 1, draw[0]);
                    watch.drawPixel(2, 2, draw[1]);
                                    // 2 field
                                    watch.drawRect(0, 1, 1, 2, draw[2]);
                                    // 3 field
                                    watch.drawRect(0, 3, 2, 5, draw[3]);
                                    // 5 field
                                    watch.drawRect(3, 1, 7, 5, draw[4]);

  }
