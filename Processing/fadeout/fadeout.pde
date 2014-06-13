// fadeout: Calculates gamma-corrected fade-out table for marquee mode.
// Copy and paste output from Processing console to Arduino sketch.

void setup() {
  int i, steps = 32, f, c = 16;

  print("static const uint8_t PROGMEM fade[] = {");
  for(i=0; i<steps; i++) {
    f = (int)(pow((float)i / (float)(steps - 1), 2.6) * 255.0 + 0.5);
    if(i > 0) print(',');
    if(++c >= 16) {
      print("\n  ");
      c = 0;
    }
    print(String.format("%3d", f));
  }
  println(" };");
  exit();
}

