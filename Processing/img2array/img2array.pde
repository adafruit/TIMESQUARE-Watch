// img2array: Converts image file to an Arduino PROGMEM array.
// Copy and paste output from Processing console to Arduino sketch,
// change array name as needed ('img[]' used by default).

void setup() {

  String filename;
  PImage img;

  if(((filename = selectInput("Select image file...")) != null) &&
     ((img      = loadImage(filename))                 != null)) {

    int i, p, c = 16; // Column counter -- force initial line wrap

    img.filter(GRAY); // Convert image to grayscale
    img.loadPixels(); // Make pixels[] array readable

    print("PROGMEM uint8_t img[] = {");
    for(i=0; i<img.pixels.length; i++) { // For each pixel...
      p = img.pixels[i] & 0xFF;          // Get 8-bit value
      if(i > 0) print(',');              // Print preceding comma
      if(++c >= 16) {                    // Every 16 columns...
        print("\n  ");                   //   Wrap output
        c = 0;                           //   And reset column counter
      }
      print("0x"); print(hex(p, 2));     // Output pixel as 2-digit hex #
    }
    println(" };");
  }

  exit();
}

