// img2array: Converts image file to an Arduino PROGMEM array.
// Copy and paste output from Processing console to Arduino sketch,
// change array name as needed ('img[]' used by default).
void setup() {

  String filename;
  PImage img;
  int    i, p, c, gamma[] = new int[256];

  if(((filename = selectInput("Select image file...")) != null) &&
     ((img      = loadImage(filename))                 != null)) {

    for(i=0; i<256; i++)
      gamma[i] = (int)(pow(((float)i / 255.0), 2.6) * 255.0 + 0.5);

    c = 16; // Column counter -- force initial line wrap

    img.filter(GRAY); // Convert image to grayscale
    img.loadPixels(); // Make pixels[] array readable

    print("static const uint8_t PROGMEM img[] = {");
    for(i=0; i<img.pixels.length; i++) { // For each pixel...
      p = gamma[img.pixels[i] & 0xFF];   // Get 8-bit value
      if(i > 0) print(',');              // Print preceding comma
      if(++c >= 15) {                    // Every 15 columns of data...
        print("\n  ");                   //   Wrap output (fits 80 cols)
        c = 0;                           //   And reset column counter
      }
      print("0x"); print(hex(p, 2));     // Output pixel as 2-digit hex #
    }
    println(" };");
  }

  exit();
}

