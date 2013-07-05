// Library for Adafruit 8x8 LED matrix watch.  (C) Adafruit Industries.
// Displays 1- to 8-bit monochrome graphics with one row of 8, 4, 2 or 1
// LED enabled at a time, using a fast timer interrupt and bit angle
// modulation (vs PWM).  AVR sleep modes are used where possible to
// conserve power.  Requires Adafruit_GFX library.

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "Watch.h"

// This code looks ridiculous and requires quite a bit of explanation...

// First, some parts might be painful to read in that 'row' and 'column'
// here refer to the hardware pin functions as described in the LED matrix
// datasheet...but, with the matrix installed sideways in the watch for
// better component placement, these aren't the same as 'row' and 'column'
// visually as we usually consider them in computer graphics.  So...the
// higher-level graphics drawing functions use conventional X/Y coordinates
// from the top left of the face of the watch as normally worn, while the
// lower-level code (most of this library) takes care of mapping this to
// the native orientation of the LED matrix.

// Maybe more important is memory usage.  This library is the biggest RAM
// hog in the history of RAM hogs, but this came about by design rather
// than negligence.  The primary focus of the entire library is to reduce
// the matrix-refresh interrupt handler to the fewest instructions and the
// shortest possible interval, permitting smoother screen refresh (less
// apparent flicker) and more opportunities to put the CPU in a low-power
// sleep state.  All other considerations take a back seat to getting in
// and out of that interrupt as quickly as possible.  The shortest route
// to achieve this is to store a ton of precomputed stuff in RAM.

// The 8x8 LED matrix, like most, is 'multiplexed' to display one row or
// column at a time (this library does one column at a time...but that's
// 'column' in the datasheet sense, recall from above that this is 'row'
// in the visual sense).  The row and column lines that drive the display
// are spread across three PORT registers.  Rather than store a memory-
// efficient representation of the screen bitmap (8 bytes for an 8x8, 1-bit
// image) and then having code rearrange this across the three PORTs on
// *every single interrupt call*, screen data is instead stored in a format
// that can be copied directly to the three PORTs...so each 8-pixel, 1-bit
// column of the display actually consumes three bytes of RAM instead of
// one (quite a few bits go unused).  A full 8x8 pixel 1-bit image thus
// requires 24 bytes of RAM, not 8 bytes.  Oink oink factor 3.

// For a multi-bit image (i.e. 'grayscale,' disregarding the color of the
// LED matrix), each additional bitplane consumes another 24 bytes.  So a
// 1-bit image is 24 bytes, while a 4-bit image (16 brightness levels)
// needs 24x4 or 96 bytes, and an 8-bit image (256 levels) requires 24x8
// or 192 bytes.

// Because the watch runs off a single lithium coin cell, there's a very
// real need to conserve power....heavy current draw will deplete the
// battery with disproportionate speed.  A significant power saving can be
// achieved by driving fewer than the full 8 LEDs of a column and using
// broader multiplexing...for example, rather than 8 LEDs x 8 columns, the
// library can multiplex 4 LEDs x 16 half-columns, 2 LEDs x 32 quarter-
// columns, or 1 LED x 64 eighth-columns.  But again, with the interrupt
// speed taking precedence, this is achieved by storing multiple full 8-LED
// columns that each only have some fraction of the LEDs enabled, because
// this is faster than masking out bits every single time.  Somewhat
// counter-intuitively, the fewer LEDs on at a time, the more RAM is spent
// in this manner.  For example, driving 4 (rather than 8) LEDs at a time,
// there are essentially two copies of image data in RAM -- one with even
// rows enabled and one with odd rows -- but each still requiring the full
// 24 bytes of data (times the number of bitplanes).  An 8-bit image
// displaying 4 LEDs at a time now requires 24x8x2 or 384 bytes just for
// this small 8x8 grid.  2 LEDs at a time requires 4 passes, while 1 LED
// requires 8 passes.  It becomes necessary to cut back the number of
// bitplanes as this approaches the total free RAM available on the MCU!
// Running more LEDs at a time saves RAM, but draws more power.  The right
// balance must be found between looking nice and preserving scarce battery
// power.  Since the application has only modest needs beyond showing a
// watch face, there's little guilt in sacrificing ridiculous amounts of
// RAM to meet these goals.

// Finally, double-buffering -- a means of achieving flicker-free animation
// by maintaining separate on- and off-screen buffers -- doubles the screen
// memory requirements yet again.

// Screen RAM requirements can thus be calculated as follows:
// 24 bytes * bitplanes * passes * buffers, where:
// Brightness levels = 2^bitplanes; e.g. 1 plane = 2 levels, 2 planes = 4...
// LEDs = 2^(4-passes); e.g. 8 LEDs = 1 pass, 4 LEDs = 2 passes, etc.
// Buffers = 1 (single-buffered) or 2 (double-buffered).
// e.g. 16-level, 4 LEDs at a time, double buffered = 24*4*2*2 = 384 bytes.

// The available display RAM has been fixed at 768 bytes.  This allows
// the following maximum bit depths for various multiplex factors:
// 8 LED (1 pass), double buf: 8 bits max
// 4 LED (2 pass), double buf: 8 bits max
// 2 LED (4 pass), double buf: 4 bits max
// 1 LED (8 pass), double buf: 2 bits max

// The 'off' state for rows and columns is different because one represents
// anodes and the other cathodes.  So this weird combination of bits sets
// all rows and columns to their respective 'off' states:
#define PORTB_OFF B11001010
#define PORTC_OFF B00110011 // PC4, PC5 are set to enable I2C pullups
#define PORTD_OFF B11001100 // PD2, PD3 are set to enable button pullups

// Because the LED matrix ties up so many of the microcontroller's pins,
// the idea of object-orientating the code for multiple instances is a bit
// nonsensical.  It's done insofar as Adafruit_GFX can be used, but all
// the matrix-specific variables and such are simply declared in the code
// here rather than in private vars.  Keeps the interrupt code simple.
static uint8_t
  imgSpace[768],          // RAM devoted to matrix-driving operations
  *img[2],                // Pointers to 'front' and 'back' display buffers
  planes,                 // Number of bitplanes
  passes,                 // Number of matrix passes for multiplexing
  plex,                   // LED multiplex factor (LED_PLEX_* in .h)
  napThreshold;           // If OCR2A >= this, OK for power-saving mode
static uint16_t
  fps,                    // Estimated frames-per-second
  holdTime,               // Frames per 1.5 sec
  bufSize,                // Size of display buffer, in bytes
  mV       = 0;           // Battery voltage, in millivolts
static volatile uint8_t
  plane,                  // Current bitplane being displayed
  pass,                   // Current matrix pass being displayed
  col,                    // Current column being displayed
  swapFlag = 0,           // If set, swap front/back buffers
  *ptr,                   // Current pointer into front buffer
  frontIdx = 0,           // Buffer # being displayed (vs modified)
  bSave,                  // Last button state
  bAction  = ACTION_NONE, // Last button action
  sCount   = 6;
static volatile boolean
  wakeFlag = false,
  holdFlag = false,
  dbuf     = false;
static volatile uint16_t
  bCount   = 0,           // Button hold counter
  cCount   = 1,           // Cursor blink counter
  timeout  = 10;          // Countdown to sleep() (in frames)

// Constructor
Watch::Watch(uint8_t nPlanes, uint8_t nLEDs, boolean doubleBuffer) :
Adafruit_GFX(8, 8) {
  img[0] = imgSpace;
  plex   = nLEDs;
  planes = nPlanes;
  dbuf   = doubleBuffer;
}

// Battery monitoring idea adapted from JeeLabs article:
// jeelabs.org/2012/05/04/measuring-vcc-via-the-bandgap/
static void readVoltage() {

  int     i, prev;
  uint8_t count;

  power_adc_enable();
  ADMUX  = _BV(REFS0) |                        // AVcc voltage reference
           _BV(MUX3)  | _BV(MUX2) | _BV(MUX1); // Bandgap (1.8V) input
  ADCSRA = _BV(ADEN)  |             // Enable ADC
           _BV(ADPS2) | _BV(ADPS1); // 1/64 prescaler (8 MHz -> 125 KHz)
  // Datasheet notes that the first bandgap reading is usually garbage as
  // voltages are stabilizing.  It practice, it seems to take a bit longer
  // than that (perhaps due to sleep).  Tried various delays, but this was
  // still inconsistent and kludgey.  Instead, repeated readings are taken
  // until four concurrent readings stabilize within 10 mV.
  for(prev=9999, count=0; count<4; ) {
    for(ADCSRA |= _BV(ADSC); ADCSRA & _BV(ADSC); ); // Start, await ADC conv.
    i  = ADC;                                       // Result
    mV = i ? (1100L * 1023 / i) : 0;                // Scale to millivolts
    if(abs((int)mV - prev) <= 10) count++;   // +1 stable reading
    else                          count = 0; // too much change, start over
    prev = mV;
  }
  ADCSRA = 0; // ADC off
  power_adc_disable();
}

// Initialize PORT registers and enable timer and button interrupts.
void Watch::begin() {

  readVoltage();     // Battery voltage at startup

  PORTB = PORTB_OFF; // Turn all rows/columns off
  PORTC = PORTC_OFF;
  PORTD = PORTD_OFF;
  DDRB  = B11111111; // And enable outputs
  DDRC  = B00001111;
  DDRD  = B11110000;

  // Disable unused peripherals
  ADCSRA &= ~_BV(ADEN); // ADC off
  // Timer0 interrupt is disabled as this throws off the delicate PWM
  // timing.  Unfortunately this means delay(), millis() won't work.
  TIMSK0 = 0;
  // Disable peripherals that aren't used by this code,
  // maybe save a tiny bit of power.
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_usart0_disable(); // Comment this out if using Serial for debugging

  // Set up interrupt-on-change for buttons.
  EICRA = _BV(ISC10)  | _BV(ISC00);  // Trigger on any logic change
  EIMSK = _BV(INT1)   | _BV(INT0);   // Enable interrupts on pins
  bSave = PIND & (_BV(PORTD3) | _BV(PORTD2)); // Get initial button state

  // Set up Timer2 for matrix interrupt
  TCCR2A   = _BV(WGM21); // Mode 2 (CTC), OC2A,2B off
  fps      = setDisplayMode(planes, plex, dbuf); // Prescale (TCCR2B) set here
  holdTime = fps * 3 / 2;

  sei(); // Enable global interrupts
}

uint16_t Watch::setDisplayMode(uint8_t nPlanes, uint8_t nLEDs,
   boolean doubleBuffer) {

  // Stop Timer2 and interrupt if running
  TCCR2B  = 0;
  TIMSK2 &= ~_BV(OCIE2A);

  // Validate inputs (1-8 planes, 1/2/4/8 LEDs).  This checks for limits
  // on the individual parameters but currently does not look for illegal
  // combinations (those that would exceed imageSpace[]).
  if(nPlanes > 8)          nPlanes = 8;
  if(nLEDs   > LED_PLEX_1) nLEDs   = LED_PLEX_1;

  // Set plane/pass limits and reset all counters:
  planes = nPlanes;
  plex   = nLEDs;
  passes = 1 << plex;
  plane  = planes - 1; // plane and pass are set to the max values
  pass   = passes - 1; // so they'll 'wrap' on the next interrupt.
  col    = 8;          // Likewise, will roll over to start.
  TCNT2  = OCR2A = 0;
  dbuf   = doubleBuffer;

  // Always 3 bytes/row * 8 rows...
  // then multiply by number of planes and passes for total byte count:
  bufSize = 3 * 8 * planes * passes;

  // Clear front image buffer
  frontIdx = 0;
  ptr      = img[0];
  for(uint16_t i=0; i<bufSize;) {
    ptr[i++] = PORTB_OFF;
    ptr[i++] = PORTC_OFF;
    ptr[i++] = PORTD_OFF;
  }

  // If double-buffered, copy front image buffer to back:
  if(dbuf) {
    img[1] = &img[0][bufSize];
    memcpy(img[1], img[0], bufSize);
  } else {
    img[1] = img[0]; // Else both point to the same address
  }

  // Set the Timer2 prescaler to a value low enough to reduce flicker
  // but high enough to allow free cycles for screen drawing, sleep, etc.
  uint16_t prescale, res = ((1 << planes) - 1) * passes;
  if       (res >= 192) {
    prescale     = 64;
    TCCR2B       = _BV(CS22);
    napThreshold = 16;
  } else if(res >=  96) {
    prescale     = 128;
    TCCR2B       = _BV(CS22) | _BV(CS20);
    napThreshold = 8;
  } else if(res >=  24) {
    prescale     = 256;
    TCCR2B       = _BV(CS22) | _BV(CS21);
    napThreshold = 8;
  } else {
    prescale     = 1024;
    TCCR2B       = _BV(CS22) | _BV(CS21) | _BV(CS20);
    napThreshold = 1;
  }
  fps      = F_CPU / (9L * res * prescale); // Estimated frame refresh rate
  holdTime = fps * 3 / 2;
  cCount   = fps >> 1; // Reset cursor blink counter

  if(timeout < fps) timeout = fps; // Application can override this

  TIMSK2 |= _BV(OCIE2A); // (Re)start Timer2 interrupt

  return fps;
}

// For double-buffered animation, call this function to display new data.
void Watch::swapBuffers(uint8_t frames, boolean copy) {
  // Swap actually takes place at specific point in interrupt.
  // Set flag to request swap, then wait for change to complete:
  for(swapFlag = frames; swapFlag; );
  if(copy) memcpy(img[1 - frontIdx], img[frontIdx], bufSize);
}

// These tables help facilitate pixel drawing.  They're intentionally
// NOT placed in PROGMEM in order to save a few instruction cycles.
static const uint8_t
  rowBitPortB[]    = {    0, 0x20,    0, 0x10, 0x04,    0, 0x01,    0},
  rowBitPortC[]    = {    0,    0, 0x08,    0,    0, 0x04,    0,    0},
  rowBitPortD[]    = { 0x10,    0,    0,    0,    0,    0,    0, 0x20},
  passOffset[4][8] = { { 0,   0,   0,   0,   0,   0,   0,   0 },
                       { 0,  24,   0,  24,   0,  24,   0,  24 },
                       { 0,  48,  24,  72,   0,  48,  24,  72 },
                       { 0,  96,  48, 144,  24, 120,  72, 168 } };

// Basic pixel-drawing function for Adafruit_GFX.
void Watch::drawPixel(int16_t x, int16_t y, uint16_t c) {

  if((x < 0) || (y < 0) || (x > 7) || (y > 7)) return; // Clip

  switch(rotation) {
   case 1:
    swap(x, y);
    x = WIDTH  - 1 - x;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    swap(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  uint8_t bmask = rowBitPortB[x],
          cmask = rowBitPortC[x],
          dmask = rowBitPortD[x],
          c8    = (uint8_t)c,
          *p    = (uint8_t *)&img[1 - frontIdx][y * 3] +
                  passOffset[plex][x],
          bit,
          inc   = (passes << 4) + (passes << 3), // passes * 24
          done  = 1 << planes; // Will be 0 in 8-bit case, is normal

  for(bit = 1; bit != done; bit <<= 1, p += inc) {
    if(c8 & bit) {
      p[0] |=  bmask;
      p[1] |=  cmask;
      p[2] |=  dmask;
    } else {
      p[0] &= ~bmask;
      p[1] &= ~cmask;
      p[2] &= ~dmask;
    }
  }
}

uint8_t *Watch::backBuffer(void) {
  return img[1 - frontIdx];
}

uint8_t Watch::action(void) {
  uint8_t a = bAction;
  bAction   = ACTION_NONE;
  return  a;
}

void Watch::setTimeout(uint16_t t) {
  timeout = t;
}

uint16_t Watch::getTimeout(void) {
  return timeout;
}

// Puts watch into extremely low-power state, nearly all peripherals
// disabled.  Resumes on pin change interrupt (either button).  This
// function is not exposed to the outside world; only the timer
// interrupt may invoke it.
static void sleep(void) {

  uint8_t save;

  // Set all ports to high-impedance input mode, enable pullups
  // on all pins (seems to use ever-so-slightly less sauce).
  DDRB  = 0   ; DDRC  = 0   ; DDRD  = 0;
  PORTB = 0xff; PORTC = 0xff; PORTD = 0xff;

  // Stop Timer2, disable interrupt.
  save    = TCCR2B;
  TCCR2B  = 0;
  TIMSK2 &= ~_BV(OCIE2A);
  power_timer2_disable();
  power_twi_disable(); // Stop I2C

  // VITALLY IMPORTANT: brown-out reset (BOR) is NOT disabled, even though
  // this can save many micro-Amps during power-down sleep.  As the coin
  // cell runs down, real brown-outs are quite likely.  And when this
  // happens, if BOR is disabled, the MCU will behave erratically and may
  // jump to any random location...and if this leads into any bootloader
  // code that erases or writes a flash page, the application -- or much
  // worse, the bootloader itself -- can become corrupted, leaving no easy
  // way to re-flash the watch.  This is NOT the unlikely one-in-a-million
  // chance you might think...actual odds seem to be about 1% -- the
  // phenomenon has been observed in the wild with other projects and even
  // while developing this code.  So BOR is left enabled to provide a
  // proper safety net.  Really, DO NOT go adding BOR-disabling code,
  // you'll regret it later.  Just don't.  Okay?  Don't.  Thanks.

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei(); // Keep interrupts enabled during sleep
  sleep_mode();
  // Execution resumes here on wake.

  // Timeout is immediately set to ~1 sec.  The application can then
  // override this if desired; it just needs to be set to something
  // to keep the matrix interrupt from returning to sleep.
  timeout = fps;

  PORTB = PORTB_OFF; // Turn all rows/columns off
  PORTC = PORTC_OFF;
  PORTD = PORTD_OFF;
  DDRB  = B11111111; // And enable outputs
  DDRC  = B00001111;
  DDRD  = B11110000;

  // Upon wake, a battery voltage reading is taken before PWM restarts.
  // This gives a better impression of the 'resting' voltage of the
  // battery, as it will drop immediately once the matrix starts up.
  readVoltage();

  // wakeFlag stops clicks from falling through on wake
  // (e.g first click won't advance digit in time-setting mode).
  wakeFlag = true;
  bAction  = ACTION_WAKE;
  sCount   = 6; // Reset the "smoosh detect"

  // The front image buffer is cleared upon wake.  This avoids a frame
  // of old content from showing when the matrix interrupt is re-enabled
  // before the application code can process the input and redraw.
  uint8_t *p = img[frontIdx];
  for(uint16_t i=0; i<bufSize;) {
    p[i++] = PORTB_OFF;
    p[i++] = PORTC_OFF;
    p[i++] = PORTD_OFF;
  }

  // Enable only those peripherals used by the watch code
  power_twi_enable();
  Wire.begin(); // App note recommends reinitializing TWI on wake
  power_timer2_enable();
  TCCR2B  = save;        // Restore Timer2 settings
  TIMSK2 |= _BV(OCIE2A); // and re-enable interrupt
}

uint8_t Watch::getPlex(void) {
  return plex;
}

uint8_t Watch::getDepth(void) {
  return planes;
}

uint16_t Watch::getFPS(void) {
  return fps;
}

uint16_t Watch::getmV(void) {
  return mV;
}

boolean Watch::getCursorBlink(void) {
  return (cCount > (fps >> 2)); // Blink 2X/sec
}

// Matrix-multiplexing interrupt code

// Turn prior column off (set appropriate bit on appropriate PORT)
#define COLOFF(port, bit) \
    asm volatile("sbi %0,%1\n" :: "I"(_SFR_IO_ADDR(port)), "I"(bit));

// Turn new column on (clear appropriate bit on appropriate PORT) and
// advance 'col' to the next column value.  Columns advance in a bizarre
// interleaved order of horizontal lines in order to reduce apparent
// flicker and make multiplexing artifacts slightly less objectionable.
#define COLON(idx, port, bit, nxt) \
    asm volatile(                  \
     "ld  __tmp_reg__,%a0\n\t"     \
     "out %1,__tmp_reg__\n\t"      \
     "ld  __tmp_reg__,%a2\n\t"     \
     "out %3,__tmp_reg__\n\t"      \
     "ld  __tmp_reg__,%a4\n\t"     \
     "out %5,__tmp_reg__\n\t"      \
     "cbi %6,%7\n"                 \
     ::                            \
     "e"(&p[idx]),                 \
     "I"(_SFR_IO_ADDR(PORTB)),     \
     "e"(&p[idx+1]),               \
     "I"(_SFR_IO_ADDR(PORTC)),     \
     "e"(&p[idx+2]),               \
     "I"(_SFR_IO_ADDR(PORTD)),     \
     "I"(_SFR_IO_ADDR(port)),      \
     "I"(bit));                    \
    col = nxt;


ISR(TIMER2_COMPA_vect, ISR_BLOCK) {

  uint8_t *p = (uint8_t *)ptr;

  switch(col) {
   case 0:  asm("nop; nop;"); COLON( 0, PORTD, 6, 4); break;
   case 1:  COLOFF(PORTB, 3); COLON( 3, PORTB, 6, 5); break;
   case 2:  COLOFF(PORTC, 0); COLON( 6, PORTC, 1, 6); break;
   case 3:  COLOFF(PORTB, 7); COLON( 9, PORTB, 1, 7); break;
   case 4:  COLOFF(PORTD, 6); COLON(12, PORTC, 0, 2); break;
   case 5:  COLOFF(PORTB, 6); COLON(15, PORTB, 7, 3); break;
   case 6:  COLOFF(PORTC, 1); COLON(18, PORTB, 3, 1); break;
   case 7:  COLOFF(PORTB, 1); COLON(21, PORTD, 7, 8); break;
   default: COLOFF(PORTD, 7);

    // There is no physical column #8 on the LED matrix.  Think of this
    // as something like a vertical blank period between one frame and
    // the next.  After column #7 is switched off, various counters,
    // timers, etc. are updated before the next frame starts.  Some
    // attempts were made at handling this in the column 0 or 7 cases,
    // but always ran into issues where the extra time needed here would
    // throw off the PWM timing.  Handling it during a discrete 'LEDs
    // off' case avoids a whole lot of uglies.

    ptr += 24;
    if(++pass >= passes) {    // Advance pass counter; last pass reached?
      pass = 0;               // Reset back to pass #0
      if(++plane >= planes) { // Advance plane counter; last plane reached?
        OCR2A = plane = 0;                // Reset back to plane #0
        if(swapFlag && (--swapFlag == 0)) // If requested, swap buffers
          frontIdx ^= 1;                  // on return to first column
        ptr = img[frontIdx];  // Reset ptr to start of img

        // Dirty hack: in certain cases the button-handling logic doesn't
        // quite fit into the interrupt period.  Matrix is off during this
        // time, so the ugly workaround isn't visible: Timer2 is briefly
        // stopped while this part of the interrupt runs, resumes later.
        uint8_t kludge = TIMSK2;
        TIMSK2 = 0;
        // Watch for button 'hold' conditions
        if(bSave != (_BV(PORTD3) | _BV(PORTD2))) { // button(s) held
          if(bCount >= holdTime) { // ~1.5 second hold
            // If held for 9+ sec, assume watch has been "smooshed"
            // (e.g. in pocket), not intentionally pressed.
            if(!sCount) sleep();
            sCount--;
            if     (bSave == _BV(PORTD3)) bAction = ACTION_HOLD_LEFT;
            else if(bSave == _BV(PORTD2)) bAction = ACTION_HOLD_RIGHT;
            else                          bAction = ACTION_HOLD_BOTH;
            holdFlag = (bAction >= ACTION_HOLD_LEFT);
            bCount   = 0;  // Reset debounce counter
          } else bCount++; // else keep counting...
        } else {
          if(!timeout) sleep();
          timeout--;
        }
        TIMSK2 = kludge; // Resume Timer2
        if(--cCount == 0) cCount = fps >> 1; // Cursor blink counter
      } else OCR2A = (OCR2A << 1) | 1; // Last bitplane not yet reached
    } // else last pass not reached for this bitplane
    col = 0; // Resume at column 0 on next invocation
    break;

  } // end switch(col)

  // Time permitting, go into power-saving mode; CPU stops but LEDs
  // remain lit.  This is ONLY done on higher (long duration) bitplanes
  // and if swapFlag is set -- the latter indicates that the higher-
  // level application code has finished rendering a frame and does not
  // need further CPU cycles to itself.  This does impact the brightness
  // of the first column very slightly as there's a few instructions'
  // delay while the CPU wakes from sleep, but it's a fair price to pay
  // as this provides a pretty substantial power savings (20-30%).

  if(swapFlag && (OCR2A >= napThreshold)) {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    sei(); // Keep interrupts enabled during sleep
    sleep_mode();
  }
}

ISR(INT0_vect) {

  uint8_t b = PIND & (_BV(PORTD3) | _BV(PORTD2));

  // Any button press/release will reset timeout to ~1 sec.
  // Mode-specific code can override this based on action.
  if(timeout < fps) timeout = fps;

  if(b == (_BV(PORTD3) | _BV(PORTD2))) { // Buttons released
    if((bCount > 3)) {                   // Past debounce threshold?
      if(wakeFlag == true) {
        // First click (release) on wake does NOT perform corresponding
        // action (e.g. don't advance digit in time-setting mode).
        bAction  = ACTION_NONE;
        wakeFlag = false;
      } else {
        // If we arrived here by a mode switch (extended hold),
        // the button release should NOT register as a tap.
        if(holdFlag) {
          bAction  = ACTION_NONE;
          holdFlag = false;
        } else {
          if     (bSave == _BV(PORTD3)) bAction = ACTION_TAP_LEFT;
          else if(bSave == _BV(PORTD2)) bAction = ACTION_TAP_RIGHT;
        }
      }
    }
    bCount = 0; sCount = 6;
  } else if(b != bSave) {
    bCount = 0; sCount = 6; // Button press; clear counters
  }
  bSave = b; // Note last button state
}

ISR(INT1_vect, ISR_ALIASOF(INT0_vect));

