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

// This code looks insane and requires quite a bit of explanation...

// First, some of this code might be painful to read in that 'row' and
// 'column' here refer to the hardware pin functions as described in the
// LED matrix datasheet...but with the matrix installed sideways in the
// watch for better component placement, these aren't the same as 'row'
// and 'column' as we usually consider them in computer graphics.  So...
// the higher-level graphics drawing functions use conventional X/Y
// coordinates from the top left of the watch display as normally worn,
// while the lower-level code (most of this library) takes care of mapping
// this to the native orientation of the LED matrix.

// Maybe more important is memory usage.  This library is the biggest
// memory hog in the history of memory hogs, but this came about by design
// rather than negligence.  The primary focus of the entire library is to
// reduce the PWM interrupt handler to the fewest instructions and the
// shortest possible interval, permitting smoother screen refresh (less
// apparent flicker) and more opportunities to put the CPU in a low-power
// sleep state.  All other considerations take a back seat to getting in
// and out of that interrupt as quickly as possible.  Period.

// The row and column lines that drive the display are spread across three
// PORT registers.  Rather than storing a memory-efficient representation
// of the screen bitmap that's then dismantled in code across the three
// PORTs on every interrupt call, screen data is instead stored in a format
// that can be copied directly to the three PORTs...so each 8-pixel, 1-bit
// column of the display actually consumes three bytes instead of one.  A
// full 8x8 pixel 1-bit image requires 24 bytes.

// For a multi-bit image (i.e. 'grayscale,' if we disregard the color of
// the LED matrix), each additional bitplane consumes 24 bytes.  So a
// 1-bit image is only 24 bytes, but a 4-bit image (16 brightness levels)
// needs 24x4 or 96 bytes, and an 8-bit image (256 levels) requires 24x8
// or 192 bytes.

// In the interest of conserving power, we might not always want to run
// all 8 LEDs of a column simultaneously (especially as we're using a
// lithium coin cell, where heavy current draw will deplete this with
// disproportionate speed).  The library can be throttled back to drive
// 4, 2 or even just 1 LED in a column at a time.  But again, with the
// interrupt speed taking precedence, this is achieved by storing multiple
// full 8-LED columns that each have only half the LEDs enabled (because
// this is faster than masking out bits every single time).  For example,
// with 4 (rather than 8) LEDs on at a time, there are essentially two
// copies of image data in RAM -- one with even rows enabled and one with
// odd rows -- but each still requiring the full 24 bytes of data (times
// the number of bitplanes).  An 8-bit image displaying 4 LEDs at a time
// now requires 24x8x2 or 384 bytes for the small 8x8 grid.  2 LEDs at a
// time requires 4 passes, while 1 LED requires 8 passes.  It becomes
// necessary to cut back the number of bitplanes as this approaches the
// total free RAM available on the MCU!  Running more LEDs at a time saves
// RAM, but draws more power.  The right balance must be found between
// looking nice and preserving scarce battery power.  Since the application
// has only modest needs beyond showing a watch face, there's little guilt
// in sacrificing ridiculous amounts of RAM to meet these goals.

// Finally, double-buffering -- a means of achieving flicker-free animation
// by maintaining separate on- and off-screen buffers -- doubles the screen
// memory requirements yet again.

// Screen RAM requirements can thus be calculated as follows:
// 24 bytes * bitplanes * passes * buffers, where:
// Brightness levels = 2^bitplanes; e.g. 1 plane = 2 levels, 2 planes = 4...
// 8 LEDs = 1 pass, 4 LEDs = 2 passes, 2 LEDs = 4 passes, 1 LED = 8 passes.
// Buffers = 1 (single-buffered) or 2 (double-buffered).
// e.g. 16-level, 4 LEDs at a time, double buffered = 24*4*2*2 = 384 bytes.





// These tables help facilitate pixel drawing.  They are intentionally
// NOT placed in PROGMEM in order to save a few instruction cycles.
// (might change that.  Also might use RTClite library to save RAM)
static const uint8_t
  rowBitPortB[] = {    0, 0x20,    0, 0x10, 0x04,    0, 0x01,    0},
  rowBitPortC[] = {    0,    0, 0x08,    0,    0, 0x04,    0,    0},
  rowBitPortD[] = { 0x10,    0,    0,    0,    0,    0,    0, 0x20};

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
  *img[2];                // Pointers to 'front' and 'back' display buffers
static volatile uint8_t
  planes,
  passes,
  plane,
  pass,
  fmode,
  col,
  *ptr,                   // Current pointer into front buffer
  frontIdx = 0,           // Buffer # being displayed (vs modified)
  bSave,                  // Last button state
  bCount   = 0,           // Button hold counter
  bAction  = ACTION_NONE, // Last button action
  frames   = 0;           // Counter for delay()
static volatile boolean
  swapFlag = false,
  wakeFlag = false;
static volatile uint16_t
  timeout = 0;            // Countdown to sleep() (in frames)

// Constructor
Watch::Watch(uint8_t nPlanes, uint8_t nLEDs, boolean dbuf) {
  img[0] = NULL;                        // Image memory not yet alloc'd
  setDisplayMode(nPlanes, nLEDs, dbuf); // Alloc happens here
  constructor(8, 8);                    // Init Adafruit_GFX (8x8 image)
}

// might return FPS!
void Watch::setDisplayMode(uint8_t nPlanes, uint8_t nLEDs, boolean dbuf) {

  // Validate inputs (1-8 planes, 1/2/4/8 LEDs)
  if(nPlanes > 8)            nPlanes = 8;
  if(nLEDs   > WATCH_LEDS_1) nLEDs   = WATCH_LEDS_1;

  // Set plane/pass limits and reset counters
  planes = nPlanes;
  fmode  = nLEDs;
  passes = 1 << nLEDs;
  plane  = planes - 1;
  pass   = passes - 1;
  col    = 7;

  // Always 3 bytes/row * 8 rows...
  // then multiply by number of planes and passes for total byte count.
  int bufSize   = 3 * 8 * planes * passes,
      allocSize = (dbuf == true) ? (bufSize * 2) : bufSize;
// very wasteful on low-nLED displays...that's by design.
// this WILL run out of memory in some situations!

  if(img[0] != NULL) free(img[0]);

  // Allocate and initialize front image buffer:
  if(NULL == (img[0] = (uint8_t *)malloc(allocSize))) return;

  // Clear image buffer
  ptr = img[0];
  for(uint8_t i=0; i<bufSize;) {
    ptr[i++] = PORTB_OFF;
    ptr[i++] = PORTC_OFF;
    ptr[i++] = PORTD_OFF;
  }

  // If double-buffered, copy front image buffer to back
  if(dbuf) {
    img[1] = &img[0][bufSize];
    memcpy(img[1], img[0], bufSize);
  } else {
    img[1] = img[0]; // Else both point to the same address
  }
}

// Initialize PORT registers and enable timer and button interrupts.
void Watch::begin() {

  // Disable unused peripherals
  ADCSRA &= ~_BV(ADEN); // ADC off
  // Timer0 interrupt is disabled as this throws off the delicate PWM
  // timing.  Unfortunately this means delay(), millis() won't work,
  // so we have our own methods instead for passing time.
  TIMSK0 = 0;
  // Disable peripherals that aren't used by this code,
  // maybe save a tiny bit of power.
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  // Comment out this line to use Serial for debugging, etc.:
  power_usart0_disable();

  PORTB   = PORTB_OFF; // Turn all rows/columns off
  PORTC   = PORTC_OFF;
  PORTD   = PORTD_OFF;
  DDRB    = B11111111; // And enable outputs
  DDRC    = B00001111;
  DDRD    = B11110000;

  // Set up Timer2 for matrix interrupt.  Mode 2 (CTC), OC2A off, 1/64 prescale
  TCCR2A  = _BV(WGM21); // Mode 2(CTC), OC2A,2B off
  TCCR2B  = _BV(CS22);  // 1/64 prescale
  //TCCR2B  = _BV(CS21) | _BV(CS20);  // 1/32 prescale
  OCR2A   = 255;
  TIMSK2 |= _BV(OCIE2A);

  // Set up interrupt-on-change for buttons.
  EICRA   = _BV(ISC10)  | _BV(ISC00);  // Trigger on any logic change
  EIMSK   = _BV(INT1)   | _BV(INT0);   // Enable interrupts on pins
  bSave   = PIND & (_BV(PORTD3) | _BV(PORTD2)); // Get initial button state

  sei(); // Enable global interrupts
}

// For double-buffered animation, call this function to display new data.
void Watch::swapBuffers(boolean copy) {
  // Swap actually takes place at specific point in interrupt.
  // Set flag to request swap, then wait for change to complete:
  for(swapFlag = true; swapFlag; );
  if(copy) memcpy(img[1 - frontIdx], img[frontIdx], 3 * 8 * planes * passes);
}

PROGMEM uint8_t flarp[4][8] = {
  { 0,   0,   0,   0,   0,   0,   0,   0 },
  { 0,  24,   0,  24,   0,  24,   0,  24 },
  { 0,  48,  24,  72,   0,  48,  24,  72 },
  { 0,  96,  48, 144,  24, 120,  72, 168 } };

// Basic pixel-drawing function for Adafruit_GFX.
void Watch::drawPixel(int16_t x, int16_t y, uint16_t c) {
  if((x >= 0) && (y >= 0) && (x < 8) && (y < 8)) {

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
                    pgm_read_byte(&flarp[fmode][x]);

//    for(uint8_t bit = 1; bit; bit <<= 1) {
int bit, maxbit;
uint8_t inc = 24 * passes;
maxbit = 1 << planes;
    for(bit = 1; bit < maxbit; bit <<= 1) {
      if(c8 & bit) {
        p[0] |=  bmask;
        p[1] |=  cmask;
        p[2] |=  dmask;
      } else {
        p[0] &= ~bmask;
        p[1] &= ~cmask;
        p[2] &= ~dmask;
      }
      p += inc;
    }
  }
}

// Because Timer0 is disabled (throws off LED brightness), our own delay
// function is provided.  Unlike normal Arduino delay(), the units here
// are NOT milliseconds, but frames.  As noted below, one frame is about
// 1/65 second(ish).  Second parameter is a bitmask of actions that will
// abort the current delay operation (or 0 if no abort option).
void Watch::delay(uint8_t f) {
  for(frames = f; frames; );
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

  unsigned char tmp;

  // Set all ports to high-impedance input mode, enable pullups
  // on all pins (seems to use ever-so-slightly less sauce).
  DDRB  = 0   ; DDRC  = 0   ; DDRD  = 0;
  PORTB = 0xff; PORTC = 0xff; PORTD = 0xff;

  power_timer2_disable();
  power_twi_disable();

  // VITALLY IMPORTANT: note that BOR (brown-out reset) is NOT disabled,
  // even though this can save many micro-Amps during power-down sleep.
  // As the coin cell runs down, brown-outs are actually quite likely.
  // When this happens, if BOR is disabled, the MCU will behave erratically
  // and may jump to any random location...and if this leads into any
  // bootloader code that erases or writes a flash page, the application --
  // or worse, the bootloader itself -- can become corrupted, leaving no
  // easy way to re-flash the watch.  This is NOT the unlikely one-in-a-
  // million chance you might think.  Actual odds seem to be about 1% --
  // the phenomenon has been observed in the wild and even while developing
  // this code.  So BOR is left enabled to provide a proper safety net.
  // Really, do NOT go adding BOR-disabling code, you'll regret it.

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei(); // Keep interrupts enabled during sleep
  sleep_mode();
  // Execution resumes here on wake.

  // wakeFlag stops clicks from falling through on wake
  // (e.g first click won't advance digit in time-setting mode).
  wakeFlag = true;
  bAction  = ACTION_WAKE;

  // Enable only those peripherals used by the watch code
  power_twi_enable();
  Wire.begin(); // App note recommends reinitializing TWI on wake
  power_timer2_enable();

  PORTB = PORTB_OFF; // Turn all rows/columns off
  PORTC = PORTC_OFF;
  PORTD = PORTD_OFF;
  DDRB  = B11111111; // And enable outputs
  DDRC  = B00001111;
  DDRD  = B11110000;
}


// OVERHEAD is the estimated instruction cycle count for the stack work
// done entering and exiting the timer interrupt.  LEDMINTIME is the
// shortest LED 'on' time and must be more than OVERHEAD.  Total PWM cycle
// time will be LEDMINTIME * 255, total refresh time will be 8X cycle time.
#define OVERHEAD   53
#define LEDMINTIME 60
// Issue: with low-bit-depth displays, mintime needs to incporporate
// extra cycles to allow for the "7th row" math.  This isn't needed
// at higher bit depths because of the time doubling.
//#define LEDMINTIME 240
// 60 * 255 = 15300, * 8 = 122400, 8M / 122400 = ~65 Hz
// Because columns are cycled within PWM intervals, the appearance is more
// like 2X this (~130 Hz), though the actual full frame rate is still ~65.

// Turn prior column off, then load new row bits on all 3 PORTs
#define COLSTART(n, port, bit, idx) \
   case n:                     \
    asm volatile(              \
     "sbi %0,%1\n\t"           \
     "ld  __tmp_reg__,%a2\n\t" \
     "out %3,__tmp_reg__\n\t"  \
     "ld  __tmp_reg__,%a4\n\t" \
     "out %5,__tmp_reg__\n\t"  \
     "ld  __tmp_reg__,%a6\n\t" \
     "out %7,__tmp_reg__\n\t"  \
     ::                        \
     "I"(_SFR_IO_ADDR(port)),  \
     "I"(bit),                 \
     "e"(&p[idx]),             \
     "I"(_SFR_IO_ADDR(PORTB)), \
     "e"(&p[idx+1]),           \
     "I"(_SFR_IO_ADDR(PORTC)), \
     "e"(&p[idx+2]),           \
     "I"(_SFR_IO_ADDR(PORTD)));

// Advance 'col' to next column, then enable current column loaded above.
// Columns advance in a bizarre interleaved order of horizontal lines in
// order to reduce apparent flicker and to make multiplexing artifacts
// less objectionable, esp. when scrolling text horizontally.
#define COLEND(port, bit, nxt) \
    col = nxt; \
    asm volatile("cbi %0,%1" :: "I"(_SFR_IO_ADDR(port)), "I"(bit)); \
    break;

// Might use Timer2 for PWM with 64:1 prescaler
// because this is close to the MINTIME value
// 16320 inst in PWM cycle, *8 = 130560 = 61 Hz

// Plan is to eventually make this a 'naked' interrupt w/100% assembly,
// avr-gcc output looks a little bloaty esp. in the stack work...if this
// can be tightened up, OVERHEAD and LEDMINTIME constants above can be
// reduced and a a better refresh rate should be possible.  Already
// unrolled this (e.g. no array lookups), just needs a bit more TLC.
ISR(TIMER2_COMPA_vect, ISR_BLOCK) {

  uint8_t *p = (uint8_t *)ptr;

  switch(col) {
    COLSTART(0, PORTD, 7,  0)
//      OCR1A = (LEDMINTIME << plane) - OVERHEAD; // Interrupt time for plane
OCR2A = 255 >> (8 - plane);
    COLEND(PORTD, 6, 4)
    COLSTART(1, PORTB, 3,  3) COLEND(PORTB, 6, 5)
    COLSTART(2, PORTC, 0,  6) COLEND(PORTC, 1, 6)
    COLSTART(3, PORTB, 7,  9) COLEND(PORTB, 1, 7)
    COLSTART(4, PORTD, 6, 12) COLEND(PORTC, 0, 2)
    COLSTART(5, PORTB, 6, 15) COLEND(PORTB, 7, 3)
    COLSTART(6, PORTC, 1, 18) COLEND(PORTB, 3, 1)
    COLSTART(7, PORTB, 1, 21)
      ptr += 24;
      if(++pass >= passes) {    // Advance pass counter
        pass = 0;               // Reset back to pass #0
        if(++plane >= planes) { // Advance plane counter
          plane = 0;            // Reset back to plane #0
          if(swapFlag) {        // If requested, swap
// this may throw off the sleep, below
            frontIdx ^= 1;      // buffers on return
            swapFlag  = false;  // to first column
          }
          ptr = img[frontIdx];  // Reset ptr to start of img

          // Think of this as something like a vertical blank period.
          // Various counters, etc. occur before next frame starts.

          // Watch for button 'hold' conditions
          if(bSave != (_BV(PORTD3) | _BV(PORTD2))) {
// FPS will now be variable, based on planes & passes, blargh
            if(bCount >= (WATCH_FPS * 2)) { // ~2 second hold
              if     (bSave == _BV(PORTD3)) bAction = ACTION_HOLD_LEFT;
              else if(bSave == _BV(PORTD2)) bAction = ACTION_HOLD_RIGHT;
              else if(bSave == 0          ) bAction = ACTION_HOLD_BOTH;
              bSave = bCount = 0;    // So button release code isn't confused
            } else bCount++;         // else keep counting...
          }

          if(frames > 0)  frames--;  // Counter for delay() function
          if(timeout > 0) timeout--; // Counter for sleep timeout
          else            sleep();
        } // else not advancing plane #
      }   // else not advancing pass #
    COLEND(PORTD, 7, 0)
  }
  // Reset Timer0 counter.  This is done so that the LED 'on' time is
  // more precise -- the above plane-advancing conditional logic won't
  // throw the brightness off.  Need consistent time from end of
  // interrupt to start of next, not uniform start-to-start interval.
//  TCNT1 = 0;

  if(swapFlag && (OCR2A > 4)) {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    sei(); // Keep interrupts enabled during sleep
    sleep_mode();
  }
}

ISR(INT0_vect) {

  uint8_t b = PIND & (_BV(PORTD3) | _BV(PORTD2));

  // Any button press/release will reset timeout to ~4 sec.
  // Mode-specific code can override this based on action.
  if(timeout < (WATCH_FPS * 4)) timeout = (WATCH_FPS * 4);

  if(b == (_BV(PORTD3) | _BV(PORTD2))) { // Buttons released
    if((bCount > 4)) {                   // Past debounce threshold?
      if(wakeFlag == true) {
        // First click (release) on wake does NOT perform corresponding
        // action (e.g. don't advance digit in time-setting mode).
        bAction  = ACTION_NONE;
        wakeFlag = false;
      } else {
        if     (bSave == _BV(PORTD3)) bAction = ACTION_TAP_LEFT;
        else if(bSave == _BV(PORTD2)) bAction = ACTION_TAP_RIGHT;
      }
    }
  } else if(b != bSave) bCount = 0; // Button press; clear debounce counter
  bSave = b; // Note last button state
}

ISR(INT1_vect, ISR_ALIASOF(INT0_vect));

