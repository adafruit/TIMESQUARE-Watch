/*
Idea: have both 8-bit (high quality) and 1-bit (low power) display
modes.  The time-setting code can use the 1-bit stuff (since it's
seldom seen), and the display code has the option of 8-bit display.
This will save a ton of power during the initial (lengthy) time-
setting process.

Power-down code will all be moved here, not just the higher-level stuff.
Watch modes will just set a timeout variable (or call a function to set
it) rather than invoking sleep directly.  Timer counter will count DOWN
rather than up.  Go to deep sleep when it reaches 0.  Any operation while
on (button press, etc.) just resets the counter to a higher number.
This way the counter can also be used for a nice fade out on the display
(rather than just shutting off when sleeping).  Counter will need to be
a 16-bit type for longer timeouts to work.
*/




// Library for Adafruit 8x8 LED matrix watch.  This version displays 8-bit
// monochrome graphics with one row enabled at any given time, using a fast
// timer interrupt and bit angle modulation (vs PWM).
// (C) Adafruit Industries.

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

// Some of this code might be painful to read in that 'row' and 'column'
// here refer to the hardware pin functions as described in the LED matrix
// datasheet...but with the matrix installed sideways in the watch for
// better component placement, these aren't the same as 'row' and 'column'
// as we usually consider them in computer graphics.  So...the graphics
// drawing functions use conventional intuitive X/Y coordinates from the
// top left of the watch display, and the lower-level code takes care of
// mapping this to the oddly rotated layout of the LED matrix.

// These tables help facilitate pixel drawing.  They are intentionally
// NOT placed in PROGMEM in order to save a few instruction cycles.
// There's ample RAM for this as the screen buffer isn't very large
// (192 bytes single-buffered, 384 bytes double-buffered).  Oink oink!
static const uint8_t
  rowBitPortB[] = {    0, 0x20,    0, 0x10, 0x04,    0, 0x01,    0},
  rowBitPortC[] = {    0,    0, 0x08,    0,    0, 0x04,    0,    0},
  rowBitPortD[] = { 0x10,    0,    0,    0,    0,    0,    0, 0x20};

// The 'off' state for rows and columns is different because one represents
// anodes and the other cathodes.  So this weird combination of bits sets
// all rows and columns to their respective 'off' states:
#define PORTB_OFF B11001010
#define PORTC_OFF B00110011 // PC4, PC5 are set to enable I2C pinups
#define PORTD_OFF B11001100 // PD2, PD3 are set to enable button pullups

// Because the LED matrix ties up so many of the microcontroller's pins,
// the idea of object-orientating the code for multiple instances is a bit
// nonsensical.  It's done insofar as Adafruit_GFX can be used, but all
// the matrix-specific variables and such are simply declared in the code
// here rather than in private vars.  Keeps the interrupt code simple.
static uint8_t
  *img[2];                // Display data
static volatile uint8_t
  plane    = 7,
  col      = 7,
  *ptr,                   // Current pointer into front buffer
  frontIdx = 0,           // Buffer # being displayed (vs modified)
  bSave,                  // Last button state
  bCount   = 0,           // Timer2 overflow counter
  bAction  = ACTION_NONE, // Last button action
  frames   = 0,           // For delay() counter
  mode     = MODE_8BIT;   // Default display mode = 8-bit
static volatile boolean
  swapFlag = false;

// Constructor: pass 'true' to enable double-buffering (default = false).
Watch::Watch(boolean dbuf) {
  int bufSize   = 3 * 8 * 8, // 3 bytes/row * 8 rows * 8 planes
      allocSize = (dbuf == true) ? (bufSize * 2) : bufSize;

  // Allocate and initialize front image buffer:
  if(NULL == (img[0] = (uint8_t *)malloc(allocSize))) return;
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
  constructor(8, 8); // Init Adafruit_GFX
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
  power_usart0_disable();

  PORTB = PORTB_OFF; // Turn all rows/columns off
  PORTC = PORTC_OFF;
  PORTD = PORTD_OFF;
  DDRB  = B11111111; // And enable outputs
  DDRC  = B00001111;
  DDRD  = B11110000;

  // Set up Timer1 for matrix interrupt.  Mode 4 (CTC), OC1A off, no prescale.
  TCCR1A  = 0;
  TCCR1B  = _BV(WGM12) | _BV(CS10);
  OCR1A   = 100;
  TIMSK1 |= _BV(OCIE1A);

  // Set up Timer2 for button-hold counter.  Mode 0, OC2A off, 1024x prescale.
  TCCR2A = 0;
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  // Timer2 interrupt is not enabled until a button is pressed.
// Will likely remove Timer2 use and instead use Timer1 counter for
// double duty.

  // Set up interrupt-on-change for buttons.
  EICRA = _BV(ISC10)  | _BV(ISC00);  // Trigger on any logic change
  EIMSK = _BV(INT1)   | _BV(INT0);   // Enable interrupts on pins
  bSave = _BV(PORTD3) | _BV(PORTD2); // Get initial button state

  sei(); // Enable global interrupts
}

// For double-buffered animation, call this function to display new data.
void Watch::swapBuffers(boolean copy) {
  // Swap actually takes place at specific point in interrupt.
  // Set flag to request swap, then wait for change to complete:
  for(swapFlag = true; swapFlag; );
  if(copy) {
    memcpy(img[1 - frontIdx], img[frontIdx],
      (mode == MODE_8BIT) ? (3 * 8 * 8) : (3 * 8));
  }
}

// Basic pixel-drawing function for Adafruit_GFX.
void Watch::drawPixel(int16_t x, int16_t y, uint16_t c) {
  if((x >= 0) && (y >= 0) && (x < 8) && (y < 8)) {
    if(mode == MODE_8BIT) {
      uint8_t bmask = rowBitPortB[x],
              cmask = rowBitPortC[x],
              dmask = rowBitPortD[x],
              c8    = (uint8_t)c,
              *p    = (uint8_t *)&img[1 - frontIdx][y * 3];
      for(uint8_t bit = 1; bit; bit <<= 1) {
        if(c8 & bit) {
          p[0] |=  bmask;
          p[1] |=  cmask;
          p[2] |=  dmask;
        } else {
          p[0] &= ~bmask;
          p[1] &= ~cmask;
          p[2] &= ~dmask;
        }
        p += 24;
      }
    } else {
    }
  }
}

// Because Timer0 is disabled (throws off LED brightness), our own delay
// function is provided.  Unlike normal Arduino delay(), the units here
// are NOT milliseconds, but frames.  As noted below, one frame is about
// 1/65 second(ish).  Second parameter is a bitmask of actions that will
// abort the current delay operation (or 0 if no abort option).
void Watch::delay(uint8_t f) {
  for(frames = 0; frames < f;);
}

uint8_t *Watch::backBuffer(void) {
  return img[1 - frontIdx];
}

uint8_t Watch::action(void) {
  uint8_t a = bAction;
  bAction   = ACTION_NONE;
  return  a;
}


// OVERHEAD is the estimated instruction cycle count for the stack work
// done entering and exiting the timer interrupt.  LEDMINTIME is the
// shortest LED 'on' time and must be more than OVERHEAD.  Total PWM cycle
// time will be LEDMINTIME * 255, total refresh time will be 8X cycle time.
#define OVERHEAD   53
#define LEDMINTIME 60
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

// Plan is to eventually make this a 'naked' interrupt w/100% assembly,
// avr-gcc output looks a little bloaty esp. in the stack work...if this
// can be tightened up, OVERHEAD and LEDMINTIME constants above can be
// reduced and a a better refresh rate should be possible.  Already
// unrolled this (e.g. no array lookups), just needs a bit more TLC.
ISR(TIMER1_COMPA_vect, ISR_BLOCK) {

// Make this do 1-bit/8-bit magic now

  uint8_t *p = (uint8_t *)ptr;

  switch(col) {
    COLSTART(0, PORTD, 7,  0)
      OCR1A = (LEDMINTIME << plane) - OVERHEAD; // Interrupt time for plane
    COLEND(PORTD, 6, 4)
    COLSTART(1, PORTB, 3,  3) COLEND(PORTB, 6, 5)
    COLSTART(2, PORTC, 0,  6) COLEND(PORTC, 1, 6)
    COLSTART(3, PORTB, 7,  9) COLEND(PORTB, 1, 7)
    COLSTART(4, PORTD, 6, 12) COLEND(PORTC, 0, 2)
    COLSTART(5, PORTB, 6, 15) COLEND(PORTB, 7, 3)
    COLSTART(6, PORTC, 1, 18) COLEND(PORTB, 3, 1)
    COLSTART(7, PORTB, 1, 21)
      if(++plane >= 8) { // Advance plane counter
        plane = 0;       // Back to plane 0
        if(swapFlag) {       // If requested, swap
          frontIdx ^= 1;     // buffers on return
          swapFlag  = false; // to first column
        }
        ptr = img[frontIdx];
        frames++; // For delay() function
      } else ptr += 24;
    COLEND(PORTD, 7, 0)
  }
  // Reset Timer0 counter.  This is done so that the LED 'on' time is
  // more precise -- the above plane-advancing conditional logic won't
  // throw the brightness off.  Need consistent time from end of
  // interrupt to start of next, not uniform start to start interval.
  TCNT1 = 0;
}

ISR(INT0_vect) {

  uint8_t b = PIND & (_BV(PORTD3) | _BV(PORTD2));

  if(b == (_BV(PORTD3) | _BV(PORTD2))) { // Buttons released
    TIMSK2 &= ~_BV(TOIE2);               // Disable Timer2 interrupt
    if((bCount > 2)) {                   // Past debounce threshold?
      if     (bSave == _BV(PORTD3)) bAction = ACTION_TAP_LEFT;
      else if(bSave == _BV(PORTD2)) bAction = ACTION_TAP_RIGHT;
    }
  } else {                 // Button pressed
    if(b == bSave) return; // Debounce
    bCount  = TCNT2 = 0;   // Clear counter + timer
    TIMSK2 |= _BV(TOIE2);  // Enable Timer2 interrupt
  }
  bSave = b; // Note last button change
}

ISR(INT1_vect, ISR_ALIASOF(INT0_vect));

// Maybe change this to 100 Hz interval for easier timeout math.
// On the other hand, minimizing interrupts will keep display looking
// smoother, and provides more deep sleep opportunities.

// Overflow = 256 * 1024 inst.  8MHz / (256 * 1024) = ~30.5 Hz, ~33 msec.
ISR(TIMER2_OVF_vect) {
  if(bCount >= 60) {              // ~2 second hold
    TIMSK2 &= ~_BV(TOIE2);        // Stop interrupt
    if     (bSave == _BV(PORTD3)) bAction = ACTION_HOLD_LEFT;
    else if(bSave == _BV(PORTD2)) bAction = ACTION_HOLD_RIGHT;
    else if(bSave == 0          ) bAction = ACTION_HOLD_BOTH;
    bSave = bCount = 0;           // So button release code isn't confused
  } else bCount++;                // else keep counting...
}

// Puts watch into extremely low-power state, disabling all peripherals and
// brownout detection.  Resumes on pin change interrupt (either button).
void Watch::sleep(void) {

  unsigned char tmp;

  // Set all ports to high-impedance input mode, enable pullups
  // on all pins (seems to use ever-so-slightly less sauce).
  DDRB  = 0   ; DDRC  = 0   ; DDRD  = 0;
  PORTB = 0xff; PORTC = 0xff; PORTD = 0xff;

  TCCR2B = 0; // Stop Timer2
  power_timer2_disable();
  power_timer1_disable();
  power_twi_disable();

  // BOD disable code adapted from Rocket Scream's LowPower library
  // https://github.com/rocketscream/Low-Power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  asm volatile(
    "in   %[tmp]  , %[mcucr]\n"
    "ori  %[tmp]  , %[bods_bodse]\n"
    "out  %[mcucr], %[tmp]\n"
    "andi %[tmp]  , %[not_bodse]\n"
    "out  %[mcucr], %[tmp]" :
    [tmp]        "=&d"(tmp) :
    [mcucr]      "I"  _SFR_IO_ADDR(MCUCR),
    [bods_bodse] "i"  (_BV(BODS) | _BV(BODSE)),
    [not_bodse]  "i"  (~_BV(BODSE)));
  sei();
  sleep_cpu();
  // Execution resumes here on wake

  // Enable only those peripherals used by the watch code
  power_twi_enable();
  Wire.begin(); // App note recommends reinitializing TWI on wake
  power_timer1_enable();
  power_timer2_enable();
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // Restore Timer2 settings

  PORTB = PORTB_OFF; // Turn all rows/columns off
  PORTC = PORTC_OFF;
  PORTD = PORTD_OFF;
  DDRB  = B11111111; // And enable outputs
  DDRC  = B00001111;
  DDRD  = B11110000;
}

Watch::setMode(uint8_t newMode) {

  if(mode == newMode) return;  // Same mode as before -- do nothing
  mode = newMode;

  switch(mode) {
   case MODE_1BIT:
    break;
   case MODE_8BIT:
    break;
  }

// Stop current interrupt at last position
// Clear display buffers (front and back)
// Reset row/plane counters
// Start interrupt anew


}

