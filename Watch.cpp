// Library for Adafruit 8x8 LED matrix watch.  This version displays 8-bit
// monochrome graphics with one row enabled at any given time, using a fast
// timer interrupt and bit angle modulation (vs PWM).
// (C) Adafruit Industries.

// To do: add basic button debounce / interpretation to lib

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif
#include <avr/pgmspace.h>
#include "Watch.h"

// Some of this code might be painful to read in that 'row' and 'column'
// here refer to the hardware pin functions as described in the LED matrix
// datasheet...but with the matrix installed sideways in the watch for
// better component placement, these aren't the same as 'row' and 'column'
// as we usually consider them in computer graphics.  What's more, the
// matrix is refreshed bottom to top (columns 7 through 0 in datasheet
// terminology) because this makes multiplexing artifacts least
// objectionable when scrolling text right to left (the most common case).
// The graphics drawing functions use conventional X/Y coordinates from
// the top left of the watch display, and the low-level driver code takes
// care of mapping this to the bizarre rotated layout of the LED matrix.

// Each LED row or column corresponds to a single bit on a specific PORT.
// These tables help facilitate drawing and matrix refresh.  They are
// intentionally NOT placed in PROGMEM in order to save a few cycles in
// the interrupt function -- the faster that runs, the less flicker is
// apparent.  There's ample RAM for this as the screen buffer isn't very
// large (128 bytes for 8x8, 8 bits w/double buffering).  Oink oink!
static volatile unsigned char
  *colPort[]    = {&PORTD,&PORTB,&PORTB,&PORTC,&PORTB,&PORTC,&PORTB,&PORTD};
static const uint8_t
  colBit[]      = {_BV(7),_BV(3),_BV(7),_BV(0),_BV(1),_BV(1),_BV(6),_BV(6)},
  rowBitPortB[] = {    0 ,_BV(5),    0 ,_BV(4),_BV(2),    0 ,_BV(0),    0 },
  rowBitPortC[] = {    0 ,    0 ,_BV(3),    0 ,    0 ,_BV(2),    0 ,    0 },
  rowBitPortD[] = {_BV(4),    0 ,    0 ,    0 ,    0 ,    0 ,    0 ,_BV(5)};

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
  img[2][8 * 8 * 3]; // Display data (double-buffered)
static volatile uint8_t
  plane = 7,
  col   = 7,
  *backBuf,  // Buffer being modified
  *frontBuf, // Buffer being displayed
  *ptr;      // Current pointer into frontBuf
static volatile boolean
  swapFlag = false;

// Constructor: pass 'true' to enable double-buffering (default = false).
Watch::Watch(boolean doubleBuffer) {
  ptr = frontBuf = &img[0][0];
  for(uint8_t i=0; i<(8*8*3);) {
    frontBuf[i++] = PORTB_OFF;
    frontBuf[i++] = PORTC_OFF;
    frontBuf[i++] = PORTD_OFF;
  }
  if(doubleBuffer) {
    backBuf = &img[1][0];
    memcpy((void *)backBuf, (void *)frontBuf, 8 * 8 * 3);
  } else {
    backBuf = frontBuf;
  }
  constructor(8, 8); // Init Adafruit_GFX
}

// Initialize PORT registers and enable timer interrupt for matrix refresh.
void Watch::begin() {
  PORTB = PORTB_OFF; // Turn all rows/columns off
  PORTC = PORTC_OFF;
  PORTD = PORTD_OFF;
  DDRB  = B11111111; // And enable outputs
  DDRC  = B00001111;
  DDRD  = B11110000;

  // Set up interrupt-on-change for buttons
  EICRA = _BV(ISC10) | _BV(ISC00); // Trigger on any logic change
  EIMSK = _BV(INT1)  | _BV(INT0);  // Enable interrupts on pins

  // Set up Timer1 for interrupt:
  TCCR1A  = _BV(WGM11); // Mode 14 (fast PWM), OC1A off
  TCCR1B  = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Mode 14, no prescale
  ICR1    = 100;
  TIMSK1 |= _BV(TOIE1); // Enable Timer1 interrupt

  sei(); // Enable global interrupts
}

// For double-buffered animation, call this function to display new data.
void Watch::swapBuffers(boolean copy) {
  // Swap actually takes place at specific point in interrupt.
  // Set flag to request swap, then wait for change to complete:
  for(swapFlag = true; swapFlag; );
  if(copy) memcpy((void *)backBuf, (void *)frontBuf, 8 * 8 * 3);
}

// Basic pixel-drawing function for Adafruit_GFX.
void Watch::drawPixel(int16_t x, int16_t y, uint16_t c) {
  if((x >= 0) && (y >= 0) && (x < 8) && (y < 8)) {
    uint8_t bmask = rowBitPortB[x],
            cmask = rowBitPortC[x],
            dmask = rowBitPortD[x],
            *p    = (uint8_t *)&backBuf[(7 - y) * 3];
// If doing plane, row:
//            *p    = (uint8_t *)&backBuf[(7 - y) * 24];
    for(uint8_t bit = 1; bit; bit <<= 1) {
      if(c & bit) {
        *p++ |=  bmask;
        *p++ |=  cmask;
        *p++ |=  dmask;
      } else {
        *p++ &= ~bmask;
        *p++ &= ~cmask;
        *p++ &= ~dmask;
      }
      p += 21; // Remove if doing plane, row
    }
  }
}

// These are approximate instruction cycle counts for entering and exiting
// the timer interrupt, and minimum time spent in body of interrupt.
// Derived from disassembled code, might not be 100% accurate, good enough.
#define CALLOVERHEAD 28
#define LOOPTIME     37

ISR(TIMER1_OVF_vect, ISR_BLOCK) {

  // Disable current column.  This is done even when advancing planes
  // on the same column, to avoid momentary flicker when the three PORT
  // registers are reloaded.
  *colPort[col] |= colBit[col];

  if(++col >= 8) {     // Advance column counter
    col = 0;           // Back to column 0
    if(++plane >= 8) { // Advance plane counter
      plane = 0;       // Back to plane 0
      if(swapFlag) {            // If requested,
        volatile uint8_t *temp; // swap buffers
        temp     = frontBuf;    // on return to
        frontBuf = backBuf;     // first column.
        backBuf  = temp;
        swapFlag = false;
      }
      ptr = frontBuf; // Reset image pointer to start
    }
  }

  // Set new row bits (columns are off at this point).  The compiler
  // seems to miss out on an opportunity to use Z+ addressing mode,
  // so a tiny bit of assembly is used to save a few instructions.
  // Temporary variable 'p' is needed because volatile 'ptr' doesn't
  // take well to optimization.  Weird stuff.
  uint8_t *p = (uint8_t *)ptr;
  asm volatile("ld  __tmp_reg__,%a0+" :: "e"(p));
  asm volatile("out %0,__tmp_reg__" :: "I"(_SFR_IO_ADDR(PORTB)));
  asm volatile("ld  __tmp_reg__,%a0+" :: "e"(p));
  asm volatile("out %0,__tmp_reg__" :: "I"(_SFR_IO_ADDR(PORTC)));
  asm volatile("ld  __tmp_reg__,%a0+" :: "e"(p));
  asm volatile("out %0,__tmp_reg__" :: "I"(_SFR_IO_ADDR(PORTD)));
  ptr = p;

  // Set interval for next interrupt
  ICR1  = ((LOOPTIME + CALLOVERHEAD * 2) << plane) - CALLOVERHEAD;
  TCNT1 = 0; // Restart interrupt timer

  // Enable new column
  *colPort[col] &= ~colBit[col];

  TIFR1 |= TOV1; // Clear Timer1 interrupt flag
}

static uint8_t foo = 0;

uint8_t Watch::buttons(void) {
	return foo;
}

ISR(INT0_vect) {
	switch(PIND & B00001100) {
	   case B00000000:
		foo = 3;
		break;
	   case B00000100:
		foo = 2;
		break;
	   case B00001000:
		foo = 1;
		break;
	   case B00001100:
		foo = 0;
		break;
	}
}

ISR(INT1_vect, ISR_ALIASOF(INT0_vect));
