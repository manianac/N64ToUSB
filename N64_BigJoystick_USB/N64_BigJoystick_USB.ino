/* Arduino N64 Controller To USB */
/* Robert Kirk
 * Big Joystick Demo Author: Darran Hunt
 * Controller Communication Assembly Author: Michele Perla (the.mickmad@gmail.com)
 * Released into the public domain.
 */

#define DEBUG
#define VERBOSE
#undef SKIP_LAGFILTER
#undef SKIP_THRESHOLD_ADJUSTER
//Threshold adjust requires lag filter to be on

#include "pins_arduino.h"
#define N64_PIN 2
#define N64_PIN_DIR DDRD
// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define N64_HIGH DDRD &= ~0x04
#define N64_LOW DDRD |= 0x04
#define N64_QUERY (PIND & 0x04)

#define NUM_BUTTONS	40
#define NUM_AXES	8	       // 8 axes, X, Y, Z, etc

#define LAG_COEFFECIENT 0.25f

typedef struct joyReport_t {
  int16_t axis[NUM_AXES];
  uint8_t button[(NUM_BUTTONS+7)/8]; // 8 buttons per byte
} 
joyReport_t;

joyReport_t joyReport;
// bits: 0, 0, 0, start, y, x, b, a
// bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
char N64_raw_dump[33]; // 1 received bit per byte
void setup(void);
void loop(void);
void sendJoyReport(joyReport_t *report);
void translate_raw_data();
void N64_send(unsigned char *buffer, char length);
void N64_get();

static int16_t max_stickx;
static int16_t max_sticky;
static int16_t min_stickx;
static int16_t min_sticky;

static int16_t old_stickx;
static int16_t old_sticky;

void setup() 
{
  //Initial values to prevent div/0 bugs
  max_stickx = max_sticky = 1000;
  min_stickx = min_sticky = -1000;
  old_stickx = old_sticky = 0;
  Serial.begin(115200);
  delay(200);
  // Communication with n64 controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(N64_PIN, LOW);  
  pinMode(N64_PIN, INPUT);
}

// Send an HID report to the USB interface, or debug vales to the serial console
void sendJoyReport(struct joyReport_t *report)
{
#ifndef DEBUG
  Serial.write((uint8_t *)report, sizeof(joyReport_t));
#else
  Serial.print("Start: ");
  Serial.print(joyReport.button[0] & 16 ? 1:0);

  Serial.print("Z:     ");
  Serial.print(joyReport.button[0] & 32 ? 1:0);

  Serial.print("B:     ");
  Serial.print(joyReport.button[0] & 64 ? 1:0);

  Serial.print("A:     ");
  Serial.print(joyReport.button[0] & 128 ? 1:0);

  Serial.print("L:     ");
  Serial.print(joyReport.button[1] & 32 ? 1:0);
  Serial.print("R:     ");
  Serial.print(joyReport.button[1] & 16 ? 1:0);

  Serial.print("Cup:   ");
  Serial.print(joyReport.button[1] & 0x08 ? 1:0);
  Serial.print("Cdown: ");
  Serial.print(joyReport.button[1] & 0x04 ? 1:0);
  Serial.print("Cright:");
  Serial.print(joyReport.button[1] & 0x01 ? 1:0);
  Serial.print("Cleft: ");
  Serial.print(joyReport.button[1] & 0x02 ? 1:0);

  Serial.print("Dup:   ");
  Serial.print(joyReport.button[0] & 0x08 ? 1:0);
  Serial.print("Ddown: ");
  Serial.print(joyReport.button[0] & 0x04 ? 1:0);
  Serial.print("Dright:");
  Serial.print(joyReport.button[0] & 0x01 ? 1:0);
  Serial.print("Dleft: ");
  Serial.print(joyReport.button[0] & 0x02 ? 1:0);

  Serial.print("Stick X:");
  Serial.print(joyReport.axis[0], DEC);
  Serial.print("Stick Y:");
  Serial.print(joyReport.axis[1], DEC);
  Serial.println();
#ifdef VERBOSE
  Serial.print(" Min_StickX: ");
  Serial.print(min_stickx);
  Serial.print(" Min_StickY: ");
  Serial.print(min_sticky);
  Serial.print(" Max_StickX: ");
  Serial.print(max_stickx);
  Serial.print(" Max_StickY: ");
  Serial.print(max_sticky);
  Serial.print(" Old Stick X: ");
  Serial.print(old_stickx);
  Serial.print(" Old Stick Y: ");
  Serial.println(old_sticky);
#endif
#endif
}



/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * Oh, it destroys the buffer passed in as it writes it
 */
void N64_send(unsigned char *buffer, char length)
{
  // Send these bytes
  char bits;

  bool bit;

  // This routine is very carefully timed by examining the assembly output.
  // Do not change any statements, it could throw the timings off
  //
  // We get 16 cycles per microsecond, which should be plenty, but we need to
  // be conservative. Most assembly ops take 1 cycle, but a few take 2
  //
  // I use manually constructed for-loops out of gotos so I have more control
  // over the outputted assembly. I can insert nops where it was impossible
  // with a for loop

  asm volatile (";Starting outer for loop");
outer_loop:
  {
    asm volatile (";Starting inner for loop");
    bits=8;
inner_loop:
    {
      // Starting a bit, set the line low
      asm volatile (";Setting line to low");
      N64_LOW; // 1 op, 2 cycles

      asm volatile (";branching");
      if (*buffer >> 7) {
        asm volatile (";Bit is a 1");
        // 1 bit
        // remain low for 1us, then go high for 3us
        // nop block 1
        asm volatile ("nop\nnop\nnop\nnop\nnop\n");

        asm volatile (";Setting line to high");
        N64_HIGH;

        // nop block 2
        // we'll wait only 2us to sync up with both conditions
        // at the bottom of the if statement
        asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          );

      } 
      else {
        asm volatile (";Bit is a 0");
        // 0 bit
        // remain low for 3us, then go high for 1us
        // nop block 3
        asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\nnop\n"  
          "nop\n");

        asm volatile (";Setting line to high");
        N64_HIGH;

        // wait for 1us
        asm volatile ("; end of conditional branch, need to wait 1us more before next bit");

      }
      // end of the if, the line is high and needs to remain
      // high for exactly 16 more cycles, regardless of the previous
      // branch path

      asm volatile (";finishing inner loop body");
      --bits;
      if (bits != 0) {
        // nop block 4
        // this block is why a for loop was impossible
        asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
          "nop\nnop\nnop\nnop\n");
        // rotate bits
        asm volatile (";rotating out bits");
        *buffer <<= 1;

        goto inner_loop;
      } // fall out of inner loop
    }
    asm volatile (";continuing outer loop");
    // In this case: the inner loop exits and the outer loop iterates,
    // there are /exactly/ 16 cycles taken up by the necessary operations.
    // So no nops are needed here (that was lucky!)
    --length;
    if (length != 0) {
      ++buffer;
      goto outer_loop;
    } // fall out of outer loop
  }

  // send a single stop (1) bit
  // nop block 5
  asm volatile ("nop\nnop\nnop\nnop\n");
  N64_LOW;
  // wait 1 us, 16 cycles, then raise the line 
  // 16-2=14
  // nop block 6
  asm volatile ("nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"  
    "nop\nnop\nnop\nnop\n");
  N64_HIGH;

}

void N64_get()
{
  // listen for the expected 8 bytes of data back from the controller and
  // blast it out to the N64_raw_dump array, one bit per byte for extra speed.
  // Afterwards, call translate_raw_data() to interpret the raw data and pack
  // it into the N64_status struct.
  asm volatile (";Starting to listen");
  unsigned char timeout;
  char bitcount = 32;
  char *bitbin = N64_raw_dump;

  // Again, using gotos here to make the assembly more predictable and
  // optimization easier (please don't kill me)
read_loop:
  timeout = 0x3f;
  // wait for line to go low
  while (N64_QUERY) {
    if (!--timeout)
      return;
  }
  // wait approx 2us and poll the line
  asm volatile (
  "nop\nnop\nnop\nnop\nnop\n"  
    "nop\nnop\nnop\nnop\nnop\n"  
    "nop\nnop\nnop\nnop\nnop\n"  
    "nop\nnop\nnop\nnop\nnop\n"  
    "nop\nnop\nnop\nnop\nnop\n"  
    "nop\nnop\nnop\nnop\nnop\n"  
    );
  *bitbin = N64_QUERY;
  ++bitbin;
  --bitcount;
  if (bitcount == 0)
    return;

  // wait for line to go high again
  // it may already be high, so this should just drop through
  timeout = 0x3f;
  while (!N64_QUERY) {
    if (!--timeout)
      return;
  }
  goto read_loop;

}

void translate_raw_data()
{
  int16_t new_stickx = 0;
  int16_t new_sticky = 0;
  // The get_N64_status function sloppily dumps its data 1 bit per byte
  // into the get_status_extended char array. It's our job to go through
  // that and put each piece neatly into the struct N64_status
  memset(&joyReport, 0, sizeof(joyReport_t));
  // line 1
  // bits: A, B, Z, Start, Dup, Ddown, Dleft, Dright
  // line 2
  // bits: 0, 0, L, R, Cup, Cdown, Cleft, Cright
  // line 3
  // bits: joystick x value
  // These are 8 bit values centered at 0x80 (128)
  // line 4
  // bits: joystick 4 value
  // These are 8 bit values centered at 0x80 (128)
  for (int i=0; i<8; i++) {
    joyReport.button[0] |= N64_raw_dump[i] ? (0x80 >> i) : 0;
    joyReport.button[1] |= N64_raw_dump[8+i] ? (0x80 >> i) : 0;
    new_stickx |= (N64_raw_dump[16+i] ? (0x8000 >> i) : 0);
    new_sticky |= (N64_raw_dump[24+i] ? (0x8000 >> i) : 0);
  }
#ifndef SKIP_LAGFILTER
  //Simple operation to smooth movement over 16 bits (Since the N64 controller only provides 8bit resolution)
  int16_t lag_stickx = old_stickx + ((new_stickx - old_stickx) * LAG_COEFFECIENT);
  int16_t lag_sticky = old_sticky + ((new_sticky - old_sticky) * LAG_COEFFECIENT);
  joyReport.axis[0] = lag_stickx;
  joyReport.axis[1] = lag_sticky;
  old_stickx = new_stickx;
  old_sticky = new_sticky;
#else
  joyReport.axis[0] = new_stickx;
  joyReport.axis[1] = new_sticky;
#endif
#ifndef SKIP_THRESHOLD_ADJUSTER
  //This starts out by storing the min/max values of the joystick
  max_stickx = max(max_stickx, new_stickx);
  max_sticky = max(max_sticky, new_sticky);
  min_stickx = min(min_stickx, new_stickx);
  min_sticky = min(min_sticky, new_sticky);
  //Then we take the known min/max and scale it up to 16bit max to provide simulated range for damage joysticks
  joyReport.axis[0] = (int16_t)(lag_stickx / (float)(lag_stickx >= 0.0f ? max_stickx : min_stickx) * 32767.0f);
  if (lag_stickx < 0) joyReport.axis[0] = -joyReport.axis[0]; //Cheap hack since the above operation removes signage
  joyReport.axis[1] = (int16_t)(lag_sticky / (float)(lag_sticky >= 0.0f ? max_sticky : min_sticky) * 32767.0f);
  if (lag_sticky < 0) joyReport.axis[1] = -joyReport.axis[1]; //Cheap hack since the above operation removes signage
#endif
}

void loop() 
{
  // Command to send to the gamecube
  // The last bit is rumble, flip it to rumble
  // yes this does need to be inside the loop, the
  // array gets mutilated when it goes through N64_send
  unsigned char command[] = {
    0x01  };
  //unsigned char command2[] = {0x01};
  // don't want interrupts getting in the way
  noInterrupts();
  // send those 3 bytes
  N64_send(command, 1);
  // read in data and dump it to N64_raw_dump
  N64_get(); 
  interrupts();
  translate_raw_data();
  sendJoyReport(&joyReport);
  //If this delay is too low, the 8u2 avr (r1 and r2 UNOs) overwrites data and sends back garbage
  delay(30);
}












