//********************************************************************************************************//
//           Teensy 4.0 Program to test high speed DMA from memory and GPIO2 to memory for uMD2           //
//                Copyright® Jan Beck and Sam Goldwasser, 1994-2025, all rights reserved.                 //
//      The code may be freely used or distributed for non-commercial (mostly) educational purposes.      //
//   One issue is that it isn't very high speed, stuck at 100 ns/32 bit word copy.  There may be others.  //
//********************************************************************************************************//

#include <Arduino.h>
#include "DMAChannel.h"

#define Print_Verbose 0    // If 1, print verbose raw buffer data
#define VERBOSE_SIZE 1024  // Number of values to print: first 32, then 1 every 32nd sample
#define Print_Analysis 1   // If 1, print transition counts, waveform periods (samples and duration), and pin frequencies

#define BUFFER_SIZE 10000    // Total buffer size in 32 bit words for samples
#define TRANSFER_SIZE 10000  // Transfer size in 32 bit words

// REFI, MEAS1, MEAS1I, MEAS2I, and MEAS3I are on GPIO2; REF and MEAS3 on GPIO1.
//  (GPIO7 and GPIO2, respectively, have the same signals but cannot use DMA.)
#define REF_Pin 0      // REF     GPIO2 bit 03 0x8
#define REFI_Pin 6     // REFI    GPIO2 bit 10 0x400
#define MEAS1_Pin 9    // MEAS1   GPIO2 bit 11 0x800
#define MEAS1I_Pin 8   // MEAS1I  GPIO2 bit 16 0x10000
#define MEAS2_Pin 10   // MEAS2   GPIO2 bit 00 0x1
#define MEAS2I_Pin 11  // MEAS2I  GPIO2 bit 02 0x4
#define MEAS3_Pin 14   // MEAS3   GPIO1 bit 18 0x40000
#define MEAS3I_Pin 12  // MEAS3I  GPIO1 bit 01 0x2

// Create bitmasks for each pin
const uint32_t REF_MASK = digitalPinToBitMask(REF_Pin);
const uint32_t REFI_MASK = digitalPinToBitMask(REFI_Pin);
const uint32_t MEAS1_MASK = digitalPinToBitMask(MEAS1_Pin);
const uint32_t MEAS1I_MASK = digitalPinToBitMask(MEAS1I_Pin);
const uint32_t MEAS2_MASK = digitalPinToBitMask(MEAS2_Pin);
const uint32_t MEAS2I_MASK = digitalPinToBitMask(MEAS2I_Pin);
const uint32_t MEAS3_MASK = digitalPinToBitMask(MEAS3_Pin);
const uint32_t MEAS3I_MASK = digitalPinToBitMask(MEAS3I_Pin);

const uint32_t ALL_PINS_MASK = REF_MASK | REFI_MASK | MEAS1_MASK | MEAS1I_MASK | MEAS2_MASK | MEAS2I_MASK | MEAS3_MASK | MEAS3I_MASK;

uint32_t TRANSFER_COUNT = TRANSFER_SIZE;

float Start_Time = 0;  // Measure time spent in transfer (us)
float End_Time = 0;
float Elapsed_Time_Float = 0;     // End Time - Start Time (us)
float Period_Float = 0;           // Average number of transfers between edges
float Time_per_Copy_Float = 0;    // (ns)
float Time_per_Period_Float = 0;  // (ns)
float Transfer_Size_Float = 0;    // Points
float Frequency_Float = 0;        // (MHz)

#define GPIO7 0x42004008  // Address of GPIO PSR

// DMA buffer
DMAMEM static uint32_t srcBuffer[BUFFER_SIZE] __attribute__((aligned(32)));
DMAMEM static uint32_t destBuffer[BUFFER_SIZE] __attribute__((aligned(32)));

// Completion flag
volatile bool transferComplete = false;
int GPIO_Print = GPIO7;  // Only for the print info

// DMA channel
DMAChannel dma;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial
  Serial.begin(115200);

  // Print info
  Serial.print("\n******** Direct DMA Tests from Memery and GPIO to Memory *******\n\n\n\n\n\n\n\n");

  /***** Transfer of FAKE DATA from memory to memory. *****/
  
  Serial.printf("\n  Transfer of %d 32 bit words from srcBuffer to destBuffer.\n", TRANSFER_SIZE);
  Serial.println("  Source data: Squarewave REF with a period of 64; Destination prefilled with 0xDEADFEEDs");

  // Initialize buffers with FAKE DATA (squarewave) and constant 0xDEADFEEDs
  for (int i = 0; i < BUFFER_SIZE; i++) {
    destBuffer[i] = 0xDEADFEED;
    if ((i & 0X20) == 0x20) srcBuffer[i] = REF_MASK;
    else srcBuffer[i] = 0;
  }

  // Initialize DMA memory to memory transfer
  dma.begin(true);
  dma.sourceBuffer(srcBuffer, BUFFER_SIZE);              // BUFFER_SIZE doesn't appear to make a difference
  dma.destinationBuffer(destBuffer, TRANSFER_SIZE * 4);  // Must be 4x?
  dma.disableOnCompletion();
  dma.attachInterrupt(dma_complete_isr);                 // ISR diables DMA
  dma.interruptAtCompletion();
  dma.triggerContinuously();

  // Flush cache - may be needed to not keep copying old stuff
  arm_dcache_flush_delete(srcBuffer, sizeof(srcBuffer));
  arm_dcache_flush_delete(destBuffer, sizeof(destBuffer));

  transferComplete = false;
  dma.enable();  // Do it!

  Start_Time = micros();
  while (!transferComplete && (micros() - Start_Time < 1000)) {};  // Wait for completion or timeout
  End_Time = micros();

  analyzeCapture();

  /***** Transfer of GPIO2_PSR to memory. *****/

  Serial.printf("\n  Transfer of %d 32 bit words from GPIO2_PSR to destBuffer.\n", TRANSFER_SIZE);
  Serial.println("  Source data: GPIO_PSR; Destination prefilled with 0xDEADFEEDs");

  // Load srcBuffer with constant length random numbers and destBuffer with 0xDEADFEEDs
  for (int i = 0; i < BUFFER_SIZE; i++) {
    destBuffer[i] = 0xDEADFEED;
  }

  // Initialize DMA fpr GPIO2 transfer.  PSR="Pin State Register".  Do NOT use GPIO_DR as it is really for output.
  dma.begin(true);
  dma.sourceBuffer(&GPIO2_PSR, BUFFER_SIZE);             // BUFFER_SIZE doesn't appear to make a difference
  dma.destinationBuffer(destBuffer, TRANSFER_SIZE * 4);  // Must be 4x?
  dma.disableOnCompletion();
  dma.attachInterrupt(dma_complete_isr);                 // ISR disables DMA
  dma.interruptAtCompletion();
  dma.triggerContinuously();
  dma.TCD->SOFF = 0;                                     // Don't increment source address for GPIO

  // Flush cache - needed to force copy
  arm_dcache_flush_delete(destBuffer, sizeof(destBuffer));

  Serial.print("\n  Starting DMA transfer.  ");

  transferComplete = false;
  dma.enable();  // Do it!

  Start_Time = micros();
  while (!transferComplete && (micros() - Start_Time < 1000)) {};  // Wait for completion or timeout
  End_Time = micros();

  analyzeCapture();

/***** End *****/

  Serial.print("\nTests Complete.\n\n");
}

// ISR for DMA completion
void dma_complete_isr(void) {
  transferComplete = true;
  dma.clearInterrupt();
}

// Analysis of the captured data edge count, period in edges, and frequency in MHz
void analyzeCapture() {

  // Display transfer time, time/copy, and copy frequency
  Serial.printf("\n    Elapsed Time: ");
  Elapsed_Time_Float = End_Time - Start_Time;
  Serial.print(Elapsed_Time_Float);
  Serial.printf(" Microseconds.  Time per Copy: ");
  Transfer_Size_Float = TRANSFER_SIZE;
  Serial.print(Time_per_Copy_Float = (1000 * Elapsed_Time_Float) / Transfer_Size_Float);
  Serial.printf(" ns.  Transfer Rate: ");
  Serial.print(1000 / Time_per_Copy_Float);
  Serial.printf(" MW/s.\n");

#if Print_Verbose == 1
  // Display the raw captured data in a readable format
  Serial.println("\n   Sample  RAW HEX   REF   REFI   MEAS1   MEAS1I   MEAS2   MEAS2I   MEAS3   MEAS3I");
  Serial.println("  ---------------------------------------------------------------------------------");

  // Print each sample with pin data bit values
  for (uint32_t i = 0; i < VERBOSE_SIZE; i++) {
    // Only print first 32 samples, then every 32nd sample to avoid flooding serial
    if (i < 32 || i % 32 == 0) {
      uint32_t value = destBuffer[i];

      // Print sample number and raw hex value
      Serial.printf("    %4d", i);
      Serial.printf("   0x%06X", value);

      // Extract and print individual pin values
      Serial.print("   ");
      Serial.print((value & REF_MASK) ? "1" : "0");
      Serial.print("     ");
      Serial.print((value & REFI_MASK) ? "1" : "0");
      Serial.print("       ");
      Serial.print((value & MEAS1_MASK) ? "1" : "0");
      Serial.print("       ");
      Serial.print((value & MEAS1I_MASK) ? "1" : "0");
      Serial.print("        ");
      Serial.print((value & MEAS2_MASK) ? "1" : "0");
      Serial.print("       ");
      Serial.print((value & MEAS2I_MASK) ? "1" : "0");
      Serial.print("        ");
      Serial.print((value & MEAS3_MASK) ? "1" : "0");
      Serial.print("       ");
      Serial.print((value & MEAS3I_MASK) ? "1" : "0");
      Serial.println();
    }

    // Print a summary message if we're skipping a lot of data
    if (i == 32) {
      Serial.println("  ... (printing every 32nd sample) ...");
    }
  }
#endif

// Display transition counts, sample period and duration, pin frequency
#if Print_Analysis == 1
  uint32_t transitions[8] = { 0, 0, 0, 0, 0, 0 };
  uint32_t masks[8] = { REF_MASK, REFI_MASK, MEAS1_MASK, MEAS1I_MASK, MEAS2_MASK, MEAS2I_MASK, MEAS3_MASK, MEAS3I_MASK };
  const char* pinNames[8] = { "REF", "REFI", "MEAS1", "MEAS1I", "MEAS2", "MEAS2I", "MEAS3", "MEAS3I" };

  // Get initial pin states
  uint32_t prevStates[8];
  for (int pin = 0; pin < 8; pin++) {
    prevStates[pin] = destBuffer[0] & masks[pin] ? 1 : 0;
  }

  // Count transitions
  for (uint32_t i = 1; i < BUFFER_SIZE; i++) {
    for (int pin = 0; pin < 8; pin++) {
      uint32_t currentState = destBuffer[i] & masks[pin] ? 1 : 0;
      if (currentState != prevStates[pin]) {
        transitions[pin]++;
        prevStates[pin] = currentState;
      }
    }
  }

  // Print transition counts
  Serial.println("\n                  Pin       <----- Period ----->           Pin");
  Serial.println("     Pin      Transitions   Samples     Duration        Frequency");
  Serial.println("    --------------------------------------------------------------");

  for (int pin = 0; pin < 8; pin++) {
    Serial.print("     ");
    Serial.print(pinNames[pin]);
    Serial.printf("   \t%4d", transitions[pin]);
    if (transitions[pin] != 0) {
      Serial.printf("\t     ");
      Serial.print(Period_Float = (Transfer_Size_Float * 2) / transitions[pin]);
      Serial.printf("\t");
      Serial.print(Time_per_Period_Float = Time_per_Copy_Float * Period_Float);
      Serial.print(" ns\t");
      Serial.print(1000 / Time_per_Period_Float);
      Serial.println(" MHz");
    } else Serial.println("           -            -               -");
  }
#endif
}

// Show activity using LED_BUILTIN indicating that code hasn't crashed totally
void loop() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(100);
}
