//********************************************************************************************************//
//           Teensy 4.0 program to demonstrate DMA transfers from one memory buffer to another.           //
//                Copyright® Jan Beck and Sam Goldwasser, 1994-2025, all rights reserved.                 //
//      The code may be freely used or distributed for non-commercial (mostly) educational purposes.      // 
//      Known issue: The time spent in DMA appears to be >15 times longer than should be possible.        //                                                                                                         //
//********************************************************************************************************//

#include <Arduino.h>
#include "DMAChannel.h"
 
#define BUFFER_SIZE 64     // Total buffer size for samples
#define TRANSFER_SIZE 32   // Transfer size - do half for this test

// DMA buffer
DMAMEM static uint32_t srcBuffer[BUFFER_SIZE] __attribute__((aligned(32)));  // Source buffer
DMAMEM static uint32_t destBuffer[BUFFER_SIZE] __attribute__((aligned(32))); // Destination buffer

// Completion flag
volatile bool transferComplete = false; // This gets set by the DMA complete interrupt service routine

// DMA channel
DMAChannel dma;

// Interrupt Service Routine (ISR) for DMA completion
void dma_complete_isr(void) {
  transferComplete = true;
  dma.clearInterrupt();
}

uint32_t Start_Time_Stamp = 0; // Measure time spent in DMA.  See note above.
uint32_t End_Time_Stamp = 0;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize serial.
  Serial.begin(115200);
//  delay(3000);                //  Optional pause to allow time to start Serial Monitor (Ctrl-Shift-M) in another window
  Serial.println("\n******** Direct Teensy 4.0 DMA Test *******\n");

  // Print info
  Serial.printf("Transfer %d ", TRANSFER_SIZE);
  Serial.printf("words from source buffer to %d word destination buffer\n\n", BUFFER_SIZE);
  Serial.println("Source data: Hexidecimal random numbers; Destination initially filled with 0xDEADFEEDs");

  // Load srcBuffer with random numbers
  for (int i = 0; i < BUFFER_SIZE; i++) {
    srcBuffer[i] = rand() & 0xffffffff; // Generate constant length random number
  }

  // Load destBuffer with 0xDEADFEED to know if it has been overwritten
  for (int i = 0; i < BUFFER_SIZE; i++) {
    destBuffer[i] = 0xDEADFEED;
  }

  // Initialize DMA
  dma.begin(true);

  dma.sourceBuffer(srcBuffer, BUFFER_SIZE);                 // BUFFER_SIZE doesn't appear to make a difference
// dma.TCD->SOFF = 0;                                       // Don't increment source address to transfer from a single address
  dma.destinationBuffer(destBuffer, TRANSFER_SIZE * 4);     // Must be 4x?
  dma.disableOnCompletion();
  dma.attachInterrupt(dma_complete_isr);                    // Set up termination interrupt
  dma.interruptAtCompletion();
  dma.triggerContinuously();                                // Set up trigger


  // Flush caches - needed to not keep copying old stuff
  arm_dcache_flush_delete(destBuffer, sizeof(destBuffer));
  arm_dcache_flush_delete(srcBuffer, sizeof(srcBuffer));

  // Start DMA
  Serial.print("\nStarting DMA capture\n");
  transferComplete = false;
  dma.enable();
  Start_Time_Stamp = micros();
  while (!transferComplete) {}; // Wait for DMA to complete
  End_Time_Stamp = micros();
  Serial.printf("Elapsed Time: %d Microseconds", End_Time_Stamp - Start_Time_Stamp);

  // Print data
  Serial.println("\n\nData:\n");

  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.printf("%02d",i);
    Serial.printf(": Src 0x%08X", srcBuffer[i]);
    Serial.printf("; Dest 0x%08X \n", destBuffer[i]);
    }

 // Print completion
  Serial.printf("\nTransferred %d Words\n", TRANSFER_SIZE);
}

void loop() {
  // Show activity using LED_BUILTIN indicating that code hasn't totally crashed
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(100);
}