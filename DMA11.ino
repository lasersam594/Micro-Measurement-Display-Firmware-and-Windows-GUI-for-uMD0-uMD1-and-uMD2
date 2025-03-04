/*********** Program to test high speed DMA from GPIO to memory *************/

#include <Arduino.h>
#include "DMAChannel.h"
 
#define BUFFER_SIZE 64     // Total buffer size for samples
#define TRANSFER_SIZE 32   // Transfer size - do half for this test
#define GPIO6 0x42004008   // Address of GPIO PSR

// DMA buffer
DMAMEM static uint32_t srcBuffer[BUFFER_SIZE] __attribute__((aligned(32)));
DMAMEM static uint32_t destBuffer[BUFFER_SIZE] __attribute__((aligned(32)));

// Completion flag
volatile bool transferComplete = false;
int GPIO_Print = GPIO6; // Only for the print info

// DMA channel
DMAChannel dma;

uint32_t Start_Time_Stamp = 0; // Measure time spent in DMA
uint32_t End_Time_Stamp = 0;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize serial
  Serial.begin(115200);
  delay(3000);

  // Print info
  Serial.println("\n******** Direct DMA Test *******\n");
// Serial.printf("GPIO6 Address: 0X%X \n", GPIO_Print, HEX);
  Serial.printf("Transfer %d ", TRANSFER_SIZE);
  Serial.printf("words from source buffer to %d word destination buffer\n\n", BUFFER_SIZE);
  Serial.println("Source data: Random Hexidecimal numbers; Destination initially filled with 0xDEADFEEDs");

  // Load srcBuffer with random numbers
  for (int i = 0; i < BUFFER_SIZE; i++) {
    srcBuffer[i] = rand() & 0xffffffff; // Generate constant length random number
  }

  // Load destBuffer with 0xDEADFEED
  for (int i = 0; i < BUFFER_SIZE; i++) {
    destBuffer[i] = 0xDEADFEED;
  }

  // Initialize DMA
  dma.begin(true);
  
/*
  // DMA source 
  dma.TCD->SADDR = (void*)GPIO6;      // Source address GPIO6_PSR
  dma.TCD->ATTR_SRC = 2;              // 32-bit source size
  dma.TCD->NBYTES = 4;                // 4 bytes per transfer
  dma.TCD->SLAST = 0;                 // Don't adjust source address at end of major loop
  dma.transferCount(TRANSFER_SIZE / 2); // These don't appear to be needed
  dma.transferSize(TRANSFER_SIZE / 2);  
*/

  dma.sourceBuffer(srcBuffer, BUFFER_SIZE);                   // BUFFER_SIZE doesn't appear to make a difference
  dma.destinationBuffer(destBuffer, TRANSFER_SIZE * 4);       // Must be 4x?
  dma.disableOnCompletion();
  dma.attachInterrupt(dma_complete_isr);
  dma.interruptAtCompletion();

//  dma.TCD->SOFF = 0;                  // Don't increment source address - Adding this works

  // Setup trigger
  dma.triggerContinuously();

  // Flush cache - needed to not keep copying old stuff
  arm_dcache_flush_delete(srcBuffer, sizeof(srcBuffer));
  arm_dcache_flush_delete(destBuffer, sizeof(destBuffer));

  // Start DMA
  Serial.print("\nStarting DMA transfer\n");

  transferComplete = false;

  dma.enable();
  Start_Time_Stamp = micros();
  
  // Wait for completion or timeout
  unsigned long start = micros();
  while (!transferComplete && (micros() - start < 1000)) {};

//  while (!transferComplete) {}; // Wait for DMA to complete, no timeout
  End_Time_Stamp = micros();
  Serial.printf("Elapsed Time: %d Microseconds", End_Time_Stamp - Start_Time_Stamp);

  // Print data
  Serial.println("\n\nData:\n");

  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.printf("%02d",i);
    Serial.printf(": Src 0x%08X", srcBuffer[i]);
    Serial.printf("; Dst 0x%08X \n", destBuffer[i]);
    }
}

// ISR for DMA completion
void dma_complete_isr(void) {
  transferComplete = true;
  dma.clearInterrupt();
}

// Show activity using LED_BUILTIN indicating that code hasn't crashed totally
void loop() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(100);
}