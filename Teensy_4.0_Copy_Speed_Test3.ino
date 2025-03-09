//********************************************************************************************************//
//          Teensy 4.0 program to demonstrate speed of transfers from memory and GPIO to memory.         //
//                Copyright® Jan Beck and Sam Goldwasser, 1994-2025, all rights reserved.                 //
//      The code may be freely used or distributed for non-commercial (mostly) educational purposes.      // 
//********************************************************************************************************//

#include <Arduino.h>

#define Print_Verbose 0   // Print verbose buffer data if 1
#define VERBOSE_SIZE 1024  // # Values to print but see comments

// GPIO7 pins to monitor
#define REF_Pin      0    // REF
#define REFI_Pin     6    // REFI  
#define MEAS1_Pin    9    // MEAS1  
#define MEAS1I_Pin   8    // MEAS1I  
#define MEAS2_Pin    10   // MEAS2  
#define MEAS2I_Pin   11   // MEAS2I  
#define MEAS3_Pin    14   // MEAS3  
#define MEAS3I_Pin   12   // MEAS3I

// Create bitmasks for each pin
const uint32_t REF_MASK = digitalPinToBitMask(REFI_Pin);
const uint32_t REFI_MASK = digitalPinToBitMask(REFI_Pin);
const uint32_t MEAS1_MASK = digitalPinToBitMask(MEAS1_Pin);
const uint32_t MEAS1I_MASK = digitalPinToBitMask(MEAS1I_Pin);
const uint32_t MEAS2_MASK = digitalPinToBitMask(MEAS2_Pin);
const uint32_t MEAS2I_MASK = digitalPinToBitMask(MEAS2I_Pin);
const uint32_t MEAS3_MASK = digitalPinToBitMask(MEAS3_Pin);
const uint32_t MEAS3I_MASK = digitalPinToBitMask(MEAS3I_Pin);

const uint32_t ALL_PINS_MASK = REF_MASK | REFI_MASK | MEAS1_MASK | MEAS1I_MASK | MEAS2_MASK | MEAS2I_MASK | MEAS3_MASK | MEAS3I_MASK;

#define BUFFER_SIZE 10000   // Total buffer size in 32 bit words for samples
#define TRANSFER_SIZE 10000 // Transfer size in 32 bit words

// Memory buffers
DMAMEM static uint32_t srcBuffer[BUFFER_SIZE] __attribute__((aligned(32)));  // Source buffer for memory to memory copy ONLY
DMAMEM static uint32_t destBuffer[BUFFER_SIZE] __attribute__((aligned(32))); // Destination buffer

uint32_t Start_Time = 0;    // Measure time in microseonds spent in transfer
uint32_t End_Time = 0;
uint32_t Elapsed_Time = 0;  // End Time - Start Time
uint32_t Period = 0;        // Number of transfers between edges
float Time_per_Copy = 0;    // Nanoseconds
float Time_per_Period = 0;  // Nanoseconds
float Frequency = 0;        // MHz

void setup() {

  // Configure all data pins as inputs
  pinMode(REF_Pin, INPUT);
  pinMode(REFI_Pin, INPUT);
  pinMode(MEAS1_Pin, INPUT);
  pinMode(MEAS1I_Pin, INPUT);
  pinMode(MEAS2_Pin, INPUT);
  pinMode(MEAS2I_Pin, INPUT);
  pinMode(MEAS3_Pin, INPUT);
  pinMode(MEAS3I_Pin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize serial.
  Serial.begin(115200);
  Serial.println("\n\n\n\n\n\n\n******** Teensy 4.0 Memory and GPIO Copy 10,000 Word Speed Test *******\n");

/***** Memory to memory copy using FAKE DATA acquired from GPIO7 *****/

  Serial.printf("  Memory to Memory Copy using FAKE DATA acquired from GPIO7:");

  for (int i = 0; i < BUFFER_SIZE; i++) {
      srcBuffer[i] = GPIO7_DR;
  }

  // Flush caches - needed to not keep copying old stuff
  arm_dcache_flush_delete(srcBuffer, sizeof(srcBuffer));
  arm_dcache_flush_delete(destBuffer, sizeof(destBuffer));

  // Start transfer

  Start_Time = micros();
 
  for (int i = 0; i < BUFFER_SIZE; i++) {
    destBuffer[i] = srcBuffer[i];
  }

  End_Time = micros();

  Elapsed_Time = End_Time - Start_Time;
  Serial.printf("\n    Elapsed Time: %d Microseconds.", Elapsed_Time);
  Time_per_Copy = Elapsed_Time;
  Time_per_Copy /= (TRANSFER_SIZE / 1000);
  Serial.printf("  Time per Copy = ");
  Serial.print(Time_per_Copy);
  Serial.printf(" ns.\n");

  analyzeCapture();

/***** GPIO1_DR to Memory Copy *****/

  Serial.printf("\n  GPIO1_DR to Memory Copy:");

  // Start transfer

  Start_Time = micros();
 
   for (int i = 0; i < BUFFER_SIZE; i++) {
      destBuffer[i] = GPIO2_DR;
  }

  End_Time = micros();

  Elapsed_Time = End_Time - Start_Time;
  Serial.printf("\n    Elapsed Time: %d Microseconds.", Elapsed_Time);
  Time_per_Copy = Elapsed_Time;
  Time_per_Copy /= (TRANSFER_SIZE / 1000);
  Serial.printf("  Time per Copy = ");
  Serial.print(Time_per_Copy);
  Serial.printf(" ns.\n");

  analyzeCapture();

/***** GPIO7_DR to Memory Copy *****/

  Serial.printf("\n  GPIO7_DR to Memory:");

  // Start transfer

  Start_Time = micros();
 
   for (int i = 0; i < BUFFER_SIZE; i++) {
      destBuffer[i] = GPIO7_DR;
  }

  End_Time = micros();

  Elapsed_Time = End_Time - Start_Time;
  Serial.printf("\n    Elapsed Time: %d Microseconds.", Elapsed_Time);
  Time_per_Copy = Elapsed_Time;
  Time_per_Copy /= (TRANSFER_SIZE / 1000);
  Serial.printf("  Time per Copy = ");
  Serial.print(Time_per_Copy);
  Serial.printf(" ns.\n");

  analyzeCapture();

   // Process and display the data
    Serial.println("\nTest complete.\n\n");
}

// Analysis of the captured data edge count, period in edges, and frequency in MHz
void analyzeCapture() {

// Display the captured data in a readable format
#if Print_Verbose == 1
  //  Serial.printf("GPIO7 Data Copied:\n\n");
  Serial.println("\n   Sample  RAW HEX   REF   REFI   MEAS1   MEAS1I   MEAS2   MEAS2I   MEAS3   MEAS3I");
  Serial.println("  ---------------------------------------------------------------------------------");
  
   // Print each sample with pin values
   for (uint32_t i = 0; i < VERBOSE_SIZE; i++) {
   // Only print first 32 samples, then every 32nd sample to avoid flooding serial
     if (i < 32 || i % 32 == 0) {
       uint32_t value = destBuffer[i];
       
       // Print sample number and raw hex value
       Serial.printf("    %4d", i);
       Serial.print("   0x");
       Serial.print(value, HEX);
       
        // Extract and print individual pin values
        Serial.print("   ");
        Serial.print((value & REF_MASK) ?  "1" : "0");
        Serial.print("     ");
        Serial.print((value & REFI_MASK) ?  "1" : "0");
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
        Serial.println("... (printing every 32nd sample) ...");
     }
    }  
 #endif

  // Data analysis
  uint32_t transitions[8] = {0, 0, 0, 0, 0, 0};
  uint32_t masks[8] = {REF_MASK, REFI_MASK, MEAS1_MASK, MEAS1I_MASK, MEAS2_MASK, MEAS2I_MASK, MEAS3_MASK, MEAS3I_MASK};
  const char* pinNames[8] = {"REF", "REFI", "MEAS1", "MEAS1I", "MEAS2", "MEAS2I", "MEAS3", "MEAS3I"};
  
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
  Serial.println("\n    Pin      Transitions  Samples/Period  Frequency");
  Serial.println("   ------------------------------------------------------");
  
  for (int pin = 0; pin < 8; pin++) {
    Serial.print("    ");
    Serial.print(pinNames[pin]);
    Serial.printf("   \t%4d", transitions[pin]);
    if (transitions[pin] != 0) {
      Period = 2 * TRANSFER_SIZE / transitions[pin];
      Serial.printf("          %4d        ", Period);
      Time_per_Period = Period * Time_per_Copy;
      Frequency = 1000 / Time_per_Period;
      Serial.print(Frequency);    
      Serial.println(" MHz");
    }
    else Serial.println("            NA           NA");
  }  
}

void loop() {
  // Show activity using LED_BUILTIN indicating that code hasn't totally crashed
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(100);
}