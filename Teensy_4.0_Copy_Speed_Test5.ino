//********************************************************************************************************//
//     Teensy 4.0 program to benchmark the speed of transfers from memory and GPIO to memory for µMD2.    //
//            This version uses both c and assembly code, each instance believed to be optimal.           //
//                Copyright® Jan Beck and Sam Goldwasser, 1994-2025, all rights reserved.                 //
//      The code may be freely used or distributed for non-commercial (mostly) educational purposes.      // 
//********************************************************************************************************//

#include <Arduino.h>

#define Print_Verbose 0    // If 1, print verbose raw buffer data
#define VERBOSE_SIZE 256   // Number of values to print, but see comments on skipping
#define Print_Analysis 1   // If 1, print transition counts, waveform periods (samples and duration), and pin frequencies

// GPIO pins to monitor.  Default uses GPIO7 (and GPIO2) have REFI, MEAS1, MEAS1I, MEAS2I, and MEAS3I.
//  They are the same except for speed; GPO6 (and GPO1) have REF and MEAS3.
#define REF_Pin       0   // REF     GPIO6 bit 03 0x8
#define REFI_Pin      6   // REFI    GPIO7 bit 10 0x400
#define MEAS1_Pin     9   // MEAS1   GPIO7 bit 11 0x800
#define MEAS1I_Pin    8   // MEAS1I  GPIO7 bit 16 0x10000
#define MEAS2_Pin    10   // MEAS2   GPIO7 bit 00 0x1
#define MEAS2I_Pin   11   // MEAS2I  GPIO7 bit 02 0x4
#define MEAS3_Pin    14   // MEAS3   GPIO6 bit 18 0x40000
#define MEAS3I_Pin   12   // MEAS3I  GPIO7 bit 01 0x2

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

#define BUFFER_SIZE 10000   // Total buffer size in 32 bit words for samples
#define TRANSFER_SIZE 10000 // Transfer size in 32 bit words

uint32_t TRANSFER_COUNT = TRANSFER_SIZE;

// Memory buffers.  What does DMAMEM do?
static unsigned int srcBuffer[BUFFER_SIZE] __attribute__((aligned(32)));  // Source buffer for memory to memory copy ONLY
static unsigned int destBuffer[BUFFER_SIZE] __attribute__((aligned(32))); // Destination buffer

// Addressses for asm
unsigned int *src_base = srcBuffer;       // Fake data buffer
unsigned int *srcBuffer_ptr;
unsigned int  gpio_base = 0x401BC008;     // GPIO port
unsigned int* gpio_base_ptr;

uint32_t Start_Time = 0;    // Measure time spent in transfer (us)
uint32_t End_Time = 0;
uint32_t Elapsed_Time = 0;  // End Time - Start Time (us)
float Period = 0;           // Average number of transfers between edges
float Time_per_Copy = 0;    // (ns)
float Time_per_Period = 0;  // (ns)
float Frequency = 0;        // (MHz)

//void Waveform_Capture_Assembly_Block1(unsigned int* src_base, unsigned int* src_increment) __attribute__((optimize("-O0")));
void Waveform_Capture_Assembly_Block1(unsigned int* src_base) __attribute__((optimize("-O0")));

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
  Serial.print("\n\n\n\n\n\n\n******** Teensy 4.0 Memory and GPIO Copy 10,000 (32 bit) Word Speed Test *******\n");

/***** Memory to memory copy of FAKE DATA using c code *****/

  Serial.printf("\n  Memory to memory copy of FAKE DATA using c code:");

  // Initialize buffers with FAKE DATA (squarewave) and constant 0xDEADFEEDs
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if ((i & 0X20) == 0x20) srcBuffer[i] = REF_MASK;     
    else srcBuffer[i] = 0;
  }

  for (int i = 0; i < BUFFER_SIZE; i++) destBuffer[i] = 0xDEADFEED;

  // Flush caches - needed to not keep copying old stuff
  arm_dcache_flush_delete(srcBuffer, sizeof(srcBuffer));
  arm_dcache_flush_delete(destBuffer, sizeof(destBuffer));

  Start_Time = micros();
  for (int i = 0; i < BUFFER_SIZE; i++) {
    destBuffer[i] = srcBuffer[i];
  }
  End_Time = micros();
  analyzeCapture();

/***** GPIO2_DR to memory copy using c code *****/

  Serial.printf("\n  GPIO2_DR to memory copy using c code:");

  Start_Time = micros();
   for (int i = 0; i < BUFFER_SIZE; i++) {
      destBuffer[i] = GPIO2_DR;
  }
  End_Time = micros();  
  analyzeCapture();

/***** GPIO7_DR to memory copy using c code *****/

  Serial.printf("\n  GPIO7_DR to memory copy using c code:");

  Start_Time = micros();
   for (int i = 0; i < BUFFER_SIZE; i++) {
      destBuffer[i] = GPIO7_DR;
  }
  End_Time = micros();  
  analyzeCapture();

/***** Memory to memory copy of FAKE DATA using assembly code *****/

  Serial.printf("\n  Memory to memory copy of FAKE DATA using assembly code:");

  // Initialize buffers with FAKE DATA (squarewave) and constant 0xDEADFEEDs
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if ((i & 0X40) == 0x40) srcBuffer[i] = MEAS3I_MASK;     
    else srcBuffer[i] = 0;
  }

  for (int i = 0; i < BUFFER_SIZE; i++) destBuffer[i] = 0xDEADFEED;

  srcBuffer_ptr = srcBuffer;
  Start_Time = micros();
  Waveform_Capture_Assembly_Block1(srcBuffer_ptr);
  End_Time = micros();
  analyzeCapture();

/***** GPIO2_DR to memory copy using assembly code *****/

  Serial.printf("\n  GPIO2_DR to memory copy using assembly code:");

  gpio_base = 0x401BC008;
  gpio_base_ptr = (unsigned int*)gpio_base;  // Cast integer to pointer
  Start_Time = micros();
  Waveform_Capture_Assembly_Block1(gpio_base_ptr);
  End_Time = micros();
  analyzeCapture();

  /***** GPIO7_DR to memory copy using assembly code *****/

  Serial.printf("\n  GPIO7_DR to memory copy using assembly code:");

  Start_Time = micros();
  gpio_base = 0x42004008;
  gpio_base_ptr = (unsigned int*)gpio_base;  // Cast integer to pointer
  Waveform_Capture_Assembly_Block1(gpio_base_ptr);
  End_Time = micros();
  analyzeCapture();

/***** End *****/

  Serial.println("\nTest complete.\n\n");
}

// Analysis of the captured data edge count, period in edges, and frequency in MHz
void analyzeCapture() {

// Display transfer time, time/copy, and copy frequency
  Elapsed_Time = End_Time - Start_Time;
  Serial.printf("\n    Elapsed Time: %d Microseconds.", Elapsed_Time);
  Time_per_Copy = Elapsed_Time;
  Time_per_Copy /= (TRANSFER_SIZE / 1000);
  Serial.printf("  Time per Copy: ");
  Serial.print(Time_per_Copy);
  Serial.printf(" ns.  Transfer Rate: ");
  Serial.print(1000 / Time_per_Copy);
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
      Serial.println("  ... (printing every 32nd sample) ...");
    }
  }  
#endif

// Display transition counts, sample period and duration, pin frequency
#if Print_Analysis == 1
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
  Serial.print("  \n                 Pin       <---- Period ---->       Pin");
  Serial.println("\n    Pin      Transitions   Samples   Duration    Frequency");
  Serial.println("   --------------------------------------------------------");
  
  for (int pin = 0; pin < 8; pin++) {
    Serial.print("    ");
    Serial.print(pinNames[pin]);
    Serial.printf("   \t%4d", transitions[pin]);
    if (transitions[pin] != 0) {
      Serial.printf("        ");
      Period = 2 * TRANSFER_SIZE;
      Serial.print(Period /= transitions[pin]);
      Serial.printf("   ");
      Serial.print(Time_per_Period = Period * Time_per_Copy);
      Serial.print(" ns    ");
      Serial.print(1000 / Time_per_Period);    
      Serial.println(" MHz");
    }
    else Serial.println("          -         -           -");
  }  
#endif
}

void loop() {
  // Show activity using LED_BUILTIN indicating that code hasn't totally crashed
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(100);
}

void Waveform_Capture_Assembly_Block1 (unsigned int* base) {

  asm volatile("ldr    r8, [sp,#4]     \n\t" // Load address of source into r8
               "mov    r9, %1          \n\t" // Copy address of array into r9, index = 0
               "mov    r10, r9         \n\t" // Copy r9 to r10 to start on end-of-loop-condition
               "ldr    r12, %0         \n\t" // Load Loop_Count into r12
               "lsls   r12, r12, #2    \n\t" // Shift left by 2 => multiply by 4
               "subs   r12, r12, #4    \n\t" // Subtract 4 to reduce count by 1 since the loop does count+1
               "add    r10, r10, r12   \n\t" // Add it to r10 used for loop limit
               "tst r8, 0x40000000     \n\t" // Test high order bit for GPIO address
               "beq nextdata2          \n\t" // Use increment for buffer copy

    "nextdata1:                        \n\t" // GPIO capture
               "ldr    r3, [r8]        \n\t" // Load value of GPIO_DR into r3
               "str    r3, [r9], #4    \n\t" // Store value into gpioDataArray and then add 4 bytes to the index
               "cmp    r9, r10         \n\t" // Check loop counter against loop limit
               "ble    nextdata1       \n\t" // Loop if limit not reached
               "b      done            \n\t" // End

   "nextdata2:                         \n\t" // Buffer copy
               "ldr    r3, [r8], #4    \n\t" // Load value of src buffer into r3 and then add 4 bytes to the index
               "str    r3, [r9], #4    \n\t" // Store value into gpioDataArray and then add 4 bytes to the index
               "cmp    r9, r10         \n\t" // Check loop counter against loop limit
               "ble    nextdata2       \n\t" // Loop if limit not reached
   
   "done:                              \n\t" // Buffer copy
              "nop                     \n\t" //

               : "=m" (TRANSFER_COUNT)          // Output operand list
               : "r" (destBuffer)               // Input operand list
               : "r3", "r8", "r9", "r10", "r12" // Clobber list
              );
}
