#define FirmwareVersion 5704

// #define UseSensors 1 // Uncomment this line for sensor support

#define REF_Sync 1      // Enable synchronization of MEAS capture to REF edge for interpolation.

// Written by Jan Beck and heavily modified by Sam Goldwasser.  This code is in the public domain. Do with it what you like.

// The design of this system is based on 5 free-running counters.
// The first counter (1) (using the Timer1 hardware) is on the internal CPU clock and provides the sampling interrupt on overflow.
// The second counter (2) (using the Timer3 hardware) is on REF (laser reference signal).
// The third  counter (3) (using the Timer5 hardware) is on MEAS1 (axis 1 interferometer signal).
// The forth  counter (4) (using the Timer4 hardware) is on MEAS2 (optional axis 2 interferometer signal).
// The fifth  counter (5) (using the Timer2 hardware) is on MEAS3 (optional axis 3 interferometer signal).

// This code is set up to capture sample buffers with an initial REF sample synchronized to REF edge and then 32 consecutive MEAS samples.
// (The last 4 MEAS samples occur slightly delayed due to lack of CPU registers but are better than nothing to get below
// a laser REF frequency of 1.5 MHz with the CPU clock at 40 MHz; close to 1 MHz with the CPU clock set to 30 MHz.)
// The REF count is only saved once along with MEAS(s) to acquire the reference REF and MEAS counter values.

// It can select any of the REF/MEAS sample buffers to use for the phase estimation and will do averaging.
// Locates REF change to start capture, so the REF sample prior to change is not stored.

// It then captures up to 31 unsynchronized sequences of 30 PORTBs (which include the REF and MEAS1/2/3 clock input
// bits and uses these for averaging.  

// The interpolation code should work with any in-spec common HP/Agilent with REF from below 1.5 MHz to above 4.0 MHz.

// V56.xx supports up to 3 axes of REF/MEAS and PORTB acquisition and interpolation with optimized inner loops.
// V56.04 includes hooks for digital sensors with conditional sensor support - UseSsnsors must be defined
// V56.05 added automatic CPU clock frequency opimization.
// V56.06 optimizes searches using coarse-fine algorithm.
// V56.08 cleaned up averaging code.
// V56.09 cleaned up these comments. :)
// V56.10 added sensors but only acquired once after reset.
// V57.01 fixed parameters for axes 2 and 3.
// V57.04 added ability to disable REF sync if not using interpolation.

#include <plib.h>
#include "p32xxxx.h"

#ifdef UseSensors
  #include <Wire.h>
  #include <Adafruit_BMP085.h>
  #include "dht.h"

  Adafruit_BMP085 bmp;  // instantiate object to read pressure/temperature sensor
  dht DHT;              // instantiate humidity/temperature sensor object  
#endif

bool newDataAvailable = false; // indicator of whether to send another data packet out on the serial bus; semaphore....
bool bmpPresent = true;       // indicator of whether BMP sensor is present.
bool dhtPresent = true;       // indicator of whether AM2302 sensor is present.

// display values won't be written to unless newDataAvailable is false; used to publish information from interrupt routine to USB send routine

struct dataPoint                  // stucture for one data packet; The FIFO queue is a queue of these structs
  {
    INT64 displayREFFrequencyCount;       // REF count in 1/610s
    INT64 displayMEAS1FrequencyCount;     // MEAS1 count in 1/610s
    INT64 displayTotalDifference1;        // Displacement1
    INT64 displayVelocity1Count;          // Velocity1 count in 1/610s
    INT64 displayPhase1;                  // Phase1 location of MEAS edge relative to REF1(n)-REF1(n-1)
    INT64 displaySequenceNumber;          // Sequence number
    INT64 displayLowSpeedCode;            // Select type of low speed data
    INT64 displayLowSpeedData;            // Low speed data

    INT64 displayMEAS2FrequencyCount;     // MEAS2 count in 1/610s
    INT64 displayTotalDifference2;        // Displacement2
    INT64 displayVelocity2Count;          // Velocity2 count in 1/610s
    INT64 displayPhase2;                  // Phase2 location of MEAS edge relative to REF2(n)-REF2(n-1)
    INT64 displayMEAS3FrequencyCount;     // MEAS3count3 in 1/610s
    INT64 displayTotalDifference3;        // Displacement3
    INT64 displayVelocity3Count;          // Velocity3 count in 1/610s
    INT64 displayPhase3;                  // Phase location of MEAS3 edge relative to REF3(n)-REF3(n-1)
  };

#define FIFO_SIZE 25 // size of circular buffer

struct dataPoint dataPoint[FIFO_SIZE]; // circular buffer of data to be sent to USB

int head = 0;  // beginning of circular buffer; empty when both head and tail are the same value
int tail = 0;  // end of circular buffer; full when tail is one slot before head we are full

static char buffer[300];    // make some space for the routine to convert numbers to text

// UINT16 FirmwareVersion = 5608;

// Loop counters

INT32 i = 1;
INT32 j = 1;
INT32 k = 1;
INT32 m = 1;
INT32 n = 1;

// Indexes to reduce computation time

INT32 kx40 = 0;
INT32 kx41 = 0;
INT32 apm1x40 = 0;
INT32 apm4 = 0;
INT32 ipkx40papm1x40 = 0;
INT32 ipkx40papm1x40m1 = 0;
INT32 kx40papm1x40 = 0;
INT32 ipkx40 = 0;
INT32 ipkx40m1 = 0;
    
// REF and MEAS counter overflow correction 16 bit to 64 bit variables
static INT64 Counter2_OverflowCounter[3]; // how often has the 16 bit counter (2) overflown
static INT64 Counter3_OverflowCounter[3]; // how often has the 16 bit counter (3) overflown
static INT64 TotalCounter2[3];            // this is the total count for (2)
static INT64 TotalCounter3[3];
static INT64 PreviousTotalCounter2[3];
static INT64 PreviousTotalCounter3[3];
static INT64 TotalDifference[3];          // difference between the two totals. This is the measured distance
static INT64 PreviousTotalDifference[3];  // previous difference stored. This is the measured velocity
static UINT16 CounterValue2[3];           // current value of 16 bit counter (2)/(3)  
static UINT16 CounterValue3[3];           // current value of 16 bit counter (2)/(3)
static UINT16 PreviousCounterValue2[3];   // previous value of 16 bit counter (2)/(3) for overflow check
static UINT16 PreviousCounterValue3[3];   // previous value of 16 bit counter (2)/(3) for overflow check
static INT32 REFFrequencyCount[3];        // Number of REF clocks in 1/610.35 s
static INT32 MEASFrequencyCount[3];       // Number of MEAS clocks in 1/610.35 s
static INT64 tempTotalCounter = 0;        // temporary variable to handle the overflow checks of (2) and (3)    

static INT32 ActiveAxes = 1;              // Flag word: Bit 0: Axis 1, bit 1: Axis 2, bit 2: Axis 3.  Default Axis 1
static INT32 NumberOfAxes = 1;

static INT32 REFFrequency = 0;

static INT32 temperature1 = -1;
static INT32 temperature2 = -1;
static INT32 pressure = -1;
static INT32 humidity = -1;
static INT32 sensorDelayCounter = 0;

static INT32 Timer1PRCCounts = 0;
static INT32 Timer1USBCounts = 0;

void __attribute__((interrupt)) handlerForTimer1Ints()
{ // microchip timer reference http://ww1.microchip.com/downloads/en/DeviceDoc/61105E.pdf
    static INT32 SequenceNumber = 0; // serial number for data packets and allows detection of missed data
    INT64 bufferOverflowCounter = 0; // check up on the FIFO to gauge USB quality
 
    INTDisableInterrupts();
    
    static INT32 CV23[(32*40)+16]; // 1296 word array for temporary storage in assembly block defined as follows:
     // 32 sample REF/MEAS acquisitions with space for 8 additional values for each accessible by the assembly block.
     // These are used in the first pass to acquire the REF/MEAS values but averaging may also be performed.  Index by "m"
     // from 0 to (TotalMEASAverages-1).  TotalMEASAverages = NumberOf MEAS1Averages + NumberOfMEASA2verages + NumberOfMEAS3Averages.
     // CV23[0 + m*40] Base address for raw CounterValue3 (MEAS) values
     // CV23[32 + m*40] Address to save CounterValue2 (REF) during first pass

     // 32 sample PORTB averages with space for 8 additional values for each accessible by the assembly block.
     // These are used in the second averaging pass to acquire PORTB values.
     // NumberOfPORTBAverages must be > 0 and <= 31  Index by "n", n from 0 to 30.
     // Base address for PORTB (REF,MEAS1,MEAS2,MEAS3) values follows REF/MEAS buffers
     // CV23[TotalMEASAverages*40 + n*40].
    
     // Definitions of words used within the 8 word blocks.  The index "p" used for REF/MEAS or PORTB:
     // CV23[39 + p*40] AverageMEASCount: Counter variable for all MEAS acquisition
     // CV23[38 + p*40] AveragePORTBCount: Counter variable for PORTB acquisition
     // CV23[37 + p*40] Spare 1
     // CV23[36 + p*40] Spare 2
     // CV23[35 + p*40] AddressOffset: Address Offset value used in assembly block
     // CV23[34 + p*40] TimerOffset: Timer bus address offset relative to 0xbf800000 for REF/MEAS captures
     // CV23[32 + p*40] Save location for REF reference for MEAS1, MEAS2, MEAS3 capture
     
     // 16 word block at end for general variables that may need to be accessible by the assembly code.
     // CV23[SPSaveOffset] (1295): stack pointer save.  DO NOT TOUCH!
     // CV23[Timer2PrefetchOffset] (1291): Timer2 prefetch (for more stable REF/MEAS display (future if needed)
     // CV23[Timer3PrefetchOffset] (1292): Timer3 prefetch (for more stable REF/MEAS display (future if needed)
     // CV23[Timer4PrefetchOffset] (1293): Timer4 prefetch (for more stable REF/MEAS display (future if needed)
     // CV23[Timer5PrefetchOffset] (1294): Timer5 prefetch (for more stable REF/MEAS display (future if needed)ddressOffset: Address Offset value to use during assembly capture

    static INT32 CV2[3][32]; // Array for CounterValue2
    static INT32 CV3[3][32]; // Array for CounterValue3
    
    #define SPSaveOffset 1295         // Stack pointer save DON'T MESS WITH THIS! :-)
 
    #define Timer2PrefetchOffset 1291 // These are for only computing the frequencies
    #define Timer3PrefetchOffset 1292
    #define Timer4PrefetchOffset 1293
    #define Timer5PrefetchOffset 1294
    
    #define AverageMEASCount  39
    #define AveragePORTBCount 38
    #define TimerOffset 37
    #define AddressOffset 35
    
    static INT32 Timer2Prefetch = 0;
    static INT32 Timer3Prefetch = 0;
    static INT32 Timer4Prefetch = 0;
    static INT32 Timer5Prefetch = 0;

    static INT32 PreviousTimer2Prefetch = 0;
    static INT32 PreviousTimer3Prefetch = 0;
    static INT32 PreviousTimer4Prefetch = 0;
    static INT32 PreviousTimer5Prefetch = 0;
    
 // Averaging values for REF/MEAS and PORTB.  Total must be less than or equal to 32.
    static INT32 NumberOfMEAS1Averages = 1;  // # REF/MEAS1 averages.  Valid range is 1-32.
    static INT32 NumberOfMEAS2Averages = 1;  // # REF/MEAS2 averages.  Valid range is 0-32. (0 only if no axis 2.)
    static INT32 NumberOfMEAS3Averages = 1;  // # REF/MEAS3 averages.  Valid range is 0-32. (0 only if no axis 3.)
    static INT32 TotalMEASAverages = 0;      // This will be filled in automatically.
    static INT32 NumberOfPORTBAverages = 20; // # PORTB averages.  Valid range is 0-31.  It may be better to be even.

    static INT32 SampleSelect = 0;           // Sample sub-buffer to use when not averaging

    static INT32 FirstREFChange[3][36];
    static INT32 SecondREFChange[3][36];
    static INT32 MEASChange[3][36];
    static INT32 REFToMEASBuffer[3][36];
    static INT32 REFPeriodxNOMA[3];
    static INT32 REFPeriodxNOPBA[3];
    static INT32 REFToMEASMAccum[3];
    static INT32 REFToMEASPBAccum[3];
    static INT32 REFToMEASMAverage[3];
    static INT32 REFToMEASPBAverage[3];
    static INT32 PhaseBufferAccum[3];
    static INT32 NoChange[3];
    static INT32 MEASPhase[3];
    static INT32 PORTBPhase[3];
    static INT32 REFPeriod = 0;
    static INT32 SampleFrequency = 610;
    static INT32 SampleFrequencyX100 = 61035;
    static INT32 CPUClock = 40000000;
    static INT32 ClkSkipCount = 2;
    
    static INT32 LowSpeedCode = 0;
    static INT32 LowSpeedData = 0;
    static INT32 LowSpeedCodeSelect = 0;
    
    static INT32 AxisParam[3][5];  // [Axis1,Axis2,Axis3][Size,CV23Start,TimerAddressOffset,NumberOfPORTBAverages,MEASMask]
   
    SequenceNumber++; // count how often the 16 bit timer driven by internal clock overflows to have a serial number for each data set

    // Set up low speed data/diagnostic values to send back  

    LowSpeedCode = 0; // No sensor data at present but used for diagnostics
    LowSpeedData = 0;
    
 //   digitalWrite(PIN_LED3, 0); // Turn on LED3 at start of axis capture and analysis

    // Load values for assembly code capture

    AxisParam[0][0] = NumberOfMEAS1Averages; // Size of MEAS1 capture block
    AxisParam[0][1] = 0;                     // Start of MEAS1 capture block
    AxisParam[0][2] = 3600;                  // MEAS1 timer Timer5
    AxisParam[0][3] = NumberOfPORTBAverages; // Number of Axis 1 PORTB averages
    AxisParam[0][4] = 0x1;                   // Axis 1 PORTB MEAS mask RPB0
    
    AxisParam[1][0] = NumberOfMEAS2Averages; // Size of MEAS2 capture block
    AxisParam[1][1] = NumberOfMEAS1Averages; // Start of MEAS2 capture block
    AxisParam[1][2] = 3088;                  // MEAS2 timer Timer4
    AxisParam[1][3] = NumberOfPORTBAverages; // Number of Axis 2 PORTB averages
    AxisParam[1][4] = 0x4;                   // Axis 2 PORTB MEAS mask RPB2
   
    AxisParam[2][0] = NumberOfMEAS3Averages; // Size of MEAS3 capture block
    AxisParam[2][1] = NumberOfMEAS1Averages + NumberOfMEAS2Averages; // Start of MEAS3 capture block
    AxisParam[2][2] = 2064;                  // MEAS3 timer Timer2
    AxisParam[2][3] = NumberOfPORTBAverages; // Number of Axis 3 PORTB averages
    AxisParam[2][4] = 0x8;                   // Axis 3 PORTB MEAS mask RPB3
  
    TotalMEASAverages = NumberOfMEAS1Averages + NumberOfMEAS2Averages + NumberOfMEAS3Averages;
    
    CV23[AverageMEASCount] = TotalMEASAverages;       // Initialize averaging REF/MEAS loop counter (39)
    CV23[AveragePORTBCount] = NumberOfPORTBAverages;  // Initialize averaging PORTB loop counter (38)
    CV23[AddressOffset] = 0;                          // Address offset to restore after all capture complete (35)

// Load CV23 TimerOffset for each MEAS sample set
    for (m = 0;m < 3;m++) // Axis: MEAS1=0, MEAS2=1, MEAS3=2
      {
        kx40papm1x40 = AxisParam[m][1]*40;
        for (k = 0;k < AxisParam[m][0];k++) // Number of samples sets for each axis
          {
//         CV23[34 + ((AxisParam[m][1] + k)*40)] = AxisParam[m][2]; // TimerOffset for each sample set
           CV23[34 + kx40papm1x40] = AxisParam[m][2]; // TimerOffset for each sample set
           kx40papm1x40 += 40;
          }
      }
   
  // Assembly code to read TotalMEASAverages sets of 1 REF and 30 MEAS samples as rapidly as possible and store in
  // array CV23[32 + 40*p] and CV23[0-29 + 40*p], incrementing p after each set, and then NumberOfPORTBAverages of 30 PORTB values
  // and store in CV23[0-29 + 40*p] and incrementing p after each set.
  
  // Save 28 registers to stack. (some extras but who cares, better safe.)
  
   __asm__ __volatile__(

     "addi $sp, $sp, -112 \n\t" // Decrement stack pointer by 112

     "sw  $3,  0*4($sp) \n\t"  // Save $3 to stack
     "sw  $4,  1*4($sp) \n\t"
     "sw  $5,  2*4($sp) \n\t"
     "sw  $6,  3*4($sp) \n\t"
     "sw  $7,  4*4($sp) \n\t"
     "sw  $8,  5*4($sp) \n\t"
     "sw  $9,  6*4($sp) \n\t"
     "sw $10,  7*4($sp) \n\t"
     "sw $11,  8*4($sp) \n\t"
     "sw $12,  9*4($sp) \n\t"
     "sw $13, 10*4($sp) \n\t"
     "sw $14, 11*4($sp) \n\t"
     "sw $15, 12*4($sp) \n\t"
     "sw $16, 13*4($sp) \n\t"
     "sw $17, 14*4($sp) \n\t"
     "sw $18, 15*4($sp) \n\t"
     "sw $19, 16*4($sp) \n\t"
     "sw $20, 17*4($sp) \n\t"
     "sw $21, 18*4($sp) \n\t"
     "sw $22, 19*4($sp) \n\t"
     "sw $23, 20*4($sp) \n\t"
     "sw $24, 21*4($sp) \n\t"
     "sw $25, 22*4($sp) \n\t"
     "sw $26, 23*4($sp) \n\t"
     "sw $27, 24*4($sp) \n\t"
     "sw $28, 25*4($sp) \n\t"
     "sw $30, 26*4($sp) \n\t"
     "sw $31, 27*4($sp) \n\t"   // Save $31 to stack
     
     "sw $29, 1295*4(%0) \n\t"  // Save sp stack pointer in safe place (CV23[1079])
 
   // Wait here until Timer1 >= 512 to conceal ISR entry delay
   // C code: while (ReadTimer1() < 512) {}

      "lui $a0, 0xbf80 \n\t"       // Base address for Timers
      "lw $s0, 1552($a0) \n\t"     // Read Timer1
      "slti $s0, $s0, 512 \n\t"    // Test if Timer1 < 512. $s1 -> 1 if true
      "bne $s0, $zero, .-12 \n\t"  // Branch if result is 1
      "nop \n\t"
 
   // REF and MEAS counter prefetch to reduce REF/MEAS display jitter (future if needed)
   
      "lw  $v0, 2064($a0) \n\t"    // Read Timer2 for frequency count
      "lw  $v1, 2576($a0) \n\t"    // Read Timer3 for frequency count
      "lw  $s0, 3088($a0) \n\t"    // Read Timer4 for frequency count
      "lw  $s1, 3600($a0) \n\t"    // Read Timer5 for frequency count

      "sw  $v0, 1291*4(%0) \n\t"   // Store Timer2 in CV23[1291]
      "sw  $v1, 1292*4(%0) \n\t"   // Store Timer3 in CV23[1292]
      "sw  $s0, 1293*4(%0) \n\t"   // Store Timer4 in CV23[1293]
      "sw  $s1, 1294*4(%0) \n\t"   // Store Timer5 in CV23[1294]
     
  // Start of first pass averaging capture REF (3600) and MEAS (2576)

  "AveragingLoop1: \n\t"
 
      "lui $s0, 0xbf80 \n\t"       // Base address for Timers, used for REF (Timer3)
      "lw $a0, 34*4(%0) \n\t"      // Fetch Timer Offset for MEASs from CV23[34]
      "add $a0, $s0, $a0 \n\t"     // Timer offset from 0xbf800000 for MEASs
 
  // Sync with change in REF (Timer3)
      "lw $v1, 2576($s0) \n\t" // Load REF
      "lw $v0, 2576($s0) \n\t" // Load REF

#ifndef REF_Sync
      "j REFEdgeFound1 \n\t"       // Skip REF sync if no interpolation
#endif

      "bne $v1, $v0, REFEdgeFound1 \n\t"  // End if not equal
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"   
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"
      "lw $v0, 2576($s0) \n\t"
      "bne $v1, $v0, REFEdgeFound1 \n\t"

   // Save Timer3 (REF) and read 30 Timer5s (MEASs) as quickly as possible

"REFEdgeFound1: \n\t"

     "sw  $v0, 32*4(%0) \n\t"  // save REF sample 0 into CV23[32] (same as sample -1)
     "lw  $v0, 0($a0) \n\t"    // read MEAS sample 0
     "lw  $v1, 0($a0) \n\t"    // read MEAS sample 1
     "lw  $a3, 0($a0) \n\t"    // read MEAS sample 2 
     "lw  $a2, 0($a0) \n\t"    // read MEAS sample 3
     "lw  $t0, 0($a0) \n\t"
     "lw  $t1, 0($a0) \n\t"
     "lw  $t2, 0($a0) \n\t"
     "lw  $t3, 0($a0) \n\t"
     "lw  $t4, 0($a0) \n\t"
     "lw  $t5, 0($a0) \n\t"
     "lw  $t6, 0($a0) \n\t"
     "lw  $t7, 0($a0) \n\t"
     "lw  $s0, 0($a0) \n\t"
     "lw  $s1, 0($a0) \n\t"
     "lw  $s2, 0($a0) \n\t"
     "lw  $s3, 0($a0) \n\t"
     "lw  $s4, 0($a0) \n\t"
     "lw  $s5, 0($a0) \n\t"
     "lw  $s6, 0($a0) \n\t"
     "lw  $s7, 0($a0) \n\t"
     "lw  $t8, 0($a0) \n\t"
     "lw  $t9, 0($a0) \n\t"
     "lw  $k0, 0($a0) \n\t"
     "lw  $k1, 0($a0) \n\t"
     "lw  $gp, 0($a0) \n\t"
     "lw  $sp, 0($a0) \n\t"
     "lw  $fp, 0($a0) \n\t"   // read MEAS sample 26
     "lw  $ra, 0($a0) \n\t"   // read MEAS sample 27

  // Save Timer3/5 values into CV23 (REF at offset 0; MEAS at offset 32) arrays

     "sw $v0, 0*4(%0) \n\t"   // save MEAS sample 0 into CV23[0]
     "lw $v0, 0($a0) \n\t"    // read MEAS sample 28
     "sw $v0, 28*4(%0) \n\t"  // save MEAS sample 28 into CV23[28]
     "lw $v0, 0($a0) \n\t"    // read MEAS sample 29
     "sw $v0, 29*4(%0) \n\t"  // save MEAS sample 29 into CV23[29]
     "lw $v0, 0($a0) \n\t"    // read MEAS sample 30
     "sw $v0, 30*4(%0) \n\t"  // save MEAS sample 30 into CV23[31]
     "lw $v0, 0($a0) \n\t"    // read MEAS sample 31
     "sw $v0, 31*4(%0) \n\t"  // save MEAS sample 31 into CV23[31]
     "sw $v1, 1*4(%0) \n\t"   // save MEAS sample 1 into CV23[1]
     "sw $a3, 2*4(%0) \n\t"   // save MEAS sample 2 into CV23[2]
     "sw $a2, 3*4(%0) \n\t"   // save MEAS sample 3 into CV23[3]
     "sw $t0, 4*4(%0) \n\t"
     "sw $t1, 5*4(%0) \n\t"
     "sw $t2, 6*4(%0) \n\t"
     "sw $t3, 7*4(%0) \n\t"
     "sw $t4, 8*4(%0) \n\t"
     "sw $t5, 9*4(%0) \n\t"
     "sw $t6, 10*4(%0) \n\t"
     "sw $t7, 11*4(%0) \n\t"
     "sw $s0, 12*4(%0) \n\t"
     "sw $s1, 13*4(%0) \n\t"
     "sw $s2, 14*4(%0) \n\t"
     "sw $s3, 15*4(%0) \n\t"
     "sw $s4, 16*4(%0) \n\t"
     "sw $s5, 17*4(%0) \n\t"
     "sw $s6, 18*4(%0) \n\t"
     "sw $s7, 19*4(%0) \n\t"
     "sw $t8, 20*4(%0) \n\t"
     "sw $t9, 21*4(%0) \n\t"
     "sw $k0, 22*4(%0) \n\t"
     "sw $k1, 23*4(%0) \n\t"
     "sw $gp, 24*4(%0) \n\t"
     "sw $sp, 25*4(%0) \n\t"
     "sw $fp, 26*4(%0) \n\t"   // save MEAS sample 26 into CV23[32]
     "sw $ra, 27*4(%0) \n\t"   // save MEAS sample 27 into CV23[27+32]
     
  // End of first pass capture
  
  // Update variables from first pass

"MEAS1PassEnd: \n\t"

     "lw $s0, 39*4(%0) \n\t"     // Load MEAS1 average count
     "addi $s0, $s0,-1 \n\t"     // Decrement
     "lw $s3, 38*4(%0) \n\t"     // Load PORTB average count
     "lw $s4, 35*4(%0) \n\t"     // Load current address offset
     "addi $s4, $s4, 40*4 \n\t"  // Update address offset 
    
     "addi %0, %0, 40*4 \n\t"    // Update CV23 pointer register += 40*4

     "sw $s0, 39*4(%0) \n\t"     // Save decremented MEAS average count
     "sw $s3, 38*4(%0) \n\t"     // Save PORTB average count unchanged
     "sw $s4, 35*4(%0) \n\t"     // Save updated address offset

     "bne $s0,$zero, AveragingLoop1 \n\t" // Do another capture if AverageCount1 not 0
     "nop \n\t"
 
   // Start of averaging using PORTB
   
     "lw $s0, 38*4(%0) \n\t"      // Fetch current average count 2
     "beq $s0, $zero, Exit2 \n\t" // Check for special case of 0 PORTB captures

  "PORTBAveragingCaptureLoop: \n\t"

     "lui $a0, 0xbf88 \n\t"       // Base address for PORTB

     "lw  $v0, 24864($a0) \n\t"   // read PORTB sample 0
     "lw  $v1, 24864($a0) \n\t"   // read PORTB sample 1
     "lw  $a2, 24864($a0) \n\t"
     "lw  $a3, 24864($a0) \n\t"
     "lw  $t0, 24864($a0) \n\t"
     "lw  $t1, 24864($a0) \n\t"
     "lw  $t2, 24864($a0) \n\t"
     "lw  $t3, 24864($a0) \n\t"
     "lw  $t4, 24864($a0) \n\t"
     "lw  $t5, 24864($a0) \n\t"
     "lw  $t6, 24864($a0) \n\t"
     "lw  $t7, 24864($a0) \n\t"
     "lw  $s0, 24864($a0) \n\t"
     "lw  $s1, 24864($a0) \n\t"
     "lw  $s2, 24864($a0) \n\t"
     "lw  $s3, 24864($a0) \n\t"
     "lw  $s4, 24864($a0) \n\t"
     "lw  $s5, 24864($a0) \n\t"
     "lw  $s6, 24864($a0) \n\t"
     "lw  $s7, 24864($a0) \n\t"
     "lw  $t8, 24864($a0) \n\t"
     "lw  $t9, 24864($a0) \n\t"
     "lw  $k0, 24864($a0) \n\t"
     "lw  $k1, 24864($a0) \n\t"
     "lw  $gp, 24864($a0) \n\t"
     "lw  $sp, 24864($a0) \n\t"
     "lw  $fp, 24864($a0) \n\t"   // read PORTB sample 26
     "lw  $ra, 24864($a0) \n\t"   // read PORTB sample 27

  // Save PORTB values into CV23 starting at offset 0 arrays

     "sw $v0,  0*4(%0) \n\t"      // save PORTB sample 0 to CV23[0]
     "lw $v0, 24864($a0) \n\t"    // read PORTB sample 28
     "sw $v0, 28*4(%0) \n\t"      // save PORTB sample 28 to CV23[28]
     "lw $v0, 24864($a0) \n\t"    // read PORTB sample 29
     "sw $v0, 29*4(%0) \n\t"      // save PORTB sample 29 to CV23[29]
     "lw $v0, 24864($a0) \n\t"    // read PORTB sample 30
     "sw $v0, 30*4(%0) \n\t"      // save PORTB sample 30 to CV23[30]
     "lw $v0, 24864($a0) \n\t"    // read PORTB sample 31
     "sw $v0, 31*4(%0) \n\t"      // save PORTB sample 31 to CV23[31]
     "sw $v1,  1*4(%0) \n\t"      // save PORTB sample 1 to CV23[1]
     "sw $a2,  2*4(%0) \n\t"
     "sw $a3,  3*4(%0) \n\t"
     "sw $t0,  4*4(%0) \n\t"
     "sw $t1,  5*4(%0) \n\t"
     "sw $t2,  6*4(%0) \n\t"
     "sw $t3,  7*4(%0) \n\t"
     "sw $t4,  8*4(%0) \n\t"
     "sw $t5,  9*4(%0) \n\t"
     "sw $t6, 10*4(%0) \n\t"
     "sw $t7, 11*4(%0) \n\t"
     "sw $s0, 12*4(%0) \n\t"
     "sw $s1, 13*4(%0) \n\t"
     "sw $s2, 14*4(%0) \n\t"
     "sw $s3, 15*4(%0) \n\t"
     "sw $s4, 16*4(%0) \n\t"
     "sw $s5, 17*4(%0) \n\t"
     "sw $s6, 18*4(%0) \n\t"
     "sw $s7, 19*4(%0) \n\t"
     "sw $t8, 20*4(%0) \n\t"
     "sw $t9, 21*4(%0) \n\t"
     "sw $k0, 22*4(%0) \n\t"
     "sw $k1, 23*4(%0) \n\t"
     "sw $gp, 24*4(%0) \n\t"
     "sw $sp, 25*4(%0) \n\t"
     "sw $fp, 26*4(%0) \n\t"   // save PORTB sample 26 to CV23[26]
     "sw $ra, 27*4(%0) \n\t"   // save PORTB sample 27 to CV23[27]
   
     "lw $s0, 38*4(%0) \n\t"      // Fetch current PORTB average count 
     "addi $s0, $s0,-1 \n\t"      // Decrement
     "beq $s0, $zero, Exit2 \n\t" // End of capture
     "lw $s3, 35*4(%0) \n\t"      // Load current address offset
     "addi $s3, $s3, 40*4 \n\t"   // Update address offset 
 
     "addi %0, %0, 40*4 \n\t"     // Update CV23 pointer register += 40*4
     "sw $s0, 38*4(%0) \n\t"      // Save updated PORTB average count
     "sw $s3, 35*4(%0) \n\t"      // Save address offset
   
     "j PORTBAveragingCaptureLoop \n\t"
     
"Exit2: \n\t"
     "nop \n\t"
     "lw $s3, 35*4(%0) \n\t"    // Retrieve address offset to restore original %0
     "sub %0, %0, $s3 \n\t"     // Should put base address back where it was originally
     
  // End of averaging capture

  // Restore registers from stack
  
     "lw  $29, 1295*4(%0) \n\t"  // Restore stack pointer to sp
  
     "lw  $3,  0*4($sp) \n\t"    // Restore from stack to $3
     "lw  $4,  1*4($sp) \n\t"
     "lw  $5,  2*4($sp) \n\t"
     "lw  $6,  3*4($sp) \n\t"
     "lw  $7,  4*4($sp) \n\t"
     "lw  $8,  5*4($sp) \n\t"
     "lw  $9,  6*4($sp) \n\t"
     "lw $10,  7*4($sp) \n\t"
     "lw $11,  8*4($sp) \n\t"
     "lw $12,  9*4($sp) \n\t"
     "lw $13, 10*4($sp) \n\t"
     "lw $14, 11*4($sp) \n\t"
     "lw $15, 12*4($sp) \n\t"
     "lw $16, 13*4($sp) \n\t"
     "lw $17, 14*4($sp) \n\t"
     "lw $18, 15*4($sp) \n\t"
     "lw $19, 16*4($sp) \n\t"
     "lw $20, 17*4($sp) \n\t"
     "lw $21, 18*4($sp) \n\t"
     "lw $22, 19*4($sp) \n\t"
     "lw $23, 20*4($sp) \n\t"
     "lw $24, 21*4($sp) \n\t"
     "lw $25, 22*4($sp) \n\t"
     "lw $26, 23*4($sp) \n\t"
     "lw $27, 24*4($sp) \n\t"
     "lw $28, 25*4($sp) \n\t"
     "lw $30, 26*4($sp) \n\t"
     "lw $31, 27*4($sp) \n\t"     // Restore from stack to $31

     "addi $sp, $sp, 112"        // Restore stack pointer to where it was before assembly block - increment by 112

     : // we handle this manually now
     : "r" (CV23) // we handle this manually now 
     :"%a0","%v0","%v1","%a2","%a3","%t0","%t1","%t2","%t3","%t4","%t5","%t6","%t7","%s0","%s1","%s2","%s3","%s4","%s5","%s6","%s7","%t8","%t9","%k0","%k1","%fp","%ra","memory");   // clobbered registers; memory

   // leave out "%a1" to give the compiler a place to store CV23;
   // memory means don't move assembly block

// Save Timer values for REF and MEAS frequency estimation.

     Timer2Prefetch = CV23[Timer2PrefetchOffset];
     Timer3Prefetch = CV23[Timer3PrefetchOffset];
     Timer4Prefetch = CV23[Timer4PrefetchOffset];
     Timer5Prefetch = CV23[Timer5PrefetchOffset];
     
// Beginning of phase stuff

    for (m = 0;m < NumberOfAxes;m++)
       {
         NoChange[m] = 0;
         if (REFFrequencyCount[0] > 0)
           {
 //          REFPeriod =  ((40000000 + (REFFrequencyCount[0] * 305)) / REFFrequencyCount[0]) / 610; // REFPeriod = (40 MHz) / (REF Frequency)
             REFPeriod =  ((CPUClock + (REFFrequencyCount[0] * SampleFrequency/2)) / REFFrequencyCount[0]) / SampleFrequency; // REFPeriod = CPUClock / (REF Frequency)
             REFPeriodxNOMA[m] = AxisParam[m][0] * REFPeriod; // MEAS Averages
             REFPeriodxNOPBA[m] = AxisParam[m][3] * REFPeriod; // PORTB Averages
           }
         else NoChange[m] = 0x4200; // No REF clocks
       } 
  
// Averaging REF/MEAS analysis loop for REF/MEAS acquisition

      for (m = 0;m < NumberOfAxes;m++)
        {
          apm1x40 = AxisParam[m][1] * 40;
          REFToMEASMAverage[m] = 0;

          for(k = 0;k < AxisParam[m][0];k++)
            {
              kx40papm1x40 = k * 40 + apm1x40;            
      
       // Find first REF change
       // m is axis, k is MEAS average number
       
//             NoChange[m] = 0;
 
             if (REFFrequencyCount[0] < 100) NoChange[m] = 0x4200;
         
             FirstREFChange[m][k] = -1;        // First REF change value by definition
//           CV2[m][k] = CV23[32 + (k*40) + (AxisParam[m][1]*40)];    // CounterValue2 (REF) values for displacement
             CV2[m][k] = CV23[32 + kx40papm1x40];    // CounterValue2 (REF) values for displacement
//           CV3[m][k] = CV23[0 + (k*40) + (AxisParam[m][1]*40)];     // CounterValue3 (MEAS) values for displacement
             CV3[m][k] = CV23[kx40papm1x40];         // CounterValue3 (MEAS) values for displacement
  
    // Find change in MEAS after first REF change
    // (Samples 28 and 29 do not occur at precisely the correct time but are better than nothing!)

             i = 5;
             j = 32;
             MEASChange[m][k] = 30;

             ipkx40papm1x40 = kx40papm1x40;

             while (i < 31)
               {
                 ipkx40papm1x40 += 5;
//               if (CV23[i + (k * 40) + (AxisParam[m][1] * 40)] !=  CV23[i - 1 + (k * 40) + (AxisParam[m][1] * 40)]) // Check for MEAS change
                 if (CV23[ipkx40papm1x40] !=  CV23[ipkx40papm1x40 - 5]) // Check for MEAS change
                   {
                     j = i; // MEAS edge found
                     i = 31; // Drop out
                   }
                 else
                   {
                     i += 5;
                   }
               }

             if (j != 32) // Found an edge in previous 5 points, need to home in on it
               {
                 ipkx40papm1x40 -= 5; // Back up 5.
                 i = j - 5;
                 n = j;

                 while (i < n) // Find edge within block of 5 points
                   {
                     ipkx40papm1x40++;
                     if (CV23[ipkx40papm1x40] !=  CV23[ipkx40papm1x40 - 1]) // Check for MEAS change
                       {
                         j = i; // MEAS edge found
                         i = n; // Drop out
                       }
                     else
                       {
                         i++;
                       }
                   }
                   MEASChange[m][k] = j;
               }
             else NoChange[m] |= 0x800; // No first pass MEAS change found

             REFToMEASBuffer[m][k] = 3 - MEASChange[m][k]; // Range of 2 when MEAS is at REF+2 to (3-REFPeriod) when MEAS is at far end RP+1
   
      // REFToMEASMAverage += MEASChange[k];

             if (REFToMEASBuffer[m][k] > (REFToMEASBuffer[m][0] + (REFPeriod / 2))) REFToMEASBuffer[m][k] -= REFPeriod;
             if (REFToMEASBuffer[m][k] < (REFToMEASBuffer[m][0] - (REFPeriod / 2))) REFToMEASBuffer[m][k] += REFPeriod;
           }

         CounterValue2[m] = CV2[m][0]; // CounterValue2 (REF)
         CounterValue3[m] = CV3[m][0]; // CounterValue3 (MEAS)
       }

// Averaging computation loop using PORTB
     // Locate FirstREFChange, MEASChange

  for (m=0;m<NumberOfAxes;m++)
    {
         apm4 = AxisParam[m][4];
         REFToMEASPBAverage[m] = 0;
            
         for(k = TotalMEASAverages;k < (TotalMEASAverages+NumberOfPORTBAverages);k++)   
           {

      // Find REF clock rising edge in PORTB data (RPB1)
             i = 5; // Start at beginning  
             j = 0;
             kx40 = k * 40;

             ipkx40 = kx40;

             while (i < 31)
               {
                 ipkx40 += 5;

//               if ((CV23[i + (k * 40)] & 0x20) == ((CV23[i - 1 + (k * 40)] & 0x20) + 0x20)) // Check for REF clock rising edge
                 if ((CV23[ipkx40] & 0x20) > (CV23[ipkx40 - 5] & 0x20)) // Check for REF clock rising edge RPB5  
                   {
                     j = i;  // REF edge found
                     i = 31; // Drop out
                   }
                 else i += 5;
               }

             if (j != 0)
               {
                 ipkx40 -= 5;
                 i = j - 5;
                 n = j;

                 while (i < n)
                   {
                     ipkx40++;

                      if ((CV23[ipkx40] & 0x20) > (CV23[ipkx40 - 1] & 0x20)) // Check for REF clock rising edge RPB5 
                        {
                          j = i;  // REF edge found
                          i = n;  // Drop out  
                        }
                      else i++;
                   }
                 FirstREFChange[m][k] = j;
               }

             else NoChange[m] |= 0x3000; // No REF changes found              

      // Find MEAS clock rising edge in PORTB data (RPB0)

             i = 5;
             j = 31;
             ipkx40 = kx40;

             while (i < 31)
               {
                 ipkx40 += 5;
//               if ((CV23[i + (k * 40)] & AxisParam[m][4]) == ((CV23[i - 5 + (k * 40)] & AxisParam[m][4]) + AxisParam[m][4])) // Check for MEAS clock rising edge
                 if ((CV23[ipkx40] & apm4) > (CV23[ipkx40 - 5] & apm4)) // Check for MEAS clock rising edge
                   {
                     j = i;  // MEAS edge found
                     i = 31; // Drop out
                   }
                 else i += 5;
               }

             if (j != 31)
               {
                 i = j - 5;
                 n = j;
                 ipkx40 -= 5;

                 while (i < n)
                   {
                     ipkx40 += 1;
                     if ((CV23[ipkx40] & apm4) > (CV23[ipkx40 - 1] & apm4)) // Check for MEAS clock rising edge 
                       {
                         j = i;  // MEAS edge found
                         i = 31; // Drop out
                       }
                     else i++;
                   }
                 MEASChange[m][k] = j;
               }

             else  NoChange[m] |= 0x4000; // No PORTB MEAS change found

             REFToMEASBuffer[m][k] = FirstREFChange[m][k] - MEASChange[m][k];

             if (REFToMEASBuffer[m][k] < 0) REFToMEASBuffer[m][k] += REFPeriod; // Range of 0 to REFPeriod-1

      // Adjust for relative location of REF to MEAS Timers compared to PORTB samples
             REFToMEASBuffer[m][k] += (11 - REFPeriod); // 11 should work for 5517A-D and I (Sam) have no idea why. :-)

             if (REFToMEASBuffer[m][k] > (REFToMEASBuffer[m][0] + (REFPeriod / 2))) REFToMEASBuffer[m][k] -= REFPeriod - 0;
             if (REFToMEASBuffer[m][k] < (REFToMEASBuffer[m][0] - (REFPeriod / 2))) REFToMEASBuffer[m][k] += REFPeriod + 0;
           }
         }

// End of average change computation loop

// Compute averages

    for (m=0;m<NumberOfAxes;m++)
       {
         if ((NoChange[m] & 0xe00) != 0) // Check for problems with REF/MEAS data
           {
             REFPeriodxNOMA[m] = 27 * AxisParam[m][0]; 
             MEASPhase[m] = NoChange[m];
           }
         else // REF/MEAS average calculation
           {
             REFToMEASMAccum[m] = 0; 
             for(k = 0;k < AxisParam[m][0];k++)
               {
                 REFToMEASMAccum[m] += REFPeriod + REFToMEASBuffer[m][k];
               }
//               REFToMEASMAverage[m] = REFToMEASMAccum[m] / AxisParam[m][0]; // For diagnostic readout only
               REFToMEASMAccum[m] -= (2 * AxisParam[m][0]);                   
                   // Equivelent to: MEASPhase = (1 + REFToMEAS/REFPeriod + 2/REFPeriod) * 256
               if (REFPeriod > 0) MEASPhase[m] = (REFToMEASMAccum[m] * 256) / REFPeriodxNOMA[m]; // Offset * 256 / REF period: Main result
               else MEASPhase[m] = 0;                 
             }
        }
 
     for (m=0;m<NumberOfAxes;m++)
       {
         if ((NoChange[m] & 0x7000) != 0) // Check for problems with PORTB data
           {
             REFPeriodxNOPBA[m] = 27 * AxisParam[m][3]; 
             PORTBPhase[m] = NoChange[m];
           }
         else // PORTB average calculation
           {
             PhaseBufferAccum[m] = 0;
             for(k = TotalMEASAverages;k < (TotalMEASAverages + AxisParam[m][3]);k++)
               {
                 PhaseBufferAccum[m] += REFPeriod + REFToMEASBuffer[m][k];
               }
               REFToMEASPBAverage[m] = PhaseBufferAccum[m] / AxisParam[m][3]; // For diagnostic readout only
               PhaseBufferAccum[m] -= ((AxisParam[m][3] * 5) / 2);  // Zero correction
                  // Equivelent to: PORTBPhase = (1 + REFToMEAS/REFPeriod + 5/2/REFPeriod) * 256
               if (REFPeriod > 0) PORTBPhase[m] = (PhaseBufferAccum[m] * 256) / REFPeriodxNOPBA[m]; // Offset * 256 / REF period: Main result
               else PORTBPhase[m] = 0;
       }
     }
 
/* Phase error codes:
    Phase & 0x7e = 0: Valid data.  Range is -512 to +511.  Most of the time between 0 and 255.
    Phase with bit 0x200 set: No REF/MEAS first REF change found - not possible
    Phase with bit 0x400 set: No REF/MEAS second REF change found - no REF counts
    Phase with bit 0x800 set: No REF/MEAS MEAS change found
    Phase with bit 0x1000 set: No PORTB first REF change found
    Phase with bit 0x2000 set: No PORTB second REF change found - not possible
    Phase with bit 0x4000 set: No PORTB MEAS change found
*/

// End of phase stuff

// Activity detection for Axis 2 and 3

    // Turn on multiple axis mode if activity is detected on Axes 2 or 3

    if ((Timer4Prefetch - PreviousTimer4Prefetch) > 100)
      {
        ActiveAxes |= 0xa;                       // Enable multiaxis mode and Axis 2
        NumberOfMEAS2Averages = 1;               // Number of Axis 2 MEAS averages
        AxisParam[2][3] = NumberOfPORTBAverages; // Number of Axis 2 PORTB averages
      }   
    if ((Timer2Prefetch - PreviousTimer2Prefetch) > 100)
      {
        ActiveAxes |= 0xc;                       // Enable multiaxis mode and Axis 3
        NumberOfMEAS3Averages = 1;               // Number of Axis 3 MEAS averages
        AxisParam[3][3] = NumberOfPORTBAverages; // Number of Axis 3 PORTB averages
      }

    if (ActiveAxes > 1) NumberOfAxes = 3;
    
    PreviousTimer2Prefetch = Timer2Prefetch;
    PreviousTimer3Prefetch = Timer3Prefetch;
    PreviousTimer4Prefetch = Timer4Prefetch;
    PreviousTimer5Prefetch = Timer5Prefetch;

// Adjust CPU clock speed to optimize interpolation.

    REFFrequency = REFFrequencyCount[0] * SampleFrequency;

   if (ClkSkipCount > 0)
     {
     	ClkSkipCount--;
     }
   else
     {
       if (CPUClock == 30000000)
         {
           if (REFFrequency > 1500000)
             {
               CPUClock = 40000000;
               OSCConfig (OSC_POSC_PLL,OSC_PLL_MULT_20,OSC_PLL_POST_2,OSC_FRC_POST_2); // 40Mhz
               SampleFrequencyX100 = 61035;
               SampleFrequency = 610;
               ClkSkipCount = 2;
             } 
         }
       else if (CPUClock == 40000000)
         {
           if (REFFrequency < 1450000)
             {
               CPUClock = 30000000;
               OSCConfig (OSC_POSC_PLL,OSC_PLL_MULT_15,OSC_PLL_POST_2,OSC_FRC_POST_2); // 30Mhz
               SampleFrequencyX100 = 45776;
               SampleFrequency = 458;
               ClkSkipCount = 2;
             }
           else if (REFFrequency > 1775000) // Disabled due to weird behavior at 48 MHz, should be 2250000
             {
               CPUClock = 48000000;
               OSCConfig (OSC_POSC_PLL,OSC_PLL_MULT_24,OSC_PLL_POST_2,OSC_FRC_POST_2); // 48Mhz
               SampleFrequencyX100 = 73242;
               SampleFrequency = 732;
               ClkSkipCount = 2;
             }      
         }
       else if (CPUClock == 48000000)
         {
           if (REFFrequency < 1725000)
             {
               CPUClock = 40000000;
               OSCConfig (OSC_POSC_PLL,OSC_PLL_MULT_20,OSC_PLL_POST_2,OSC_FRC_POST_2); // 40Mhz
               SampleFrequencyX100 = 61035;
               SampleFrequency = 610;
               ClkSkipCount = 2;
             }
         }
     }
     
// Update counters
    for (m=0;m<NumberOfAxes;m++) // MEAS1=0, MEAS2=1, MEAS3=2
      {    
        if (PreviousCounterValue2[m] > CounterValue2[m])    // check for overflow of (2);
          {    	
            Counter2_OverflowCounter[m]++;                  // count how often (2) overflows
          }
        PreviousCounterValue2[m] = CounterValue2[m];    // needed to check for overflow the next time this function is called.
        tempTotalCounter = Counter2_OverflowCounter[m]; // combine values for (2)
        tempTotalCounter = tempTotalCounter<<16;        // combine values for (2)
        tempTotalCounter += CounterValue2[m];           // combine values for (2)
        PreviousTotalCounter2[m] = TotalCounter2[m];
        TotalCounter2[m] = tempTotalCounter;            // this is now the new value for (2)

        if (PreviousCounterValue3[m] > CounterValue3[m])    // check for overflow of (3); this could have happened anytime before here including during the 2 previous lines
          {
            Counter3_OverflowCounter[m]++;	                // count how often (3) overflows
          }
        PreviousCounterValue3[m] = CounterValue3[m];    // needed to check for overflow the next time this function is called.
        tempTotalCounter = Counter3_OverflowCounter[m]; // combine values for (3)
        tempTotalCounter = tempTotalCounter<<16;        // combine values for (3)
        tempTotalCounter += CounterValue3[m];           // combine values for (3)
        PreviousTotalCounter3[m] = TotalCounter3[m];
        TotalCounter3[m] = tempTotalCounter;            // this is now the new value for (3)
      
// prepare for sending out next data packet
    
        REFFrequencyCount[m] = TotalCounter2[m] - PreviousTotalCounter2[m];
        MEASFrequencyCount[m] = TotalCounter3[m] - PreviousTotalCounter3[m];

        PreviousTotalDifference[m] = TotalDifference[m];
        TotalDifference[m] = TotalCounter2[m] - TotalCounter3[m]; // assign value to be displayed
    }

    LowSpeedCode = 0; // Default to no low speed data
    LowSpeedData = 0;
    LowSpeedCodeSelect = SequenceNumber & 0x1f;

    if (LowSpeedCodeSelect == 1) // Send firmware version
      {
        LowSpeedCode = 10;
        LowSpeedData = FirmwareVersion;
      }
    else if (LowSpeedCodeSelect == 2) // # 40 MHz CPU clocks spent in capture and analysis
      {
        LowSpeedCode = 8;
        LowSpeedData = SampleFrequencyX100;
      }
   else if (LowSpeedCodeSelect == 3) // Send temperature 1
      {
        LowSpeedCode = 3;
        LowSpeedData = temperature1;
      }
    else if (LowSpeedCodeSelect == 4) // Send temperature 2
      {
        LowSpeedCode = 4;
        LowSpeedData = temperature2;
      }   
    else if (LowSpeedCodeSelect == 5) // Send pressure
      {
        LowSpeedCode = 5;
        LowSpeedData = pressure;
      }   
    else if (LowSpeedCodeSelect == 6) // Send humidity
      {
        LowSpeedCode = 6;
        LowSpeedData = humidity;
      } 
    else if (LowSpeedCodeSelect == 7) // REF/MEAS1 averaging REFPeriod and REFToMEAS1AVerage values
      {
        LowSpeedCode = 101;
        LowSpeedData = 0x8080;
        if ((NoChange[0] & 0x600) == 0) LowSpeedData = 0x8000 + (REFPeriod & 0xff);
        if ((NoChange[0] & 0xc00) == 0) LowSpeedData = (LowSpeedData & 0xff) + ((REFToMEASMAverage[0] & 0xff) << 8);
      }
    else if (LowSpeedCodeSelect == 8) // REF/MEAS1 averaging FirstREFChange and SecondREFChange values
      {
        LowSpeedCode = 102;
        LowSpeedData = 0x8080;
        if ((NoChange[0] & 0x200) == 0) LowSpeedData = 0x8000 + (FirstREFChange[0][0] & 0xff);
        if ((NoChange[0] & 0x400) == 0) LowSpeedData = (LowSpeedData & 0xff) + (((FirstREFChange[0][0] + REFPeriod) & 0xff) << 8);
      }  
    else if (LowSpeedCodeSelect == 9) // PORTB averaging REFPeriod and REFToMEAS1Average values
      {
        LowSpeedCode = 111;
        LowSpeedData = 0x8080;
        if ((NoChange[0] & 0x600) == 0) LowSpeedData = 0x8000 + (REFPeriod & 0xff);
        if ((NoChange[0] & 0xc00) == 0) LowSpeedData = (LowSpeedData & 0xff) + ((REFToMEASPBAverage[0] & 0xff) << 8);
      }
    else if (LowSpeedCodeSelect == 10) // PORTB averaging FirstREFChange and SecondREFChange values
      {
        LowSpeedCode = 112;
        LowSpeedData = 0x8080;
        if ((NoChange[0] & 0x200) == 0) LowSpeedData = 0x8000 + (FirstREFChange[0][0] & 0xff);
        if ((NoChange[0] & 0x400) == 0) LowSpeedData = (LowSpeedData & 0xff) + (((FirstREFChange[0][0] + REFPeriod) & 0xff) << 8);
      }
    else if (LowSpeedCodeSelect == 11) // # CPU clocks spent in capture and analysis
      {
        LowSpeedCode = 121;
        LowSpeedData = Timer1PRCCounts;
      }
    else if (LowSpeedCodeSelect == 12) // # 40 MHz CPU clocks spent in capture and analysis
      {
        LowSpeedCode = 122;
        LowSpeedData = Timer1USBCounts;
      }


    if (!(((0==tail)&&((FIFO_SIZE-1)==head))||(head==(tail-1)))) // check if buffer is full; either tail is at beginning of array and head is at end, or head is right before tail
      {
      // head points to the element to store next data
        dataPoint[head].displayREFFrequencyCount =  REFFrequencyCount[0];
        dataPoint[head].displayMEAS1FrequencyCount =  MEASFrequencyCount[0];
        dataPoint[head].displayTotalDifference1 = TotalDifference[0];
        dataPoint[head].displayVelocity1Count = TotalDifference[0] - PreviousTotalDifference[0];
        dataPoint[head].displayPhase1 = PORTBPhase[0];
        dataPoint[head].displaySequenceNumber = SequenceNumber;
        dataPoint[head].displayLowSpeedCode = LowSpeedCode;
        dataPoint[head].displayLowSpeedData = LowSpeedData;

        dataPoint[head].displayMEAS2FrequencyCount =  MEASFrequencyCount[1];
        dataPoint[head].displayTotalDifference2 = TotalDifference[1];
        dataPoint[head].displayVelocity2Count = TotalDifference[1] - PreviousTotalDifference[1];
        dataPoint[head].displayPhase2 = PORTBPhase[1];
        dataPoint[head].displayMEAS3FrequencyCount =  MEASFrequencyCount[2];
        dataPoint[head].displayTotalDifference3 = TotalDifference[2];
        dataPoint[head].displayVelocity3Count = TotalDifference[2] - PreviousTotalDifference[2];
        dataPoint[head].displayPhase3 = PORTBPhase[2];

      // increment head
        if ((FIFO_SIZE-1)==head)
          head = 0;
        else
          head = head +1;
      }

    else
      {
        bufferOverflowCounter++;  // buffer is full. Increment counter
      }
 
 //  INTEnableSystemMultiVectoredInt();
    if ((SequenceNumber & 0x10) != 0) digitalWrite(PIN_LED3, 1);
    else digitalWrite(PIN_LED3, 0);
    
    Timer1PRCCounts = ReadTimer1();  // Save # of 40 MHz CPU clocks spent in capture and analysis
    mT1ClearIntFlag();  
}

void setup() {
//  OSCConfig (OSC_POSC_PLL,OSC_PLL_MULT_20,OSC_PLL_POST_2,OSC_FRC_POST_2); // 48 Mhz
    delay(1000); // wait for USB enumeration; this is really only important for the DP32 board; others probably would not need it
    Serial.begin(115200); // Buad rate (probably doesn't matter for USB)

#ifdef UseSsnsors
// Sensor code - currently only reads after RESET.

    if (!bmp.begin()) 
      {
        Serial.println("Could not find a valid BMP180 sensor, check wiring!");
      }
    else 
      {
      	 temperature1 = bmp.readTemperature()*100; // value to send out over USB
   	     pressure = bmp.readPressure();
      }

    if (DHTLIB_OK != DHT.read22(1)) 
      {
        Serial.println("Could not find a valid AM2302 sensor, check wiring!");
      }
    else
      {
        temperature2 = DHT.temperature*100;                 // value to send out over USB
        humidity =     DHT.humidity*10;                     // value to send out over USB
      }
#endif

// set up pps pin for external counters

// REF/MEAS1/MEAS2/MEAS3 wiring:
// REF: J4 1st pin labeled 0 is RPB5 REF Timer3.
// MEAS1 J3 3rd pin labeled 11 is P32-PGD RPB0 Timer5, also goes to JP6-4 via 50 ohms and LED4.
// MEAS2 J3 5th pin labeled 13 is RPB2 Timer4, also goes to LED2.
// MEAS3 J3 6th pin labeled 14 is RBB3 Timer2, also goes to LED1.

// Set up counters

// Sample interrupt overflow counter:
    OpenTimer1( T1_ON | T1_SOURCE_INT | T1_PS_1_1, 0xFFFF);  // Timer1 internal clock based counter

// REF:
    pinMode(0, INPUT);  // REF (RPB5)
    T3CKR = 0b0001;     // REF Timer3 on RPB5 J4-1 (0)
    OpenTimer3( T3_ON | T3_SOURCE_EXT | T3_PS_1_1, 0xFFFF);  // Timer3 address: 0xbf800000 + 2576 (REF)

// MEAS for axis 1:
    pinMode(11, INPUT); // MEAS1 (RPB0) LED4
    T5CKR = 0b0010;     // MEAS1 Timer5 on RPB0 J3-3 (11) LED4
    OpenTimer5( T5_ON | T5_SOURCE_EXT | T5_PS_1_1, 0xFFFF);  // Timer5 address: 0xbf800000 + 3600 (MEAS1)

// Axes 2 and 3.  Uncomment this code only if LEDs are disabled.

// MEAS for axis 2:
    pinMode(13, INPUT); // MEAS2 (RPB2) LED2
    T4CKR = 0b0100;     // MEAS2 Timer4 on RPB2 J3-5 (13) LED2
    OpenTimer4( T4_ON | T4_SOURCE_EXT | T4_PS_1_1, 0xFFFF);  // Timer4 address: 0xbf800000 + 3088 (MEAS2)

// MEAS for axis 3:
    pinMode(14, INPUT); // MEAS3 (RPB3) LED1
    T2CKR = 0b0001;     // MEAS3 Timer2 on RPB3 J3-6 (14) LED1
    OpenTimer2( T2_ON | T2_SOURCE_EXT | T2_PS_1_1, 0xFFFF);  // Timer2 address: 0xbf800000 + 2064 (MEAS3)

// Heartbeat/status LED
    pinMode(PIN_LED3, OUTPUT);
    
// set up interrupt for internal timer
    setIntVector(_TIMER_1_VECTOR,handlerForTimer1Ints);      // Configure interrupt for (1)
    ConfigIntTimer1( T1_INT_ON | T1_INT_PRIOR_1);            // enable interrupt for (1)

    // Initialize counter variables
    for (m=0;m<3;m++)
      {
        Counter2_OverflowCounter[m] = 0;
        Counter3_OverflowCounter[m] = 0;
        CounterValue2[m] = 0;
        CounterValue3[m] = 0;
        PreviousCounterValue2[m] = 0;
        PreviousCounterValue3[m] = 0;
        REFFrequencyCount[m] = 0;
        MEASFrequencyCount[m] = 0;
      }
    INTEnableSystemMultiVectoredInt();                       // Turn on all interrupts
 }

void loop() {

  int length = 0;
  int Timer1Start = 0;
  int Timer1End = 0;
  
// Determine time spent in this Sensor/Comm routine
  Timer1Start = ReadTimer1();
  
  if (head != tail)  // only send a new data packet if the queue is not empty; probably superfluous since serial.println blocks :/
  {
    length += sprintf(buffer,"%d ",dataPoint[head].displayREFFrequencyCount);          // REF Frequency Count
    length += sprintf(buffer+length,"%d ",dataPoint[head].displayMEAS1FrequencyCount); // MEAS1 Frequency Count
    length += sprintf(buffer+length,"%d ",dataPoint[head].displayTotalDifference1);    // Distance1
    length += sprintf(buffer+length,"%d ",dataPoint[head].displayVelocity1Count);      // Velocity1 Count
    length += sprintf(buffer+length,"%d ",dataPoint[head].displayPhase1);              // Phase1
    length += sprintf(buffer+length,"%d ",dataPoint[head].displaySequenceNumber);      // Sequence Number
    length += sprintf(buffer+length,"%d ",dataPoint[head].displayLowSpeedCode);        // Code to specify low speed data type
    length += sprintf(buffer+length,"%d",dataPoint[head].displayLowSpeedData);         // Low speed data to send back
       
    if (ActiveAxes > 1)
      {
        length += sprintf(buffer+length," %d ",dataPoint[head].displayMEAS2FrequencyCount); // MEAS2 Frequency Count
        length += sprintf(buffer+length,"%d ",dataPoint[head].displayTotalDifference2);    // Distance2
        length += sprintf(buffer+length,"%d ",dataPoint[head].displayVelocity2Count);      // Velocity2 Count
        length += sprintf(buffer+length,"%d ",dataPoint[head].displayPhase2);              // Phase2
        length += sprintf(buffer+length,"%d ",dataPoint[head].displayMEAS3FrequencyCount); // MEAS3 Frequency Count
        length += sprintf(buffer+length,"%d ",dataPoint[head].displayTotalDifference3);    // Distance3
        length += sprintf(buffer+length,"%d ",dataPoint[head].displayVelocity3Count);      // Velocity3 Count
        length += sprintf(buffer+length,"%d",dataPoint[head].displayPhase3);               // Phase3
      }
 
    // Send data
//  Serial.flush();
    Serial.println(buffer);

   // update queue
    if ((FIFO_SIZE-1)==tail)  // check for wrap around
       tail = 0;                    // update tail location
    else  
       tail = tail +1;              // update tail location
        
    Timer1End = ReadTimer1(); // Deterine time spent in this routine
    Timer1USBCounts = Timer1End - Timer1Start;
  }
}
