//****************************************************************************************//
// Micro Measurement Display 2 - uMD2 - Heterodyne interferometer firmware for Teensy 4.0 //
//                        Jan Beck and Sam Goldwasser 2021                                //
//****************************************************************************************//

// V2.00 Formatted for GUI and OLED.
// V2.01 First version that talks to GUI. :)
// V2.02 Changed OLED to display counters XOR displacements.  Counts are screwy due to noise or crosstalk or....
// V2.04 First working heterodyne version.  Removed analogwrite stuff, fixed serial formatting.
// V2.05 Changed to double clocking, resolution 79 nm with PMI, Still slow drift (~2 um/min) if OLED is enabled, probably noise.
// V2.06 NC
// V2.07 Moved MEAS2 from D13 to D0 to free up status LED.
// V2.08 Firmware version format, status LED.
// V2.09 Added code to turn off unused LEDs. (Only will work if no line receivers installed.)

// V2.20 Moved REF, MEAS1, MEAS2 to shorten traces on V1.2 PCB (jumpers on V1.0 PCB).
//        REF   TMR2 D0  Pin 2  (REF was TMR4 D9 Pin 11, D0 was MEAS2)
//        MEAS1 TMR4 D9  Pin 11 (MEAS1 was TMR1 D10 Pin 12, D9 was REF)
//        MEAS2 TMR1 D10 Pin 12 (MEAS2 was TMR2 D0 Pin 2, D10 was MEAS1)
//        MEAS3 TMR3 D14 Pin 16

// V2.23 Added initial code for interpolation, requires additional jumpers to put REF,MEAS1,MEAS2,MEAS3 in a single IO address
//        REF D6 Pin 8
//        MEAS1 D8 Pin 10
//        MEAS2 D11 Pin 13
//        MEAS3 D12 Pin 14

// V2.24 Cleanup.  Tested confirmed working WIITHOUT interpolation code
// V2.25 First test with some parts of interpolation.  Appears to find REF edges and average period correctly.
// V2.26 In-line assembly code to capture waveform data.  Appears to be slightly SLOWER than loop.  One clock cycle/loop.  Depricated.
// V2.27 REF period average sent via Low Speed Code 101 for diagnostics
// V2.28 Added MEAS offsets UNTESTED probably not needed, removed in future revs
// V2.29 Initial version of complete interpolation for MEAS1.  Compiles but NOT TESTED.
// V2.32 First test of simple interpolation using GPIO values only.  One axis partially working
// V2.33 Refine interpolation code.  50 tests for REF edge.
// V2.34 Interpolation working but with (expected) glitches and small jump.
// V2.35 Interpolation continued.
// V2.36 Interpolation continued.
// V2.42 Test for inconsistent displacement and phase - displacement change with no phase change.
// V2.47 Best so far but glitches.
// V2.47b Put QuadTimer init and assembly code into subroutines.
// V2.47c Use C code only for REF Sync.  Appears similar to using assembly.
// V2.47d Attempt to sync REF on GPIO bit 10 instead of TMR2.
// V2.47e Use C code only for REF Sync and GPIO reads.  Similar to best using assembly.
// V2.47f Use GPIO for REF Sync in assembly; GPIO for waveform capture in C.
// V2.47g Use C code only for REF Sync and GPIO reads.  Similar to best using assembly.
// V2.47h Use assembly loop for REF Sync with GPIO, C for waveform capture. OK.
// V2.47i Use assembly loops with GPIO for both REF Sync and waveform capture. OK.
// V2.47j Ditto, streamlined one bit. ;-)
// V2.47k Added assembly test for no REF edges and cleaned up roundoff in phase.
// V2.47l Ditto, now seems to work OK.
// V2.47m Added optional OLED display of REF frequency average).
// V2.47n Formatting of OLED frequency counter.
// V2.47o Optimized OLED frequency counter update.
// V2.47p Optimized OLED sn and DISP update.
// V2.47q Ditto.
// V2.47r Ditto.  Changed register assignments in assembly code to get around possible compiler glitch.

#define FirmwareVersion 247  // Firmware version (x100) used by OLED and GUI About.

#define Heterodyne 1           // Set to the number of axes for heterodyne, sample rate always 1 kHz.

#define Multiplier 1           // Integer counts/cycle.  1 for rising edge; 2 for double clocking.  Do not mess with this - used for TMR_CTRL_CM value.

#define Interpolation 0        // O for none, 1 for enabled
#define REF_Sync 0             // Required for interpolation

#define OLED 1                 // OLED display of banner and version if defined
#define OLED_Displacement 1    // OLED display of sn, Disp1, Disp2, Disp3 if defined
#define OLED_REF 2             // REF frequency display if defined; Value is text size.  1 (default) and 2 (large) valid.

// u8g2 graphics library available though the Library Manager

#ifdef OLED
#include <U8x8lib.h>
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
#endif

float FirmwareFloat = FirmwareVersion;
char buffer[100];
int32_t Scale_Shift   = 0;

IMXRT_TMR_t * TMR1 = (IMXRT_TMR_t *)&IMXRT_TMR1;
IMXRT_TMR_t * TMR2 = (IMXRT_TMR_t *)&IMXRT_TMR2;
IMXRT_TMR_t * TMR3 = (IMXRT_TMR_t *)&IMXRT_TMR3;
IMXRT_TMR_t * TMR4 = (IMXRT_TMR_t *)&IMXRT_TMR4;

#define IMXRT_GPIO6_DIRECT  (*(volatile uint32_t *)0x42004008)

IntervalTimer usbTimer;            // send USB data at predefinded rate to make frequency analysis work in the GUI

#ifdef OLED
char oled_buffer[16];
char old_oled_buffer[16];
char REF_oled_buffer[16];
char old_REF_oled_buffer[16];
char sn_oled_buffer[16];
char old_sn_oled_buffer[16];
char DISP1_oled_buffer[16];
char old_DISP1_oled_buffer[16];
char DISP2_oled_buffer[16];
char old_DISP2_oled_buffer[16];
char DISP3_oled_buffer[16];
char old_DISP3_oled_buffer[16];
      
uint8_t tiles[8] = {0x0, 0x80, 0x7c, 0x40, 0x40, 0x40, 0x7c, 0x0}; // Tail on "micro"
#endif

int64_t counter_REF   =  0;
int64_t counter_MEAS1 =  0;
int64_t counter_MEAS2 =  0;
int64_t counter_MEAS3 =  0;

int64_t counter_REF_save   =  0;
int64_t counter_MEAS1_save =  0;
int64_t counter_MEAS2_save =  0;
int64_t counter_MEAS3_save =  0;

int32_t OLED_MEAS1 =  0;
int32_t OLED_MEAS2 =  0;
int32_t OLED_MEAS3 =  0;

int32_t REF_Frequency_Total = 0;
int32_t REF_Frequency_Total_Count = 0;
  
float   REF_Frequency = 0;
float   REF_Frequency_Old = 0;
float   REF_Frequency_New = 0;
float   REF_Frequency_Average = 0;

float   OLED_REF_Frequency = 0;
float   OLED_REF_Frequency_save = 0;

int32_t REF =  0;
int32_t MEAS1 =  0;
int32_t MEAS2 =  0;
int32_t MEAS3 =  0;

int32_t OLED_REF_save = 0;
int32_t OLED_MEAS1_save = 0;
int32_t OLED_MEAS2_save = 0;
int32_t OLED_MEAS3_save = 0;

int32_t OLED_DISP1 =  0;
int32_t OLED_DISP2 =  0;
int32_t OLED_DISP3 =  0;

int32_t OLED_DISP1_save = 0;
int32_t OLED_DISP2_save = 0;
int32_t OLED_DISP3_save = 0;

int32_t encoder1Value = 0;
int32_t encoder2Value = 0;
int32_t encoder3Value = 0;

int32_t encoder1ValueSave = 0;
int32_t encoder2ValueSave = 0;
int32_t encoder3ValueSave = 0;

int32_t compareValue  = 0;

int32_t displacement1 = 0;
int32_t displacement2 = 0;
int32_t displacement3 = 0;

int32_t previous_displacement1 = 0;
int32_t previous_displacement2 = 0;
int32_t previous_displacement3 = 0;

int32_t velocity1     = 0;
int32_t velocity2     = 0;
int32_t velocity3     = 0;

uint64_t sequenceNumber = 0;
uint32_t sn             = 0;
uint32_t old_sn = 0;

int32_t LowSpeedCode = 0;
int32_t LowSpeedData = 0;
int32_t LowSpeedCodeSelect = 0;

uint32_t LEDValue = 0;
int32_t LEDIncrement = 1;

uint32_t i = 0;
uint32_t j = 0;
uint32_t k = 0;
uint32_t l = 0;
uint32_t m = 0;
uint32_t n = 0;

uint32_t data = 0;

uint32_t REF_Save = 0;
uint32_t MEAS1_Save = 0;

uint32_t REF_Waveform[1050];
uint32_t MEAS1_Waveform[1050];
uint32_t MEAS2_Waveform[1050];
uint32_t MEAS3_Waveform[1050];

uint32_t gpioData[1000];

int32_t MEAS1_Phase[500];
int32_t MEAS2_Phase[500];
int32_t MEAS3_Phase[500];

uint32_t MEAS1_Valid = 0;
uint32_t MEAS2_Valid = 0;
uint32_t MEAS3_Valid = 0;

int32_t REF_Offset = 0;
int32_t MEAS1_Offset = 0;
int32_t MEAS2_Offset = 0;
int32_t MEAS3_Offset = 0;

int32_t Phase1_Average = 0;
int32_t Phase2_Average = 0;
int32_t Phase3_Average = 0;

int32_t Phase1_Total = 0;
int32_t Phase2_Total = 0;
int32_t Phase3_Total = 0;

int32_t Phase1_Total_x256 = 0;
int32_t Phase2_Total_x256 = 0;
int32_t Phase3_Total_x256 = 0;

int32_t Phase1_Average_x256 = 0;
int32_t Phase2_Average_x256 = 0;
int32_t Phase3_Average_x256 = 0;

int32_t Phase1_Offset = 0;
int32_t Phase2_Offset = 0;
int32_t Phase3_Offset = 0;

int32_t Phase1_Offset_Save = 0;
int32_t Phase2_Offset_Save = 0;
int32_t Phase3_Offset_Save = 0;

int32_t Previous_Phase1_Offset = 0;
int32_t Previous_Phase2_Offset = 0;
int32_t Previous_Phase3_Offset = 0;

int32_t Phase1 = 0;
int32_t Phase2 = 0;
int32_t Phase3 = 0;

int32_t Previous_Phase1 = 0;
int32_t Previous_Phase2 = 0;
int32_t Previous_Phase3 = 0;

int32_t Phase1_Save = 0;
int32_t Phase2_Save = 0;
int32_t Phase3_Save = 0;

uint32_t REF_Edge_Count = 0;
uint32_t MEAS1_Edge_Count = 0;
uint32_t MEAS2_Edge_Count = 0;
uint32_t MEAS3_Edge_Count = 0;

uint32_t REF_Edge_Positions[500];
uint32_t MEAS1_Edge_Positions[500];
uint32_t MEAS2_Edge_Positions[500];
uint32_t MEAS3_Edge_Positions[500];

uint32_t REF_Period_Average = 0;
uint32_t MEAS1_Period_Average = 0;
uint32_t MEAS2_Period_Average = 0;
uint32_t MEAS3_Period_Average = 0;

uint32_t REF_Period_Average_x1024 = 0;
uint32_t MEAS1_Period_Average_x1024 = 0;
uint32_t MEAS2_Period_Average_x1024 = 0;
uint32_t MEAS3_Period_Average_x1024 = 0;

uint32_t REF_Period_Total = 0;
uint32_t MEAS1_Period_Total = 0;
uint32_t MEAS2_Period_Total = 0;
uint32_t MEAS3_Period_Total = 0;

int32_t REF_to_MEAS1_Average = 0;
int32_t REF_to_MEAS2_Average = 0;
int32_t REF_to_MEAS3_Average = 0;

uint32_t REF_NoChange = 0;
uint32_t MEAS1_NoChange = 0;
uint32_t MEAS2_NoChange = 0;
uint32_t MEAS3_NoChange = 0;

uint32_t Waveform_Length = 1000;

volatile uint32_t No_REF = 0;
volatile uint32_t No_MEAS1 = 0;
volatile uint32_t No_MEAS2 = 0;
volatile uint32_t No_MEAS3 = 0;

#define Normal_Phase 0x1
#define DISP_Dec_Phase_Drop 0x2
#define DISP_Dec_Phase_NC 0x4
#define DISP_Dec_Phase_Jump 0x8
#define DISP_NC_Phase_Drop 0x10
#define DISP_NC_Phase_NC 0x20
#define DISP_NC_Phase_Jump 0x40
#define DISP_Inc_Phase_Drop 0x80
#define DISP_Inc_Phase_NC 0x100
#define DISP_Inc_Phase_Jump 0x200
#define Phase_Invalid 0x400

uint32_t Phase_State = Normal_Phase;

void REF_Sync_Assembly_Block() __attribute__((optimize("-O0")));

void setup()
{
  pinMode(0, INPUT_PULLUP);  // REF input
  pinMode(1, INPUT_PULLUP);  // Hom 1A wired to REF
  pinMode(2, INPUT_PULLUP);  // Hom 1B wired to MEAS1
  pinMode(9, INPUT_PULLUP);  // MEAS1 input
  pinMode(10, INPUT_PULLUP); // MEAS2 input
  pinMode(14, INPUT_PULLUP); // MEAS3 input
  pinMode(6, INPUT);         // REF   interpolation input
  pinMode(8, INPUT);         // MEAS1 interpolation input
  pinMode(11, INPUT);        // MEAS2 interpolation input
  pinMode(12, INPUT);        // MEAS3 interpolation input

  // Turn off unused LEDs for Homodyne channel 3 IFF no line receivers present.

  pinMode(5, OUTPUT);        // Hom 3A
  digitalWrite (5, HIGH);    // Turn off LED for Hom 3A
  pinMode(7, OUTPUT);        // Hom 3B
  digitalWrite (7, HIGH);    // Turn off LED for Hom 3B

  // Turn off unused LEDs for MEAS2 and MEAS3 (and Homodyne channel 2) if no line receivers present.
  if (Heterodyne == 1)
  {
    pinMode(3, OUTPUT);      // Hom 2A wired to MEAS2
    digitalWrite (3, HIGH);  // Turn off LED for Hom 2A
    pinMode(4, OUTPUT);      // Hom 2B wired to MEAS3
    digitalWrite (4, HIGH);  // Turn off LED for Hom 2B
  }

  pinMode(13, OUTPUT);       // Status LED

  Scale_Shift = Multiplier >> 1;

  Serial.begin(2000000);

#ifdef OLED
  // Initialize and clear display
  u8x8.begin();
  u8x8.setPowerSave(0);

  // Banner and sequence number display
  u8x8.setFont(u8x8_font_chroma48medium8_r);  // Default font (thin)
  u8x8.drawString(0, 0, " -  MD2 V     -");
  u8x8.drawTile(3, 0, 1, tiles);              // Needed for tail of micro symbol

  sprintf(buffer, "%.2f", FirmwareFloat / 100);
  u8x8.drawString(9, 0, buffer);

  for (j = 0; j < 16; j++)
    {
      oled_buffer[j] = 0;
      old_oled_buffer[j] = 0;
      REF_oled_buffer[j] = 0;
      old_REF_oled_buffer[j] = 0;      
      DISP1_oled_buffer[j] = 0;
      old_DISP1_oled_buffer[j] = 0;
      DISP2_oled_buffer[j] = 0;
      old_DISP2_oled_buffer[j] = 0;
      DISP3_oled_buffer[j] = 0;
      old_DISP3_oled_buffer[j] = 0;
      sn_oled_buffer[j] = 0;
      old_sn_oled_buffer[j] = 0;
    }


#ifdef OLED_REF
  // Initialize REF frequency line

  if (OLED_REF == 1)
    {
      u8x8.drawString(0, 3, "REF: 0.0000 MHz");     // Default font
    }

  if (OLED_REF == 2)
    {
       u8x8.setFont(u8x8_font_8x13_1x2_f);         // Medium font
       u8x8.drawString(0, 6, "   REF (MHz)   ");
       u8x8.setFont(u8x8_font_chroma48medium8_r);  // Default font
    }

#endif
#endif

#ifdef OLED_Displacement

  u8x8.drawString(0, 2, "Seq#:        ");

#if OLED_REF != 2

  u8x8.drawString(0, 4, "Disp1:");

  if (Heterodyne > 1)
  {
    u8x8.drawString(0, 5, "Disp2:");
    u8x8.drawString(0, 6, "Disp3:");
  }

#endif
#endif

  QuadTimer_Setup ();                    // Initialize counters for REF, MEAS1, MEAS2, MEAS3

  usbTimer.begin(USBSender, 1000);       // Send USB data every 1000 microseconds
  usbTimer.priority(200);                // Lower numbers are higher priority, with 0 the highest and 255 the lowest. Most other interrupts default to 128

  // Clear junk from waveform buffers.
  for (j = 0; j < Waveform_Length; j++)
  {
    REF_Waveform[j] = 0;
    MEAS1_Waveform[j] = 0;
    MEAS2_Waveform[j] = 0;
    MEAS3_Waveform[j] = 0;
  }

  // Reset counters again.  Only should need to do LSBs.
  TMR2->CH[0].CNTR = 0;   // set REF count to 0
  TMR4->CH[0].CNTR = 0;   // set MEAS1 count to 0
  TMR1->CH[0].CNTR = 0;   // set MEAS2 count to 0
  TMR3->CH[0].CNTR = 0;   // set MEAS3 count to 0
}

void USBSender()
{
  // Sync on REF counter input being low in preparation for assembly sync on rising edge
  for (j = 0; j < 100; j++) // Loop until REF clock is high
  {
    if ((IMXRT_GPIO6_DIRECT & 0x400) != 0)
    {
      j = 100;
    }
  }

  for (j = 0; j < 100; j++) // Loop until REF clock goes low and capture REF counter value
  {
    if ((IMXRT_GPIO6_DIRECT & 0x400) == 0)
    {
      j = 100;
    }
  }

  REF_Save = TMR2->CH[0].CNTR; // This will be REF before incrementing
  
  REF_Sync_Assembly_Block (); // Sync to REF clock rising edge and capture GPIO waveforms

  // Load counter values

  counter_REF   =  TMR2->CH[3].HOLD;
  counter_REF   =  counter_REF * 65536  + TMR2->CH[2].HOLD;
  counter_REF   =  counter_REF * 65536  + TMR2->CH[1].HOLD;
  counter_REF   =  counter_REF * 65536  + TMR2->CH[0].HOLD;

  counter_MEAS1 =  TMR4->CH[3].HOLD;
  counter_MEAS1 =  counter_MEAS1 * 65536  + TMR4->CH[2].HOLD;
  counter_MEAS1 =  counter_MEAS1 * 65536  + TMR4->CH[1].HOLD;
  counter_MEAS1 =  counter_MEAS1 * 65536  + TMR4->CH[0].HOLD;

  counter_MEAS2 =  TMR1->CH[3].HOLD;
  counter_MEAS2 =  counter_MEAS2 * 65536  + TMR1->CH[2].HOLD;
  counter_MEAS2 =  counter_MEAS2 * 65536  + TMR1->CH[1].HOLD;
  counter_MEAS2 =  counter_MEAS2 * 65536  + TMR1->CH[0].HOLD;

  counter_MEAS3 =  TMR3->CH[3].HOLD;
  counter_MEAS3 =  counter_MEAS3 * 65536  + TMR3->CH[2].HOLD;
  counter_MEAS3 =  counter_MEAS3 * 65536  + TMR3->CH[1].HOLD;
  counter_MEAS3 =  counter_MEAS3 * 65536  + TMR3->CH[0].HOLD;

  REF   = (counter_REF   - counter_REF_save)   >> Scale_Shift;
  MEAS1 = (counter_MEAS1 - counter_MEAS1_save) >> Scale_Shift;
  MEAS2 = (counter_MEAS2 - counter_MEAS2_save) >> Scale_Shift;
  MEAS3 = (counter_MEAS3 - counter_MEAS3_save) >> Scale_Shift;

  // Update values

  counter_REF_save = counter_REF;
  counter_MEAS1_save = counter_MEAS1;
  counter_MEAS2_save = counter_MEAS2;
  counter_MEAS3_save = counter_MEAS3;

  previous_displacement1 = displacement1;
  previous_displacement2 = displacement2;
  previous_displacement3 = displacement3;

  displacement1 = (counter_MEAS1 - counter_REF);
  displacement2 = (counter_MEAS2 - counter_REF);
  displacement3 = (counter_MEAS3 - counter_REF);

  velocity1 = displacement1 - previous_displacement1;
  velocity2 = displacement2 - previous_displacement2;
  velocity3 = displacement3 - previous_displacement3;

#ifdef Interpolation

  // Test for REF activity
  if (No_REF != 0x12345) // REF OK)
    {
      // Extract bits corresponding to REF, MEAS1, MEAS2, MEAS3 to separate waveform arrays for convenience and possibly speed.
      REF_Offset = 4; // 31
      MEAS1_Offset = 0; // 0

      for (j = 0; j < Waveform_Length; j++)
      {
        data = gpioData[j]; // copy 32 bit value
        REF_Waveform[j + REF_Offset] = (data >> 10) & 0x1;   // REF is IO register bit D6
        MEAS1_Waveform[j + MEAS1_Offset] = (data >> 16) & 0x1; // MEAS1 is IO register bit D8;
        MEAS2_Waveform[j] = (data >> 2) & 0x1;  // MEAS2 is IO register bit D11;
        MEAS3_Waveform[j] = (data >> 1) & 0x1;  // MEAS3 is IO register bit D12;
      }

      // Initial version of interpolation will work with rising edge counting ONLY

      // Locate and store REF and MEAS edge positions for this sample
      REF_Edge_Count = 0;
      MEAS1_Edge_Count = 0;
      MEAS2_Edge_Count = 0;
      MEAS3_Edge_Count = 0;

      for (j = 1; j < Waveform_Length; j++)
        {
          if ((REF_Waveform[j] - REF_Waveform[j - 1]) == 1)  // Look for rising edges on REF waveform
            {
              REF_Edge_Positions[REF_Edge_Count] = j;        // Store position of found REF edge
              REF_Edge_Count++;                              // Increment REF edge count
            }

          if ((MEAS1_Waveform[j] - MEAS1_Waveform[j - 1]) == 1)
            {
              MEAS1_Edge_Positions[MEAS1_Edge_Count] = j; // Store position of found MEAS1 edge
              MEAS1_Edge_Count++; // Increment MEAS1 edge count
            }
       }

    // Force # REF edges to equal # MEAS edges
      if (REF_Edge_Count > MEAS1_Edge_Count) REF_Edge_Count = MEAS1_Edge_Count;
      if (REF_Edge_Count < MEAS1_Edge_Count) MEAS1_Edge_Count = REF_Edge_Count;

    // Calculate average REF period.  Difference between first and last REF edge divided by # REF edges
      if (REF_Edge_Count >= 4)
        {
          REF_Period_Total = REF_Edge_Positions[REF_Edge_Count - 1] - REF_Edge_Positions[0];
          REF_Period_Average_x1024 = ((REF_Period_Total << 10) + 512) / REF_Edge_Count; // x1024 to maintain precision without FP
          REF_Period_Average = REF_Period_Average_x1024 >> 10;
        }

    // Calculate average MEAS1 period.  Difference between first and last MEAS1 edge divided by # MEAS1 edges
      if (MEAS1_Edge_Count >= 4)
        {
          MEAS1_Period_Total = MEAS1_Edge_Positions[MEAS1_Edge_Count - 1] - MEAS1_Edge_Positions[0];
          MEAS1_Period_Average_x1024 = ((MEAS1_Period_Total << 10) + 512) / MEAS1_Edge_Count; // x1024 to maintain precision without FP
          MEAS1_Period_Average = MEAS1_Period_Average_x1024 >> 10;
        }

    // Initialize sampling variables for averaging

    // THE Phase 1 calculation. :-)
      Previous_Phase1_Offset = Phase1_Offset;
      Phase1_Total = 0;
      for (j = 1; j < MEAS1_Edge_Count; j++)
        {
          Phase1_Total += (MEAS1_Edge_Positions[j] - REF_Edge_Positions[j]);  // Sum of phase values
        }
      Phase1_Total_x256 = (Phase1_Total << 8);                              // Sum of phase values * 256
      Phase1_Average_x256 = (Phase1_Total_x256 + (MEAS1_Edge_Count >> 1)) / MEAS1_Edge_Count;     // Average of phase values * 256
      Phase1_Offset = (Phase1_Average_x256 + (MEAS1_Period_Average >> 1)) / MEAS1_Period_Average;                // Average of phase values range ~0 to 255 (might go slightly below 0)

    // Boundary correction kludges
    // When DISP decrements, Phase should become -128
    // When DISP increments, Phase should become +127

     Phase1_Offset_Save =  Phase1_Offset;

     Phase_State = Normal_Phase;

  //      digitalWrite (13, 1);

  switch (Phase_State) {

    // Normal behavior between boundaries.  Note: Relationship of DISP and Phase seems backwards but
    // that is how it works so be it and I'm not changing the GUI. ;-)

    case Normal_Phase:
      //                 ________    ________
      //  DISP  ________|                    |________
      //                 ________    ________
      //  Phase ________|                    |________
      //            DISP Inc             DISP Dec

      if (displacement1 > previous_displacement1)      // Did DISP change?
      {                                                // Yes, it incremented
        if (Phase1_Offset < 127)                       // Is phase low?
        { // Yes
          //                _________
          //  DISP  _______|
          //                  _______
          //  Phase _________|
          //                ^

          Phase1_Offset = 255;                         // Set to 255
          Phase_State = DISP_Inc_Phase_Drop;           // DISP inc but no Phase jump
        }
      }

      else if (displacement1 < previous_displacement1) // Did DISP change?
      { // Yes, it decremented
        if (Phase1_Offset > 127)                       // Is phase high?
        { // Yes
          //        ________
          //  DISP          |______
          //        __________
          //  Phase           |________
          //                 ^

          Phase1_Offset = 0;                           // Set to 0
          Phase_State = DISP_Dec_Phase_Jump;
        }
      }
      /*
         else if (previous_displacement1 == displacement1)
          {
            if (Phase1_Offset - Previous_Phase1_Offset > 127)          // Did Phase jump?
        //        _______ __ ______
        //  DISP  _______|__|______
        //                 ________
        //  Phase ________|
        //                 ^
              {                                                        // Yes, near DISP change
                Phase1_Offset = 0;
                Phase_State = DISP_NC_Phase_Jump;
              }

            else if ((Phase1_Offset - Previous_Phase1_Offset) < -127)  // Did Phase drop?
        //        _______ __ ______
        //  DISP  _______|__|______
        //        ________
        //  Phase         |________
        //                 ^
              {                                              // Yes, near DISP change
                Phase1_Offset = 255;
                Phase_State = DISP_NC_Phase_Drop;
              }
           }


          case DISP_Dec_Phase_Drop:                          // Back to normal
          Phase_State = Normal_Phase;
          break;

          case DISP_Dec_Phase_NC:
          break;

          case DISP_Dec_Phase_Jump:
          break;

          case DISP_NC_Phase_Drop:
          break;

          case DISP_NC_Phase_NC:
          break;

          case DISP_NC_Phase_Jump:
          break;

          case DISP_Inc_Phase_Drop: // DISP increment, and Phase is low, set to 255

          if (displacement1 < previous_displacement1)        // Did DISP decrement?
            {                                                // Yes
              if (Phase1_Offset < 127)                       // Is phase now low?
                {                                            // Yes
                  Phase_State = Normal_Phase;                // Return to normal
                }
              else
                {                                            // No
                  Phase_State = DISP_NC_Phase_NC;            //
                }
            }
          else if (displacement1 > previous_displacement1)   // Did DISP increment?
            {                                                // Yes
              Phase_State = Phase_Invalid;                   // Phase changing too quickly
            }

          else if (displacement1 == previous_displacement1)  // Is DISP unchanged?
            {                                                // Yes
              if (Phase1_Offset > 127)                       // Is phase now high?
                {                                            // Yes
                  Phase_State = DISP_NC_Phase_Jump;
                }
              else
                {                                            // No, Phase is low
                  Phase_State = DISP_Inc_Phase_Drop;         // State unchanged
                }
          break;

          case DISP_Inc_Phase_NC:                                 // No change
          break;

          case DISP_Inc_Phase_Jump:                               // Return to normal
          Phase_State = Normal_Phase;
          break;
      */
  }
  

    Phase1 = (Phase1_Offset & 0xff) - 128;
    Phase1_Save = (Phase1_Offset_Save & 0xff) - 128;
}
#endif

  sequenceNumber++;
  sn = sequenceNumber;

  if ((sn & 0x40) == 0x40) digitalWrite (13, 1);
  else digitalWrite (13, 0);

#ifdef OLED_REF

noInterrupts();

  REF_Frequency_Total += REF;
  REF_Frequency_Total_Count++;

interrupts();

#endif

  // Set up appropriate low speed data
  LowSpeedCode = 0;                  // Default to no low speed data
  LowSpeedData = 0;
  LowSpeedCodeSelect = sequenceNumber & 0x1f;

  if (LowSpeedCodeSelect == 1)       // Send firmware version
  {
    LowSpeedCode = 10;
    LowSpeedData = FirmwareVersion;
  }

  else if (LowSpeedCodeSelect == 2)  // Sammple frequency x 100
  {
    LowSpeedCode = 8;
    LowSpeedData = 100000;
  }

  else if (LowSpeedCodeSelect == 3) // Tell GUI this is not a homodyne system
  {
    LowSpeedCode = 20;
    LowSpeedData = (Multiplier * 256) + 0; // Tell GUI this is not a homodyne system; Multiplier must be present or GUI NaN.
  }

  else if (LowSpeedCodeSelect == 4) // # CPU clocks spent in capture and analysis
  {
    LowSpeedCode = 121;
    LowSpeedData = 1;
  }

  else if (LowSpeedCodeSelect == 5) // # CPU clocks spent in communications
  {
    LowSpeedCode = 122;
    LowSpeedData = 1;                // Timer1USBCounts not implemented yet
  }

  else if (LowSpeedCodeSelect == 6) // # CPU clocks spent in communications
  {
    LowSpeedCode = 15;
    LowSpeedData = gpioData[0];                // GPIO
  }

  // Low speed codes 101, 102, 111, 112 available for diagnostics
  else if (LowSpeedCodeSelect == 7) // REF period average and REF to MEAS1 average values
  {
    LowSpeedCode = 101;
    LowSpeedData = (REF_Period_Average & 0xff) + ((Phase1 & 0xff) << 8);
    //    LowSpeedData = (REF_Period_Average & 0xff) + ((REF_to_MEAS1_Average & 0xff) << 8);
  }

  else if (LowSpeedCodeSelect == 8) // REF period average and REF to MEAS1 average values
  {
    LowSpeedCode = 111;
    LowSpeedData = (MEAS1_Period_Average & 0xff) + ((Phase1 & 0xff) << 8);
  }

  if (Serial.availableForWrite() > 256)
  {
    Serial.printf("%li ", REF);                    // 1: REF
    Serial.printf("%li ", MEAS1);                  // 2: MEAS1
    Serial.printf("%li ", displacement1);          // 3: Displacement 1
    //      Serial.printf("%li ", velocity1);              // 4: Velocity Count 1
    Serial.printf("%li ", No_REF); // Phase1_Save);            // 4: Diagnostic Phase1 without correction
    Serial.printf("%li ", Phase1);                 // 5: Phase 1
    Serial.printf("%llu ", sequenceNumber);        // 6: Sequence Number
    Serial.printf("%li ", LowSpeedCode);           // 7: LowSpeedCode
    Serial.printf("%li", LowSpeedData);            // 8: LowSpeedData

    if (Heterodyne > 1)
    {
      Serial.printf(" %li ", MEAS2);              // 9: MEAS2
      Serial.printf("%li ", displacement2);       // 10: Displacement 2
      Serial.printf("%li ", velocity2);           // 11: Velocity Count 2
      Serial.printf("%li ", Phase2);              // 5: Phase 2
      Serial.printf("%li ", MEAS3);               // 13: MEAS3
      Serial.printf("%li ", displacement3);       // 14: Displacement 3
      Serial.printf("%li ", velocity3);           // 15: Velocity Count 3
      Serial.printf("%li ", Phase3);              // 16: Phase 3
    }
    Serial.println();
  }
}

void loop()
{
#ifdef OLED
#ifdef OLED_REF
  noInterrupts();
  REF_Frequency = REF_Frequency_Total;
  REF_Frequency_Average = REF_Frequency / REF_Frequency_Total_Count;
  REF_Frequency_Total_Count = 0;
  REF_Frequency_Total = 0;
  interrupts();
   
  if (REF_Frequency_Average != OLED_REF_Frequency_save)
    {
      OLED_REF_Frequency_save = REF_Frequency_Average;

      if (OLED_REF == 1)                                      // Default font
        {
          if (REF_Frequency_Average < 10000) // Need extra digit?
            {
              sprintf(REF_oled_buffer, "%.4f", REF_Frequency_Average / 1000);
            }
          else 
            {
              sprintf(REF_oled_buffer, "%.3f", REF_Frequency_Average / 1000);
            }
          update_OLED_X1 (5, 3, 6, REF_oled_buffer, old_REF_oled_buffer);
        }

      if (OLED_REF == 2)
        {
          u8x8.setFont(u8x8_font_courB18_2x3_n);             // Large font
          if (REF_Frequency_Average < 10000) // Need extra digit?
            {
              sprintf(REF_oled_buffer, "%.6f", REF_Frequency_Average / 1000);
            }
          else 
            {
              sprintf(REF_oled_buffer, "%.5f", REF_Frequency_Average / 1000);
            }
          update_OLED_X2 (0, 3, 8, REF_oled_buffer, old_REF_oled_buffer);
          u8x8.setFont(u8x8_font_chroma48medium8_r);         // Restore normal font
        }
   }
#endif

#ifdef OLED_Displacement

  sprintf(sn_oled_buffer, "%li", sn);
  update_OLED_X1 (6, 2, 8, sn_oled_buffer, old_sn_oled_buffer);

#if OLED_REF != 2

  OLED_DISP1 = displacement1;
  if (OLED_DISP1 != OLED_DISP1_save)
    {
      OLED_DISP1_save = OLED_DISP1;
      sprintf(DISP1_oled_buffer, "%li", OLED_DISP1);
      update_OLED_X1 (7, 4, 8, DISP1_oled_buffer, old_DISP1_oled_buffer);
    }

  if (Heterodyne > 1)
  {
    OLED_DISP2 = displacement2;
    if (OLED_DISP2 != OLED_DISP2_save)
      {
        OLED_DISP2_save = OLED_DISP2;
        sprintf(DISP2_oled_buffer, "%li", OLED_DISP2);
        update_OLED_X1 (7, 5, 8, DISP2_oled_buffer, old_DISP2_oled_buffer);
      }

    OLED_DISP3 = displacement3;
    if (OLED_DISP3 != OLED_DISP3_save)
      {
        OLED_DISP3_save = OLED_DISP3;
        sprintf(DISP3_oled_buffer, "%li", OLED_DISP3);
        update_OLED_X1 (7, 6, 8, DISP3_oled_buffer, old_DISP3_oled_buffer);
      }
  }
#endif
#endif
#endif

  delay(100);
}

void update_OLED_X1 (unsigned int xpos, unsigned int ypos, unsigned int slength, char *oled_buffer, char *old_oled_buffer)
{
  uint32_t index = 0;
  for (index = 0; index < slength; index++) // Only update numbers that changed
    {
      if (oled_buffer[index] != old_oled_buffer[index])
        {
          old_oled_buffer[index] = oled_buffer[index];
          u8x8.setCursor(xpos + index, ypos);
          u8x8.print(oled_buffer[index]);
        }                     
    }
}

void update_OLED_X2 (unsigned int xpos, unsigned int ypos, unsigned int slength, char *oled_buffer, char *old_oled_buffer)
{
  uint32_t index = 0;
  for (index = 0; index < slength; index++) // Only update numbers that changed
    {
      if (oled_buffer[index] != old_oled_buffer[index])
        {
          old_oled_buffer[index] = oled_buffer[index];
          u8x8.setCursor(xpos + 2 * index, ypos);
          u8x8.print(oled_buffer[index]);
        }
    }
}

void xbar_connect(unsigned int input, unsigned int output)
{
  // function to make setting the crossbar SEL fields easier; 2 of these SEL fields are fit into a 32 register; there are many of these fields....
  if (input >= 88) return;
  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}

void QuadTimer_Setup () {
  // QuadTimer setup - DO NOT MESS WITH THIS!!!

  // The IOMUX is also used to configure other pin characteristics, such as voltage level, drive strength, and hysteresis. These may not be set optimally. More experimentation / real world data is necessary

  // set up QuadTimer1: MEAS2 on D10
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);                // enable QTMR1 clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 1;                       // QuadTimer1 Counter 0 on pin D10 using ALT1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 |= 0b1000000000000000;     // enable hysteresis in pin D10
  TMR1->CH[0].CTRL = 0;                                       // stop
  TMR1->CH[1].CTRL = 0;                                       // stop
  TMR1->CH[2].CTRL = 0;                                       // stop
  TMR1->CH[3].CTRL = 0;                                       // stop
  TMR1->CH[0].CNTR = 0;                                       // set count to 0
  TMR1->CH[1].CNTR = 0;                                       // set count to 0
  TMR1->CH[2].CNTR = 0;                                       // set count to 0
  TMR1->CH[3].CNTR = 0;                                       // set count to 0
  TMR1->CH[0].LOAD = 0;
  TMR1->CH[1].LOAD = 0;
  TMR1->CH[2].LOAD = 0;
  TMR1->CH[3].LOAD = 0;
  TMR1->CH[0].SCTRL = TMR1->CH[0].CSCTRL = 0;
  TMR1->CH[1].SCTRL = TMR1->CH[1].CSCTRL = 0;
  TMR1->CH[2].SCTRL = TMR1->CH[2].CSCTRL = 0;
  TMR1->CH[3].SCTRL = TMR1->CH[3].CSCTRL = 0;
  TMR1->CH[0].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR1->CH[1].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR1->CH[2].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR1->CH[3].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR1->CH[0].CMPLD1 =  0xffff;
  TMR1->CH[1].CMPLD1 =  0xffff;
  TMR1->CH[2].CMPLD1 =  0xffff;
  TMR1->CH[3].CMPLD1 =  0xffff;
  TMR1->CH[3].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR1->CH[3].CTRL |= TMR_CTRL_PCS(6);                       // Primary Count Source: CH[2] output
  TMR1->CH[2].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR1->CH[2].CTRL |= TMR_CTRL_PCS(5);                       // Primary Count Source: CH[1] output
  TMR1->CH[1].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR1->CH[1].CTRL |= TMR_CTRL_PCS(4);                       // Primary Count Source: CH[0] output
  TMR1->CH[0].CTRL  = TMR_CTRL_CM (Multiplier);              // Count Mode: Count rising edges or both edges of primary source
  TMR1->CH[0].CTRL |= TMR_CTRL_PCS(0);                       // Primary Count Source: Counter 0 input pin

  // set up QuadTimer2: REF on D0
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);                 // turn clock on for XBAR1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 1;                   // IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 (pin 0) to ALT1 mux port: XBAR1_INOUT17
  IOMUXC_XBAR1_IN17_SELECT_INPUT = 1 ;                       // XBAR1_INOUT17 has several inputs to choose from. Pick IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03
  IOMUXC_GPR_GPR6 |= 0b0000000010000;                        // connect XBAR as input for QTIMER2_TIMER0
  xbar_connect(17, XBARA1_OUT_QTIMER2_TIMER0);               // connect XBAR1_INOUT17 to XBARA1_OUT_QTIMER2_TIMER0
  CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);               // enable QTMR2 clock
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 |= 0b1000000000000000; // enable hysteresis in pin D0
  TMR2->CH[0].CTRL = 0;                                      // stop
  TMR2->CH[0].SCTRL = TMR2->CH[0].CSCTRL = 0;
  TMR2->CH[1].CTRL = 0;                                      // stop
  TMR2->CH[2].CTRL = 0;                                      // stop
  TMR2->CH[3].CTRL = 0;                                      // stop
  TMR2->CH[0].CNTR = 0;                                      // set count to 0
  TMR2->CH[1].CNTR = 0;                                      // set count to 0
  TMR2->CH[2].CNTR = 0;                                      // set count to 0
  TMR2->CH[3].CNTR = 0;                                      // set count to 0
  TMR2->CH[0].LOAD = 0;
  TMR2->CH[1].LOAD = 0;
  TMR2->CH[2].LOAD = 0;
  TMR2->CH[3].LOAD = 0;
  TMR2->CH[0].SCTRL = TMR2->CH[0].CSCTRL = 0;
  TMR2->CH[1].SCTRL = TMR2->CH[1].CSCTRL = 0;
  TMR2->CH[2].SCTRL = TMR2->CH[2].CSCTRL = 0;
  TMR2->CH[3].SCTRL = TMR2->CH[3].CSCTRL = 0;
  TMR2->CH[0].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR2->CH[1].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR2->CH[2].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR2->CH[3].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR2->CH[0].CMPLD1 =  0xffff;
  TMR2->CH[1].CMPLD1 =  0xffff;
  TMR2->CH[2].CMPLD1 =  0xffff;
  TMR2->CH[3].CMPLD1 =  0xffff;
  TMR2->CH[3].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR2->CH[3].CTRL |= TMR_CTRL_PCS(6);                       // Primary Count Source: CH[2] output
  TMR2->CH[2].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR2->CH[2].CTRL |= TMR_CTRL_PCS(5);                       // Primary Count Source: CH[1] output
  TMR2->CH[1].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR2->CH[1].CTRL |= TMR_CTRL_PCS(4);                       // Primary Count Source: CH[0] output
  TMR2->CH[0].CTRL  = TMR_CTRL_CM (Multiplier);              // Count Mode: Count rising edges or both edges of primary source
  TMR2->CH[0].CTRL |= TMR_CTRL_PCS(0);                       // Primary Count Source: Counter 0 input pin

  // set up QuadTimer3: MEAS3 on D14
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);               // enable QTMR3 clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 1;                   // Daisy Chain 1 - QT3 Counter 2 conects to pin D14 ALT1
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT  = 1 ;                  // Daisy Chain 2 - QT3 Counter 2 conects to pin D14 ALT1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 |= 0b1000000000000000; // enable hysteresis in pin D14
  TMR3->CH[0].CTRL = 0;                                      // stop
  TMR3->CH[2].SCTRL = TMR3->CH[2].CSCTRL = 0;
  TMR3->CH[1].CTRL = 0;                                      // stop
  TMR3->CH[2].CTRL = 0;                                      // stop
  TMR3->CH[3].CTRL = 0;                                      // stop
  TMR3->CH[0].CNTR = 0;                                      // set count to 0
  TMR3->CH[1].CNTR = 0;                                      // set count to 0
  TMR3->CH[2].CNTR = 0;                                      // set count to 0
  TMR3->CH[3].CNTR = 0;                                      // set count to 0
  TMR3->CH[0].LOAD = 0;
  TMR3->CH[1].LOAD = 0;
  TMR3->CH[2].LOAD = 0;
  TMR3->CH[3].LOAD = 0;
  TMR3->CH[0].SCTRL = TMR3->CH[0].CSCTRL = 0;
  TMR3->CH[1].SCTRL = TMR3->CH[1].CSCTRL = 0;
  TMR3->CH[2].SCTRL = TMR3->CH[2].CSCTRL = 0;
  TMR3->CH[3].SCTRL = TMR3->CH[3].CSCTRL = 0;
  TMR3->CH[0].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR3->CH[1].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR3->CH[2].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR3->CH[3].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR3->CH[0].CMPLD1 =  0xffff;
  TMR3->CH[1].CMPLD1 =  0xffff;
  TMR3->CH[2].CMPLD1 =  0xffff;
  TMR3->CH[3].CMPLD1 =  0xffff;
  TMR3->CH[3].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR3->CH[3].CTRL |= TMR_CTRL_PCS(6);                       // Primary Count Source: CH[2] output
  TMR3->CH[2].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR3->CH[2].CTRL |= TMR_CTRL_PCS(5);                       // Primary Count Source: CH[1] output
  TMR3->CH[1].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR3->CH[1].CTRL |= TMR_CTRL_PCS(4);                       // Primary Count Source: CH[0] output
  TMR3->CH[0].CTRL  = TMR_CTRL_CM (Multiplier);              // Count Mode: Count rising edges or both edges of primary source
  TMR3->CH[0].CTRL |= TMR_CTRL_PCS(2);                       // Primary Count Source: Counter 2 input pin

  // set up QuadTimer4: MEAS1 on D9
  CCM_CCGR6 |= CCM_CCGR6_QTIMER4(CCM_CCGR_ON);               // enable QTMR4 clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 1;                      // QuadTimerT4 Counter 2 on pin D9
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 |= 0b1000000000000000;    // enable hysteresis in pin D9
  TMR4->CH[0].CTRL = 0;                                      // stop
  TMR4->CH[1].CTRL = 0;                                      // stop
  TMR4->CH[2].CTRL = 0;                                      // stop
  TMR4->CH[3].CTRL = 0;                                      // stop
  TMR4->CH[0].CNTR = 0;                                      // set count to 0
  TMR4->CH[1].CNTR = 0;                                      // set count to 0
  TMR4->CH[2].CNTR = 0;                                      // set count to 0
  TMR4->CH[3].CNTR = 0;                                      // set count to 0
  TMR4->CH[0].LOAD = 0;
  TMR4->CH[1].LOAD = 0;
  TMR4->CH[2].LOAD = 0;
  TMR4->CH[3].LOAD = 0;
  TMR4->CH[0].SCTRL = TMR4->CH[0].CSCTRL = 0;
  TMR4->CH[1].SCTRL = TMR4->CH[1].CSCTRL = 0;
  TMR4->CH[2].SCTRL = TMR4->CH[2].CSCTRL = 0;
  TMR4->CH[3].SCTRL = TMR4->CH[3].CSCTRL = 0;
  TMR4->CH[0].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR4->CH[1].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR4->CH[2].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR4->CH[3].COMP1 =  0xffff;                               // send count signal to next counter on overflow at 0xffff
  TMR4->CH[0].CMPLD1 =  0xffff;
  TMR4->CH[1].CMPLD1 =  0xffff;
  TMR4->CH[2].CMPLD1 =  0xffff;
  TMR4->CH[3].CMPLD1 =  0xffff;
  TMR4->CH[3].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR4->CH[3].CTRL |= TMR_CTRL_PCS(6);                       // Primary Count Source: CH[2] output
  TMR4->CH[2].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR4->CH[2].CTRL |= TMR_CTRL_PCS(5);                       // Primary Count Source: CH[1] output
  TMR4->CH[1].CTRL  = TMR_CTRL_CM (7);                       // Count Mode:           Cascaded counter mode
  TMR4->CH[1].CTRL |= TMR_CTRL_PCS(4);                       // Primary Count Source: CH[0] output
  TMR4->CH[0].CTRL  = TMR_CTRL_CM (Multiplier);              // Count Mode: Count rising edges or both edges of primary source
  TMR4->CH[0].CTRL |= TMR_CTRL_PCS(2);                       // Primary Count Source: Counter 2 input pin
}

void REF_Sync_Assembly_Block () {

  asm volatile("ldr    r0, =0x401dc00a \n\t" // load address of MEAS2 (TMR1_CNTR0) into r0
               "ldr    r1, =0x401e000a \n\t" // load address of REF (TMR2_CNTR0) into r1
               "ldr    r2, =0x401e400a \n\t" // load address of MEAS3 (TMR3_CNTR0) into r2
               "ldr    r3, =0x401e800a \n\t" // load address of MEAS1 (TMR4_CNTR0) into r3

               "ldr    r8, =0x42004008 \n\t" // load address of GPIO6_PSR into r8
               "mov    r9, %1          \n\t" // copy address of array into r9, index = 0
               "mov    r10, r9         \n\t" // copy r9 to r10 to start on end-of-loop-condition
               "add    r10, #3996      \n\t" // end-of-loop condition. we want Waveform_Length 4 byte values, so the end is at 4000 - 4 = 3996 bytes after the beginning of the array

               "mov    r12, #0          \n\t" // reset error code to 0
               "str    r12, %0          \n\t" //
               "mov    r12, #0x400      \n\t" // load bit to test in r5
               "mov    r11, #100        \n\t" // load loop count in r4
               
#ifndef REF_Sync
               "b REFEdgeFound1         \n\t" // Do NOT sync on REF edges//
#endif

              // Sync with rising edge on REF clock (Timer3, GPIO bit 0x400)

    "REFSyncLoop:                      \n\t" //
               "ldr    r6, [r8]        \n\t" // Load value of GPIO_PSR into r6
               "tst    r6, r12          \n\t" // Check GPIO_PSR REF counter clock bit
               "bne    REFEdgeFound1   \n\t" // End if REF clock high
               "subs   r11, r11, #1      \n\t" //
               "bne    REFSyncLoop     \n\t" // Do it again if loop count not 0

    "NoREFEdgeFound1:                  \n\t" // No high REF clock detected
               "ldrh   r12, [r1], #0    \n\t" // hold TMR2 by reading TMR2_CNTR0 (REF)
               "ldrh   r12, [r3], #0    \n\t" // hold TMR4 by reading TMR4_CNTR0 (MEAS1)
               "movw   r12, #12345      \n\t" // load error code - 12345 for now
               "str    r12, %0          \n\t"
               "b      Abort           \n\t" // Skip gpioData capture if no REFs
                       
    "REFEdgeFound1:                    \n\t" // Breakout
               "ldrh   r5, [r1], #0    \n\t" // hold TMR2 by reading TMR2_CNTR0 (REF)
               "ldrh   r5, [r3], #0    \n\t" // hold TMR4 by reading TMR4_CNTR0 (MEAS1)
               
  // read GPIO directly for interpolation calculation

    "nextdata:                        \n\t"  //
               "ldr    r3, [r8]       \n\t"  // load value of GPIO6_PSR into r3
               "str    r3, [r9], #4   \n\t"  // store value into gpioDataArray and then add 4 bytes to the index
               "cmp    r9, r10        \n\t"  // check loop counter against loop limit
               "ble    nextdata       \n\t"  // loop if limit not reached

    "Abort:                           \n\t"  //  
               "nop                   \n\t"  //

               : "=m" (No_REF)         // output operand list
               : "r" (gpioData)              // input operand list
               : "r0", "r1", "r2", "r3", "r6", "r8", "r9", "r10", "r11", "r12"
              );
}
