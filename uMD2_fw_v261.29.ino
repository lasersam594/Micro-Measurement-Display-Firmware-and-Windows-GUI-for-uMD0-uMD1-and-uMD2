//********************************************************************************************************//
//         Micro Measurement Display 2 - uMD2 - Heterodyne interferometer firmware for Teensy 4.0         //
//                 Copyright® Jan Beck and Sam Goldwasser, 1994-2025, all rights reserved                 //
//********************************************************************************************************//

// V2.61.01 New branch.  First re-attempt at heuristic deglitching 18-Jan-2025.  Changed to numeric firmware
//          sub-versions instead of letters to display in "COM %" field of GUI without modifications. 
// V2.61.02 Now we start hacking the interpolation for real. ;-)
// V2.61.03 Built state transition table for interpolation.  Still buggy.
// V2.61.04 Reducing glitches.
// V2.61.05 Very close to being clean.  Added diagnostic serial monitor options.
// V2.61.06 Cleaned a couple more bogus state errors  At least one more to go.
// V2.61.07 Cleaned up state transition 25.
// V2.61.08 Interpolation appears to be 99.99999% glitch-free but still some noise at kludge points, probably
//           nothing can be done to remedy that.  This version works with and without interpolation.
// V2.61.09 Aborted - crashes.
// V2.61.10 First test of three axis interpolation, at least to compile and not crash or mess up axis 1.
// V2.61.11 First to have QuadTimer_Setup.cpp .
// V2.61.12 Skip.
// V2.61.13 Skip superstition. ;-)
// V2.61.14 First to have Next_State.cpp.  Seems OK, usually. ;-( ;-)
// V2.61.15 Stopped working. ;-(
// V2.61.16 Skipped accidentally.
// V2.61.17 Cleaned up a bit.
// V2.61.18 Cleaned up a bit more.
// V2.61.19 Reverted to single sketch file.  Still may be an unitialized variable.  Added LED Axis blink code. ;-)
// V2.61.20 Test of single axis interpolation using floating point.
// V2.61.21 Update Axes 2 and 3 interpolation using floating point.  NOT TESTED.
// V2.61.22 Moved Phase(1,2,3) calculation to Phase_Execute().
// V2.61.23 Minor cleanup.
// V261.24  Added 16 bit Phase default and new firmware version format XXX.YY for use with 29-Jan-2025 uMD GUI or later.
// V261.25  Cleanup
// V261.26  Aborted
// V261.27  Upgraded to aborted V261.26.  Cleaned up 8 and 16 bit Phase calculations.
// V261.28  Converted uMD GUI to default to 16 bit Phase to minimize possibility of large offset at startup.

// Version with three axis interpolation capability.  Only Axis 1 has been fully tested.  Interpolation
// provides sub-nm-scale resolution and precision (accuracy subject to enbironmental compensation) for slew rates of
// around 0.01 mm/second or less and normal 79 or 158 nm resolution (depending on interferometer optics) above that.

#define Diagnostics 0           // 0 normal uMD2.
                                // 1 for diagnostics inside uMD GUI but display may be strange with some data in axes 2 and 3 fields. ;-)
                                // 2 for extended printout using Serial Monitor or terminal program but incompatible with uMD GUI.

#define Heterodyne 1            // Set to the number of axes for heterodyne, sample rate always 1 kHz.

#define Interpolation 1         // O for none, 1 for enabled.
#define Old_uMD_GUI 0           // Set to 0 for 16 bit Interpolation with 2025+ GUI, 1 for 8 bit with all GUIs.

#define OLED 1                  // OLED display of banner and version if defined
#define OLED_Displacement 1     // OLED display of SN, Disp1, Disp2, Disp3 if defined
#define OLED_REF 2              // REF frequency display if defined; Value is text size.  1 (default) and 2 (large) valid.

// No user or hacker sericeable parts beyond this point. ;-)

#define FirmwareVersion 261     // Firmware version used by OLED and GUI About.
#define FirmwareSubVersion 29   // Now numeric in XXX.YY format.
#define FirmwareVersion_Flag 4  // Set to 4 to tell 2025+ GUI to use new firmware Version/Subversion format for About.

#define Multiplier 2            // Integer for REF/MEAS1/MEAS2/MEAS3 counts/cycle.  1 for rising edge; 2 for double clocking.  Used for TMR_CTRL_CM value.  Do NOT mess with that!

#define Phase_8bit_Flag 2       // Tells 2025+ GUI to use 8 bit Phase values if Old_uMD_GUI set to 1, 16 bit Phase is the default.  There is no good reason to use 8 bit with the 2025+ GUI.
#define REF_Sync 1              // May be required for interpolation

#ifdef OLED
#include <U8x8lib.h> // Graphics library available though the Library Manager
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
#endif

float FirmwareFloat = FirmwareVersion;
uint OLEDSubVersion = FirmwareSubVersion;

char buffer[100];
int32_t Scale_Shift   = 0;

// DON'T mess with these!
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

int32_t displacement1 = 0;
int32_t displacement2 = 0;
int32_t displacement3 = 0;

int32_t previous_displacement1 = 0;
int32_t previous_displacement2 = 0;
int32_t previous_displacement3 = 0;

int32_t Displacement_Sent = 0;
int32_t Displacement1_Sent = 0;
int32_t Displacement2_Sent = 0;
int32_t Displacement3_Sent = 0;

int32_t velocity1     = 0;
int32_t velocity2     = 0;
int32_t velocity3     = 0;

uint64_t sequenceNumber = 0;
uint32_t sn             = 0;
uint32_t old_sn = 0;

int32_t LowSpeedCode = 0;
int32_t LowSpeedData = 0;
int32_t LowSpeedCodeSelect = 0;

uint32_t j = 0;

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

uint32_t MEAS_Valid = 0;
uint32_t MEAS1_Valid = 0;
uint32_t MEAS2_Valid = 0;
uint32_t MEAS3_Valid = 0;

int32_t REF_Offset = 0;
int32_t MEAS1_Offset = 0;
int32_t MEAS2_Offset = 0;
int32_t MEAS3_Offset = 0;

// Variables for Phase1 calculation

double REF_Period_Total_float = 0;
double REF_Edge_Count_Adj_float = 0;
double REF_Period_Average_float = 0;

double MEAS_Period_Total_float = 0;
double MEAS_Edge_Count_Adj_float = 0;
double MEAS_Period_Average_float = 0;
double Phase_Average_float = 0;
double Phase_Total_float = 0;

int32_t Phase_Average = 0;
int32_t Phase_Total = 0;

int32_t Phase_Offset = 0;

int32_t Phase = 0;
int32_t Phase1 = 0;
int32_t Phase2 = 0;
int32_t Phase3 = 0;

int32_t Phase_8bit = 0;
int32_t Phase1_8bit = 0;
int32_t Phase2_8bit = 0;
int32_t Phase3_8bit = 0;

int32_t Phase1_Sent = 0;
int32_t Phase2_Sent = 0;
int32_t Phase3_Sent = 0;

int32_t Previous_Phase = 0;
int32_t Previous_Phase1 = 0;
int32_t Previous_Phase2 = 0;
int32_t Previous_Phase3 = 0;

uint32_t REF_Edge_Count = 0;
uint32_t MEAS1_Edge_Count = 0;
uint32_t MEAS2_Edge_Count = 0;
uint32_t MEAS3_Edge_Count = 0;

uint32_t REF_Edge_Count_Adj = 0;
uint32_t MEAS_Edge_Count_Adj = 0;

uint32_t REF_Edge_Positions[500];
uint32_t MEAS1_Edge_Positions[500];
uint32_t MEAS2_Edge_Positions[500];
uint32_t MEAS3_Edge_Positions[500];

uint32_t REF_Period_Average = 0;
uint32_t MEAS_Period_Average = 0;
uint32_t MEAS1_Period_Average = 0;

uint32_t REF_Period_Total = 0;
uint32_t MEAS_Period_Total = 0;

uint32_t REF_NoChange = 0;
uint32_t Waveform_Length = 1000;

volatile uint32_t No_REF = 0x12345; // Will be changed by any REF activity in the assembly block.  This is probably silly but it doesn't seem to hurt. ;-)
 
int Exit = 0;
int Test_Flags = 0;

/* DISP=Displacement Count, Phase=Interpolated Value
   NC=No Change, DEC=DECrement, INC=INCrement
   SC=Small Change of less than +/-63, Drop=Decline by more than 127, Jump=Increase by more than 127 */

#define Normal_State_0  0  // DISP NC, Phase SC, DISP DEC, Phase Drop, DISP INC, Phase Jump.

#define Bogus_State_1  1   // DISP NC.  Phase Jump.
#define Bogus_State_2  2   // DISP NC.  Phase Drop. 
#define Bogus_State_3  3   // DISP DEC. Phase SC. 
#define Bogus_State_4  4   // DISP INC. Phase SC. 
#define Bogus_State_5  5   // DISP DEC. Phase Jump.  Error.
#define Bogus_State_6  6   // DISP INC. Phase Drop.  Error.
#define Bogus_State_7  7   // Reserved

#define Error_State_15 15  // Transitions that are generally not possible even with jittery REF and MEAS edges.

uint32_t Phase_State  = Normal_State_0;
uint32_t Phase_State1 = Normal_State_0;
uint32_t Phase_State2 = Normal_State_0;
uint32_t Phase_State3 = Normal_State_0;

void REF_Sync_Assembly_Block() __attribute__((optimize("-O0")));

void setup() {

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
  if (Heterodyne == 1) {
    pinMode(3, OUTPUT);      // Hom 2A wired to MEAS2
    digitalWrite (3, HIGH);  // Turn off LED for Hom 2A
    pinMode(4, OUTPUT);      // Hom 2B wired to MEAS3
    digitalWrite (4, HIGH);  // Turn off LED for Hom 2B
  }

  pinMode(LED_BUILTIN, OUTPUT);  // Status LED

  Scale_Shift = Multiplier >> 1;

  Serial.begin(2000000);

#ifdef OLED
  // Initialize and clear display
  u8x8.begin();
  u8x8.setPowerSave(0);

  // Banner and sequence number display
  u8x8.setFont(u8x8_font_chroma48medium8_r);  // Default font (thin)
  u8x8.drawString(0, 0, "   MD2 V      ");
  u8x8.drawTile(2, 0, 1, tiles);              // Needed for tail of micro symbol
  sprintf(buffer, "%3d", FirmwareVersion);
  u8x8.drawString(8, 0, buffer);
  u8x8.drawString(11, 0, ".");
  sprintf(buffer, "%02d", OLEDSubVersion );
  u8x8.drawString(12, 0, buffer);

  for (j = 0; j < 16; j++) {
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

  if (OLED_REF == 1) {
    u8x8.drawString(0, 3, "REF: 0.0000 MHz");     // Default font
  }

  if (OLED_REF == 2) {
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

  if (Heterodyne > 1) {
    u8x8.drawString(0, 5, "Disp2:");
    u8x8.drawString(0, 6, "Disp3:");
  }

 #endif
#endif

  QuadTimer_Setup ();                    // Initialize counters for REF, MEAS1, MEAS2, MEAS3

  usbTimer.begin(USBSender, 1000);       // Send USB data every 1000 microseconds
  usbTimer.priority(200);                // Lower numbers are higher priority, with 0 the highest and 255 the lowest. Most other interrupts default to 128

  // Clear junk from waveform buffers.
  for (j = 0; j < Waveform_Length; j++) {
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
  // Sync on REF counter input being low in preparation for assembly sync on rising edge.  This may be redundant.
  for (j = 0; j < 100; j++) {  // Loop until REF clock is high
    if ((IMXRT_GPIO6_DIRECT & 0x400) != 0) {
      j = 100;
    }
  }

  for (j = 0; j < 100; j++) {  // Loop until REF clock goes low and capture REF counter value
    if ((IMXRT_GPIO6_DIRECT & 0x400) == 0) {
      j = 100;
    }
  }

  REF_Save = TMR2->CH[0].CNTR; // This will be REF before incrementing

  REF_Sync_Assembly_Block ();  // Sync to REF clock rising edge and capture GPIO waveforms if interpolation enabled

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

  REF   = (counter_REF   - counter_REF_save)   >> Scale_Shift; // Reduction of GUI frequency counter resolution if Multiplier set to 2 (Scale_Shift = 1).
  MEAS1 = (counter_MEAS1 - counter_MEAS1_save) >> Scale_Shift; // This does not affect displacement.
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

#if Interpolation == 0
  Displacement1_Sent = displacement1;
  Displacement2_Sent = displacement2;
  Displacement3_Sent = displacement3;
#endif
 
#if Interpolation == 1
Interpolation_Execute();
#endif

  sequenceNumber++;
  sn = sequenceNumber;

// Numder of axes LED blink code
          if                      ((sn & 0x1F0) == 0x040)  digitalWrite (LED_BUILTIN, 1); // 1 Blink =  1 Axis
     else if ((Heterodyne > 1) && ((sn & 0x1F0) == 0x0C0)) digitalWrite (LED_BUILTIN, 1); // 2 Blinks = 2 Axes
     else if ((Heterodyne > 2) && ((sn & 0x1F0) == 0x140)) digitalWrite (LED_BUILTIN, 1); // 3 Blinks = 3 Axes
     else                                                  digitalWrite (LED_BUILTIN, 0); // Black space

#ifdef OLED_REF
  noInterrupts();
  REF_Frequency_Total += REF;
  REF_Frequency_Total_Count++;
  interrupts();
#endif

  // Set up appropriate low speed data
  LowSpeedCode = 0;                        // Default to no low speed data
  LowSpeedData = 0;
  LowSpeedCodeSelect = sequenceNumber & 0x1f;

  if (LowSpeedCodeSelect == 1) {           // Send firmware version
    LowSpeedCode = 10;
    LowSpeedData = FirmwareVersion;
#if Old_uMD_GUI == 0
    LowSpeedData += (FirmwareSubVersion << 16);
#endif
  }

  else if (LowSpeedCodeSelect == 2) {      // Sammple frequency x 100
    LowSpeedCode = 8;
    LowSpeedData = 100000;
  }

  else if (LowSpeedCodeSelect == 3) {      // Tell GUI this is not a homodyne system
    LowSpeedCode = 20;
    LowSpeedData = (Multiplier * 256);     // Tell GUI this is not a homodyne system; Multiplier must be present or GUI NaN.
  }

  else if (LowSpeedCodeSelect == 4) {      // Was # CPU clocks spent in capture and analysis, not implemented.
    LowSpeedCode = 121;
    LowSpeedData = 0;
  }

  else if (LowSpeedCodeSelect == 5) {      // Firmware sub-version hack (Was # CPU clocks spent in communications)
    LowSpeedCode = 122;
    LowSpeedData = 0;
  }

  else if (LowSpeedCodeSelect == 6) {      // # CPU clocks spent in communications
    LowSpeedCode = 15;
    LowSpeedData = gpioData[0];            // GPIO
  }

  // Low speed codes 101, 102, 111, 112 available for diagnostics
  else if (LowSpeedCodeSelect == 7) {      // REF period average and REF to MEAS1 average values
    LowSpeedCode = 101;
    LowSpeedData = (REF_Period_Average & 0xff) + ((Phase1 & 0xff) << 8);
  }

  else if (LowSpeedCodeSelect == 8) {      // REF period average and REF to MEAS1 average values
    LowSpeedCode = 111;
    LowSpeedData = (MEAS1_Period_Average & 0xff) + ((Phase1 & 0xff) << 8);
  }

  else if (LowSpeedCodeSelect == 10) {     // Firmware_Flags for 2025 and beyond uMD GUI, ignored by previous versions
    LowSpeedCode = 112;
#if (Old_uMD_GUI == 1)
    LowSpeedData = Phase_8bit_Flag;        // Tells the 2025+ GUI to use 8 bit Phase.
#endif
    LowSpeedData |= FirmwareVersion_Flag;  // Tells the GUI to use decimal Firmware Subversion
  }

#if Old_uMD_GUI == 1                       // Use 8 bit Phase if it is selected or using old uMD GUI
      Phase1_Sent = Phase1_8bit;
      Phase2_Sent = Phase2_8bit;
      Phase3_Sent = Phase3_8bit;
#else
      Phase1_Sent = Phase1;
      Phase2_Sent = Phase2;
      Phase3_Sent = Phase3;
#endif

#if Diagnostics == 0  // Normal uMD GUI
  if (Serial.availableForWrite() > 256) {
    Serial.printf("%li ", REF);                      // 1: REF
    Serial.printf("%li ", MEAS1);                    // 2: MEAS1
    Serial.printf("%li ", Displacement1_Sent);       // 3: Displacement 1
    Serial.printf("%li ", velocity1);                // 4: Velocity Count 1
    Serial.printf("%li ", Phase1_Sent);              // 5: Phase 1
    Serial.printf("%llu ", sequenceNumber);          // 6: Sequence Number
    Serial.printf("%li ", LowSpeedCode);             // 7: LowSpeedCode
    Serial.printf("%li", LowSpeedData);              // 8: LowSpeedData

    if (Heterodyne > 1) {
      Serial.printf(" %li ", MEAS2);                 // 9: MEAS2
      Serial.printf("%li ", Displacement2_Sent);     // 10: Displacement 2
      Serial.printf("%li ", velocity2);              // 11: Velocity Count 2
      Serial.printf("%li ", Phase2_Sent);            // 5: Phase 2
      Serial.printf("%li ", MEAS2);                  // 13: MEAS3
      Serial.printf("%li ", Displacement3_Sent);     // 14: Displacement 3
      Serial.printf("%li ", velocity3);              // 15: Velocity Count 3
      Serial.printf("%li", Phase3_Sent);             // 16: Phase 3
    }
    Serial.println();
  }
#endif

#if Diagnostics == 1  // Interpolation diagnostic values within uMD GUI
  if (Serial.availableForWrite() > 256) {
    Serial.printf("%li ", REF);                      // 1: REF
    Serial.printf("%li ", MEAS1);                    // 2: MEAS1
    Serial.printf("%li ", Displacement1_Sent);       // 3: Displacement Sent 1
    Serial.printf("%li ", displacement1);            // 4: Raw displacement  // Velocity Count 1
    Serial.printf("%li ", Phase1_Sent);              // 5: Phase 1
    Serial.printf("%llu ", sequenceNumber);          // 6: Sequence Number
    Serial.printf("%li ", LowSpeedCode);             // 7: LowSpeedCode
    Serial.printf("%li", LowSpeedData);              // 8: LowSpeedData

int zero = 0;
    if (Heterodyne > 1) {
      Serial.printf(" %li " , Exit);                 // State exit identifier  // MEAS2
      Serial.printf("%li ", Phase_State1);           // State flags            // Velocity2
      Serial.printf("%li ", Phase2_Sent);            // Phase2);        // 5: Phase 2
      Serial.printf("%li ", zero);                   // velocity2);     // 11: Velocity Count 2
      Serial.printf("%li ", zero);                   // MEAS3);         // 13: MEAS3
      Serial.printf("%li ", zero);                   // displacement3); // 14: Displacement 3
      Serial.printf("%li ", zero);                   // velocity3);     // 15: Velocity Count 3
      Serial.printf("%li", zero);                    // Phase3);        // 16: Phase 3
    }
    Serial.println();
  }
#endif

#if Diagnostics == 2 // Interpolation diagnostic values NOT compatible with uMD GUI, only serial monitor
  if (Serial.availableForWrite() > 256) {
    Serial.printf("SN: ");
    Serial.printf("%7llu ", sequenceNumber);         // Sequence Number
    Serial.printf(" DISP Raw: " );
    Serial.printf("%2li ", displacement1);           // displacement 1
    Serial.printf(" Sent: ");
    Serial.printf("%2li ", Displacement1_Sent);      // Displacement 1 Sent
    Serial.printf(" Phase: ");
    Serial.printf("%4li ", Phase1_Sent);             // Phase 1
    Serial.printf(" Exit: ");
    Serial.printf("%3li " , Exit);                   // State exit identifier   Serial.printf(" State: ");  
    Serial.printf("Goto: %2li ", Phase_State1);      // State flags
    Serial.printf(" Flags: D");
    if ((Test_Flags & 1) != 0)  Serial.printf(" ");  // DISP and Phase change flags
    if ((Test_Flags & 2) != 0)  Serial.printf("-");
    if ((Test_Flags & 4) != 0)  Serial.printf("+");
    Serial.printf(" P");
    if ((Test_Flags & 8) != 0)  Serial.printf(" ");
    if ((Test_Flags & 16) != 0) Serial.printf("-");
    if ((Test_Flags & 32) != 0) Serial.printf("+");
    Serial.printf(" DS");
    Serial.printf(" %4li ", Displacement1_Sent);
    Serial.printf(" P");
    Serial.printf(" %4li", Phase1);

    if (Heterodyne > 1) {
      Serial.printf(" %li ", MEAS2);                 // 9: MEAS2
      Serial.printf("%li ", displacement2);          // 10: Displacement 2
      Serial.printf("%li ", velocity2);              // 11: Velocity Count 2
      Serial.printf("%li ", Phase2_Sent);            // 5: Phase 2
      Serial.printf("%li ", MEAS3);                  // 13: MEAS3
      Serial.printf("%li ", displacement3);          // 14: Displacement 3
      Serial.printf("%li ", velocity3);              // 15: Velocity Count 3
      Serial.printf("%li", Phase3_Sent);             // 16: Phase 3
    }
    Serial.println();
  }
#endif
}

void loop() { // Only for OLED
  OLED_DISPLAY();
  delay(100);
}

#if Interpolation == 1
void Interpolation_Execute() {
   
  // Test for REF activity
  if (No_REF != 0x12345) { // REF OK)

    // Extract bits corresponding to REF, MEAS1, MEAS2, MEAS3 to separate waveform arrays for convenience and possibly speed.
    REF_Offset = 4; // 1-6 appear to work; 4 is default. 
    MEAS1_Offset = 0; // Only 0 appears to work.
    MEAS2_Offset = 0;
    MEAS3_Offset = 0;

    for (j = 0; j < Waveform_Length; j++) {
      data = gpioData[j]; // copy 32 bit value
      REF_Waveform[j + REF_Offset] = (data >> 10) & 0x1;     // REF is IO register bit D6
      MEAS1_Waveform[j + MEAS1_Offset] = (data >> 16) & 0x1; // MEAS1 is IO register bit D8;
      MEAS2_Waveform[j + MEAS2_Offset] = (data >> 2) & 0x1;  // MEAS2 is IO register bit D11;
      MEAS3_Waveform[j + MEAS3_Offset] = (data >> 1) & 0x1;  // MEAS3 is IO register bit D12;
    }

    // Locate and store REF and MEAS edge positions for this sample
    REF_Edge_Count = 0;
    MEAS1_Edge_Count = 0;
    MEAS2_Edge_Count = 0;
    MEAS3_Edge_Count = 0;

#if Multiplier == 1 // Look for rising edges and save positions
    for (j = 1; j < Waveform_Length; j++) {
      if ((REF_Waveform[j] - REF_Waveform[j - 1]) == 1) {      // Look for rising edges on REF waveform
        REF_Edge_Positions[REF_Edge_Count] = j;                // Store position of found REF edge
        REF_Edge_Count++;                                      // Increment REF edge count
      }

      if ((MEAS1_Waveform[j] - MEAS1_Waveform[j - 1]) == 1) {  // Look for rising edges on MEAS1 waveform
        MEAS1_Edge_Positions[MEAS1_Edge_Count] = j;            // Store position of found MEAS1 edge
        MEAS1_Edge_Count++;                                    // Increment MEAS1 edge count
      }

  #if Heterodyne > 1
      if ((MEAS2_Waveform[j] - MEAS2_Waveform[j - 1]) == 1) {  // Look for rising edges on MEAS2 waveform
        MEAS2_Edge_Positions[MEAS2_Edge_Count] = j;            // Store position of found MEAS2 edge
        MEAS2_Edge_Count++;                                    // Increment MEAS2 edge count
      }
  #endif

  #if Heterodyne > 2
      if ((MEAS3_Waveform[j] - MEAS3_Waveform[j - 1]) == 1) {  // Look for rising edges on MEAS3 waveform
        MEAS3_Edge_Positions[MEAS3_Edge_Count] = j;            // Store position of found MEAS3 edge
        MEAS3_Edge_Count++;                                    // Increment MEAS3 edge count
      }
  #endif
  }
#endif

#if Multiplier == 2 // Look for rising and falling edges and store positions
    for (j = 1; j < Waveform_Length; j++) {
      if (((REF_Waveform[j] - REF_Waveform[j - 1]) == 1) ||  ((REF_Waveform[j-1] - REF_Waveform[j]) == 1)) {          // Look for rising or falling edges on REF waveform
        REF_Edge_Positions[REF_Edge_Count] = j;                // Store position of found REF edge
        REF_Edge_Count++;                                      // Increment REF edge count
      }

      if (((MEAS1_Waveform[j] - MEAS1_Waveform[j - 1]) == 1) ||  ((MEAS1_Waveform[j-1] - MEAS1_Waveform[j]) == 1)) {  // Look for rising or falling edges on MEAS1 waveform
        MEAS1_Edge_Positions[MEAS1_Edge_Count] = j;            // Store position of found MEAS1 edge
        MEAS1_Edge_Count++;                                    // Increment MEAS1 edge count
      }

  #if Heterodyne > 1
      if (((MEAS2_Waveform[j] - MEAS2_Waveform[j - 1]) == 1) ||  ((MEAS2_Waveform[j-1] - MEAS2_Waveform[j]) == 1)) {  // Look for rising or falling edges on MEAS1 waveform
        MEAS2_Edge_Positions[MEAS2_Edge_Count] = j;            // Store position of found MEAS1 edge
        MEAS2_Edge_Count++;                                    // Increment MEAS1 edge count
      }
  #endif

  #if Heterodyne > 2
      if (((MEAS3_Waveform[j] - MEAS3_Waveform[j - 1]) == 1) ||  ((MEAS3_Waveform[j-1] - MEAS3_Waveform[j]) == 1)) {  // Look for rising or falling edges on MEAS1 waveform
        MEAS3_Edge_Positions[MEAS3_Edge_Count] = j;            // Store position of found MEAS1 edge
        MEAS3_Edge_Count++;                                    // Increment MEAS1 edge count
      }
  #endif  
    }
#endif

    // Axis 1 Phase calculation only if change in displacement is small
    if (abs(displacement1 - previous_displacement1) <= 1) {
      Previous_Phase1 = Phase1;
      Phase_Execute (REF_Edge_Count, MEAS1_Edge_Count, MEAS1_Edge_Positions);
      Phase1 = Phase;
      Phase1_8bit = Phase_8bit;
   
      Phase_State = Phase_State1;
      Next_State(displacement1, previous_displacement1, Phase1, Previous_Phase1);
      Phase_State1 = Phase_State;
 
      Displacement1_Sent = Displacement_Sent;
    }
    else {
      Displacement1_Sent = displacement1;
      Phase1 = 0;
      Phase1_8bit = 0;
    }
    
#if Heterodyne > 1
    // Axis 2 Phase calculation only if change in displacement is small
    if (abs(displacement2 - previous_displacement2) <= 1) {
      Previous_Phase2 = Phase2;
      Phase_Execute (REF_Edge_Count, MEAS2_Edge_Count, MEAS2_Edge_Positions);
      Phase2 = Phase;   
      Phase2_8bit = Phase_8bit;

      Phase_State = Phase_State2;
      Next_State(displacement2, previous_displacement2, Phase2, Previous_Phase2);
      Phase_State2 = Phase_State;
 
      Displacement2_Sent = Displacement_Sent;
    }
    else {
      Displacement2_Sent = displacement2;
      Phase2 = 0;
      Phase2_8bit = 0;
    }
#endif

#if Heterodyne > 2
    // Axis 3 Phase calculation only if change in displacement is small
    if (abs(displacement3 - previous_displacement3) <= 1) {
      Previous_Phase3 = Phase3;
      Phase_Execute (REF_Edge_Count, MEAS3_Edge_Count, MEAS3_Edge_Positions);
      Phase3 = Phase;  
      Phase3_8bit = Phase_8bit;

      Phase_State = Phase_State3;
      Next_State(displacement3, previous_displacement3, Phase3, Previous_Phase3);
      Phase_State3 = Phase_State;
 
      Displacement3_Sent = Displacement_Sent;
    else {
      Displacement3_Sent = displacement3;
      Phase3 = 0;
      Phase3_8bit = 0;
    }
#endif
  }
}
#endif

void OLED_DISPLAY() {
#ifdef OLED
 #ifdef OLED_REF
  noInterrupts();
  REF_Frequency = REF_Frequency_Total;
  REF_Frequency_Average = REF_Frequency / REF_Frequency_Total_Count;
  REF_Frequency_Total_Count = 0;
  REF_Frequency_Total = 0;
  interrupts();
   
  if (REF_Frequency_Average != OLED_REF_Frequency_save) {
      OLED_REF_Frequency_save = REF_Frequency_Average;
      if (OLED_REF == 1) {                        // Default font
          if (REF_Frequency_Average < 10000) {    // Need extra digit?
              sprintf(REF_oled_buffer, "%.4f", REF_Frequency_Average / 1000);
            }
          else  {
              sprintf(REF_oled_buffer, "%.3f", REF_Frequency_Average / 1000);
            }
          update_OLED_X1 (5, 3, 6, REF_oled_buffer, old_REF_oled_buffer);
        }

      if (OLED_REF == 2) {
          u8x8.setFont(u8x8_font_courB18_2x3_n);  // Large font
          if (REF_Frequency_Average < 10000) {    // Need extra digit?
              sprintf(REF_oled_buffer, "%.6f", REF_Frequency_Average / 1000);
            }
          else {
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
  if (OLED_DISP1 != OLED_DISP1_save) {
      OLED_DISP1_save = OLED_DISP1;
      sprintf(DISP1_oled_buffer, "%li", OLED_DISP1);
      update_OLED_X1 (7, 4, 8, DISP1_oled_buffer, old_DISP1_oled_buffer);
    }

  if (Heterodyne > 1) {
    OLED_DISP2 = displacement2;
    if (OLED_DISP2 != OLED_DISP2_save) {
        OLED_DISP2_save = OLED_DISP2;
        sprintf(DISP2_oled_buffer, "%li", OLED_DISP2);
        update_OLED_X1 (7, 5, 8, DISP2_oled_buffer, old_DISP2_oled_buffer);
      }

    OLED_DISP3 = displacement3;
    if (OLED_DISP3 != OLED_DISP3_save) {
        OLED_DISP3_save = OLED_DISP3;
        sprintf(DISP3_oled_buffer, "%li", OLED_DISP3);
        update_OLED_X1 (7, 6, 8, DISP3_oled_buffer, old_DISP3_oled_buffer);
      }
  }
  #endif
 #endif
#endif
}

void update_OLED_X1 (unsigned int xpos, unsigned int ypos, unsigned int slength, char *oled_buffer, char *old_oled_buffer) {
  uint32_t index = 0;
  for (index = 0; index < slength; index++) { // Only update numbers that changed
    if (oled_buffer[index] != old_oled_buffer[index]) {
      old_oled_buffer[index] = oled_buffer[index];
      u8x8.setCursor(xpos + index, ypos);
      u8x8.print(oled_buffer[index]);
    }                     
  }
}

void update_OLED_X2 (unsigned int xpos, unsigned int ypos, unsigned int slength, char *oled_buffer, char *old_oled_buffer) {
  uint32_t index = 0;
  for (index = 0; index < slength; index++) { // Only update numbers that changed
    if (oled_buffer[index] != old_oled_buffer[index]) {
      old_oled_buffer[index] = oled_buffer[index];
      u8x8.setCursor(xpos + 2 * index, ypos);
      u8x8.print(oled_buffer[index]);
    }
  }
}

// ************************************************************************************************ //
//         DO NOT EVEN THINK ABOUT MESSING WITH CODE BEYOND THIS POINT OF NO RETURN. ;-( ;-)        //
//      IF THE UNIVERSE GETS SUCKED INTO A BLACK HOLE, IT WILL BE YOUR FAULT! YOU WERE WARNED!      //
// ************************************************************************************************ //

void Phase_Execute(uint32_t REF_Edge_Count, uint32_t MEAS_Edge_Count, uint32_t MEAS_Edge_Positions[]) {

   if ((REF_Edge_Count >= 4) && (MEAS_Edge_Count >= 4)) {

      // Force # REF edges to equal # MEAS edges
      if (REF_Edge_Count > MEAS_Edge_Count) { REF_Edge_Count_Adj = MEAS_Edge_Count; MEAS_Edge_Count_Adj = MEAS_Edge_Count; }
      if (REF_Edge_Count < MEAS_Edge_Count) { MEAS_Edge_Count_Adj = REF_Edge_Count; REF_Edge_Count_Adj = REF_Edge_Count; }

        // Calculate average REF period.  Difference between first and last REF edge divided by # REF edges.  Redundant for Axes 2 and 3 but so be it. ;-)
        REF_Period_Total = REF_Edge_Positions[REF_Edge_Count_Adj - 1] - REF_Edge_Positions[0];
        REF_Period_Total_float = REF_Period_Total;
        REF_Edge_Count_Adj_float = REF_Edge_Count_Adj;
        REF_Period_Average_float = REF_Period_Total_float / REF_Edge_Count_Adj_float;      // Using floating point
        REF_Period_Average = REF_Period_Average_float;

      // Calculate average MEAS1 period.  Difference between first and last MEAS1 edge divided by # MEAS1 edges
        MEAS_Period_Total = MEAS_Edge_Positions[MEAS_Edge_Count_Adj - 1] - MEAS_Edge_Positions[0];
        MEAS_Period_Total_float = MEAS_Period_Total;
        MEAS_Edge_Count_Adj_float = MEAS_Edge_Count_Adj;
        MEAS_Period_Average_float = MEAS_Period_Total_float / MEAS_Edge_Count_Adj_float;   // Using floating point
        MEAS_Period_Average = MEAS_Period_Average_float;

      // THE Phase calculation. :-)
 
      Phase_Total = 0;
      for (j = 1; j < MEAS_Edge_Count_Adj; j++) {
        Phase_Total += (MEAS_Edge_Positions[j] - REF_Edge_Positions[j]);         // Sum of phase values
      }

      Phase_Total_float = Phase_Total;                                           // Sum of phase values * 256
      Phase_Average_float = (Phase_Total_float / MEAS_Edge_Count);               // Average of phase values * 256

      Phase_Offset = (Phase_Average_float * 65536) / MEAS_Period_Average_float;  // 16 bit Phase: Average of phase values range from ~0 to 65535 (might go slightly below).
      Phase = (Phase_Offset & 0xffff) - 32768;

      Phase_8bit = ((Phase_Offset / 256) & 0xff) - 128;                          // 8 bit Phase: Average of phase values range from ~0 to 255 (might go slightly below).
    }
  else {}; // Was used to clear Phase_State(1,2,3) if no activity but no udea why that might be useful. ;-)
}

// Test for change in Displacement and/or Phase and set flags

bool DISP_NC;
bool DISP_DEC;
bool DISP_INC;
bool Phase_SC;
bool Phase_Drop;
bool Phase_Jump;
bool DISP_LT_DEC;
bool DISP_GT_INC;

void Next_State(int displacement, int previous_displacement, int phase, int previous_phase) {

Test_Flags = 0;  // DISP and Phase change flags
DISP_NC = false;
DISP_DEC = false;
DISP_INC = false;
DISP_GT_INC = false;
DISP_LT_DEC = false;

       if ((displacement - previous_displacement) == -1) { DISP_DEC = true; Test_Flags |= 2; }
  else if ((displacement - previous_displacement) == 1)  { DISP_INC = true; Test_Flags |= 4; }
  else if ((displacement - previous_displacement) < -1)  { DISP_LT_DEC = true; Test_Flags |= 64; }
  else if ((displacement - previous_displacement) > 1)   { DISP_GT_INC = true; Test_Flags |= 128; }
  else                                                   { DISP_NC = true; Test_Flags |= 1; }

  Phase_SC = false;
  Phase_Drop = false;
  Phase_Jump = false;

#define Phase_16bit_Threshold 16384
#define Phase_8bit_Threshold 64

       if ((phase < -Phase_16bit_Threshold) && (previous_phase > Phase_16bit_Threshold)) { Phase_Drop = true; Test_Flags |= 16; }
  else if ((phase > Phase_16bit_Threshold) && (previous_phase < -Phase_16bit_Threshold)) { Phase_Jump = true; Test_Flags |= 32; }
  else                                                                                   { Phase_SC = true;   Test_Flags |= 8; }

// Determine next Phase_State and displacement adjustment based DISP and Phase flags.  DO NOT even think about messing with this! ;-)

  switch (Phase_State) {
  case Normal_State_0: // DISP_NC and Phase_SC. 
         if ((DISP_NC == true) && (Phase_SC == true))    { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 0;}       //
    else if ((DISP_NC == true) && (Phase_Drop == true))  { Phase_State = Bogus_State_2; Displacement_Sent = displacement - 1; Exit = 1; }   //
    else if ((DISP_NC == true) && (Phase_Jump == true))  { Phase_State = Bogus_State_1; Displacement_Sent = displacement + 1; Exit = 2; }   //

    else if ((DISP_DEC == true) && (Phase_SC == true))   { Phase_State = Bogus_State_3; Displacement_Sent = displacement + 1; Exit = 3; }   //    
    else if ((DISP_DEC == true) && (Phase_Drop == true)) { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 4; }      //
    else if ((DISP_DEC == true) && (Phase_Jump == true)) { Phase_State = Bogus_State_5; Displacement_Sent = displacement - 1; Exit = 5; }   //

    else if ((DISP_INC == true) && (Phase_SC == true))   { Phase_State = Bogus_State_4; Displacement_Sent = displacement - 1; Exit = 6; }   //
    else if ((DISP_INC == true) && (Phase_Drop == true)) { Phase_State = Bogus_State_6; Displacement_Sent = displacement; Exit = 7; }       //
    else if ((DISP_INC == true) && (Phase_Jump == true)) { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 8; }      //
    break;

  case Bogus_State_1: // DISP_NC and Phase_Jump 
         if ((DISP_NC == true) && (Phase_SC == true))    { Phase_State = Bogus_State_1; Displacement_Sent = displacement + 1; Exit = 10; }  //
    else if ((DISP_NC == true) && (Phase_Drop == true))  { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 11; }     //
    else if ((DISP_NC == true) && (Phase_Jump == true))  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 12; }     //

    else if ((DISP_DEC == true) && (Phase_SC == true))   { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 13; }     //    
    else if ((DISP_DEC == true) && (Phase_Drop == true)) { Phase_State = Bogus_State_3; Displacement_Sent = displacement - 1; Exit = 14; }  //    
    else if ((DISP_DEC == true) && (Phase_Jump == true)) { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 15; }     //

    else if ((DISP_INC == true) && (Phase_SC == true))   { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 16; }     //    
    else if ((DISP_INC == true) && (Phase_Drop == true)) { Phase_State = Bogus_State_4; Displacement_Sent = displacement - 1; Exit = 17; }  //    
    else if ((DISP_INC == true) && (Phase_Jump == true)) { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 18; }     //
    break;

  case Bogus_State_2: // DISP_NC and Phase_Drop
         if ((DISP_NC == true) && (Phase_SC == true))    { Phase_State = Bogus_State_2; Displacement_Sent = displacement - 1; Exit = 20; }  //
    else if ((DISP_NC == true) && (Phase_Drop == true))  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 21; }     //
    else if ((DISP_NC == true) && (Phase_Jump == true))  { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 22; }     //

    else if ((DISP_INC == true) && (Phase_SC == true))    { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 23; }    //
    else if ((DISP_INC == true) && (Phase_Drop == true))  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 24; }    //
    else if ((DISP_INC == true) && (Phase_Jump == true))  { Phase_State = Bogus_State_5; Displacement_Sent = displacement; Exit = 25; }     //

    else if ((DISP_DEC == true) && (Phase_SC == true))    { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 26; }    //
    else if ((DISP_DEC == true) && (Phase_Drop == true))  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 27; }    //
    else if ((DISP_DEC == true) && (Phase_Jump == true))  { Phase_State = Bogus_State_3; Displacement_Sent = displacement + 1; Exit = 28; } //

    else                                                  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 29; }    //
    break;

  case Bogus_State_3: // DISP_DEC and Phase_SC
         if ((DISP_NC == true) && (Phase_SC == true))    { Phase_State = Bogus_State_3; Displacement_Sent = displacement + 1; Exit = 30; }  //
    else if ((DISP_NC == true) && (Phase_Drop == true))  { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 31; }     //
    else if ((DISP_NC == true) && (Phase_Jump == true))  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 32; }     //

    else if ((DISP_INC == true) && (Phase_SC == true))    { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 33; }    //
    else if ((DISP_INC == true) && (Phase_Drop == true))  { Phase_State = Bogus_State_2; Displacement_Sent = displacement - 1; Exit = 34; } //
    else if ((DISP_INC == true) && (Phase_Jump == true))  { Phase_State = Bogus_State_1; Displacement_Sent = displacement + 1; Exit = 35; } //

    else                                                  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 36; }    //
    break;

  case Bogus_State_4: // DISP_INC and Phase_SC
         if ((DISP_NC == true) && (Phase_SC == true))    { Phase_State = Bogus_State_4; Displacement_Sent = displacement - 1; Exit = 40; }  //
    else if ((DISP_NC == true) && (Phase_Drop == true))  { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 41; }     //
    else if ((DISP_NC == true) && (Phase_Jump == true))  { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 42; }     //

    else if ((DISP_DEC == true) && (Phase_SC == true))   { Phase_State = Normal_State_0; Displacement_Sent = displacement; Exit = 43; }     //
    else if ((DISP_DEC == true) && (Phase_Drop == true)) { Phase_State = Error_State_15; Displacement_Sent = displacement; Exit = 44; }     //
    else if ((DISP_DEC == true) && (Phase_Jump == true)) { Phase_State = Bogus_State_1; Displacement_Sent = displacement + 1; Exit = 45; }  //
    break;

  case Bogus_State_5: // DISP_DEC and Phase_Jump
    Phase_State = Normal_State_0;
    Exit = 50;
    break;

  case Bogus_State_6: // DISP_INC and Phase_Drop
    Phase_State = Normal_State_0;
    Exit = 60;
    break;

  case Bogus_State_7:
    Phase_State = Normal_State_0;
    Exit = 70;
    break;

  case Error_State_15:
    Phase_State = Normal_State_0;
    Exit = 99;
    break;
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
  }
  else {
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

// Assembly code to capture REF, MEAS1, MEAS2, and MEAS3 waveforms as fast as possible.  DO NOT mess with this! ;-)
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

    "REFSyncLoop:                       \n\t" //
               "ldr    r6, [r8]         \n\t" // Load value of GPIO_PSR into r6
               "tst    r6, r12          \n\t" // Check GPIO_PSR REF counter clock bit
               "bne    REFEdgeFound1    \n\t" // End if REF clock high
               "subs   r11, r11, #1     \n\t" //
               "bne    REFSyncLoop      \n\t" // Do it again if loop count not 0

    "NoREFEdgeFound1:                   \n\t" // No high REF clock detected
               "ldrh   r12, [r1], #0    \n\t" // hold TMR2 by reading TMR2_CNTR0 (REF)
               "ldrh   r12, [r3], #0    \n\t" // hold TMR4 by reading TMR4_CNTR0 (MEAS1)
               "movw   r12, #12345      \n\t" // load error code - 12345 for now
               "str    r12, %0          \n\t"
               "b      Abort            \n\t" // Skip gpioData capture if no REFs
                       
    "REFEdgeFound1:                     \n\t" // Breakout
               "ldrh   r5, [r1], #0     \n\t" // hold TMR2 by reading TMR2_CNTR0 (REF)
               "ldrh   r5, [r3], #0     \n\t" // hold TMR4 by reading TMR4_CNTR0 (MEAS1)
               
  // read GPIO directly for interpolation calculation

    "nextdata:                          \n\t"  //
               "ldr    r3, [r8]         \n\t"  // load value of GPIO6_PSR into r3
               "str    r3, [r9], #4     \n\t"  // store value into gpioDataArray and then add 4 bytes to the index
               "cmp    r9, r10          \n\t"  // check loop counter against loop limit
               "ble    nextdata         \n\t"  // loop if limit not reached

    "Abort:                             \n\t"  //  
               "nop                     \n\t"  //

               : "=m" (No_REF)                 // output operand list
               : "r" (gpioData)                // input operand list
               : "r0", "r1", "r2", "r3", /* "r4", "r5",*/ "r6", "r8", "r9", "r10", "r11", "r12"
              );
}
