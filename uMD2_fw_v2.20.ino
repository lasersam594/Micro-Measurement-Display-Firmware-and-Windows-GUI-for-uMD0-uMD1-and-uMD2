//****************************************************************************************//
// Micro Measurement Display 2 - uMD2 - Heterodyne interferometer firmware for Teensy 4.0 //
//                        Jan Beck and Sam Goldwasser 2021                                //
//****************************************************************************************//

// V2.00 Formatted for GUI and OLED.
// V2.01 First version that talks to GUI. :)
// V2.02 Changed OLED to display counters XOR displacements.  Counts are screwy due to noise or crosstalk or....
// V2.04 First working heterodyne version.  Removed analogwrite stuff, fixed serial formatting.
// V2.05 Changed to double clocking, resolution 79 nm with PMI, Still slow drift (~2 um/min) if OLED is enabled.
// V2.06 NC
// V2.07 Moved MEAS2 from D13 to D0 to free up status LED.
// V2.08 Firmware version format, status LED.
// V2.09 Added code to turn off unused LEDs. (Only will work if no line receivers installed.)
// V2.20 Moved REF, MEAS1, MEAS2 to shorten traces on V2.2 PCB.
//        REF   TMR2 D0  Pin 2  (REF was TMR4 D9 Pin 11, D0 was MEAS2)
//        MEAS1 TMR4 D9  Pin 11 (MEAS1 was TMR1 D10 Pin 12, D9 was REF)
//        MEAS2 TMR1 D10 Pin 12 (MEAS2 was TMR2 D0 Pin 2, D10 was MEAS1)
//        MEAS3 TMR3 D14 Pin 16

#define FirmwareVersion 220    // Firmware version (x100) used by OLED and GUI About.

#define Heterodyne 3           // Set to the number of axes for heterodyne, sample rate always 1 kHz.

#define Multiplier 2           // Integer counts/cycle.  2 for double clocking.
#define Scale_Shift 1          // Set logbase2(Multiplier)

// #define OLED_Displacement   // OLED to display Disp1,Disp2,Disp3

// u8g2 graphics library available though the Library Manager

#include <U8x8lib.h>

float FirmwareFloat = FirmwareVersion;

U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

IMXRT_TMR_t * TMR1 = (IMXRT_TMR_t *)&IMXRT_TMR1;
IMXRT_TMR_t * TMR2 = (IMXRT_TMR_t *)&IMXRT_TMR2;
IMXRT_TMR_t * TMR3 = (IMXRT_TMR_t *)&IMXRT_TMR3;
IMXRT_TMR_t * TMR4 = (IMXRT_TMR_t *)&IMXRT_TMR4;

IntervalTimer usbTimer;            // send USB data at predefinded rate to make frequency analysis work in the GUI

// XXX The IOMUX is also used to configure other pin characteristics, such as voltage level, drive strength, and hysteresis. These may not be set optimally. More experimentation / real world data is necessary

char buffer[100];
char oled_buffer[25];
uint8_t tiles[8] = {0x0, 0x80, 0x7c, 0x40, 0x40, 0x40, 0x7c, 0x0};

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

void setup()
{
  pinMode(0, INPUT_PULLUP);  // MEAS2
  pinMode(1, INPUT_PULLUP);  // Hom 1A wired to REF
  pinMode(2, INPUT_PULLUP);  // Hom 1B wired to MEAS1
  pinMode(9, INPUT_PULLUP);  // REF
  pinMode(10, INPUT_PULLUP); // MEAS1
  pinMode(14, INPUT_PULLUP); // MEAS3

// Turn off unused LEDs for Homodyne channel 3 IFF no line receivers present.

  pinMode(5, OUTPUT);     // Hom 3A
  digitalWrite (5, HIGH); // Turn off LED for Hom 3A
  pinMode(7, OUTPUT);     // Hom 3B
  digitalWrite (7, HIGH); // Turn off LED for Hom 3B

// Turn off unused LEDs for MEAS2 and MEAS3 (and Homodyne channel 2) if no line receivers present.
  if (Heterodyne == 1)
    {
      pinMode(3, OUTPUT);     // Hom 2A wired to MEAS2
      digitalWrite (3, HIGH); // Turn off LED for Hom 2A
      pinMode(4, OUTPUT);     // Hom 2B wired to MEAS3
      digitalWrite (4, HIGH); // Turn off LED for Hom 2B
    }
 
  pinMode(13, OUTPUT);       // Status LED

  Serial.begin(2000000);
  // initialize and clear display
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  // Banner and sequence number display
  u8x8.setFont(u8x8_font_chroma48medium8_r);        // Default font (thin)

  //u8x8.setFont(u8x8_font_amstrad_cpc_extended_f); // Font with micro symbol (fatter
  u8x8.drawString(0, 0, " -  MD2 V     -");
  u8x8.drawTile(3, 0, 1, tiles);                    // Needed for tail of micro symbol

  sprintf(buffer, "%.2f", FirmwareFloat / 100);
  u8x8.drawString(9, 0, buffer);

#ifdef OLED_Displacement

  u8x8.drawString(0, 2, "Seq#:        ");
  u8x8.drawString(0, 4, "Disp1:");

  if (Heterodyne > 1)
  {
    u8x8.drawString(0, 5, "Disp2:");
    u8x8.drawString(0, 6, "Disp3:");
  }
#endif

  // set up QuadTimer1: MEAS2 on D10
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);           // enable QTMR1 clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 1;                  // QuadTimer1 Counter 0 on pin D10 using ALT1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 |= 0b1000000000000000; // enable hysteresis in pin D10
  TMR1->CH[0].CTRL = 0;                   // stop
  TMR1->CH[1].CTRL = 0;                   // stop
  TMR1->CH[2].CTRL = 0;                   // stop
  TMR1->CH[3].CTRL = 0;                   // stop
  TMR1->CH[0].CNTR = 0;                   // set count to 0
  TMR1->CH[1].CNTR = 0;                   // set count to 0
  TMR1->CH[2].CNTR = 0;                   // set count to 0
  TMR1->CH[3].CNTR = 0;                   // set count to 0
  TMR1->CH[0].LOAD = 0;
  TMR1->CH[1].LOAD = 0;
  TMR1->CH[2].LOAD = 0;
  TMR1->CH[3].LOAD = 0;
  TMR1->CH[0].SCTRL = TMR1->CH[0].CSCTRL = 0;
  TMR1->CH[1].SCTRL = TMR1->CH[1].CSCTRL = 0;
  TMR1->CH[2].SCTRL = TMR1->CH[2].CSCTRL = 0;
  TMR1->CH[3].SCTRL = TMR1->CH[3].CSCTRL = 0;
  TMR1->CH[0].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR1->CH[1].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR1->CH[2].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR1->CH[3].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR1->CH[0].CMPLD1 =  0xffff;
  TMR1->CH[1].CMPLD1 =  0xffff;
  TMR1->CH[2].CMPLD1 =  0xffff;
  TMR1->CH[3].CMPLD1 =  0xffff;
  TMR1->CH[3].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR1->CH[3].CTRL |= TMR_CTRL_PCS(6);    // Primary Count Source: CH[2] output
  TMR1->CH[2].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR1->CH[2].CTRL |= TMR_CTRL_PCS(5);    // Primary Count Source: CH[1] output
  TMR1->CH[1].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR1->CH[1].CTRL |= TMR_CTRL_PCS(4);    // Primary Count Source: CH[0] output
  TMR1->CH[0].CTRL  = TMR_CTRL_CM (2);    // Count Mode:           Count rising edges of primary source
  TMR1->CH[0].CTRL |= TMR_CTRL_PCS(0);    // Primary Count Source: Counter 0 input pin

  // set up QuadTimer2: REF on D0
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);                  // turn clock on for XBAR1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 1;                    // IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 (pin 0) to ALT1 mux port: XBAR1_INOUT17
  IOMUXC_XBAR1_IN17_SELECT_INPUT = 1 ;                        // XBAR1_INOUT17 has several inputs to choose from. Pick IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03
  IOMUXC_GPR_GPR6 |= 0b0000000010000;                         // connect XBAR as input for QTIMER2_TIMER0
  xbar_connect(17, XBARA1_OUT_QTIMER2_TIMER0);                // connect XBAR1_INOUT17 to XBARA1_OUT_QTIMER2_TIMER0
  CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);                // enable QTMR2 clock
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 |= 0b1000000000000000;  // enable hysteresis in pin D0
  TMR2->CH[0].CTRL = 0;                   // stop
  TMR2->CH[0].SCTRL = TMR2->CH[0].CSCTRL = 0;
  TMR2->CH[1].CTRL = 0;                   // stop
  TMR2->CH[2].CTRL = 0;                   // stop
  TMR2->CH[3].CTRL = 0;                   // stop
  TMR2->CH[0].CNTR = 0;                   // set count to 0
  TMR2->CH[1].CNTR = 0;                   // set count to 0
  TMR2->CH[2].CNTR = 0;                   // set count to 0
  TMR2->CH[3].CNTR = 0;                   // set count to 0
  TMR2->CH[0].LOAD = 0;
  TMR2->CH[1].LOAD = 0;
  TMR2->CH[2].LOAD = 0;
  TMR2->CH[3].LOAD = 0;
  TMR2->CH[0].SCTRL = TMR2->CH[0].CSCTRL = 0;
  TMR2->CH[1].SCTRL = TMR2->CH[1].CSCTRL = 0;
  TMR2->CH[2].SCTRL = TMR2->CH[2].CSCTRL = 0;
  TMR2->CH[3].SCTRL = TMR2->CH[3].CSCTRL = 0;
  TMR2->CH[0].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR2->CH[1].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR2->CH[2].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR2->CH[3].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR2->CH[0].CMPLD1 =  0xffff;
  TMR2->CH[1].CMPLD1 =  0xffff;
  TMR2->CH[2].CMPLD1 =  0xffff;
  TMR2->CH[3].CMPLD1 =  0xffff;
  TMR2->CH[3].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR2->CH[3].CTRL |= TMR_CTRL_PCS(6);    // Primary Count Source: CH[2] output
  TMR2->CH[2].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR2->CH[2].CTRL |= TMR_CTRL_PCS(5);    // Primary Count Source: CH[1] output
  TMR2->CH[1].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR2->CH[1].CTRL |= TMR_CTRL_PCS(4);    // Primary Count Source: CH[0] output
  TMR2->CH[0].CTRL  = TMR_CTRL_CM (2);    // Count Mode:           Count rising edges of primary source
  TMR2->CH[0].CTRL |= TMR_CTRL_PCS(0);    // Primary Count Source: Counter 0 input pin

  // set up QuadTimer3: MEAS3 on D14
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);               // enable QTMR3 clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 1;                   // Daisy Chain 1 - QT3 Counter 2 conects to pin D14 ALT1
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT  = 1 ;                  // Daisy Chain 2 - QT3 Counter 2 conects to pin D14 ALT1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 |= 0b1000000000000000; // enable hysteresis in pin D14
  TMR3->CH[0].CTRL = 0;                   // stop
  TMR3->CH[2].SCTRL = TMR3->CH[2].CSCTRL = 0;
  TMR3->CH[1].CTRL = 0;                   // stop
  TMR3->CH[2].CTRL = 0;                   // stop
  TMR3->CH[3].CTRL = 0;                   // stop
  TMR3->CH[0].CNTR = 0;                   // set count to 0
  TMR3->CH[1].CNTR = 0;                   // set count to 0
  TMR3->CH[2].CNTR = 0;                   // set count to 0
  TMR3->CH[3].CNTR = 0;                   // set count to 0
  TMR3->CH[0].LOAD = 0;
  TMR3->CH[1].LOAD = 0;
  TMR3->CH[2].LOAD = 0;
  TMR3->CH[3].LOAD = 0;
  TMR3->CH[0].SCTRL = TMR3->CH[0].CSCTRL = 0;
  TMR3->CH[1].SCTRL = TMR3->CH[1].CSCTRL = 0;
  TMR3->CH[2].SCTRL = TMR3->CH[2].CSCTRL = 0;
  TMR3->CH[3].SCTRL = TMR3->CH[3].CSCTRL = 0;
  TMR3->CH[0].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR3->CH[1].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR3->CH[2].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR3->CH[3].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR3->CH[0].CMPLD1 =  0xffff;
  TMR3->CH[1].CMPLD1 =  0xffff;
  TMR3->CH[2].CMPLD1 =  0xffff;
  TMR3->CH[3].CMPLD1 =  0xffff;
  TMR3->CH[3].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR3->CH[3].CTRL |= TMR_CTRL_PCS(6);    // Primary Count Source: CH[2] output
  TMR3->CH[2].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR3->CH[2].CTRL |= TMR_CTRL_PCS(5);    // Primary Count Source: CH[1] output
  TMR3->CH[1].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR3->CH[1].CTRL |= TMR_CTRL_PCS(4);    // Primary Count Source: CH[0] output
  TMR3->CH[0].CTRL  = TMR_CTRL_CM (2);    // Count Mode:           Count rising edges of primary source
  TMR3->CH[0].CTRL |= TMR_CTRL_PCS(2);    // Primary Count Source: Counter 2 input pin

  // set up QuadTimer4: MEAS1 D9
  CCM_CCGR6 |= CCM_CCGR6_QTIMER4(CCM_CCGR_ON);            // enable QTMR4 clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 1;                   // QuadTimerT4 Counter 2 on pin D9
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 |= 0b1000000000000000; // enable hysteresis in pin D9
  TMR4->CH[0].CTRL = 0;                   // stop
  TMR4->CH[1].CTRL = 0;                   // stop
  TMR4->CH[2].CTRL = 0;                   // stop
  TMR4->CH[3].CTRL = 0;                   // stop
  TMR4->CH[0].CNTR = 0;                   // set count to 0
  TMR4->CH[1].CNTR = 0;                   // set count to 0
  TMR4->CH[2].CNTR = 0;                   // set count to 0
  TMR4->CH[3].CNTR = 0;                   // set count to 0
  TMR4->CH[0].LOAD = 0;
  TMR4->CH[1].LOAD = 0;
  TMR4->CH[2].LOAD = 0;
  TMR4->CH[3].LOAD = 0;
  TMR4->CH[0].SCTRL = TMR4->CH[0].CSCTRL = 0;
  TMR4->CH[1].SCTRL = TMR4->CH[1].CSCTRL = 0;
  TMR4->CH[2].SCTRL = TMR4->CH[2].CSCTRL = 0;
  TMR4->CH[3].SCTRL = TMR4->CH[3].CSCTRL = 0;
  TMR4->CH[0].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR4->CH[1].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR4->CH[2].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR4->CH[3].COMP1 =  0xffff;            // send count signal to next counter on overflow at 0xffff
  TMR4->CH[0].CMPLD1 =  0xffff;
  TMR4->CH[1].CMPLD1 =  0xffff;
  TMR4->CH[2].CMPLD1 =  0xffff;
  TMR4->CH[3].CMPLD1 =  0xffff;
  TMR4->CH[3].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR4->CH[3].CTRL |= TMR_CTRL_PCS(6);    // Primary Count Source: CH[2] output
  TMR4->CH[2].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR4->CH[2].CTRL |= TMR_CTRL_PCS(5);    // Primary Count Source: CH[1] output
  TMR4->CH[1].CTRL  = TMR_CTRL_CM (7);    // Count Mode:           Cascaded counter mode
  TMR4->CH[1].CTRL |= TMR_CTRL_PCS(4);    // Primary Count Source: CH[0] output
  TMR4->CH[0].CTRL  = TMR_CTRL_CM (2);    // Count Mode:           Count rising edges of primary source
  TMR4->CH[0].CTRL |= TMR_CTRL_PCS(2);    // Primary Count Source: Counter 2 input pin

  usbTimer.begin(USBSender, 1000);        // send USB data every 1000 microseconds
  usbTimer.priority(200);                 // Lower numbers are higher priority, with 0 the highest and 255 the lowest. Most other interrupts default to 128
}

int64_t counter_REF   =  0;
int64_t counter_MEAS1 =  0;
int64_t counter_MEAS2 =  0;
int64_t counter_MEAS3 =  0;

int64_t counter_REF_save   =  0;
int64_t counter_MEAS1_save =  0;
int64_t counter_MEAS2_save =  0;
int64_t counter_MEAS3_save =  0;

int32_t OLED_REF =  0;
int32_t OLED_MEAS1 =  0;
int32_t OLED_MEAS2 =  0;
int32_t OLED_MEAS3 =  0;

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

int32_t LowSpeedCode = 0;
int32_t LowSpeedData = 0;
int32_t LowSpeedCodeSelect = 0;

uint32_t LEDValue = 0;
int32_t LEDIncrement = 1;

void USBSender()
{
  //constant delay timer read - update values.
  asm volatile("ldr    r0 ,=0x401dc00a  \n\t" // load address of TMR1_CNTR0 into r0
               "ldr    r1 ,=0x401e000a  \n\t" // load address of TMR2_CNTR0 into r1
               "ldr    r2 ,=0x401e400a  \n\t" // load address of TMR3_CNTR0 into r2
               "ldr    r3 ,=0x401e800a  \n\t" // load address of TMR4_CNTR0 into r3
               "ldrh   r4 ,[r0],#0      \n\t" // hold TMR1 by reading TMR1_CNTR0
               "ldrh   r5 ,[r1],#0      \n\t" // hold TMR2 by reading TMR2_CNTR0
               "ldrh   r6 ,[r2],#0      \n\t" // hold TMR3 by reading TMR3_CNTR0
               "ldrh   r7 ,[r3],#0      \n\t" // hold TMR4 by reading TMR4_CNTR0
               :
               :
               : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
              );

  REF   = (counter_REF   - counter_REF_save)   >> Scale_Shift;
  MEAS1 = (counter_MEAS1 - counter_MEAS1_save) >> Scale_Shift;
  MEAS2 = (counter_MEAS2 - counter_MEAS2_save) >> Scale_Shift;
  MEAS3 = (counter_MEAS3 - counter_MEAS3_save) >> Scale_Shift;

  counter_REF_save = counter_REF;
  counter_MEAS1_save = counter_MEAS1;
  counter_MEAS2_save = counter_MEAS2;
  counter_MEAS3_save = counter_MEAS3;

// Load counter values

  counter_REF   =  TMR2->CH[3].HOLD;
  counter_REF   =  counter_REF * 65536    + TMR2->CH[2].HOLD;
  counter_REF   =  counter_REF * 65536    + TMR2->CH[1].HOLD;
  counter_REF   =  counter_REF * 65536    + TMR2->CH[0].HOLD;
  
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

  previous_displacement1 = displacement1;
  previous_displacement2 = displacement2;
  previous_displacement3 = displacement3;

  displacement1 = (counter_MEAS1 - counter_REF);
  displacement2 = (counter_MEAS2 - counter_REF);
  displacement3 = (counter_MEAS3 - counter_REF);

  velocity1 = displacement1 - previous_displacement1;
  velocity2 = displacement2 - previous_displacement2;
  velocity3 = displacement3 - previous_displacement3;

  sequenceNumber++;
  sn = sequenceNumber;

  //  analogWrite (13, 255);
  //    LEDValue += LEDIncrement;
  //    if ((LEDValue >=255) || (LEDValue <= 0)) LEDIncrement = -LEDIncrement;

  if ((sn & 0x40) == 0x40) digitalWrite (13, 1);
  else digitalWrite (13, 0);

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

  else if (LowSpeedCodeSelect == 13) // Tell GUI this is not a homodyne system
  {
    LowSpeedCode = 20;
    LowSpeedData = (Multiplier * 256) + 0; // Tell GUI this is not a homodyne system
  }

  else if (LowSpeedCodeSelect == 11) // # CPU clocks spent in capture and analysis
  {
    LowSpeedCode = 121;
    LowSpeedData = 1;
  }

  else if (LowSpeedCodeSelect == 12) // # CPU clocks spent in communications
  {
    LowSpeedCode = 122;
    LowSpeedData = 1;                // Timer1USBCounts not implemented yet
  }

  if (Serial.availableForWrite() > 256)
  {
    Serial.printf("%li ", REF);                     // 1: REF
    Serial.printf("%li ", MEAS1);                   // 2: MEAS1
    Serial.printf("%li ", displacement1);           // 3: Displacement 1
    Serial.printf("%li ", velocity1);               // 4: Velocity Count 1
    Serial.print("0 ");                             // 5: Phase 1
    Serial.printf("%llu ", sequenceNumber);         // 6: Sequence Number
    Serial.printf("%li ", LowSpeedCode);            // 7: LowSpeedCode
    Serial.printf("%li", LowSpeedData);             // 8: LowSpeedData

    if (Heterodyne > 1)
    {
      Serial.printf(" %li ", MEAS2);              // 9: MEAS2
      Serial.printf("%li ", displacement2);       // 10: Displacement 2
      Serial.printf("%li ", velocity2);           // 11: Velocity Count 2
      Serial.print("0 ");                         // 12: Phase 2
      Serial.printf("%li ", MEAS3);               // 13: MEAS3
      Serial.printf("%li ", displacement3);       // 14: Displacement 3
      Serial.printf("%li ", velocity3);           // 15: Velocity Count 3
      Serial.print("0");                          // 16: Phase 3
    }
    Serial.println();
  }
}

void loop()
{
#ifdef OLED_Displacement

  u8x8.drawString(6, 2, "       ");
  sprintf(oled_buffer, "%li", sn);
  u8x8.drawString(6, 2, oled_buffer);

  OLED_DISP1 = displacement1;
  if (OLED_DISP1 != OLED_DISP1_save)
  {
    OLED_DISP1_save = OLED_DISP1;
    u8x8.drawString(7, 4, "       ");
    sprintf(oled_buffer, "%li", OLED_DISP1);
    u8x8.drawString(7, 4, oled_buffer);
  }

  if (Heterodyne > 1)
  {
    OLED_DISP2 = displacement2;
    if (OLED_DISP2 != OLED_DISP2_save)
    {
      OLED_DISP2_save = OLED_DISP2;
      u8x8.drawString(7, 5, "       ");
      sprintf(oled_buffer, "%li", OLED_DISP2);
      u8x8.drawString(7, 5, oled_buffer);
    }

    OLED_DISP3 = displacement3;
    if (OLED_DISP3 != OLED_DISP3_save)
    {
      OLED_DISP3_save = OLED_DISP3;
      u8x8.drawString(7, 6, "       ");
      sprintf(oled_buffer, "%li", OLED_DISP3);
      u8x8.drawString(7, 6, oled_buffer);
    }
  }

#endif
//  digitalWrite (13, !digitalRead (13));

  delay(100);
}
