// uMD2 homodyne interferometer firmware for Teensy 4.0.
// Jan Beck and Sam Goldwasser 2020

// V1.21 First version with timer controller loop.  1 ks/s but the GUI doesn't know this yet.
// V1.22 Added low speed data for version, homodyne parameters, sample rate
// V1.23 Fixed inconsistency bug whereby GUI and OLED would not track if both reset.
// V1.25 Fixed so OLED axes update even if com is blocked, test for # axes. 
// V1.26 Fixed OLED axes update to handle numbers larger than 99999

// Required libraries:
//  Teensy 4.0 Hardware encoder library available from https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library
//  u8g2 graphics library available though the Arduino Library Manager

#define FirmwareVersion 1.26 // Firmware version used by OLED and GUI About.

#define Homodyne 3           // Set # of axes: 1 for 1 axis, 2 for 2 axes, 3 for 3 axes, 0 for heterodyne.
                             // Sample rate for homodyne always 1 kHz.

#define HomodyneMultiplier 4 // Set to homodyne interferometer integer counts/cycle.  4 is Quadpulse and Renishaw Coarse; Fine  is 16.

#include <U8x8lib.h>
#include "QuadEncoder.h"

U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
char buffer[50];

//The Teensy 4.0 Encoders are supported on pins: 0, 1, 2, 3, 4, 5, 7, 30, 31 and 33
QuadEncoder encoder1(1, 1, 2, 0);  // Encoder on channel 1 of 4 available; Phase A (pin1), PhaseB(pin2), Pullups Req(0)
QuadEncoder encoder2(2, 3, 4, 0);  // Encoder on channel 2 of 4 available; Phase A (pin3), PhaseB(pin4), Pullups Req(0)
QuadEncoder encoder3(3, 5, 7, 0);  // Encoder on channel 3 of 4 available; Phase A (pin5), PhaseB(pin7), Pullups Req(0)

//elapsedMicros timeStampLastSerialData;  // used for serial data frequency; increments automatically
IntervalTimer usbTimer;            // send USB data at predefinded rate to make frequency analysis work in the GUI

uint8_t tiles[8] = {0x0,0x80,0x7c,0x40,0x40,0x40,0x7c,0x0};
int HeartbeatLED = 13;

void setup()
{
  pinMode(HeartbeatLED, OUTPUT);
  
  while (!Serial && millis() < 1000);

  // Initialize the ENC module.
  encoder1.setInitConfig();
  encoder1.init();
  encoder2.setInitConfig();
  encoder2.init();
  encoder3.setInitConfig();
  encoder3.init();
  
  // initialize and clear display
  u8x8.begin();
  u8x8.setPowerSave(0);

  // Banner and sequence number display
  u8x8.setFont(u8x8_font_chroma48medium8_r);    // Default font (thin)
  //u8x8.setFont(u8x8_font_amstrad_cpc_extended_f); // Font with micro symbol (fatter
  u8x8.drawString(0, 0, " -  MD2 V     -");
  u8x8.drawTile(3, 0, 1, tiles);                // Needed for tail of micro symbol

/*
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f); // Font with micro symbol (fatter)
  u8x8.drawUTF8(0, 0, "µMD2 ");
*/

  sprintf(buffer, "%.2f", FirmwareVersion);
  u8x8.drawString(9, 0, buffer);
  u8x8.drawString(0, 2, "Seq#:        ");
  u8x8.drawString(0, 5, "Ax1:");
  u8x8.drawString(0, 6, "Ax2:");
  u8x8.drawString(0, 7, "Ax3:");
   
  
  pinMode(11, OUTPUT);
  usbTimer.begin(USBSender, 1000);  // send USB data every 1000 microseconds
  usbTimer.priority(200);           // Lower numbers are higher priority, with 0 the highest and 255 the lowest. Most other interrupts default to 128
}

int32_t encoder1Value = 0;
int32_t encoder2Value = 0;
int32_t encoder3Value = 0;
int32_t encoder1ValueSave = 0;
int32_t encoder2ValueSave = 0;
int32_t encoder3ValueSave = 0;
int32_t compareValue  = 0;
int32_t velocity1     = 0;
int32_t velocity2     = 0;
int32_t velocity3     = 0;
uint64_t sequenceNumber = 0;
uint32_t sn      = 0;

int32_t LowSpeedCode = 0;
int32_t LowSpeedData = 0;
int32_t LowSpeedCodeSelect = 0;
int32_t FirmwareInt = FirmwareVersion * 100;

void USBSender()
{
  sequenceNumber++;
  sn = sequenceNumber;

// Send appropriate low speed data

  LowSpeedCode = 0; // Default to no low speed data
  LowSpeedData = 0;
  LowSpeedCodeSelect = sequenceNumber & 0x1f;

  if (LowSpeedCodeSelect == 1) // Send firmware version
    {
      LowSpeedCode = 10;
      LowSpeedData = FirmwareInt;
    }
      
  else if (LowSpeedCodeSelect == 2) // Sammple frequency x 100
    {
      LowSpeedCode = 8;
      LowSpeedData = 100000;
    }
      
  else if (LowSpeedCodeSelect == 13) // Tell GUI this is a homodyne system if true
    {
      LowSpeedCode = 20;
      LowSpeedData = (HomodyneMultiplier * 256) + Homodyne; // # counts per cycle << 8 + # homodyne axes
    }

  compareValue = encoder1.read();
  velocity1 = compareValue - encoder1Value ;         // calculate velocity 1
  encoder1Value = compareValue;

  compareValue = encoder2.read();
  velocity2 = compareValue - encoder2Value;          // calculate velocity 2
  encoder2Value = compareValue;

  compareValue = encoder3.read();
  velocity3 = compareValue - encoder3Value;          // calculate velocity 3
  encoder3Value = compareValue;

  if ((0x1f0 & sequenceNumber) == 0x1f0) analogWrite(HeartbeatLED,16);
    else analogWrite(HeartbeatLED,0);  // Heartbeat LED
    
  if (Serial.availableForWrite() > 256)
    {
      Serial.print("0 0 ");                          // 0: REF Frequency Count; 1: MEAS Frequency Count 1  
      Serial.printf("%li ", encoder1Value);          // 2: Displacement 1
      Serial.printf("%li ", velocity1);              // 3: Velocity Count 1
      Serial.print("0 ");                            // 4: Phase 1
      Serial.printf("%llu ", sequenceNumber);        // 5: Sequence Number
      Serial.printf("%li ", LowSpeedCode);           // 6: LowSpeedCode
      Serial.printf("%li", LowSpeedData);            // 7: LowSpeedData

      if (Homodyne > 1)
        {
          Serial.print(" 0 ");                            // 8: MEAS Frequency Count 2
          Serial.printf("%li ", encoder2Value);          // 9: Displacement 2
          Serial.printf("%li ", velocity2);              // 10: Velocity Count 2
          Serial.print("0 ");                            // 11: Phase 2
          Serial.print("0 ");                            // 12: MEAS Frequency Count 3 
          Serial.printf("%li ", encoder3Value);          // 13: Displacement 3
          Serial.printf("%li ", velocity3);              // 14: Velocity Count 3
          Serial.printf("0");                           // 15: Phase 3
        }
      Serial.println();
    }
  }


void loop()
{
  if (encoder1Value != encoder1ValueSave)
    {
      encoder1ValueSave = encoder1Value;
      u8x8.drawString(5, 5, "           ");
      sprintf(buffer, "%li", encoder1Value);
      u8x8.drawString(5, 5, buffer);
    }
  
  if (Homodyne > 1)
    {
      if (encoder2Value != encoder2ValueSave)
        {
          encoder2ValueSave = encoder2Value;
          u8x8.drawString(5, 6, "           ");
          sprintf(buffer, "%li", encoder2Value);
          u8x8.drawString(5, 6, buffer);
        }

      if (encoder3Value != encoder3ValueSave)
        {
          encoder3ValueSave = encoder3Value;
          u8x8.drawString(5, 7, "           ");
          sprintf(buffer, "%li", encoder3Value);
          u8x8.drawString(5, 7, buffer);
        }
    }

  sprintf(buffer, "%li", sn);
  u8x8.drawString(6, 2, buffer);
}
