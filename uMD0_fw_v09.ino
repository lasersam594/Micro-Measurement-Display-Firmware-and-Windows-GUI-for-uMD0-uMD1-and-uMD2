//*******************************************************************************************************//
//                 uMD Homodyne Interferometer Test Firmware for Atmel Microprocessors                   //
//                 Sam Goldwasser with support from Jan Beck, Copyright(c) 1994-2024                     //
//*******************************************************************************************************//

// Note that behavior for GUI Velocity and Frequency is unpredictable and probably will remain that way. ;-)

// v01 First attempt at Atmega homodyne interferometer code.
// v02 Added interrupts. (Note: Changed encoder pins to 2 and 3.)
// V03 Added separate interrupt routines for PinA (2) and PinB (3) changes.
// V04 First working version.
// V05 Fixed 32 bit string formatting resulting in jumps at displacement 32K bounderies.
// V06 Cleaned up.
// V07 Cleaned up some more.
// V08 Added interrupt lockout and test for Serial.availableForWrite.
// V09 Added 244.14 Hz timer interrupt for USB send.

#define FirmwareVersion 209

#define Homodyne 1              // Select homodyne mode and a single axis for the GUI
#define HomodyneMultiplier 4    // Set to homodyne interferometer integer counts/cycle.

bool PinA;
bool PinB;

int32_t SequenceNumber;
int16_t LowSpeedCode = 0;       // Select type of low speed data
int16_t LowSpeedData = 0;       // Low speed data
int32_t Displacement = 0;
int16_t LowSpeedCodeSelect = 0;
int16_t length = 0;

static char buffer[300];    // make some space for the routine to convert numbers to text

// Encoder input ISRs
void encoderupdatePinA()
  {
    noInterrupts();
    if ((PinA = digitalRead(2)) == HIGH)
      {
        if ((PinB = digitalRead(3)) == LOW) Displacement++;
        else Displacement--;
      }
    else if (PinB == LOW) Displacement--;
    else Displacement++;
    interrupts();
  }
  
void encoderupdatePinB()
  {
    noInterrupts();
    if ((PinB = digitalRead(3)) == HIGH)
      {
        if ((PinA = digitalRead(2)) == LOW) Displacement--;
        else Displacement++;
      }
     else if (PinA == LOW) Displacement++;
     else Displacement--;
     interrupts();
  }

void setup()
  {
    Serial.begin(115200); // Buad rate must match GUI
    Serial.println("\nMinimalist Homodyne Test:");

    pinMode(13, OUTPUT);       // Heartbeat LED
    pinMode(2, INPUT_PULLUP);  // Encoder Pin A
    pinMode(3, INPUT_PULLUP);  // Encoder Pin B

    noInterrupts(); // Disable interrupts while messing with timers

 // Initialize Timer0 PWM B for USB send interrupt at 244.140625 Hz
    TCNT0  = 0;                                      // Value
    OCR0B = 128;                                     // Compare match register
    TCCR0B &= !(_BV(CS02) | _BV(CS01) | _BV(CS00));  // Clear CS bits (0b0111)
    TCCR0B |= _BV(CS02);                             // Set prescaler to 256 (0b0100)
    TIMSK0 |= _BV(OCIE1B);                           // Enable timer compare interrupt (bit 0b0100)

// Initialize interrups for encoder inputs
    attachInterrupt(0, encoderupdatePinA, CHANGE);
    attachInterrupt(1, encoderupdatePinB, CHANGE);

    interrupts();  // Enable all interrupts
  }
  
ISR(TIMER0_COMPB_vect) // USB send at 244.14 s/s
  {
    if (Serial.availableForWrite() >= 63) // 63 seems to be the maximum available.
      {
        interrupts();                     // Enable interrupts to permit UART to function

        LowSpeedCodeSelect = SequenceNumber & 0xff;

        if (LowSpeedCodeSelect == 1)      // Send firmware version
          {
            LowSpeedCode = 10;
            LowSpeedData = FirmwareVersion;
          }
        else if (LowSpeedCodeSelect == 2) // Sample frequency
          {
            LowSpeedCode = 8;
            LowSpeedData = 24414;
          }
        else if (LowSpeedCodeSelect == 13) // Tell GUI this is a homodyne system if true
          {
            LowSpeedCode = 20;
            LowSpeedData = (HomodyneMultiplier * 256) + Homodyne; // # counts per cycle << 8 + # homodyne axes
          }
        else
          {
            LowSpeedCode = 0;
            LowSpeedData = 0;
          }
 
        length = 0;   
        length += sprintf(buffer,"0 0 ");                           // REF & MEAS Counts for heterodyne, 0 here
        ltoa(Displacement,buffer+length,10);                        // Displacement (32 bit)
        Serial.print(buffer);       
        length = 0;
        length += sprintf(buffer+length," 0 0 %d ",SequenceNumber); // Velocity Count & Phase for heterodyne, 0 here, Sequence Number
        length += sprintf(buffer+length,"%d ",LowSpeedCode);        // Code to specify low speed data type
        length += sprintf(buffer+length,"%d",LowSpeedData);         // Low speed data to send back
        Serial.println(buffer);       

        SequenceNumber++;
        if ((SequenceNumber & 0x3f) == 0x3f) digitalWrite(13, !digitalRead(13));  // Heartbeat LED
      }
  }

void loop()
  {
  }
