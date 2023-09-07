

/*
Scupt LED Linear board
This example controls one IW7039 LED drivers.
Each IW7039 has 32 LED channels.
Maximum current of each channel changes to 60mA

The circuit:
*5V - to the +5V of Arduino Uno
*DGND - to GND of Arduino Uno
*CS - to digital pin 10(chip select pin)
*MOSI - to digital pin 11 (MOSI pin)
*MISO - to digital pin 12 (MISO pin)
*SCK - to digital pin 13 (SCK pin)
*VSYNC - to digital pin 9
*FAULT - to digital pin ?? (may not required)

 */

// include SPI library:
#include <SPI.h>
#include <TimerOne.h>

// Define other pins needed by SPI library

#define Vsync 11
#define chipSelectPin 68

// Define SPI_CS_DELAY 25us
#define SPI_CS_DELAY 25

float PWM[6] = {0, 0, 1, 1, 1, 1}; // PWM duty cycle for all channels
int PWM_Reg[6];                    // R,R,G,G,B,B

void setup()
{
  Serial.begin(9600);
  // Start the SPI library:
  SPI.begin();

  // Initalize chip select pins
  pinMode(chipSelectPin, OUTPUT);

  // Add external 120Hz Vsync, 50% duty cycle
  pinMode(Vsync, OUTPUT);
  Timer1.initialize(16666);
  Timer1.pwm(Vsync, 512);

  // Delay 10ms until logic core is ready
  delay(1000); // delay 10ms until logic core is ready

  initDev();

  delay(3000); // delay 300ms
}

void initDev()
{
  unsigned int intReg_Conf_iW7039[34] = {
      0x8020, 0x00,                                                   // broadcast command with same data writting for 32 words from register 0x00
      0x0802, 0x83A2, 0x0800, 0x0FFF, 0x9FFC, 0x4927, 0x4585, 0x87FF, // 0X00 ~ 0X07
      0x0100, 0x2100, 0x0000, 0x0870, 0x0088, 0x186A, 0x5348, 0xCC0C, // 0X08 ~ 0X0F
      0xFFFF, 0xFFFF, 0x0001, 0x0080, 0x010f, 0x0000, 0x0000, 0x0000, // 0X10 ~ 0X17
      0xFFFF, 0xFFFF, 0x0000, 0x01FF, 0x0000, 0x1000, 0xFFFF, 0xFFFF  // 0X18 ~0X1F

  }; // Initializing

  unsigned int intReg_DT_iW7039[34] = {
      0x8020, 0x40,                                           //    broadcast command with same data writting for 32 words for DT
      0x00, 0x80, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380,   // 0x40 ~0x47
      0x400, 0x480, 0x500, 0x580, 0x600, 0x680, 0x700, 0x780, // 0x48 ~ 0x4F
      0x800, 0x880, 0x900, 0x980, 0xA00, 0xA80, 0xB00, 0xB80, // 0x50 ~ 0x57
      0xC00, 0xC80, 0xD00, 0xD80, 0xE00, 0xE80, 0xF00, 0xF80  // 0x58 ~ 0x5F
  };                                                          // 0x40 to 0x5F
  unsigned int intReg_HT_iW7039[34] = {
      0x8020, 0x60,                                         //    broadcast command with same data writting for 32 words for HT
      0xfff, 0Xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0x00, 0x00, // 0x60 ~ 0x67
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       // 0x68 ~ 0x6F
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       // 0x70 ~ 0x77
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00        // 0x78 ~ 0x7F
  };                                                        // 0x60 to 0x7F

  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_Conf_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(chipSelectPin, HIGH);
  delayMicroseconds(SPI_CS_DELAY);

  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_DT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(chipSelectPin, HIGH);

  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_HT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(chipSelectPin, HIGH);

  // delay(3000);

  // Reset all chip, all LEDs should turn on
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  SPI.transfer16(0x8001);
  SPI.transfer16(0x00);
  SPI.transfer16(0x0803);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(chipSelectPin, HIGH);
  delayMicroseconds(SPI_CS_DELAY);

  delay(10); // Keep all LEDs on for 10ms

  // Re-config all LED PWM duty cycles

  for (int j = 0; j < 6; j++)
  {
    PWM_Reg[j] = (int)(PWM[j] * 2000);

    // unsigned int Reg_HT_iW7039[8] = {
    //     0x8120, 0x60,                                                          // writing for 32 words for HT for IW7039
    //     PWM_Reg[0], PWM_Reg[1], PWM_Reg[2], PWM_Reg[3], PWM_Reg[4], PWM_Reg[5] // CH1 ~ CH6
    // };

    // // Send HT registers to iW7039 LED driver, CH1 to CH6
    // digitalWrite(chipSelectPin, LOW);
    // delayMicroseconds(SPI_CS_DELAY);
    // for (int i = 0; i < 8; i++)
    //   SPI.transfer16(Reg_HT_iW7039[i]);
    // SPI.transfer16(0);
    // delayMicroseconds(SPI_CS_DELAY);
    // digitalWrite(chipSelectPin, HIGH);
  }

  // delay(3000);

  unsigned int Reg_HT_iW7039[8] = {
        0x8120, 0x60,                                                          // writing for 32 words for HT for IW7039
        PWM_Reg[0], PWM_Reg[1], PWM_Reg[2], PWM_Reg[3], PWM_Reg[4], PWM_Reg[5] // CH1 ~ CH6
    };

  // Send HT registers to iW7039 LED driver, CH1 to CH6
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 8; i++)
    SPI.transfer16(Reg_HT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(chipSelectPin, HIGH);
  // delay(2000);
}

void loop()
{
  for(int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      PWM_Reg[j] = (int)(PWM[j] * 4095 / i);
      Serial.print(PWM_Reg[j]);
      Serial.print(" ");
    }
    unsigned int Reg_HT_iW7039[8] = {
        0x8120, 0x60,                                                          // writing for 32 words for HT for IW7039
        PWM_Reg[0], PWM_Reg[1], PWM_Reg[2], PWM_Reg[3], PWM_Reg[4], PWM_Reg[5] // CH1 ~ CH6
    };

    // Send HT registers to iW7039 LED driver, CH1 to CH6
    digitalWrite(chipSelectPin, LOW);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 8; i++)
      SPI.transfer16(Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    digitalWrite(chipSelectPin, HIGH);

    Serial.println(i);

    delay(1000);
  }
}