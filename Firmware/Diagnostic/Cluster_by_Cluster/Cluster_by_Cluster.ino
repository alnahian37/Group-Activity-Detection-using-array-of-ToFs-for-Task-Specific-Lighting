// include SPI, TimerOne, Ethernet library
#include <SPI.h>
#include <TimerOne.h>
#include <Ethernet.h>

// Define a flag for the code to be run on right (0) or left (1) Arduino
byte side = 1;

// Define the cluster to change value
int clusterPin = 0;

// Define other pins needed by SPI library
#define Vsync 11

//Define SPI_CS_DELAY 25us
#define SPI_CS_DELAY 25

// Defining the pin lookup arrays
// This is for the both sides side and indexes are 1-36
int pinLookup[][2] = {{1,32},{2,35},{3,38},{4,41},{5,44},{6,47},
                      {7,31},{8,34},{9,37},{10,40},{11,43},{12,46},
                      {13,30},{14,33},{15,36},{16,39},{17,42},{18,45},
                      {19,27},{20,24},{21,21},{22,18},{23,15},{24,3},
                      {25,28},{26,25},{27,22},{28,19},{29,16},{30,2},
                      {31,29},{32,26},{33,23},{34,20},{35,17},{36,14}};

// This has the data for turning all the LEDs off
word allOffData[2][4] = {{0x8001, 0x10, 0x0, 0},
                         {0x8001, 0x11, 0x0, 0}};

// This sends 4 words to the LED driver
void send_4words(word *data)
{
  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for(byte i=0; i<4; i++) SPI.transfer16(data[i]);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

// This sends 2 blocks of 4 words to the LED driver
void send_2blocks_of_4words(word data0[2][4], word milliSeconds, bool turnOff, word turnOffDelay)
{
  for(byte i=0; i<2; i++)
  {
    send_4words(data0[i]);
  }
  delay(milliSeconds);
  if(turnOff)
  {
    for(byte j=0; j<2; j++)
    {
      send_4words(allOffData[j]);
    }
    delay(turnOffDelay);
  }
}

// Turn off all lights
void all_off()
{
  send_2blocks_of_4words(allOffData, 1000, false, 0);
}

// chipUpdate updates all of the chip select pins to the same state (HIGH or LOW) for the given side
void chipUpdate(int state)
{
  if(side == 0)
  {
    if(state == 0) // corresponds to the low state
    {
      for(uint8_t i=0; i<18; i++)
      {
        digitalWrite(pinLookup[i][1], LOW); // set as LOW
      }
    }
    else // corresponds to the high state
    {
      for(uint8_t i=0; i<18; i++)
      {
        digitalWrite(pinLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else if (side == 1)
  {
    if(state == 0) // corresponds to the low state
    {
      for(uint8_t i=18; i<36; i++)
      {
        digitalWrite(pinLookup[i][1], LOW); // set as LOW
      }
    }
    else if(state == 1) // corresponds to the high state
    {
      for(uint8_t i=18; i<36; i++)
      {
        digitalWrite(pinLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else
  {
    Serial.println("Error: side not defined");
  }
}

void setup()
{
  Serial.begin(9600);
  // Start the SPI library:
  SPI.begin();

  // Initalize chip select pins
  for(uint8_t i=0; i<36; i++)
  { 
    pinMode (pinLookup[i][1], OUTPUT); // set as output
    digitalWrite(i, LOW); // set as LOW
  }

  // Add external 120Hz Vsync, 50% duty cycle
  pinMode(Vsync, OUTPUT);   
  Timer1.initialize(16666);
  Timer1.pwm(Vsync, 512);  

  delay(10); // delay 10ms until logic core is ready

  initDev();
  
  delay(300);
}

void initDev()
{
  unsigned int intReg_Conf_iW7039[34] = { 0x8020,0x00, // broadcast command with same data writting for 32 words from register 0x00
                                          0x0802, 0x83A2, 0x0800, 0x0FFF, 0x9FFC, 0x4927, 0x4585, 0x87FF, //0X00 ~ 0X07
                                          0x0100, 0x2100, 0x0000, 0x08F0, 0x0088, 0x186A, 0x5348, 0xCC0C, // 0X08 ~ 0X0F 
                                          0xFFFF, 0xFFFF, 0x0001, 0x0080, 0x010f, 0x0000, 0x0000, 0x0000, // 0X10 ~ 0X17
                                          0xFFFF, 0xFFFF, 0x0000, 0x01FF, 0x0000, 0x1000, 0xFFFF, 0xFFFF //0X18 ~0X1F
                                        }; //Initializing

  unsigned int intReg_DT_iW7039[34] = {   0x8020,0x40, // broadcast command with same data writting for 32 words for DT
                                          0x00 , 0x80 , 0x100, 0x180, 0x200, 0x280, 0x300, 0x380, //0x40 ~0x47
                                          0x400, 0x480, 0x500, 0x580, 0x600, 0x680, 0x700, 0x780, //0x48 ~ 0x4F
                                          0x800, 0x880, 0x900, 0x980, 0xA00, 0xA80, 0xB00, 0xB80, //0x50 ~ 0x57
                                          0xC00, 0xC80, 0xD00, 0xD80, 0xE00, 0xE80, 0xF00, 0xF80  //0x58 ~ 0x5F
                                      }; //0x40 to 0x5F
  unsigned int intReg_HT_iW7039[34] = {   0x8020,0x60, // broadcast command with same data writting for 32 words for HT
                                          0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, // 0x60 ~ 0x67
                                          0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, // 0x68 ~ 0x6F
                                          0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, // 0x70 ~ 0x77
                                          0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100 // 0x78 ~ 0x7F
                                      }; //0x60 to 0x7F

  Serial.println("Starting Initialization");

  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_Conf_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);

  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_DT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);

  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_HT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);

  // Reset all chip, all LEDs should turn on
  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  SPI.transfer16(0x8001);
  SPI.transfer16(0x00);
  SPI.transfer16(0x0803);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
  
  delay(10); // Keep all LEDs on for 1s

  all_off(); // Turn off all LEDs
}

void scanCluster(int pin)
{
  unsigned int intReg_Conf_iW7039[34] = { 0x8020,0x00, // broadcast command with same data writting for 32 words from register 0x00
                                          0x0802, 0x83A2, 0x0800, 0x0FFF, 0x9FFC, 0x4927, 0x4585, 0x87FF, //0X00 ~ 0X07
                                          0x0100, 0x2100, 0x0000, 0x08F0, 0x0088, 0x186A, 0x5348, 0xCC0C, // 0X08 ~ 0X0F 
                                          0xFFFF, 0xFFFF, 0x0001, 0x0080, 0x010f, 0x0000, 0x0000, 0x0000, // 0X10 ~ 0X17
                                          0xFFFF, 0xFFFF, 0x0000, 0x01FF, 0x0000, 0x1000, 0xFFFF, 0xFFFF //0X18 ~0X1F
                                        }; //Initializing

  unsigned int intReg_DT_iW7039[34] = {   0x8020,0x40, // broadcast command with same data writting for 32 words for DT
                                          0x00 , 0x80 , 0x100, 0x180, 0x200, 0x280, 0x300, 0x380, //0x40 ~0x47
                                          0x400, 0x480, 0x500, 0x580, 0x600, 0x680, 0x700, 0x780, //0x48 ~ 0x4F
                                          0x800, 0x880, 0x900, 0x980, 0xA00, 0xA80, 0xB00, 0xB80, //0x50 ~ 0x57
                                          0xC00, 0xC80, 0xD00, 0xD80, 0xE00, 0xE80, 0xF00, 0xF80  //0x58 ~ 0x5F
                                      }; //0x40 to 0x5F
  unsigned int intReg_HT_iW7039[34] = {   0x8020,0x60, // broadcast command with same data writting for 32 words for HT
                                          0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, // 0x60 ~ 0x67
                                          0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, // 0x68 ~ 0x6F
                                          0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, // 0x70 ~ 0x77
                                          0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400 // 0x78 ~ 0x7F
                                      }; //0x60 to 0x7F

  // Defines pin as the corresponding pin based on the side of the board
  if(side == 0)
  {
    pin = pinLookup[pin][1];
  }
  else if(side == 1)
  {
    pin = pinLookup[pin+18][1];
  }

  Serial.println((String)"Pin - "+pin);

  digitalWrite(pin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_Conf_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(pin, HIGH);
  delayMicroseconds(SPI_CS_DELAY);

  digitalWrite(pin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_DT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(pin, HIGH);

  digitalWrite(pin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_HT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(pin, HIGH);

  // Reset all chip, all LEDs should turn on
  digitalWrite(pin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  SPI.transfer16(0x8001);
  SPI.transfer16(0x00);
  SPI.transfer16(0x0803);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(pin, HIGH);
  delayMicroseconds(SPI_CS_DELAY);

  delay(100); // Keep all LEDs on for 100ms
}

void loop()
{
  // Looping over the 18 clusters
  Serial.print((String)"Index - "+clusterPin+", ");
  scanCluster(clusterPin); // Turn on the LED with pin number 'clusterPin'
  delay(1000); // 1s delay between each LED
  all_off(); // Comment this out if you want the LEDs to stay on
  clusterPin = clusterPin + 1;
  if(clusterPin == 18) // Exit when all the lights are turned on
  {
    exit(0);
  }
}
