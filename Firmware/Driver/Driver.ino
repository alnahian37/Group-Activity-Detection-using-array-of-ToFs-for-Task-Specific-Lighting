// Include SPI, TimerOne, Ethernet library:
#include <SPI.h>
#include <TimerOne.h>
#include <Ethernet.h>

// Define a flag for the code to be run on which troffer
byte troffer = 2;

// Define a flag for the code to be run on right (0) or left (1) Arduino
byte side = 0;

// Define the IP and MAC addresses needed for the networking of the Arduinos
// IPAddress ipAddresses[] = {IPAddress(192, 168, 0, 80),
//                            IPAddress(192, 168, 0, 81),
//                            IPAddress(192, 168, 0, 61),
//                            IPAddress(192, 168, 0, 57),
//                            IPAddress(192, 168, 0, 84),
//                            IPAddress(192, 168, 0, 85),
//                            IPAddress(192, 168, 0, 86),
//                            IPAddress(192, 168, 0, 87)};

// byte macAddresses[][6] = {{0xA8, 0x61, 0x0A, 0xAE, 0x96, 0x06},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0x96, 0x04},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0x95, 0xF5},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x7D},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x64},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x6A},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0x96, 0x0E},
//                           {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x99}};

// int arduinoIndex = ((troffer - 1) * 2) + side;

// IPAddress ipRight = ipAddresses[arduinoIndex];
// IPAddress ipLeft = ipAddresses[arduinoIndex];

// byte macRight = macAddresses[arduinoIndex];
// byte macLeft = macAddresses[arduinoIndex];

IPAddress ipRight(192, 168, 0, 54);
IPAddress ipLeft(192, 168, 0, 23);
// IPAddress ipRight(192, 168, 0, 48);
// IPAddress ipLeft(192, 168, 0, 40);
// IPAddress ipRight(192, 168, 0, 61);
// IPAddress ipLeft(192, 168, 0, 57);
// IPAddress ipRight(192, 168, 0, 46);
// IPAddress ipLeft(192, 168, 0, 24);
// IPAddress ipRight(192, 168, 0, 63);
// IPAddress ipLeft(192, 168, 0, 55);
byte macRight[] = {0xA8, 0x61, 0x0A, 0xAE, 0x96, 0x0E};
byte macLeft[] = {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x99};
// byte macRight[] = {0xA8, 0x61, 0x0A, 0xAE, 0x95, 0xF5};
// byte macLeft[] = {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x7D};
// byte macRight[] = {0xA8, 0x61, 0x0A, 0xAE, 0x95, 0xE7};
// byte macLeft[] = {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x6C};
// byte macRight[] = {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x8F};
// byte macLeft[] = {0xA8, 0x61, 0x0A, 0xAE, 0x96, 0x05};

EthernetServer server(23); // Using port 23 for the telnet server
bool gotAMessage = false;  // Whether or not you got a message from the client yet
int dataBuffer[53];        // buffer to hold the entire array of multipliers
char charBuffer;           // buffer to hold the individual characters of the input string

// Define PWM stuff
int frames = 10;     // Default number of frames to change from oldData to newData
float frameStep = 0; // Default frame step

// Define color stuff
// For spot lights
float redSpot = 1;   // Default red value
float greenSpot = 1; // Default green value
float blueSpot = 1;  // Default blue value
// For edge lights
float redEdge = 1;   // Default red value
float greenEdge = 1; // Default green value
float blueEdge = 1;  // Default blue value

// Define the frame data arrays
unsigned int oldData[53];
unsigned int newData[53];
unsigned int currentFrameData[53];
unsigned int edgeData[4];
unsigned int edgeDataRed[4];
unsigned int edgeDataGreen[4];
unsigned int edgeDataBlue[4];
unsigned int pixelData[49];
unsigned int pixelDataRed[49];
unsigned int pixelDataGreen[49];
unsigned int pixelDataBlue[49];

// byte newData2[53];

unsigned int Driver1_Reg_HT_iW7039[34];
unsigned int Driver2_Reg_HT_iW7039[34];
unsigned int Driver3_Reg_HT_iW7039[34];
unsigned int Driver4_Reg_HT_iW7039[34];
unsigned int Driver5_Reg_HT_iW7039[34];

// Define other pins needed by SPI library
#define Vsync 11

// Define SPI_CS_DELAY 25us
#define SPI_CS_DELAY 25

// Defining the pin lookup arrays
int pinPixelLookup[][2] = {{1, 32}, {2, 35}, {3, 38}, {4, 41}, {5, 44}, {6, 47}, {7, 31}, {8, 34}, {9, 37}, {10, 40}, {11, 43}, {12, 46}, {13, 30}, {14, 33}, {15, 36}, {16, 39}, {17, 42}, {18, 45}, {19, 27}, {20, 24}, {21, 21}, {22, 18}, {23, 15}, {24, 3}, {25, 28}, {26, 25}, {27, 22}, {28, 19}, {29, 16}, {30, 2}, {31, 29}, {32, 26}, {33, 23}, {34, 20}, {35, 17}, {36, 14}};
int pinEdgeLookup[][2] = {{1, 66}, {2, 67}, {3, 68}, {4, 69}};

word red_cmd_1[49] = {0x8101, 0x8101, 0x8101, 0x8101, 0x8101, 0x8101, 0x8101,
                      0x8101, 0x8101, 0x8201, 0x8101, 0x8101, 0x8101, 0x8101,
                      0x8101, 0x8101, 0x8201, 0x8201, 0x8201, 0x8201, 0x8101,
                      0x8101, 0x8201, 0x8201, 0x8201, 0x8201, 0x8101, 0x8101,
                      0x8101, 0x8201, 0x8201, 0x8201, 0x8201, 0x8101, 0x8101,
                      0x8101, 0x8201, 0x8201, 0x8201, 0x8201, 0x8101, 0x8101,
                      0x8101, 0x8101, 0x8101, 0x8101, 0x8101, 0x8101, 0x8101};
word red_cmd_2[49] = {0x11, 0x11, 0x11, 0x10, 0x11, 0x11, 0x11,
                      0x11, 0x11, 0x11, 0x10, 0x10, 0x10, 0x10,
                      0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
                      0x11, 0x11, 0x11, 0x10, 0x11, 0x10, 0x11,
                      0x10, 0x11, 0x11, 0x11, 0x11, 0x10, 0x11,
                      0x10, 0x11, 0x11, 0x11, 0x11, 0x10, 0x11,
                      0x10, 0x10, 0x10, 0x11, 0x10, 0x10, 0x10};
word red_cmd_3[49] = {0x1000, 0x800, 0x400, 0x4000, 0x200, 0x100, 0x80,
                      0x1, 0x2000, 0x1, 0x8000, 0x2000, 0x1000, 0x800,
                      0x2, 0x4000, 0x2, 0x1000, 0x800, 0x400, 0x40,
                      0x8000, 0x4, 0x2000, 0x1, 0x200, 0x400, 0x20,
                      0x1, 0x8, 0x4000, 0x8000, 0x100, 0x200, 0x10,
                      0x2, 0x10, 0x20, 0x40, 0x80, 0x100, 0x8,
                      0x4, 0x8, 0x10, 0x4, 0x20, 0x40, 0x80};

word green_cmd_1[49] = {0x8201, 0x8201, 0x8201, 0x8301, 0x8201, 0x8201, 0x8201,
                        0x8201, 0x8201, 0x8301, 0x8301, 0x8301, 0x8301, 0x8301,
                        0x8201, 0x8201, 0x8301, 0x8301, 0x8301, 0x8301, 0x8201,
                        0x8301, 0x8301, 0x8301, 0x8401, 0x8301, 0x8301, 0x8201,
                        0x8301, 0x8301, 0x8301, 0x8401, 0x8301, 0x8301, 0x8201,
                        0x8301, 0x8301, 0x8301, 0x8301, 0x8301, 0x8301, 0x8201,
                        0x8301, 0x8301, 0x8301, 0x8201, 0x8301, 0x8301, 0x8301};
word green_cmd_2[49] = {0x10, 0x10, 0x10, 0x11, 0x10, 0x10, 0x10,
                        0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11,
                        0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
                        0x11, 0x10, 0x10, 0x11, 0x10, 0x11, 0x10,
                        0x11, 0x10, 0x10, 0x11, 0x10, 0x11, 0x10,
                        0x11, 0x10, 0x10, 0x10, 0x10, 0x11, 0x10,
                        0x11, 0x11, 0x11, 0x10, 0x11, 0x11, 0x11};
word green_cmd_3[49] = {0x2000, 0x1000, 0x800, 0x8000, 0x400, 0x200, 0x100,
                        0x2, 0x4000, 0x2, 0x1, 0x4000, 0x2000, 0x1000,
                        0x4, 0x8000, 0x4, 0x2000, 0x1000, 0x800, 0x80,
                        0x1, 0x8, 0x4000, 0x2, 0x400, 0x800, 0x40,
                        0x2, 0x10, 0x8000, 0x1, 0x200, 0x400, 0x20,
                        0x4, 0x20, 0x40, 0x80, 0x100, 0x200, 0x10,
                        0x8, 0x10, 0x20, 0x8, 0x40, 0x80, 0x100};

word blue_cmd_1[49] = {0x8401, 0x8401, 0x8401, 0x8501, 0x8401, 0x8401, 0x8401,
                       0x8401, 0x8401, 0x8501, 0x8501, 0x8401, 0x8401, 0x8401,
                       0x8401, 0x8401, 0x8501, 0x8501, 0x8501, 0x8501, 0x8401,
                       0x8401, 0x8501, 0x8501, 0x8501, 0x8501, 0x8401, 0x8401,
                       0x8401, 0x8501, 0x8501, 0x8501, 0x8501, 0x8401, 0x8401,
                       0x8401, 0x8501, 0x8501, 0x8501, 0x8501, 0x8401, 0x8401,
                       0x8401, 0x8401, 0x8401, 0x8401, 0x8401, 0x8401, 0x8401};
word blue_cmd_2[49] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
                       0x11, 0x11, 0x11, 0x11, 0x10, 0x10, 0x10,
                       0x11, 0x10, 0x11, 0x11, 0x11, 0x11, 0x11,
                       0x10, 0x11, 0x11, 0x10, 0x11, 0x10, 0x11,
                       0x10, 0x11, 0x10, 0x10, 0x11, 0x10, 0x11,
                       0x10, 0x11, 0x11, 0x11, 0x11, 0x10, 0x11,
                       0x10, 0x10, 0x10, 0x11, 0x10, 0x10, 0x10};
word blue_cmd_3[49] = {0x4000, 0x2000, 0x1000, 0x1, 0x800, 0x400, 0x200,
                       0x4, 0x8000, 0x4, 0x2, 0x8000, 0x4000, 0x2000,
                       0x8, 0x1, 0x8, 0x4000, 0x2000, 0x1000, 0x100,
                       0x2, 0x10, 0x8000, 0x4, 0x800, 0x1000, 0x80,
                       0x4, 0x20, 0x1, 0x2, 0x400, 0x800, 0x40,
                       0x8, 0x40, 0x80, 0x100, 0x200, 0x400, 0x20,
                       0x10, 0x20, 0x40, 0x10, 0x80, 0x100, 0x200};

word allOffData[2][4] = {{0x8001, 0x10, 0x0, 0},
                         {0x8001, 0x11, 0x0, 0}};

word allOnData[4] = {0x8001, 0x00, 0x0803, 0};

/**
 * Sends the initialization register to the LED driver.
 *
 * @param reg An array of 34 unsigned integers representing the initialization register.
 */
void send_init_reg(unsigned int *reg)
{
  chipPixelUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(reg[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

/**
 * Sends 4 words of data to the LED driver.
 *
 * @param data An array of 4 words of data to send.
 */
void send_4words(word *data)
{
  chipPixelUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (byte i = 0; i < 4; i++)
    SPI.transfer16(data[i]);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

/**
 * Sends 3 words of data to the LED driver, followed by a zero word.
 *
 * @param cmd1 The first word of data to send.
 * @param cmd2 The second word of data to send.
 * @param cmd3 The third word of data to send.
 */
void send_3words_plus_zero(word cmd1, word cmd2, word cmd3)
{
  chipPixelUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  SPI.transfer16(cmd1);
  SPI.transfer16(cmd2);
  SPI.transfer16(cmd3);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

/**
 * Sends 2 blocks of 4 words of data to the LED driver, with a delay between each block.
 *
 * @param data0 The 2 blocks of 4 words of data to send.
 * @param milliSeconds The delay in milliseconds between each block of data.
 * @param turnOff Whether to turn off the LEDs after sending the data.
 * @param turnOffDelay The delay in milliseconds before turning off the LEDs.
 */
void send_2blocks_of_4words(word data0[2][4], word milliSeconds, bool turnOff, word turnOffDelay)
{
  for (byte i = 0; i < 2; i++)
  {
    send_4words(data0[i]);
  }
  delay(milliSeconds);
  if (turnOff)
  {
    for (byte j = 0; j < 2; j++)
    {
      send_4words(allOffData[j]);
    }
    delay(turnOffDelay);
  }
}

/**
 * Sends 3 blocks of 4 words of data to the LED driver, with a delay between each block.
 *
 * @param data0 The 3 blocks of 4 words of data to send.
 * @param milliSeconds The delay in milliseconds between each block of data.
 * @param turnOff Whether to turn off the LEDs after sending the data.
 * @param turnOffDelay The delay in milliseconds before turning off the LEDs.
 */
void send_3blocks_of_4words(word data0[3][4], word milliSeconds, bool turnOff, word turnOffDelay)
{
  for (byte i = 0; i < 3; i++)
  {
    send_4words(data0[i]);
  }
  delay(milliSeconds);
  if (turnOff)
  {
    for (byte j = 0; j < 2; j++)
    {
      send_4words(allOffData[j]);
    }
    delay(turnOffDelay);
  }
}

/**
 * Sends 4 blocks of 4 words of data to the LED driver, with a delay between each block.
 *
 * @param data0 The 4 blocks of 4 words of data to send.
 * @param milliSeconds The delay in milliseconds between each block of data.
 * @param turnOff Whether to turn off the LEDs after sending the data.
 * @param turnOffDelay The delay in milliseconds before turning off the LEDs.
 */
void send_4blocks_of_4words(word data0[4][4], word milliSeconds, bool turnOff, word turnOffDelay)
{
  for (byte i = 0; i < 4; i++)
  {
    send_4words(data0[i]);
  }
  delay(milliSeconds);
  if (turnOff)
  {
    for (byte j = 0; j < 2; j++)
    {
      send_4words(allOffData[j]);
    }
    delay(turnOffDelay);
  }
}

/**
 * Turns off all lights by sending 2 blocks of 4 words of data to the LED driver with all zeros, with a delay between each block.
 */
void all_off()
{
  send_2blocks_of_4words(allOffData, 1000, false, 0);
}

/**
 * Turns off all lights using PWM and setting newData[] to 0.
 */
void all_off_PWM()
{
  for (byte i = 0; i < 53; i++)
  {
    newData[i] = 0;
  }
  by_data_pwm(1, oldData, newData);
}

/**
 * Turns on all lights by sending 4 words of data to the LED driver with all ones.
 */
void all_on()
{
  send_4words(allOnData);
}

/**
 * Turns on all lights using PWM and setting newData[] to the specified intensity.
 *
 * @param intensity The intensity value to set for all pixels.
 */
void all_on_PWM(int intensity)
{
  for (byte i = 0; i < 53; i++)
  {
    newData[i] = intensity;
  }
  by_data_pwm(2, oldData, newData);
}

/**
 * Updates the chip select pins for the pixels to HIGH or LOW.
 *
 * @param state The state to set the pins to (0 for LOW, 1 for HIGH).
 */
void chipPixelUpdate(int state)
{
  if (side == 0)
  {
    if (state == 0) // corresponds to the low state
    {
      for (uint8_t i = 0; i < 18; i++)
      {
        digitalWrite(pinPixelLookup[i][1], LOW); // set as LOW
      }
    }
    else // corresponds to the high state
    {
      for (uint8_t i = 0; i < 18; i++)
      {
        digitalWrite(pinPixelLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else if (side == 1)
  {
    if (state == 0) // corresponds to the low state
    {
      for (uint8_t i = 18; i < 36; i++)
      {
        digitalWrite(pinPixelLookup[i][1], LOW); // set as LOW
      }
    }
    else if (state == 1) // corresponds to the high state
    {
      for (uint8_t i = 18; i < 36; i++)
      {
        digitalWrite(pinPixelLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else
  {
    Serial.println("Error: side not defined");
  }
}

// Update the chip select pings for the edges to HIGH or LOW
void chipEdgeUpdate(int state)
{
  if (side == 0)
  {
    if (state == 0) // corresponds to the low state
    {
      for (uint8_t i = 0; i < 2; i++)
      {
        digitalWrite(pinEdgeLookup[i][1], LOW); // set as LOW
      }
    }
    else // corresponds to the high state
    {
      for (uint8_t i = 0; i < 2; i++)
      {
        digitalWrite(pinEdgeLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else if (side == 1)
  {
    if (state == 0) // corresponds to the low state
    {
      for (uint8_t i = 2; i < 4; i++)
      {
        digitalWrite(pinEdgeLookup[i][1], LOW); // set as LOW
      }
    }
    else if (state == 1) // corresponds to the high state
    {
      for (uint8_t i = 2; i < 4; i++)
      {
        digitalWrite(pinEdgeLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else
  {
    Serial.println("Error: side not defined");
  }
}

void initDev()
{
  unsigned int intReg_Conf_iW7039[34] = {
      0x8020, 0x00,                                                   // broadcast command with same data writting for 32 words from register 0x00
      0x0802, 0x83A2, 0x0800, 0x0FFF, 0x9FFC, 0x4927, 0x4585, 0x87FF, // 0X00 ~ 0X07
      0x0100, 0x2100, 0x0000, 0x08F0, 0x0088, 0x186A, 0x5348, 0xCC0C, // 0X08 ~ 0X0F
      0xFFFF, 0xFFFF, 0x0001, 0x0080, 0x010f, 0x0000, 0x0000, 0x0000, // 0X10 ~ 0X17
      0xFFFF, 0xFFFF, 0x0000, 0x01FF, 0x0000, 0x1000, 0xFFFF, 0xFFFF  // 0X18 ~0X1F
  };                                                                  // Initializing

  unsigned int intReg_DT_iW7039[34] = {
      0x8020, 0x40,                                           // broadcast command with same data writting for 32 words for DT
      0x00, 0x80, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380,   // 0x40 ~0x47
      0x400, 0x480, 0x500, 0x580, 0x600, 0x680, 0x700, 0x780, // 0x48 ~ 0x4F
      0x800, 0x880, 0x900, 0x980, 0xA00, 0xA80, 0xB00, 0xB80, // 0x50 ~ 0x57
      0xC00, 0xC80, 0xD00, 0xD80, 0xE00, 0xE80, 0xF00, 0xF80  // 0x58 ~ 0x5F
  };                                                          // 0x40 to 0x5F DT: Delay Time

  unsigned int intReg_HT_iW7039[34] = {
      0x8020, 0x60,                                           // broadcast command with same data writting for 32 words for HT
      0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, // 0x60 ~ 0x67
      0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, // 0x68 ~ 0x6F
      0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, // 0x70 ~ 0x77
      0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1  // 0x78 ~ 0x7F
  };                                                          // 0x60 to 0x7F

  Serial.println("Starting Initialization");

  chipPixelUpdate(0);
  chipEdgeUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_Conf_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  chipEdgeUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);

  chipPixelUpdate(0);
  chipEdgeUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_DT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  chipEdgeUpdate(1);

  chipPixelUpdate(0);
  chipEdgeUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(intReg_HT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  chipEdgeUpdate(1);

  // Reset all chip, all LEDs should turn on
  chipPixelUpdate(0);
  chipEdgeUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  SPI.transfer16(0x8001);
  SPI.transfer16(0x00);
  SPI.transfer16(0x0803);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipPixelUpdate(1);
  chipEdgeUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);

  delay(1000); // Keep all LEDs on for 1s

  // all_off(); // Turn off all LEDs

  all_off_PWM(); // Turn off all LEDs using PWM
}

void setup()
{
  delay(1000); // Wait for the serial port to connect.

  // Open serial communications
  Serial.begin(9600);
  // Start the SPI library
  SPI.begin();

  // Networking part begins here

  // Start the Ethernet connection and the server based on the side
  if (side == 0)
  {
    Ethernet.begin(macRight, ipRight);
    Serial.println("Right side started");
  }
  else if (side == 1)
  {
    Ethernet.begin(macLeft, ipLeft);
    Serial.println("Left side started");
  }
  else
  {
    Serial.println("Error: side not defined");
  }

  delay(1000);

  // Check if the Ethernet hardware is present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found. Sorry, can't run without hardware.");
    while (true)
    {
      delay(1); // Do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("Ethernet cable is not connected.");
  }

  delay(1000);

  Serial.print("Chat server address : ");
  Serial.println(Ethernet.localIP());

  // Networking part ends here

  // Initalize all the chip select pins
  for (uint8_t i = 0; i < 36; i++)
  {
    pinMode(pinPixelLookup[i][1], OUTPUT); // Set as output
    digitalWrite(i, LOW);                  // Set as LOW
  }
  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(pinEdgeLookup[i][1], OUTPUT); // Set as output
    digitalWrite(i, LOW);                 // Set as LOW
  }

  // Add external 120Hz Vsync, 50% duty cycle
  pinMode(Vsync, OUTPUT);
  Timer1.initialize(16666);
  Timer1.pwm(Vsync, 512);

  // Delay 100ms until logic core is ready
  delay(100);

  initDev();

  // Delay after initialization
  delay(100);

  // Start listening for clients
  server.begin();
  Serial.println("Began Server");

  delay(100); // Delay 100ms
}

// Does not work as of now. Need to fix based on the edge lights code.
void by_data(byte data[])
{
  // According to binary data, all colors
  int pin = 0;
  int intensity = 0;
  for (byte j = 0; j < 4; j++)
  {
    // count = i;
    // if (data[count] == 1)
    // {
    if (side == 0)
    {
      pin = pinPixelLookup[j + 18][1];
    }
    else if (side == 1)
    {
      pin = pinPixelLookup[j + 38][1];
    }

    intensity = data[j] * 4095;

    unsigned int Edge_Reg_HT_iW7039[34] = {0x8120, 0x60, intensity, intensity, intensity, intensity, intensity, intensity};

    Serial.println((String) "Pin - " + pin);
    Serial.println((String) "Intensity - " + intensity);

    digitalWrite(pin, LOW);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 8; i++)
      SPI.transfer16(Edge_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    digitalWrite(pin, HIGH);

    delay(1000); // Keep all LEDs on for 1s
  }

  byte count = 0;

  for (byte i = 0; i < 7; i++)
  {
    for (byte j = 0; j < 7; j++)
    {
      count = (i * 7 + j) + 4; // Skipping the first four multipliers for the edge strips
      if (data[count] == 1)
      {
        Serial.print("Count - ");
        Serial.println(count);
        send_3words_plus_zero(red_cmd_1[count], red_cmd_2[count], red_cmd_3[count]);
        send_3words_plus_zero(green_cmd_1[count], green_cmd_2[count], green_cmd_3[count]);
        send_3words_plus_zero(blue_cmd_1[count], blue_cmd_2[count], blue_cmd_3[count]);
      }
    }
  }
  delay(1);
}

// Changes PWM values for all LEDs from oldData to newData in frames steps with color control
void by_data_pwm(int frames, int oldData[], int newData[])
{
  redSpot = redSpot / 255.0;
  greenSpot = greenSpot / 255.0;
  blueSpot = blueSpot / 255.0;
  redEdge = redEdge / 255.0;
  greenEdge = greenEdge / 255.0;
  blueEdge = blueEdge / 255.0;

  // Serial.println(frames);
  // Serial.println(redSpot);
  // Serial.println(greenSpot);
  // Serial.println(blueSpot);
  // Serial.println(redEdge);
  // Serial.println(greenEdge);
  // Serial.println(blueEdge);

  for (int frame = 0; frame <= frames; frame++)
  {
    frameStep = (float)frame / (float)frames;

    for (byte i = 0; i < 53; i++)
    {
      currentFrameData[i] = oldData[i] + ((newData[i] - oldData[i]) * frameStep);
      // Serial.print(currentFrameData[i]);
      // Serial.print("-");
      if (i < 4)
      {
        edgeData[i] = currentFrameData[i];
      }
      else if (i < 53)
      {
        pixelData[i - 4] = currentFrameData[i];
      }
    }
    // Serial.print("Frame = ");
    // Serial.print(frame);
    // Serial.println("_");

    // Creating new edgeData and pixelData arrays
    // edgeData array to edgeDataRed, edgeDataGreen, edgeDataBlue
    for (byte i = 0; i < 4; i++)
    {
      edgeDataRed[i] = (int)(redEdge * (float)edgeData[i]);
      edgeDataGreen[i] = (int)(greenEdge * (float)edgeData[i]);
      edgeDataBlue[i] = (int)(blueEdge * (float)edgeData[i]);
    }
    // pixelData array to pixelDataRed, pixelDataGreen, pixelDataBlue
    for (byte i = 0; i < 49; i++)
    {
      pixelDataRed[i] = (int)(redSpot * (float)pixelData[i]);
      pixelDataGreen[i] = (int)(greenSpot * (float)pixelData[i]);
      pixelDataBlue[i] = (int)(blueSpot * (float)pixelData[i]);
    }

    // Updating the edges
    int pin = 0;

    if (side == 0)
    {
      unsigned int Edge1_Reg_HT_iW7039[8] = {0x8120, 0x60, edgeDataRed[0], edgeDataRed[0], edgeDataGreen[0], edgeDataGreen[0], edgeDataBlue[0], edgeDataBlue[0]};

      pin = pinEdgeLookup[0][1];

      // Serial.println("Updating edge 1 - ");
      // Serial.print(edgeData[0]);
      // Serial.print(" - ");
      // Serial.println(pin);

      digitalWrite(pin, LOW);
      delayMicroseconds(SPI_CS_DELAY);
      for (int i = 0; i < 8; i++)
        SPI.transfer16(Edge1_Reg_HT_iW7039[i]);
      SPI.transfer16(0);
      delayMicroseconds(SPI_CS_DELAY);
      digitalWrite(pin, HIGH);

      unsigned int Edge2_Reg_HT_iW7039[8] = {0x8120, 0x60, edgeDataRed[1], edgeDataRed[1], edgeDataGreen[1], edgeDataGreen[1], edgeDataBlue[1], edgeDataBlue[1]};

      pin = pinEdgeLookup[1][1];

      digitalWrite(pin, LOW);
      delayMicroseconds(SPI_CS_DELAY);
      for (int i = 0; i < 8; i++)
        SPI.transfer16(Edge2_Reg_HT_iW7039[i]);
      SPI.transfer16(0);
      delayMicroseconds(SPI_CS_DELAY);
      digitalWrite(pin, HIGH);
    }
    else if (side == 1)
    {
      unsigned int Edge3_Reg_HT_iW7039[8] = {0x8120, 0x60, edgeDataRed[2], edgeDataRed[2], edgeDataGreen[2], edgeDataGreen[2], edgeDataBlue[2], edgeDataBlue[2]};

      pin = pinEdgeLookup[2][1];

      digitalWrite(pin, LOW);
      delayMicroseconds(SPI_CS_DELAY);
      for (int i = 0; i < 8; i++)
        SPI.transfer16(Edge3_Reg_HT_iW7039[i]);
      SPI.transfer16(0);
      delayMicroseconds(SPI_CS_DELAY);
      digitalWrite(pin, HIGH);

      unsigned int Edge4_Reg_HT_iW7039[8] = {0x8120, 0x60, edgeDataRed[3], edgeDataRed[3], edgeDataGreen[3], edgeDataGreen[3], edgeDataBlue[3], edgeDataBlue[3]};

      pin = pinEdgeLookup[3][1];

      digitalWrite(pin, LOW);
      delayMicroseconds(SPI_CS_DELAY);
      for (int i = 0; i < 8; i++)
        SPI.transfer16(Edge4_Reg_HT_iW7039[i]);
      SPI.transfer16(0);
      delayMicroseconds(SPI_CS_DELAY);
      digitalWrite(pin, HIGH);
    }

    // Updating the pixels
    // unsigned int Driver1_Reg_HT_iW7039[34] = {0x8120, 0x60, pixelData[35], pixelData[28], pixelData[3], pixelData[13], pixelData[20], pixelData[27], pixelData[34], pixelData[48], pixelData[47], pixelData[46], pixelData[44], pixelData[43], pixelData[42], pixelData[36], pixelData[29], pixelData[21], pixelData[14], pixelData[7], pixelData[0], pixelData[1], pixelData[2], pixelData[4], pixelData[5], pixelData[6], pixelData[12], pixelData[19], pixelData[26], pixelData[41], pixelData[40], pixelData[39], pixelData[45], pixelData[38]};
    // unsigned int Driver2_Reg_HT_iW7039[34] = {0x8220, 0x60, pixelData[37], pixelData[30], pixelData[22], pixelData[15], pixelData[8], pixelData[9], pixelData[10], pixelData[11], pixelData[18], pixelData[25], pixelData[33], pixelData[32], pixelData[31], pixelData[23], pixelData[16], pixelData[17], pixelData[24], pixelData[35], pixelData[28], pixelData[3], pixelData[13], pixelData[20], pixelData[27], pixelData[34], pixelData[48], pixelData[47], pixelData[46], pixelData[44], pixelData[43], pixelData[42], pixelData[36], pixelData[29]};
    // unsigned int Driver3_Reg_HT_iW7039[34] = {0x8320, 0x60, pixelData[21], pixelData[14], pixelData[7], pixelData[0], pixelData[1], pixelData[2], pixelData[4], pixelData[5], pixelData[6], pixelData[12], pixelData[19], pixelData[26], pixelData[41], pixelData[40], pixelData[39], pixelData[45], pixelData[38], pixelData[37], pixelData[30], pixelData[22], pixelData[15], pixelData[8], pixelData[9], pixelData[10], pixelData[11], pixelData[18], pixelData[25], pixelData[33], pixelData[32], pixelData[31], pixelData[23], pixelData[16]};
    // unsigned int Driver4_Reg_HT_iW7039[34] = {0x8420, 0x60, pixelData[17], pixelData[24], pixelData[35], pixelData[28], pixelData[3], pixelData[13], pixelData[20], pixelData[27], pixelData[34], pixelData[48], pixelData[47], pixelData[46], pixelData[44], pixelData[43], pixelData[42], pixelData[36], pixelData[29], pixelData[21], pixelData[14], pixelData[7], pixelData[0], pixelData[1], pixelData[2], pixelData[4], pixelData[5], pixelData[6], pixelData[12], pixelData[19], pixelData[26], pixelData[41], pixelData[40], pixelData[39]};
    // unsigned int Driver5_Reg_HT_iW7039[34] = {0x8520, 0x60, pixelData[45], pixelData[38], pixelData[37], pixelData[30], pixelData[22], pixelData[15], pixelData[8], pixelData[9], pixelData[10], pixelData[11], pixelData[18], pixelData[25], pixelData[33], pixelData[32], pixelData[31], pixelData[23], pixelData[16], pixelData[17], pixelData[24], 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    // Updating the pixels based on color
    unsigned int Driver1_Reg_HT_iW7039[34] = {0x8120, 0x60, pixelDataRed[35], pixelDataRed[28], pixelDataRed[3], pixelDataRed[13], pixelDataRed[20], pixelDataRed[27], pixelDataRed[34], pixelDataRed[48], pixelDataRed[47], pixelDataRed[46], pixelDataRed[44], pixelDataRed[43], pixelDataRed[42], pixelDataRed[36], pixelDataRed[29], pixelDataRed[21], pixelDataRed[14], pixelDataRed[7], pixelDataRed[0], pixelDataRed[1], pixelDataRed[2], pixelDataRed[4], pixelDataRed[5], pixelDataRed[6], pixelDataRed[12], pixelDataRed[19], pixelDataRed[26], pixelDataRed[41], pixelDataRed[40], pixelDataRed[39], pixelDataRed[45], pixelDataRed[38]};
    unsigned int Driver2_Reg_HT_iW7039[34] = {0x8220, 0x60, pixelDataRed[37], pixelDataRed[30], pixelDataRed[22], pixelDataRed[15], pixelDataRed[8], pixelDataRed[9], pixelDataRed[10], pixelDataRed[11], pixelDataRed[18], pixelDataRed[25], pixelDataRed[33], pixelDataRed[32], pixelDataRed[31], pixelDataRed[23], pixelDataRed[16], pixelDataRed[17], pixelDataRed[24], pixelDataGreen[35], pixelDataGreen[28], pixelDataGreen[3], pixelDataGreen[13], pixelDataGreen[20], pixelDataGreen[27], pixelDataGreen[34], pixelDataGreen[48], pixelDataGreen[47], pixelDataGreen[46], pixelDataGreen[44], pixelDataGreen[43], pixelDataGreen[42], pixelDataGreen[36], pixelDataGreen[29]};
    unsigned int Driver3_Reg_HT_iW7039[34] = {0x8320, 0x60, pixelDataGreen[21], pixelDataGreen[14], pixelDataGreen[7], pixelDataGreen[0], pixelDataGreen[1], pixelDataGreen[2], pixelDataGreen[4], pixelDataGreen[5], pixelDataGreen[6], pixelDataGreen[12], pixelDataGreen[19], pixelDataGreen[26], pixelDataGreen[41], pixelDataGreen[40], pixelDataGreen[39], pixelDataGreen[45], pixelDataGreen[38], pixelDataGreen[37], pixelDataGreen[30], pixelDataGreen[22], pixelDataGreen[15], pixelDataGreen[8], pixelDataGreen[9], pixelDataGreen[10], pixelDataGreen[11], pixelDataGreen[18], pixelDataGreen[25], pixelDataGreen[33], pixelDataGreen[32], pixelDataGreen[31], pixelDataGreen[23], pixelDataGreen[16]};
    unsigned int Driver4_Reg_HT_iW7039[34] = {0x8420, 0x60, pixelDataGreen[17], pixelDataGreen[24], pixelDataBlue[35], pixelDataBlue[28], pixelDataBlue[3], pixelDataBlue[13], pixelDataBlue[20], pixelDataBlue[27], pixelDataBlue[34], pixelDataBlue[48], pixelDataBlue[47], pixelDataBlue[46], pixelDataBlue[44], pixelDataBlue[43], pixelDataBlue[42], pixelDataBlue[36], pixelDataBlue[29], pixelDataBlue[21], pixelDataBlue[14], pixelDataBlue[7], pixelDataBlue[0], pixelDataBlue[1], pixelDataBlue[2], pixelDataBlue[4], pixelDataBlue[5], pixelDataBlue[6], pixelDataBlue[12], pixelDataBlue[19], pixelDataBlue[26], pixelDataBlue[41], pixelDataBlue[40], pixelDataBlue[39]};
    unsigned int Driver5_Reg_HT_iW7039[34] = {0x8520, 0x60, pixelDataBlue[45], pixelDataBlue[38], pixelDataBlue[37], pixelDataBlue[30], pixelDataBlue[22], pixelDataBlue[15], pixelDataBlue[8], pixelDataBlue[9], pixelDataBlue[10], pixelDataBlue[11], pixelDataBlue[18], pixelDataBlue[25], pixelDataBlue[33], pixelDataBlue[32], pixelDataBlue[31], pixelDataBlue[23], pixelDataBlue[16], pixelDataBlue[17], pixelDataBlue[24], 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    chipPixelUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver1_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipPixelUpdate(1);

    chipPixelUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver2_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipPixelUpdate(1);

    chipPixelUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver3_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipPixelUpdate(1);

    chipPixelUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver4_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipPixelUpdate(1);

    chipPixelUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver5_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipPixelUpdate(1);
  }
  Serial.println("Finished Updating");
}

void loop()
{
  // Wait for the client to connect
  EthernetClient client = server.available();
  if (client)
  {
    Serial.println("New data incoming");
    while (client.connected())
    {
      if (client.available())
      {
        String inputString = client.readStringUntil(']'); // Read the data from the server until the end of the line
        // Replacing all the characters that are not needed
        inputString.replace("[", "");
        inputString.replace("]", "");
        inputString.replace(" ", "");
        if(inputString.length() == 1)
        {
          setup();
        }
        char charBuffer[inputString.length() + 1];                     // Creating charBuffer to store the inputString as a char array
        inputString.toCharArray(charBuffer, inputString.length() + 1); // Converting the inputString to a char array
        char *ptr = strtok(charBuffer, ",");                           // Splitting the char array into a char pointer
        frames = atoi(ptr);                                            // Converting the first value to an int which is the number of frames
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        redSpot = atoi(ptr);                                           // Converting the second value to a float which is the redSpot value
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        greenSpot = atoi(ptr);                                         // Converting the third value to a float which is the greenSpot value
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        blueSpot = atoi(ptr);                                          // Converting the fourth value to a float which is the blueSpot value
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        redEdge = atoi(ptr);                                           // Converting the fifth value to a float which is the redEdge value
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        greenEdge = atoi(ptr);                                         // Converting the sixth value to a float which is the greenEdge value
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        blueEdge = atoi(ptr);                                          // Converting the seventh value to a float which is the blueEdge value
        ptr = strtok(NULL, ",");                                       // Getting ready to read the next value
        int i = 0;
        while (ptr != NULL)
        {
          dataBuffer[i++] = atoi(ptr); // Converting the char pointer to an int and storing it in the dataBuffer
          ptr = strtok(NULL, ",");     // Getting ready to read the next value
        }
        break;
      }
    }

    // client.stop();

    // Serial.println(frames);
    // Serial.println(redSpot);
    // Serial.println(greenSpot);
    // Serial.println(blueSpot);
    // Serial.println(redEdge);
    // Serial.println(greenEdge);
    // Serial.println(blueEdge);

    memcpy(newData, dataBuffer, sizeof(dataBuffer)); // Copy data from dataBuffer to newData
    Serial.println("New data copied");
    by_data_pwm(frames, oldData, newData);     // Transition lights in a PWM fashion
    memcpy(oldData, newData, sizeof(newData)); // Copy newData to oldData for next iteration

    delay(10); // Delay in between reads for stability
  }
  Ethernet.maintain();
}
