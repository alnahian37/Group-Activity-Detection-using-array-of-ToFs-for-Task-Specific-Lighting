// Include SPI, TimerOne, Ethernet library:
#include <SPI.h>
#include <TimerOne.h>
#include <Ethernet.h>

// Define a flag for the code to be run on right (0) or left (1) Arduino
byte side = 0;

// Define the IP and MAC addresses needed for the networking of the Arduinos
// IPAddress ipRight(192, 168, 0, 54);
// IPAddress ipLeft(192, 168, 0, 23);
// IPAddress ipRight(192,168,0,62);
// IPAddress ipLeft(192,168,0,70);
// byte macRight[] = {0xA8, 0x61, 0x0A, 0xAE, 0x95, 0xF4};
// byte macLeft[] = {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x7C};
// byte macRight[] = {0xA8, 0x61, 0x0A, 0xAE, 0x95, 0xF5};
// byte macLeft[] = {0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0x7D};

// EthernetServer server(23); // Using port 23 for the telnet server
// bool gotAMessage = false;  // Whether or not you got a message from the client yet
// int dataBuffer[49];        // buffer to hold the entire array of multipliers
// char charBuffer;           // buffer to hold the individual characters of the input string

// Edge lights PWM signal values
int edgeValuesPWM[6] = {4095, 4095, 4095, 4095, 4095, 4095}; // PWM duty cycle for all channels
int zeroValuesPWM[6] = {0, 0, 0, 0, 0, 0};                     // PWM duty cycle for all channels
int edgeValuesReg[6];                                          // R,R,G,G,B,B

// Define PWM stuff
int frames = 10;
float frameStep = 0;

// Define the frame data arrays
unsigned int oldData[53];
unsigned int newData[53];
unsigned int currentFrameData[53];

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
// This is for the both sides side and indexes are 1-36
int pinLookup[][2] = {{1,32},{2,35},{3,38},{4,41},{5,44},{6,47},
                      {7,31},{8,34},{9,37},{10,40},{11,43},{12,46},
                      {13,30},{14,33},{15,36},{16,39},{17,42},{18,45},
                      {19,66},{20,67},
                      {21,27},{22,24},{23,21},{24,18},{25,15},{26,3},
                      {27,28},{28,25},{29,22},{30,19},{31,16},{32,2},
                      {33,29},{34,26},{35,23},{36,20},{37,17},{38,14},
                      {39,68},{40,69}};

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

void send_init_reg(unsigned int *reg)
{
  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 34; i++)
    SPI.transfer16(reg[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

void send_4words(word *data)
{
  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  for (byte i = 0; i < 4; i++)
    SPI.transfer16(data[i]);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

void send_3words_plus_zero(word cmd1, word cmd2, word cmd3)
{
  chipUpdate(0);
  delayMicroseconds(SPI_CS_DELAY);
  SPI.transfer16(cmd1);
  SPI.transfer16(cmd2);
  SPI.transfer16(cmd3);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  chipUpdate(1);
  delayMicroseconds(SPI_CS_DELAY);
}

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

// Turn off all lights
void all_off()
{
  send_2blocks_of_4words(allOffData, 1000, false, 0);
}

// Turn off all lights using PWM and setting newData[] to 0
void all_off_PWM()
{
  for (byte i = 0; i < 49; i++)
  {
    newData[i] = 0;
  }
  by_data_pwm(1, oldData, newData);
}

// Turn on all lights
void all_on()
{
  send_4words(allOnData);
}

// Turn on all lights using PWM and setting newData[] to intensity input
void all_on_PWM(int intensity)
{
  for (byte i = 0; i < 49; i++)
  {
    newData[i] = intensity;
  }
  by_data_pwm(2, oldData, newData);
}

void chipUpdate(int state)
{
  if (side == 0)
  {
    if (state == 0) // corresponds to the low state
    {
      for (uint8_t i = 18; i < 20; i++)
      {
        digitalWrite(pinLookup[i][1], LOW); // set as LOW
      }
    }
    else // corresponds to the high state
    {
      for (uint8_t i = 18; i < 20; i++)
      {
        digitalWrite(pinLookup[i][1], HIGH); // set as LOW
      }
    }
  }
  else if (side == 1)
  {
    if (state == 0) // corresponds to the low state
    {
      for (uint8_t i = 38; i < 40; i++)
      {
        digitalWrite(pinLookup[i][1], LOW); // set as LOW
      }
    }
    else if (state == 1) // corresponds to the high state
    {
      for (uint8_t i = 38; i < 40; i++)
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

void initDev()
{
  unsigned int intReg_Conf_iW7039[34] = {
      0x8020, 0x00,                                                   // broadcast command with same data writting for 32 words from register 0x00
      0x0802, 0x83A2, 0x0800, 0x0FFF, 0x9FFC, 0x4927, 0x4585, 0x87FF, // 0X00 ~ 0X07
      0x0100, 0x2100, 0x0000, 0x08F0, 0x0088, 0x186A, 0x5348, 0xCC0C, // 0X08 ~ 0X0F
      0xFFFF, 0xFFFF, 0x0001, 0x0080, 0x010f, 0x0000, 0x0000, 0x0000, // 0X10 ~ 0X17
      0xFFFF, 0xFFFF, 0x0000, 0x01FF, 0x0000, 0x1000, 0xFFFF, 0xFFFF  // 0X18 ~ 0X1F
  };                                                                  // Initializing

  unsigned int intReg_DT_iW7039[34] = {
      0x8020, 0x40,                                           // broadcast command with same data writting for 32 words for DT
      0x00, 0x80, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380,   // 0x40 ~ 0x47
      0x400, 0x480, 0x500, 0x580, 0x600, 0x680, 0x700, 0x780, // 0x48 ~ 0x4F
      0x800, 0x880, 0x900, 0x980, 0xA00, 0xA80, 0xB00, 0xB80, // 0x50 ~ 0x57
      0xC00, 0xC80, 0xD00, 0xD80, 0xE00, 0xE80, 0xF00, 0xF80  // 0x58 ~ 0x5F
  };                                                          // 0x40 to 0x5F DT: Delay Time

  unsigned int intReg_HT_iW7039[34] = {
      0x8020, 0x60,                                           // broadcast command with same data writting for 32 words for HT
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, // 0x60 ~ 0x67
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, // 0x68 ~ 0x6F
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, // 0x70 ~ 0x77
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0  // 0x78 ~ 0x7F
  };                                                          // 0x60 to 0x7F

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

  delay(100); // Keep all LEDs on for 10ms

  // all_off(); // Turn off all LEDs

  // all_off_PWM(); // Turn off all LEDs using PWM
}

void setup()
{
  delay(1000); // Wait for the serial port to connect.

  // Open serial communications
  Serial.begin(9600);
  // Start the SPI library
  SPI.begin();

  // Networking part begins here

  // // Start the Ethernet connection and the server based on the side
  // if (side == 0)
  // {
  //   Ethernet.begin(macRight, ipRight);
  //   Serial.println("Right side started");
  // }
  // else if (side == 1)
  // {
  //   Ethernet.begin(macLeft, ipLeft);
  //   Serial.println("Left side started");
  // }
  // else
  // {
  //   Serial.println("Error: side not defined");
  // }

  // delay(1000);

  // // Check for Ethernet hardware present
  // if (Ethernet.hardwareStatus() == EthernetNoHardware)
  // {
  //   Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  //   while (true)
  //   {
  //     delay(1); // do nothing, no point running without Ethernet hardware
  //   }
  // }
  // if (Ethernet.linkStatus() == LinkOFF)
  // {
  //   Serial.println("Ethernet cable is not connected.");
  // }

  // delay(1000);

  // Serial.print("Chat server address : ");
  // Serial.println(Ethernet.localIP());

  // Networking part ends

  // Initalize chip select pins
  for (uint8_t i = 0; i < 40; i++)
  {                                   // for each pin
    pinMode(pinLookup[i][1], OUTPUT); // set as output
    digitalWrite(i, LOW);             // set as LOW
  }

  // Add external 120Hz Vsync, 50% duty cycle
  pinMode(Vsync, OUTPUT);
  Timer1.initialize(16666);
  Timer1.pwm(Vsync, 512);

  // Delay 10ms until logic core is ready
  delay(10);

  initDev();

  delay(100); // delay 100ms

  // // start listening for clients
  // server.begin();
  // Serial.print("Began Server");

  delay(100); // delay 100ms
}

void by_data(byte data[])
{
  // according to binary data, all colors
  byte count = 0;
  for (byte i = 0; i < 7; i++)
  {
    for (byte j = 0; j < 7; j++)
    {
      count = i * 7 + j;
      if (data[count] == 1)
      {
        send_3words_plus_zero(red_cmd_1[count], red_cmd_2[count], red_cmd_3[count]);
        send_3words_plus_zero(green_cmd_1[count], green_cmd_2[count], green_cmd_3[count]);
        send_3words_plus_zero(blue_cmd_1[count], blue_cmd_2[count], blue_cmd_3[count]);
      }
    }
  }
  delay(1);
}

// Changes PWM values for all LEDs from oldData to newData in frames steps
void by_data_pwm(int frames, int oldData[], int newData[])
{
  for (int frame = 0; frame <= frames; frame++)
  {
    // Serial.println("_");
    frameStep = (float)frame / (float)frames;

    for (byte i = 0; i < 49; i++)
    {
      currentFrameData[i] = oldData[i] + ((newData[i] - oldData[i]) * frameStep);
      // Serial.print(currentFrameData[i]);
      // Serial.print("-");
    }
    // Serial.print("Frame = ");
    // Serial.print(frame);
    // Serial.println("_");
    unsigned int Driver1_Reg_HT_iW7039[34] = {0x8120, 0x60, currentFrameData[35], currentFrameData[28], currentFrameData[3], currentFrameData[13], currentFrameData[20], currentFrameData[27], currentFrameData[34], currentFrameData[48], currentFrameData[47], currentFrameData[46], currentFrameData[44], currentFrameData[43], currentFrameData[42], currentFrameData[36], currentFrameData[29], currentFrameData[21], currentFrameData[14], currentFrameData[7], currentFrameData[0], currentFrameData[1], currentFrameData[2], currentFrameData[4], currentFrameData[5], currentFrameData[6], currentFrameData[12], currentFrameData[19], currentFrameData[26], currentFrameData[41], currentFrameData[40], currentFrameData[39], currentFrameData[45], currentFrameData[38]};
    unsigned int Driver2_Reg_HT_iW7039[34] = {0x8220, 0x60, currentFrameData[37], currentFrameData[30], currentFrameData[22], currentFrameData[15], currentFrameData[8], currentFrameData[9], currentFrameData[10], currentFrameData[11], currentFrameData[18], currentFrameData[25], currentFrameData[33], currentFrameData[32], currentFrameData[31], currentFrameData[23], currentFrameData[16], currentFrameData[17], currentFrameData[24], currentFrameData[35], currentFrameData[28], currentFrameData[3], currentFrameData[13], currentFrameData[20], currentFrameData[27], currentFrameData[34], currentFrameData[48], currentFrameData[47], currentFrameData[46], currentFrameData[44], currentFrameData[43], currentFrameData[42], currentFrameData[36], currentFrameData[29]};
    unsigned int Driver3_Reg_HT_iW7039[34] = {0x8320, 0x60, currentFrameData[21], currentFrameData[14], currentFrameData[7], currentFrameData[0], currentFrameData[1], currentFrameData[2], currentFrameData[4], currentFrameData[5], currentFrameData[6], currentFrameData[12], currentFrameData[19], currentFrameData[26], currentFrameData[41], currentFrameData[40], currentFrameData[39], currentFrameData[45], currentFrameData[38], currentFrameData[37], currentFrameData[30], currentFrameData[22], currentFrameData[15], currentFrameData[8], currentFrameData[9], currentFrameData[10], currentFrameData[11], currentFrameData[18], currentFrameData[25], currentFrameData[33], currentFrameData[32], currentFrameData[31], currentFrameData[23], currentFrameData[16]};
    unsigned int Driver4_Reg_HT_iW7039[34] = {0x8420, 0x60, currentFrameData[17], currentFrameData[24], currentFrameData[35], currentFrameData[28], currentFrameData[3], currentFrameData[13], currentFrameData[20], currentFrameData[27], currentFrameData[34], currentFrameData[48], currentFrameData[47], currentFrameData[46], currentFrameData[44], currentFrameData[43], currentFrameData[42], currentFrameData[36], currentFrameData[29], currentFrameData[21], currentFrameData[14], currentFrameData[7], currentFrameData[0], currentFrameData[1], currentFrameData[2], currentFrameData[4], currentFrameData[5], currentFrameData[6], currentFrameData[12], currentFrameData[19], currentFrameData[26], currentFrameData[41], currentFrameData[40], currentFrameData[39]};
    unsigned int Driver5_Reg_HT_iW7039[34] = {0x8520, 0x60, currentFrameData[45], currentFrameData[38], currentFrameData[37], currentFrameData[30], currentFrameData[22], currentFrameData[15], currentFrameData[8], currentFrameData[9], currentFrameData[10], currentFrameData[11], currentFrameData[18], currentFrameData[25], currentFrameData[33], currentFrameData[32], currentFrameData[31], currentFrameData[23], currentFrameData[16], currentFrameData[17], currentFrameData[24], 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    chipUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver1_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipUpdate(1);

    chipUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver2_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipUpdate(1);

    chipUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver3_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipUpdate(1);

    chipUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver4_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipUpdate(1);

    chipUpdate(0);
    delayMicroseconds(SPI_CS_DELAY);
    for (int i = 0; i < 34; i++)
      SPI.transfer16(Driver5_Reg_HT_iW7039[i]);
    SPI.transfer16(0);
    delayMicroseconds(SPI_CS_DELAY);
    chipUpdate(1);
  }
  Serial.println("Finished Updating");
}

// Scans through each side and turns it on
void scanEdge(int pin, int data[])
{
  // Defines pin as the corresponding pin based on the side of the board
  if (side == 0)
  {
    pin = pinLookup[pin + 18][1];
  }
  else if (side == 1)
  {
    pin = pinLookup[pin + 38][1];
  }

  Serial.println((String) "Pin - " + pin);

  unsigned int Edge_Reg_HT_iW7039[34] = {0x8120, 0x60, data[0], data[1], data[2], data[3], data[4], data[5]};

  digitalWrite(pin, LOW);
  delayMicroseconds(SPI_CS_DELAY);
  for (int i = 0; i < 8; i++)
    SPI.transfer16(Edge_Reg_HT_iW7039[i]);
  SPI.transfer16(0);
  delayMicroseconds(SPI_CS_DELAY);
  digitalWrite(pin, HIGH);

  // delay(100); // Keep all LEDs on for 100ms
}

void loop()
{
  // Loop through all 49 directions
  for (byte i = 0; i < 2; i++)
  {

    Serial.println((String) "Side - " + i);

    scanEdge(i, edgeValuesPWM);

    delay(3000);

    scanEdge(i, zeroValuesPWM);
    // all_off();
    // Turning off all the LEDs
    // for(byte j=0; j<2; j++)
    // {
    //   send_4words(allOffData[j]);
    // }
  }
}