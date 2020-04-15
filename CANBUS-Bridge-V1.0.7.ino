/*
 * #ifndef ARDUINO_AVR_UNO
 * #error "This file is for the Arduino UNO only"
 * #endif
 */

/*
 * Darren Clark - BiggRanger@tds.net
 * CAN BUS Bridge - used to determine what devices are sending CAN ID's
 * 
 * V1.0.3 (2020.01.11) - First working system
 * V1.0.4 (2020.01.12) - Add options to control output reporting
 * V1.0.5 (2020.01.13) - Add options to change speeds and save settings in EEPROM
 * V1.0.6 (2020.01.13) - Bugfix output1Mode was not being set, had output0Mode instead.
 * V1.0.7 (2020.04.14) - Add options to block or unblock CAN IDs from being transfered, made output settings easier to read.
 * 
 * Notes: 83.333K is fault tolerant and looks like is may be single ended.
 * 1. From radio leave CAN-L disconnected, connect CAN-H to CAN-H on Arduino
 * 2. On Arduino connct a 120 Ohm resistor from CAN-L to ground.
 * 3. Connect radio ground to Arduino ground.
 * 
 * Notes:
 * SPI=8MHz
 * CAN Write 8 bytes = 74.6uS CAN 500K length = 240uS
 * CAN Write 1 byte  = 57.6uS CAN 500K length = 86uS
 * CAN Write 0 byte  = 51.6uS CAN 500K length = 810uS
 * 
 * Serial=1e6 
 * 5 char = 60uS 
 * 29 chars = 328us
 * size not including terminating CR/LF
 */

/*
 * Pin assignments -> *=fixed, -=flexable
 * D0  * (RX)
 * D1  * (TX)
 * D2  * INT CAN0
 * D3  * INT CAN1
 *
 * D9  - CAN1 CS
 * D10 - CAN0 CS
 * D11 * SPI SI
 * D12 * SPI SO
 * D13 * SPI CLK
*/

#include <SPI.h>
#include <EEPROM.h>

#define REG_BFPCTRL                0x0c
#define REG_TXRTSCTRL              0x0d

#define REG_CANCTRL                0x0f

#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2a

#define REG_CANINTE                0x2b

#define FLAG_RXnIE(n)              (0x01 << n)

#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10))
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10))

#define REG_RXBnCTRL(n)            (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10))
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10))

#define FLAG_SRR                   0b00010000
#define FLAG_RTR                   0b01000000

#define FLAG_RXM0                  0b00100000
#define FLAG_RXM1                  0b01000000

struct canPacket
{
  int32_t ID;
  bool rtr;
  int8_t dlc;
  uint8_t data[8];
};

//use setPin and clrPin instead of digitalWrite since they are much faster.
#define setPin(b) ( (b)<8 ? PORTD |=(1<<(b)) : PORTB |=(1<<(b-8)) )
#define clrPin(b) ( (b)<8 ? PORTD &=~(1<<(b)) : PORTB &=~(1<<(b-8)) )
#define tstPin(b) ( (b)<8 ? (PORTD &(1<<(b)))!=0 : (PORTB &(1<<(b-8)))!=0 )

//setup pins for CAN modules
#define can0CS  9
#define can0INT 3

#define can1CS  10
#define can1INT 2

SPISettings _spiSettings = SPISettings(10E6, MSBFIRST, SPI_MODE0);  //actual SPI speed on UNO or NANO is 8MHz

//Parameters for CAN speeds. CNF(CAN crystal speed)_(CAN speed)
uint8_t CNF8E6_80E3[3]   = {0x01, 0xBF, 0x07};
uint8_t CNF8E6_83E3[3]   = {0x01, 0xBE, 0x07};
uint8_t CNF8E6_125E3[3]  = {0x01, 0xB1, 0x05};
uint8_t CNF8E6_500E3[3]  = {0x00, 0x90, 0x02};

uint8_t CNF16E6_80E3[3]  = {0x03, 0xFF, 0x07};
uint8_t CNF16E6_83E3[3]  = {0x03, 0xFE, 0x07};
uint8_t CNF16E6_125E3[3] = {0x03, 0xF0, 0x06};
uint8_t CNF16E6_500E3[3] = {0x00, 0xF0, 0x06};

uint8_t output0Mode = 2; //0=no output, 1=CAN ID only, 2=Full packet
uint8_t output1Mode = 2; //0=no output, 1=CAN ID only, 2=Full packet
uint8_t canSpeed = 7;    //6=80Kbps, 7=83.333Kbps, 8=125Kbps, 9=500Kbps

//note update signature version number only when changes break EEPROM parameters
char EEPROMVERSION[16] = "CANBUSBRIDGE106\0";

String SerialRXBuffer = "";
bool SerialRXSpecial = false;

uint16_t blockedID[128];
uint8_t blockedIDcount = 0;

void setup()
{
  Serial.begin(1E6);

  pinMode(can0CS, OUTPUT);
  pinMode(can1CS, OUTPUT);

  SPI.begin();

  Serial.println("CANBUS Bridge V1.0.7");
  eepromRead();           //read settings from EEPROM
  canSetup();             //setup CAN chips
  serialOutMode();        //display output mode
  canInterrupts(true);    //turn on interrupts to start receiving data
}

void canSetup()
{
  mcp2515Reset(can0CS);
  mcp2515Reset(can1CS);
  delay(1);

  Serial.print("Speed Selected: ");
  switch(canSpeed)
  {
    case 6:
      Serial.println("80Kbps");
      mcp2515Init(can0CS, CNF16E6_80E3);
      mcp2515Init(can1CS, CNF16E6_80E3);
      break;  
    case 7:
      Serial.println("83.333Kbps");
      mcp2515Init(can0CS, CNF16E6_83E3);
      mcp2515Init(can1CS, CNF16E6_83E3);
      break;
    case 8:
      Serial.println("125Kbps");
      mcp2515Init(can0CS, CNF16E6_125E3);
      mcp2515Init(can1CS, CNF16E6_125E3);
      break;
    case 9:
      Serial.println("500Kbps");
      mcp2515Init(can0CS, CNF16E6_500E3);
      mcp2515Init(can1CS, CNF16E6_500E3);
      break;
  }
}

void canInterrupts(bool active)
{
  if (active)
  {
    attachInterrupt(digitalPinToInterrupt(can0INT), can0Receive, LOW);
    attachInterrupt(digitalPinToInterrupt(can1INT), can1Receive, LOW);
  }
  else
  {
    detachInterrupt(digitalPinToInterrupt(can0INT));
    detachInterrupt(digitalPinToInterrupt(can1INT));
  }
}

void eepromSave()
{
  EEPROM.put(0x0000, EEPROMVERSION);
  EEPROM.put(0x0100, canSpeed);
  EEPROM.put(0x0110, output0Mode);
  EEPROM.put(0x0120, output1Mode);
}

void eepromRead()
{
  char ID[15];
  EEPROM.get(0x0000, ID);
  if (strcmp(ID, EEPROMVERSION) == 0)
  {
    EEPROM.get(0x0100, canSpeed);
    EEPROM.get(0x0110, output0Mode);
    EEPROM.get(0x0120, output1Mode);    
  }
}

void loop()
{
  if (Serial.available())
  {
    char RX = Serial.read();
    if (RX == 'X' || RX == 'x') //clear the special serial stuff...
    {
      SerialRXBuffer = "";
      SerialRXSpecial = false;
    } 
    if (!SerialRXSpecial)
    {
      switch(RX)
      {
        case '1':
        case '2':
        case '3':
          output0Mode = RX - '1';
          eepromSave();
          serialOutMode();
          break;
        case 'a':
        case 'b':
        case 'c':
          output1Mode = RX - 'a';
          eepromSave();
          serialOutMode();
          break;
        case 'A':
        case 'B':
        case 'C':
          output1Mode = RX - 'A';
          eepromSave();
          serialOutMode();
          break;
        case '6':
        case '7':
        case '8':
        case '9':
          canInterrupts(false);
          canSpeed = RX - '0';
          eepromSave();
          canSetup();
          canInterrupts(true);
          break;
        case '-':
        case '+':
          SerialRXBuffer = RX;
          SerialRXSpecial = true;
          break;
        case 'L':
        case 'l':
          for (uint8_t x = 0; x < blockedIDcount; x++)
          {
            if ( blockedID[x] != 0xFFFF )
            {
              Serial.print("Blocked ID["); Serial.print(x); Serial.print("]: ");
              Serial.println(blockedID[x], HEX);
            }
          }
          break;
      }
    }
    else
    {
      SerialRXBuffer += RX;
      if (SerialRXBuffer.length() >= 4)
      {
        //validate hex value
        String tempVal = SerialRXBuffer.substring(1,4);
        tempVal = "0x" + tempVal;
        char tempArray[8];
        tempVal.toCharArray(tempArray,sizeof(tempArray));
        int val = strtol(tempArray, 0, 0);

        if (SerialRXBuffer.charAt(0) == '+')
        {
          Serial.print("ID blocked: ");
          Serial.println(val, HEX);
          blockedID[blockedIDcount] = val;
          blockedIDcount ++;
        }

        if (SerialRXBuffer.charAt(0) == '-')
        {
          Serial.print("ID unblocked: ");
          Serial.println(val, HEX);
          for (uint8_t x = 0; x < blockedIDcount; x++)
          {
            if (blockedID[x] == val)
            {
              blockedID[x] = 0xFFFF;
              if (x+1 == blockedIDcount)
                blockedIDcount--;
            }
          }
        }
        SerialRXBuffer = "";
        SerialRXSpecial = false;
      }
    }
  }
}

void serialOutMode()
{
  Serial.print("Output mode CAN0: ");
  if (output0Mode == 0)
    Serial.print("0 (None)");
  if (output0Mode == 1)
    Serial.print("1 (ID)");
  if (output0Mode == 2)
    Serial.print("2 (Full)");

  Serial.print(" CAN1: ");
  if (output1Mode == 0)
    Serial.println("A (None)");
  if (output1Mode == 1)
    Serial.println("B (ID)");
  if (output1Mode == 2)
    Serial.println("C (Full)");
}

void can0Receive()
{
  //receive CAN packet
  canPacket data = canRead(can0CS);

  //test if the packet is blocked
  for (uint8_t x = 0; x < blockedIDcount; x++)
  {
    if (blockedID[x] == data.ID)
    {
      if (output0Mode == 1)
        serialOutSmall('9', data.ID);
      else if (output0Mode == 2)
        serialOutFull('9', data);
      return;
    }
  }

  //resend the packet
  uint8_t retVal = canWrite(can1CS, data);

  if (output0Mode == 1)
    serialOutSmall(retVal + '0', data.ID);
  else if (output0Mode == 2)
    serialOutFull(retVal + '0', data);
}

void can1Receive()
{
  //receive CAN packet
  canPacket data = canRead(can1CS);
  
  //test if the packet is blocked
  for (uint8_t x = 0; x < blockedIDcount; x++)
  {
    if (blockedID[x] == data.ID)
    {
      if (output1Mode == 1)
        serialOutSmall('X', data.ID);
      else if (output1Mode == 2)
        serialOutFull('X', data);
      return;
    }
  }

  //resend the packet
  uint8_t retVal = canWrite(can0CS, data);

  if (output1Mode == 1)
    serialOutSmall(retVal + 'A', data.ID);
  else if (output1Mode == 2)
    serialOutFull(retVal + 'A', data);
}

void serialOutSmall(char writeStatus, int16_t ID)
{
  char buff[6];
  buff[0] = writeStatus;
  buff[1] = ' ';
  buff[2] = ((ID & 0xF00) >> 8) + '0';
  buff[3] = ((ID & 0x0F0) >> 4) + '0';
  if (buff[3] > '9')
    buff[3] += 7;
  buff[4] = ((ID & 0x00F)) + '0';
  if (buff[4] > '9')
    buff[4] += 7;
  buff[5] = 0;

  Serial.println(buff);
}

void serialOutFull(char writeStatus, canPacket data)
{
  char buff[32];
  buff[0] = writeStatus;
  buff[1] = ' ';
  buff[2] = ((data.ID & 0xF00) >> 8) + '0';
  buff[3] = ((data.ID & 0x0F0) >> 4) + '0';
  if (buff[3] > '9')
    buff[3] += 7;
  buff[4] = ((data.ID & 0x00F)) + '0';
  if (buff[4] > '9')
    buff[4] += 7;

  for (uint8_t x = 0; x < data.dlc; x++)
  {
    uint8_t index = x * 3;
    buff[index + 5] = ' ';
    buff[index+ 6] = ((data.data[x] & 0xF0) >> 4) + '0';
    if (buff[index+ 6] > '9')
      buff[index + 6] += 7;
    buff[index + 7] = ((data.data[x] & 0x0F)) + '0';
    if (buff[index + 7] > '9')
      buff[index + 7] += 7;
  }
  buff[data.dlc * 3 + 5] = 0;

  Serial.println(buff);
}

uint8_t canWrite(uint8_t canCS, canPacket packet)
{
  //returns:
  //00-02 = output buffer written to.
  //04 = no output buffers available
  //05 = invalid packet ID.
  //06 = invalid packet dlc.
  
  if (packet.ID < 0 || packet.ID > 0x7FF)
    return 5;

  if (packet.dlc > 8)
    return 6;

  uint8_t n = 0;
  uint8_t TXStatus = mcp2515ReadStatus(canCS);
  if (TXStatus & 0b00000100)       //TX0 busy? no n=0
  {
    n=1;
    if (TXStatus & 0b00010000)     //TX1 busy? no n=1
    {
      n=2;
      if (TXStatus & 0b0100000)    //TX2 busy? no n=2
      {
        n = 4;                     //All buffers busy n=4
        return n;
      }
    }
  }

  registerWrite(canCS, REG_TXBnSIDH(n), packet.ID >> 3);                  //set CAN ID
  registerWrite(canCS, REG_TXBnSIDL(n), packet.ID << 5);                  //set CAN ID

  if (packet.rtr)
    registerWrite(canCS, REG_TXBnDLC(n), 0b01000000 | packet.dlc);        //set RTR + packet size
  else 
  {
    registerWrite(canCS, REG_TXBnDLC(n), packet.dlc);                     //set packet size

    SPI.beginTransaction(_spiSettings);
    clrPin(canCS);
    SPI.transfer(0b01000000 | (n * 2 + 1));
    for (uint8_t x = 0; x < packet.dlc; x++)
      SPI.transfer(packet.data[x]);
    setPin(canCS);
    SPI.endTransaction();
  }

  SPI.beginTransaction(_spiSettings);                                     //set RTS signal
  digitalWrite(canCS, LOW);
  SPI.transfer(0b10000000 | (1 << n));
  digitalWrite(canCS, HIGH);
  SPI.endTransaction();  
  
  return n;
}

canPacket canRead(uint8_t canCS)
{
  uint8_t n;
  canPacket retData;

  uint8_t RXStatus = mcp2515ReadStatus(canCS);

  if (RXStatus & 0b01) 
    n = 0;
  else if (RXStatus & 0b10) 
    n = 1;
  else
  {
    retData.ID = -1;
    retData.rtr = false;
    retData.dlc = 0;
    return retData;
  }
  
  uint8_t SIDH = registerRead(canCS, REG_RXBnSIDH(n));
  uint8_t SIDL = registerRead(canCS, REG_RXBnSIDL(n));
  retData.ID = ((SIDH << 3) & 0x7F8) | ((SIDL >> 5) & 0x07);
  retData.rtr = (SIDL & FLAG_SRR) ? true : false;
  retData.dlc = registerRead(canCS, REG_RXBnDLC(n)) & 0b00001111;

  if (!retData.rtr) 
  {
    SPI.beginTransaction(_spiSettings);
    clrPin(canCS);
    SPI.transfer(0b10010010 | (n << 2));
    for (uint8_t x = 0; x < retData.dlc; x++)
      retData.data[x] = SPI.transfer(0x00);
    setPin(canCS);
    SPI.endTransaction();
  }

  return retData;
}

uint8_t mcp2515Init(uint8_t canCS, uint8_t CNF[3])
{
  registerWrite(canCS, REG_CANCTRL, 0b10000000);                      //turn on configuration mode
  if (registerRead(canCS, REG_CANCTRL) != 0x80)                       //verify configuration mode
  {
    Serial.println("Setup Failed 1");
    return 0;
  }

  registerWrite(canCS, REG_CNF1, CNF[0]);
  registerWrite(canCS, REG_CNF2, CNF[1]);
  registerWrite(canCS, REG_CNF3, CNF[2]);
  registerWrite(canCS, REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0));   //set to interrupt on RX data in buffer 0 and 1
  registerWrite(canCS, REG_BFPCTRL, 0x00);                            //do not use RX buffer pins
  registerWrite(canCS, REG_TXRTSCTRL, 0x00);                          //do not use TX RTS pins
  registerWrite(canCS, REG_RXBnCTRL(0), FLAG_RXM1 | FLAG_RXM0);       //turn off mask filters for RX0
  registerWrite(canCS, REG_RXBnCTRL(1), FLAG_RXM1 | FLAG_RXM0);       //turn off mask filters for RX1

  registerWrite(canCS, REG_CANCTRL, 0x00);                            //turn off configuration mode
  if (registerRead(canCS, REG_CANCTRL) != 0x00)                       //verify configuration mode is off
  {
    Serial.println("Setup Failed 2");
    return 0;
  }

  return 1;
}

uint8_t registerRead(uint8_t canCS, uint8_t address)
{
  uint8_t value;

  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b00000011);
  SPI.transfer(address);
  value = SPI.transfer(0x00);
  setPin(canCS);
  SPI.endTransaction();

  return value;
}

void registerModify(uint8_t canCS, uint8_t address, uint8_t mask, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b00000101);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  setPin(canCS);
  SPI.endTransaction();
}

void registerWrite(uint8_t canCS, uint8_t address, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b00000010);
  SPI.transfer(address);
  SPI.transfer(value);
  setPin(canCS);
  SPI.endTransaction();
}

void mcp2515Reset(uint8_t canCS)
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(canCS, LOW);
  SPI.transfer(0b11000000);
  digitalWrite(canCS, HIGH);
  SPI.endTransaction();
}

uint8_t mcp2515ReadStatus(uint8_t canCS)
{
  uint8_t value;

  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b10100000);
  value = SPI.transfer(0x00);
  setPin(canCS);
  SPI.endTransaction();

  return value;  
}
/*
uint8_t mcp2515RXStatus(uint8_t canCS)
{
  uint8_t value;

  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b10110000);
  value = SPI.transfer(0x00);
  setPin(canCS);
  SPI.endTransaction();

  return value;  
}

void mcp2515LoadTXBuffer(uint8_t canCS, uint8_t TXBUFF, uint8_t data[8], uint8_t dataSize)
{
  if (dataSize == 0)
    return;

  if (TXBUFF > 2)
    return;
    
  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b01000000 | (TXBUFF * 2 + 1));
  for (uint8_t x = 0; x < dataSize; x++)
    SPI.transfer(data[x]);
  setPin(canCS);
  SPI.endTransaction();
}

void mcp2515rts(uint8_t canCS, uint8_t TXBUFF)
{
  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b10000000 | (1 << TXBUFF));
  setPin(canCS);
  SPI.endTransaction();  
}

canPacket mcp2515ReadRXBuffer(uint8_t canCS, uint8_t RXBUFF, uint8_t dataSize)
{
  canPacket data; //note only data[8] is populated in struct

  if (dataSize == 0)
    return data;
  if (RXBUFF > 1)
    return data;

  SPI.beginTransaction(_spiSettings);
  clrPin(canCS);
  SPI.transfer(0b10010010 | (RXBUFF << 2));
  for (uint8_t x = 0; x < dataSize; x++)
    data.data[x] = SPI.transfer(0x00);
  setPin(canCS);
  SPI.endTransaction();

  return data;
}
*/
