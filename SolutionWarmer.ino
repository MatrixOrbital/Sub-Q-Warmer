#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <OneWire.h>
#include <FastPID.h>
#include <stdlib.h>
#include "Eve2_81x.h"           
#include "MatrixEve2Conf.h"      // Header for EVE2 Display configuration settings
#include "process.h"
#include "Arduino_AL.h"

File myFile;
char LogBuf[WorkBuffSz];
uint8_t OWTP_addr[NumProbes][8]; // Buffer to store One wire Address
bool OWTP_Plate_Connected;       // Bool to indicate if the DS18S20 is connected
bool OWTP_Solution_Connected;    // Bool to indicate if the DS18S20 is connected

// These constructors and the abstractions below for one-wire and PID are perforce 
// in this file due to the fact that the arduino compiler expects C++ and this .ino 
// file is the only actual C++ file in the project.  Attempting to do this in 
// process.c where they are actually desired will result in compiler errors.
// You might think of this file as providing a translation between C++ and C.
//
// Both of these libraries are available via the "Manage Libraries" function under
// "Sketch/Include Library". Find and install each.
// https://github.com/PaulStoffregen/OneWire  // Reference code for OneWire
// https://github.com/mike-matera/FastPID     // Reference code for FastPID

OneWire OWTP(OneWire_PIN); 
FastPID PID_Heater(10, 0.0025, 40, 0.2, 8, false);      // FastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
FastPID PID_Load(12.0, 0.0013, 0.0, 0.0625, 16, false); 

void setup()
{
  // Initializations.  Order is important
  GlobalInit();
  FT81x_Init();
  SD_Init();

  OWTP.reset_search(); // Reset the one-wire bus to search from address zero
  
  // One wire initialization of probes
  if ( !searchTempProbe(OWTP_Solution))
  {
    //    Log("OWTP_Solution not found");
    while(1); // We can not operate without successful probe interaction
  }

  if (!searchTempProbe(OWTP_Plate))
  {
    //    Log("OWTP_Plate not found");
    while(1); // We can not operate without successful probe interaction
  }
  
  if (!LoadTouchMatrix())
  {
    // We failed to read calibration matrix values from the SD card.
    Calibrate_Manual(DWIDTH, DHEIGHT, PIXVOFFSET, PIXHOFFSET);
    SaveTouchMatrix(); // Save to flash
    LoadTouchMatrix(); // reload from flash to compare values
  }
  
//  Load_JPG(RAM_G, 0, "MainScr.jpg");  // Preload background jpg image into Eve GRAM
  Cmd_SetRotate(1);  // Rotate the display
  wr8(REG_PWM_DUTY + RAM_REG, 128);      // set backlight

  if(SD.exists("pidlog.txt"))
    SD.remove("pidlog.txt");

  SetupMainScreen();
  MainLoop(); // jump to "main()"
}

// MainLoop is called from setup() and it never leaves (which is better than loop() which is called repeatedly)
void MainLoop(void)
{
  while(1)
  {
    CheckScreen();          // Update the screen at an effective rate
    CheckSensors();         // Read the sensors at the correct rate
    CheckSolution();        // Check the sensor data and update TimeTillCat value
    CheckHeater();          // Run PID loop for heater
    CheckTouch();           // Check for user touching and update values (blocks on touch)
  }
}

// ************************************************************************************
// Following are wrapper functions for C++ Arduino functions so that they may be      *
// called from outside of C++ files.  These are also your opportunity to use a common *
// name for your hardware functions - no matter the hardware.  In Arduino-world you   *
// interact with hardware using Arduino built-in functions which are all C++ and so   *
// your "abstraction layer" must live in this xxx.ino file where C++ works.           *
//                                                                                    *
// This is also an alternative to ifdef-elif hell.  A different target                *
// processor or compiler will include different files for hardware abstraction, but   *
// the core "library" files remain unaltered - and clean.  Applications built on top  *
// of the libraries need not know which processor or compiler they are running /      *
// compiling on (in general and within reason)                                        *
// ************************************************************************************

void GlobalInit(void)
{
  Wire.begin();                          // Setup I2C bus

  Serial.begin(115200);                  // Setup serial port for debug
  while (!Serial) {;}                    // wait for serial port to connect.
  
  // Matrix Orbital Eve display interface initialization
  pinMode(EvePDN_PIN, OUTPUT);            // Pin setup as output for Eve PDN pin.
  SetPin(EvePDN_PIN, 0);                  // Apply a resetish condition on Eve
  pinMode(EveChipSelect_PIN, OUTPUT);     // SPI CS Initialization
  SetPin(EveChipSelect_PIN, 1);           // Deselect Eve
  pinMode(EveAudioEnable_PIN, OUTPUT);    // Audio Enable PIN
  SetPin(EveAudioEnable_PIN, 0);          // Disable Audio
  pinMode(ControlOutput_PIN, OUTPUT);     // Pin setup as output for the software PWM pin controlling the heater
  SetPin(ControlOutput_PIN, 0);           // Turn that heater OFF!

  SPI.begin();                            // Enable SPI
//  Log("Startup\n");
}

// Send a single byte through SPI
void SPI_WriteByte(uint8_t data)
{
  SPI.beginTransaction(SPISettings(SPISpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(EveChipSelect_PIN, LOW);

  SPI.transfer(data);
      
  digitalWrite(EveChipSelect_PIN, HIGH);
  SPI.endTransaction();
}

// Send a series of bytes (contents of a buffer) through SPI
void SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
  SPI.beginTransaction(SPISettings(SPISpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(EveChipSelect_PIN, LOW);

  SPI.transfer(Buffer, Length);
      
  digitalWrite(EveChipSelect_PIN, HIGH);
  SPI.endTransaction();
}

// Send a byte through SPI as part of a larger transmission.  Does not enable/disable SPI CS
void SPI_Write(uint8_t data)
{
//  Log("W-0x%02x\n", data);
  SPI.transfer(data);
}

// Read a series of bytes from SPI and store them in a buffer
void SPI_ReadBuffer(uint8_t *Buffer, uint32_t Length)
{
  uint8_t a = SPI.transfer(0x00); // dummy read

  while (Length--)
  {
    *(Buffer++) = SPI.transfer(0x00);
  }
}

// Enable SPI by activating chip select line
void SPI_Enable(void)
{
  SPI.beginTransaction(SPISettings(SPISpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(EveChipSelect_PIN, LOW);
}

// Disable SPI by deasserting the chip select line
void SPI_Disable(void)
{
  digitalWrite(EveChipSelect_PIN, HIGH);
  SPI.endTransaction();
}

void Eve_Reset_HW(void)
{
  // Reset Eve
  SetPin(EvePDN_PIN, 0);                    // Set the Eve PDN pin low
  MyDelay(50);                              // delay
  SetPin(EvePDN_PIN, 1);                    // Set the Eve PDN pin high
  MyDelay(100);                             // delay
}

void DebugPrint(char *str)
{
  Serial.print(str);
}

// A millisecond delay wrapper for the Arduino function
void MyDelay(uint32_t DLY)
{
  uint32_t wait;
  wait = millis() + DLY; while(millis() < wait);
}

// Externally accessible abstraction for millis()
uint32_t MyMillis(void)
{
  return millis();
}

// An abstracted pin write that may be called from outside this file.
void SetPin(uint8_t pin, bool state)
{
  digitalWrite(pin, state); 
}

// An abstracted pin read that may be called from outside this file.
uint8_t ReadPin(uint8_t pin)
{
  return(digitalRead(pin));
}

//================================== Fast-PID Functions ====================================
uint8_t PID_Heater_Step(uint16_t SetPoint, uint16_t CurrentVal)
{
  uint8_t tmp = PID_Heater.step(SetPoint, CurrentVal);
  return ( tmp );
}

uint16_t PID_Load_Step(uint16_t SetPoint, uint16_t CurrentVal)
{
  uint16_t tmp;
  tmp = PID_Load.step(SetPoint, CurrentVal);
  return ( tmp );
}

void PID_ClearAll(void)
{
  PID_Heater.clear();
  PID_Load.clear();
}

void PID_Load_SetRange(uint16_t Lowend, uint16_t Highend)
{
  PID_Load.setOutputRange(Lowend, Highend); // Set the solution PID to demand temperatures between low and high
}

//================================== One-Wire Functions ====================================
int8_t searchTempProbe(uint8_t ProbeNum)
{
  if ( !OWTP.search(OWTP_addr[ProbeNum])) {
//    Log("No more probes found\n");
    return 0;
  }

  if ( OneWire::crc8( OWTP_addr[ProbeNum], 7) != OWTP_addr[ProbeNum][7]) {
//      Log("CRC Error\n");
      return 0;
  }
  
  if ( OWTP_addr[ProbeNum][0] == 0x10) {
//      Log("DS18S20 Probe\n");
      return 1;
  }     
  else 
  {
//      Log("One-wire device not recognised\n");
      return 0;
  }
}

int16_t readTempProbe(uint8_t ProbeNum)
{
  uint8_t data[12];
  float temp;
   
  OWTP.reset();
  OWTP.select(OWTP_addr[ProbeNum]);    
  OWTP.write(0xBE);                   // read scratchpad command
  
  for ( int8_t i = 0; i < 9; i++)    // get 9 bytes of data from probe
    data[i] = OWTP.read();

  temp = (data[1] << 8) + data[0];    

  // Start the next conversion - be sure to not read for at least 750mS
  OWTP.reset();
  OWTP.select(OWTP_addr[ProbeNum]);
  OWTP.write(0x44, 0);            // start conversion then release the bus (0)

  return temp;
}     

//================================== SD Card Functions ====================================
void SD_Init(void)
{
//  Log("Initializing SD card...\n");
  if (!SD.begin(SDChipSelect_PIN)) 
  {
    Log("SD initialization failed!\n");
    return;
  }
//  Log("SD initialization done\n");
}

// Read the touch digitizer calibration matrix values from the Eve and write them to a file
void SaveTouchMatrix(void)
{
  uint8_t count = 0;
  uint32_t data;
  uint32_t address = REG_TOUCH_TRANSFORM_A + RAM_REG;
  
//  Log("Enter SaveTouchMatrix\n");
  
  // If the file exists already from previous run, then delete it.
  if(SD.exists("tmatrix.txt"))
  {
    SD.remove("tmatrix.txt");
    MyDelay(50);
  }
  
  FileOpen("tmatrix.txt", FILEWRITE);
  if(!myFileIsOpen())
  {
    Log("No create file\n");
    FileClose();
    return false;
  }
  
  do
  {
    data = rd32(address + (count * 4));
    Log("TM%dw: 0x%08lx\n", count, data);
    FileWrite(data & 0xff);                // Little endian file storage to match Eve
    FileWrite((data >> 8) & 0xff);
    FileWrite((data >> 16) & 0xff);
    FileWrite((data >> 24) & 0xff);
    count++;
  }while(count < 6);
  FileClose();
  Log("Matrix Saved\n\n");
}

// Read the touch digitizer calibration matrix values from a file and write them to the Eve.
bool LoadTouchMatrix(void)
{
  uint8_t count = 0;
  uint32_t data;
  uint32_t address = REG_TOUCH_TRANSFORM_A + RAM_REG;
  
  FileOpen("tmatrix.txt", FILEREAD);
  if(!myFileIsOpen())
  {
    Log("tmatrix.txt not open\n");
    FileClose();
    return false;
  }
  
  do
  {
    data = FileReadByte() +  ((uint32_t)FileReadByte() << 8) + ((uint32_t)FileReadByte() << 16) + ((uint32_t)FileReadByte() << 24);
    Log("TM%dr: 0x%08lx\n", count, data);
    wr32(address + (count * 4), data);
    count++;
  }while(count < 6);
  
  FileClose();
  Log("Matrix Loaded \n\n");
  return true;
}

// ************************************************************************************
// Following are abstracted file operations for Arduino.  This is possible by using a * 
// global pointer to a single file.  It is enough for our needs and it hides file     *
// handling details within the abstraction.                                           *
// ************************************************************************************
void FileOpen(char *filename, uint8_t mode)
{
  // Since one also loses access to defined values like FILE_READ from outside the .ino
  // I have been forced to make up values and pass them here (mode) where I can use the 
  // Arduino defines.
  switch(mode)
  {
  case FILEREAD:
    myFile = SD.open(filename, FILE_READ);
    break;
  case FILEWRITE:
    myFile = SD.open(filename, FILE_WRITE);
    break;
  default:;
  }
}

void FileClose(void)
{
  myFile.close();
  if(myFileIsOpen())
  {
    Log("Failed to close file\n");
  }
}

// Read a single byte from a file
uint8_t FileReadByte(void)
{
  return(myFile.read());
}

// Read bytes from a file into a provided buffer
void FileReadBuf(uint8_t *data, uint32_t NumBytes)
{
  myFile.read(data, NumBytes);
}

void FileWrite(uint8_t data)
{
  myFile.write(data);
}

// Write a string of characters to a file
// MaxChars does not include the null terminator of the source string.
// We make no attempt to detect the usage of MaxChars and simply truncate the output
void FileWriteStr(uint8_t *str, uint16_t MaxChars)
{
  int16_t count = 0; 

  FileOpen("pidlog.txt", FILEWRITE);
  if(!myFileIsOpen())
  {
    Log("No file\n");
    FileClose();
    return;
  }
  
  // Write out the string until the terminator or until we've written the max
  while( (count < MaxChars) && str[count] )
  {
    FileWrite(str[count]);
    count++;
  }
  FileClose();
}

uint32_t FileSize(void)
{
  return(myFile.size());
}

uint32_t FilePosition(void)
{
  return(myFile.position());
}

bool FileSeek(uint32_t offset)
{
  return(myFile.seek(offset));
}

bool myFileIsOpen(void)
{
  if(myFile)
    return true;
  else
    return false;
}

