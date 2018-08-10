// Process.c is the application layer.  All function calls are hardware ambivalent.
//
// NOTES:
// The gauges and trackers are scaled up by a factor of 10 in order to preserve one decimal place.
// Additionally, they are cut off on the low side to a minimum value because this thing is a heater, not a heater/chiller
// For instance, the Solution gauge is scaled to a maximum value of 200, but the selected input value is reduced by 200
// The end result is that the temperature is selected from 20.0 degrees to 40.0 degrees and the gauge ranges from 20 
// degrees to 40 degrees.
// 

#include <stdint.h>                // Find integer types like "uint8_t"  
#include "Eve2_81x.h"              // Matrix Orbital Eve2 Driver
#include "Arduino_AL.h"            // include the hardware specific abstraction layer header for the specific hardware in use.
#include "MatrixEve2Conf.h"        // Header for EVE2 Display configuration settings
#include "process.h"               // Every c file has it's header and this is the one for this file

uint32_t Time2CheckSensor = 0;     // Private variable holding time of next reading of the temperature sensors 
uint32_t Time2CheckSolution = 0;   // Private variable holding time of next check of the solution heat request PID loop
uint32_t Time2CheckHeater = 0;     // Private variable holding time of next check of the heater power request PID loop
uint32_t Time2CheckTouch = 0;      // Private variable holding time of next check for user input
uint32_t Time2UpdateScreen = 0;    // Private variable holding time of next screen update
uint32_t Time2CheckPWM = 0;        // Private variable holding time of next PWM sub-period (in resolution of CheckPWMInterval)
uint32_t PressTimeout = 0;         // Private variable counting down time you can spend pressing the screen
uint8_t  PWM_Base_Count = 0;       // Private variable - The timebase is pre-chosen to be 256 counts
uint8_t  PWM_Val;                  // Private variable - this is the "on time" per PWM base period in CheckPWMInterval counts
float HeaterVal;                   // Private variable - holds a float version of the current temperature of the plate                 
float SolutionVal;                 // Private variable - holds a float version of the current temperature of the solution 
uint16_t SaveCount = 0;

ScreenParms MainScreen;            // Global parameters related to the singular screen of the application

// This screen construction is built to work with the screen rotated.  Due to the fact that the screen has a "natural" size in
// the y direction (VSIZE) which is different than its actual size (DHEIGHT), rotating a screen made "natural" will shift it
// off of the real screen.  The required translation looks like: Ynatural + (VSIZE - DHEIGHT)
void MakeScreen_Main(void)
{
//  Log("Enter Makescreen\n");

  Send_CMD(CMD_DLSTART);
  Send_CMD(CLEAR(1,1,1));

  Cmd_Gradient(194, 21, 0x007FFF, 250, 280, 0x70FF00);                                // Diagonal gradient blueish to yellowish

  Cmd_FGcolor(0x222288);                                                              // Clear color before starting the screen
  //==================== Plate Gauge setup and implementation ============================
  if(MainScreen.HeaterOn)
    Cmd_BGcolor(0x992222);                                                            // Reddish colour indicates heater is on
  else
    Cmd_BGcolor(0x444444);                                                            // Grey colour indicates heater is off

  Send_CMD(COLOR_RGB( 0x88, 0x88, 0x88));                                             // Change color of plate temperature goal needle
  Cmd_Gauge(57, 211, 52, 0, 4, 8, MainScreen.PlateGoal, 700);                         // Show gauge needle on top for plate temperature goal
  Send_CMD(COLOR_RGB(0xFF, 0xFF, 0xFF));                                              // Change color of needle
  Cmd_Gauge(57, 211, 52, OPT_NOBACK|OPT_NOTICKS, 4, 8, MainScreen.PlateTemp, 700);    // Show gauge for plate temperature
  Send_CMD(COLOR_RGB( 0x88, 0xFF, 0x88));                                             // Change color of text temperature display
  Cmd_Text(57, 248, 27, OPT_CENTER, MainScreen.PlateTempText);                        // display the modified string on top of the control

  //==================== Solution Gauge setup and implementation ==========================
  Cmd_BGcolor(0x222288);                                                              // Background colour of the solution gauge

  Send_CMD(COLOR_RGB( 0x88, 0x88, 0x88));                                             // Change color of solution temperature goal needle
  Cmd_Gauge(165, 211, 52, 0, 4, 8, MainScreen.SolutionGoal-200, 200);                 // Show Gauge needle on top for solution temperature goal (see notes at top of file)
  Send_CMD(COLOR_RGB( 0xFF, 0xFF, 0xFF));                                             // Change color of needle
  Cmd_Gauge(165,211,52,OPT_NOBACK|OPT_NOTICKS,4,8, MainScreen.SolutionTemp-200, 200); // Show gauge for solution temperature 
  Send_CMD(COLOR_RGB( 0x88, 0xFF, 0x88));                                             // Change color of text temperature display
  Cmd_Text(165, 248, 27, OPT_CENTER, MainScreen.SolutionTempText);                    // display the modified string on top of the control

  //=================== Activation button setup and implementation ========================
  Send_CMD(COLOR_RGB( 0xAA, 0xFF, 0xAA));                                             // Change color of Text
  Send_CMD(TAG(1));                                                                   // Tag the following button as a touch region with a return value of 1
  Cmd_Button(230, 207, 124, 52, 29, 0, MainScreen.ButtonText);

  //================ Setpoint selection dial setup and implementation =====================
  Send_CMD(COLOR_RGB( 0xFF, 0xFF, 0xFF));                                             // Change color of Dial Indicator
  Cmd_Track(421, 211, 1, 1, 11);                                                      // Make a tracker matching the following tag of 11
  Send_CMD(TAG(11));                                                                  // Tag the following dial with a return value of 11
  Cmd_Dial(421, 211, 52, 0, MainScreen.SolutionGoal * 327);                           // 327 = pre-calculated scaling factor = 65536/200 where 200 is the range of the dial
  Send_CMD(COLOR_RGB( 0x88, 0xFF, 0x88));                                             // Change color of text goal display
  Cmd_Text(421, 211, 28, OPT_CENTER, MainScreen.GoalText);                            // display the modified string on top of the control

  //==================== Ready Indicator setup and implementation =========================
  if(MainScreen.Ready)
  {
    Cmd_FGcolor(0x00AA11);                                                            // Greenish colour indicates ready
    Send_CMD(COLOR_RGB( 0x55, 0xFF, 0xBB));                                           // Change color of Text
  }
  else
  {
    Cmd_FGcolor(0xBB2222);                                                            // Reddish colour indicates not ready
    Send_CMD(COLOR_RGB( 0xFF, 0xAA, 0x55));                                           // Change color of Text
  }
  Cmd_Button(230, 6 + (VSIZE-DHEIGHT), 124, 36, 27, OPT_FLAT, MainScreen.ReadyText);

  Send_CMD(DISPLAY());
  Send_CMD(CMD_SWAP);
  UpdateFIFO();                                                                      // Trigger the CoProcessor to start processing commands out of the FIFO
}

void SetupMainScreen(void)
{

  MainScreen.PlateTemp = readTempProbe(OWTP_Plate);                  // read the temp probe and initiate next measurement (comes in x2)
  MainScreen.SolutionTemp = readTempProbe(OWTP_Solution);            // read the temp probe and initiate next measurement (comes in x2)
  MyDelay(800);                                                      // We should not be reading the one-wire probes for about a second after startup

  // Initialize the current temperatures from an unfiltered reading
  MainScreen.PlateTemp = readTempProbe(OWTP_Plate);                  // read the temp probe and initiate next measurement (comes in x2)
  HeaterVal = MainScreen.PlateTemp * 5;                              // multiply the temperature by 10 to simulate a decimal place
  MainScreen.SolutionTemp = readTempProbe(OWTP_Solution);            // read the temp probe and initiate next measurement (comes in x2)
  SolutionVal = MainScreen.SolutionTemp * 5;                         // multiply the temperature by 10 to simulate a decimal place
  
  MainScreen.PlateGoal = 450;
  MainScreen.SolutionGoal = 375;
  MainScreen.Activated = false;
  sprintf(MainScreen.GoalText, "37.5");
  sprintf(MainScreen.ButtonText, "Activate");
  PID_Load_SetRange(MainScreen.SolutionGoal, 600);                   // set range of heater demand to safe levels.  Specified x10 in celsius

  MainScreen.Ready = false;
}

void CheckScreen(void)
{
  if (MyMillis() >= Time2UpdateScreen)
  {
    Time2UpdateScreen = MyMillis() + ScreenUpdateInterval;
    MakeScreen_Main();
  }
}

// As the temperatures are acquired, they are filtered by 5 samples.
// Since the numbers come in in 8 bits with a value x2 of the actual value
// and we want to multiply by 10 to gain a decimal point, the calculation 
// here would multiply by 5 to reach that.  However, we are going to filter
// by 5, so we just leave the value as is and add it to the previous number
// after subtracting 1/5 of its previous total.
//
// This filtering could be removed and quantize the selector to 0.5 degrees
// since the derivative term is not really affecting the loop anyway.
void CheckSensors(void)
{
  if (MyMillis() >= Time2CheckSensor)
  {
    Time2CheckSensor = MyMillis() + CheckSensorInterval;
    
    HeaterVal = ((HeaterVal * 4) / 5) + readTempProbe(OWTP_Plate);    // get new sample and filter by 5 samples  
    if(HeaterVal < 100) HeaterVal = 100;                              // We choose to peg the value to the lowest possible gauge value  
    MainScreen.PlateTemp = HeaterVal;                                 // Save the calculated value 
    snprintf(MainScreen.PlateTempText, 5, "%d", MainScreen.PlateTemp);
    InsertDecimal(MainScreen.PlateTempText);                          // Pre-format the aquired value into decimal number text
   
//    uint16_t newval = readTempProbe(OWTP_Solution);                 // read the temp probe and initiate next measurement (comes in x2)
//    float tmp = (SolutionVal * 4) / 5;                              // get 1/5 of the previous value and  multiply that value by 4   
//    SolutionVal = tmp + newval;                                     // add the new sample to it (which is already 1/5 scale)      

    SolutionVal = ((SolutionVal * 4) / 5) + readTempProbe(OWTP_Solution);// get new sample and filter by 5 samples
    if(SolutionVal < 200) SolutionVal = 200;                             // We choose to peg the value to the lowest possible gauge value  
    MainScreen.SolutionTemp = SolutionVal;                               // Save the calculated value 
    snprintf(MainScreen.SolutionTempText, 5, "%d", MainScreen.SolutionTemp);
    InsertDecimal(MainScreen.SolutionTempText);                          // Pre-format the aquired value into decimal number text

    if (MainScreen.SolutionTemp >= (MainScreen.SolutionGoal - 5))    // Alert the user when we get within a half degree of the goal
    {
      uint16_t sound = 0x4841;                                       // Select Xylophone note C3
      MainScreen.Ready = true;
      sprintf(MainScreen.ReadyText, "READY");
      if (MainScreen.SolutionTemp > (MainScreen.SolutionGoal + 10))  // This is too hot!  Set the danger alert  
      {
        MainScreen.Ready = false;
        sprintf(MainScreen.ReadyText, "OVER TEMP");
        sound = 0x4845;
      }
      
      SetPin(EveAudioEnable_PIN, 1);                                // Enable Audio
      wr8(REG_VOL_SOUND + RAM_REG, 0xFF);                           // Set the volume to maximum
      wr16(REG_SOUND + RAM_REG, sound);                             // 
      wr8(REG_PLAY + RAM_REG, 1);                                   // Play the sound
      while(rd8(REG_PLAY + RAM_REG));                               // Wait until sound finished
      SetPin(EveAudioEnable_PIN, 0);                                // Disable Audio
    }
    else
    {
      MainScreen.Ready = false;
      sprintf(MainScreen.ReadyText, "UNREADY");
    }

  }
}

void CheckSolution(void)
{
  if ( (MyMillis() >= Time2CheckSolution) && (MainScreen.Activated) )
  {
    Time2CheckSolution = MyMillis() + CheckSolutionInterval; // Setup timer value for next time slice
    
    // This is where we call the PID calculator for the solution
    // It generates the demand value which determines the heater setpoint goal
    MainScreen.PlateGoal = PID_Load_Step(MainScreen.SolutionGoal, MainScreen.SolutionTemp);
  }
}

void CheckHeater(void)
{
  if ( (MyMillis() >= Time2CheckHeater) && (MainScreen.Activated) )// Check for needed modifications to the output power (PWM)
  {
    Time2CheckHeater = MyMillis() + CheckHeaterInterval;
    PWM_Val = PID_Heater_Step(MainScreen.PlateGoal, MainScreen.PlateTemp);

    // Data logging for testing - save the first 1000 readings
    if (SaveCount < 1000)
    {
      // Log("%d: %d  %d\n", SaveCount, MainScreen.PlateTemp, MainScreen.SolutionTemp );
      uint8_t tmpstr[12];
      sprintf(tmpstr, "%d,%d\n", MainScreen.PlateTemp, MainScreen.SolutionTemp);
      FileWriteStr(tmpstr, 8);
      SaveCount++;
    }
  }

  if ((MyMillis() >= Time2CheckPWM) && (MainScreen.Activated)) // Actually do the PWM generation (software PWM) if system active
  {
    Time2CheckPWM = MyMillis() + CheckPWMInterval;         // Capture the time of the next PWM period
    PWM_Base_Count++;                                      // Count the number of PWM periods since the last PWM base time start
    
    if(PWM_Base_Count == 0)                                // This is the end of the PWM base period or "PWM base time start"
    {
      MainScreen.HeaterOn = true;                          // Heater tries to turn on at the beginning of every PWM base period
      SetPin(ControlOutput_PIN, 1);                        // Turn the heater ON - This is the only place where this can occur.
    }

    if(PWM_Base_Count >= PWM_Val)                          // See if we have counted beyond the PWM on time for this base period
    {
      MainScreen.HeaterOn = false;                         // Heater turns off until the next "PWM base time start"
      SetPin(ControlOutput_PIN, 0);                        // Turn the heater OFF
    }
  }
}

void CheckTouch(void)
{
  uint8_t Tag = 0;
  uint32_t tracker;
  bool FingerDown = false;
  uint32_t tmp;
  static bool FirstTouch = false;
  static uint16_t X_First, Y_First, X_Last, Y_Last;
  
  if (MyMillis() >= Time2CheckTouch)
  {
    tmp = rd32(REG_TOUCH_RAW_XY + RAM_REG); // This read eats any detected tag value (clears REG_TOUCH_TAG)

    // Check to see if we have non-FFFF values for raw X and Y coordinates
    if (tmp == 0xFFFFFFFF)
    {
      // Finger off
      if (FirstTouch)
      {
        // Time to evaluate values to detect swipe (we don't want to do this until the finger is off)
        if ( (abs(X_First - X_Last) > 0x200) && (abs(Y_First - Y_Last) > 0x100) )
        {
//         Log("\nSWIPE\n");
          Cmd_SetRotate(0);  // Rotate the display to normal orientation to do the calibration
          Calibrate_Manual(DWIDTH, DHEIGHT, PIXVOFFSET, PIXHOFFSET);
          SaveTouchMatrix();
//          LoadTouchMatrix(); // reload from flash to compare values
          Cmd_SetRotate(1);  // Rotate the display back to where it was.  The main screen is automatically redrawn.
        }
        FirstTouch = false;
      }
    }
    else                                                            
    {
      // Finger on - we might have a swipe starting or just a tag touch
      if(FirstTouch)                                             // We are already into swipe detection
      {
        X_Last = tmp >> 16;                                      // Store the latest X touch value
        Y_Last = tmp & 0xFFFF;                                   // Store the latest Y touch value
      }
      else                                                       // Finger down, but not already in swipe detection
      {
        // We need to wait here while the Eve gets another touch sample since we ate the one that got us here
        MyDelay(15);                                             
        
        Tag = rd8(REG_TOUCH_TAG + RAM_REG);                      // Check for tag touches
        Log("TAG: %d\n",Tag);
        if(Tag)
        {
          FingerDown = true;                                    // Finger is touching
          PressTimeout = MyMillis() + PressTimoutInterval;
          switch (Tag)
          {
          case 1:
            if (MainScreen.Activated)
            {
              MainScreen.Activated = false;
              MainScreen.HeaterOn = false;                       // Heater turns off when the system is deactivated
              SetPin(ControlOutput_PIN, 0);                      // Turn that heater OFF!
              sprintf(MainScreen.ButtonText, "Activate");
            }
            else
            {
              PID_ClearAll();                                   // Clean PID state in case it is still wound up from previous run.
              SaveCount = 0;                                    // Sample counter
              MainScreen.Activated = true;
              sprintf(MainScreen.ButtonText, "Deactivate");
            }

            do
            {
              MyDelay(50);                                        // Let's not do this too fast... 
              Tag = rd8(REG_TOUCH_TAG + RAM_REG);                 // Read finger status (this is a somewhat cheesy check) 
              if  ( (!Tag) || (MyMillis() >= PressTimeout) )
                FingerDown = false; 
            }while (FingerDown);
            break;
          case 11:
            // Since we need to update the screen here as we move the finger, this input function blocks all other operations.
            // If we happen to be heating the heater on entry to this function, it will not be controlled.
            // In order to limit the effect of an object resting on the screen, we start a timer here and time out the
            // input function to allow the heater to be turned off when required before it ends up here again.
        
            do
            {
              tracker = rd32(REG_TRACKER + RAM_REG);
              if ((tracker & 0xff) == 11)
              {
                MainScreen.SolutionGoal = 200 + ((tracker >> 16) * 200) / 65536; 
                PID_Load_SetRange(MainScreen.SolutionGoal, 600);  // set the requested solution temperature.  Specified x10 in celsius
              }

              // Pre-format the aquired value into decimal number text
              sprintf(MainScreen.GoalText, "%d", MainScreen.SolutionGoal);
              InsertDecimal(MainScreen.GoalText);
          
              MakeScreen_Main();                                  // Update the screen
                  
              MyDelay(30);                                        // Let's not do this too fast... 
              Tag = rd8(REG_TOUCH_TAG + RAM_REG);                 // Read finger status (this is a somewhat cheesy check)
              if ( (!Tag) || (MyMillis() >= PressTimeout) )       // You can mess with touching this for only a limited time
                FingerDown = false;                               // Finger is no longer touching
            }while (FingerDown);
            break; 
          default:                                                // Invalid tag value (importantly includes value 255)
            //============================== Start Swipe detection ==========================================
            // Store the first touch values and clear any previous last touch value
            X_First = X_Last = tmp >> 16;
            Y_First = Y_Last = tmp & 0xFFFF;
            FirstTouch = true;
            break;                                                // unrequired break
          } // switch(tag) 
        } // if (tag)
        else                                                      // Finger down, but not already detecting swipe and no tag
        {
          //============================== Start Swipe detection ==========================================
          // Store the first touch values and clear any previous last touch value
          X_First = X_Last = tmp >> 16;
          Y_First = Y_Last = tmp & 0xFFFF;
          FirstTouch = true;
        }
      }
    }
    
    Time2CheckTouch = MyMillis() + CheckTouchInterval;            // It is almost always time to chech touch
  }
}

// This define is for the size of the buffer we are going to use for data transfers.  It is 
// sitting here so uncomfortably because it is a silly tiny buffer in Arduino Uno and you
// will want a bigger one if you can get it.  Redefine this and add a nice buffer to Load_JPG()
#define COPYBUFSIZE WorkBuffSz

// Load a JPEG image from SD card into RAM_G at address "BaseAdd"
// Return value is the last RAM_G address used during the jpeg decompression operation.
uint32_t Load_JPG(uint32_t BaseAdd, uint32_t Options, char *filename)
{
  uint32_t Remaining;
  uint32_t ReadBlockSize = 0;

  // Open the file on SD card by name
  FileOpen(filename, FILEREAD);
  if(!myFileIsOpen())
  {
//    Log("%s not open\n", filename);
    FileClose();
    return false;
  }
  
  Remaining = FileSize();                                      // Store the size of the currently opened file
  
  Send_CMD(CMD_LOADIMAGE);                                     // Tell the CoProcessor to prepare for compressed data
  Send_CMD(BaseAdd);                                           // This is the address where decompressed data will go 
  Send_CMD(Options);                                           // Send options (options are mostly not obviously useful)

  while (Remaining)
  {
    if (Remaining > COPYBUFSIZE)
      ReadBlockSize = COPYBUFSIZE;
    else
      ReadBlockSize = Remaining;
    
    FileReadBuf(LogBuf, ReadBlockSize); // Read a block of data from the file
    
    // write the block to FIFO
    CoProWrCmdBuf(LogBuf, ReadBlockSize);                    // Does FIFO triggering
  
    // Calculate remaining
    Remaining -= ReadBlockSize;                              // Reduce remaining data value by amount just read
    // Log("Remaining = %ld RBS = %ld\n", Remaining, ReadBlockSize);
  }
  FileClose();

  Wait4CoProFIFOEmpty();                                     // wait here until the coprocessor has read and executed every pending command.

  // Get the address of the last RAM location used during inflation
  Cmd_GetPtr();                                              // FifoWriteLocation is updated twice so the data is returned to it's updated location - 4
  UpdateFIFO();                                              // force run the GetPtr command
  return (rd32(FifoWriteLocation + RAM_CMD - 4));            // The result is stored at the FifoWriteLocation - 4
}   

// InsertDecimal() takes a string and inserts a decimal place in the second last position
// The character array must be defined at least 6 characters in size.
void InsertDecimal(char * str)
{
  int8_t i;
  for(i = 0; i < 4; i++)
    if(str[i] == 0)                                   // search for the null terminator
      break;                                          // preserve its location when you find it
  str[i+1] = str[i];                                  // move the null terminator over
  str[i] = str[i-1];                                  // move the last digit over
  str[i-1] = '.';                                     // Insert a decimal
}

