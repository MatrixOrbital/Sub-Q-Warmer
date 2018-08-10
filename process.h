#ifndef PROCESS_H
#define PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#define CheckSensorInterval     5000  // in mS
#define CheckHeaterInterval     5000  // in mS
#define CheckSolutionInterval   16000 // in mS
#define PressTimoutInterval     4000  // in mS
#define CheckTouchInterval        15  // in mS
#define CheckSwipeInterval        60  // in mS
#define CheckPWMInterval          16  // in mS - PWM Base period = 256 * CheckPWMInterval
#define ScreenUpdateInterval      50  // in mS

// These integer values are x10 too big in order to get a decimal place but still use integers
typedef struct {
  uint16_t PlateTemp;
  uint16_t PlateGoal;
  uint16_t SolutionTemp;
  uint16_t SolutionGoal;
  char ButtonText[16];
  char ReadyText[16];
  char PlateTempText[6];
  char SolutionTempText[6];
  char GoalText[6];
  bool HeaterOn;
  bool Activated;
  bool Ready;
}ScreenParms;

extern ScreenParms MainScreen;

void MakeScreen_Main(void);
uint32_t Load_JPG(uint32_t BaseAdd, uint32_t Options, char *filename); 
void CheckScreen(void);
void CheckSensors(void);
void CheckSolution(void);   // Check the sensor data and update TimeTillCat value
void CheckHeater(void) __attribute__((__optimize__("O2")));     // Run PID loop for heater
void CheckTouch(void);      // Check for user touching and update values
void InsertDecimal(char * str);
void SetupMainScreen(void);

#ifdef __cplusplus
}
#endif

#endif
