#ifndef Fragmentor_h
#define Fragmentor_h
#include "Hardware.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

extern float RFvoltageA;
extern float RFvoltageB;
extern float DriveVolt;
extern float DriveCurrent;
extern float Power;

extern bool TuneRequest;

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "Fragmentor"
  int8_t        Rev;                    // Holds the board revision number
  int           TWIadd;
  // Fragmentor settings
  bool          Enable;
  int           Freq;                   // Fragmentor operating frequency
  int           MaxFreq;                // Upper frequency limit
  float         Drive;
  float         Setpoint;               // RF voltage setpoint in volts
  bool          Mode;                   // Mode = true for closed loop, false for open
  bool          Gate;                   // True to enable gate mode
  float         Gain;                   // Closed loop control gain
  // Limits
  float         MaxDrive;
  float         MaxPower;
  // ADC channels
  ADCchan       RFlevelA;
  ADCchan       RFlevelB;
  ADCchan       DriveV;
  ADCchan       DriveI;
  // Arc detection
  bool          arcdDetEna;
  float         arcSens;
  // Watchdog timer flag
  bool          watchDog;
  bool          serialWatchDog;
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} FragData;

typedef struct
{
  bool          update;
  // Fragmentor settings
  bool          Enable;
  int           Freq;                   // Fragmentor operating frequency
  float         Drive;
  float         Setpoint;               // RF voltage setpoint in volts
  bool          Mode;                   // Mode = true for closed loop, false for open
  bool          Gate;                   // True to enable gate mode
} State;


// Prototypes...
void ReadAllSerial(void);
void ProcessSerial(bool scan = true);
void SaveSettings(void);
void RestoreSettings(void);
void Software_Reset(void);
void FormatFLASH(void);
void Debug(int i);
void ReadADC(int chan);
void RFdriver_tune(void);
void AutoTune(void);
void AutoTuneReport(void);

void bootloader(void);
void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

#endif
