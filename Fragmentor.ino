#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_SleepyDog.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "Fragmentor.h"
#include "Errors.h"
#include "Serial.h"
#include <Adafruit_SPIFlash.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

//
// Gordon Anderson
//
// The fragmentor hardware uses an Adafruit ItsyBitsy M4 processor.
// 
// Version histoory
//
//  1.0, May12, 2023
//    - Initial release
//  1.1, Aug 27, 2023
//    - Added current calibration function
//  1.2, Oct 27, 2023
//    - Added Arc detection
//    - Added watchdog timer
//    - Added ramp up on RF
//  1.3, Dec 21, 2023
//    - Disable arc det during tune
//    - Reranged gain, multiplied by 1000
//  1.4, Sept 19, 2024
//    - Added the flash file system 
//    - Added the bootloader command
//

// PLL is not stable below 250,000Hz
#define MinFreq     400000

#define driveStep   0.1

unsigned int serialTime = 0;

const char   Version[] PROGMEM = "Fragmentor version 1.4, Sept 19, 2024";
FragData     fragdata;

FragData Rev_1_fragdata = 
{
  sizeof(FragData),"Fragmentor",1,
  0x58,
  false,
  2805000,4000000,0,500,false,false,
  4.0,
  60,25,
  0,0.37,8.07,
  1,0.45,6.28,
  2,113.07,0,
  3,305.26,406.37,
  true, 0.7,
  true,
  false,
  SIGNATURE
};

// System variables
State state;
float RFvoltageA = 0;
float RFvoltageB = 0;
float DriveVolt = 0;
float DriveCurrent = 0;
float Power = 0;

// Auto tune parameters
bool TuneRequest   = false;
bool RetuneRequest = false;
bool Tuning        = false;
bool TuneReport    = false;
// Tune states
#define TUNE_SCAN_DOWN 1
#define TUNE_SCAN_UP 2

#define MaxNumDown 5
#define MAXSTEP    100000


// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is cleared every time
// the sketch is uploaded on the board.
FlashStorage(flash_fragdata, FragData);

// for flashTransport definition
#include "flash_config.h"
Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatVolume fatfs;
File32 file;

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

void Control(void)
{
  if(fragdata.Mode)
  {
    // Here if in closed loop control mode.
    float error = fragdata.Setpoint - (RFvoltageA + RFvoltageB);
    fragdata.Drive += error * fragdata.Gain / 1000;
    if(fragdata.Drive < 0) fragdata.Drive=0;
    if(fragdata.Drive > fragdata.MaxDrive) fragdata.Drive = fragdata.MaxDrive;
  }
}

// This function is called at 40 Hz
void Update(void)
{
  float        val,val2;
  static int   holdOff = 0;
  
  serialTime += 25;
// Update all changes
  if((state.Enable != fragdata.Enable) || (state.update))
  {
    state.Enable = fragdata.Enable;
    if(fragdata.Enable)
    {
      //setDrive(fragdata.Drive);
      state.Drive = 0;
    }
    else
    {
      setDrive(0);
    }
  }
  if((state.Freq != fragdata.Freq) || (state.update))
  {
    state.Freq = fragdata.Freq;
    setFrequency(fragdata.Freq);
  }
  if((state.Drive != fragdata.Drive) || (state.update))
  {
    if(fragdata.Enable)
    {
      holdOff = 40;
      // Ramp up logic
      if(fragdata.Drive > state.Drive)
      {
        state.Drive += driveStep;
        if(state.Drive > fragdata.Drive) state.Drive = fragdata.Drive;
      }
      else state.Drive = fragdata.Drive;
      setDrive(state.Drive);
    }
  }
  if((state.Setpoint != fragdata.Setpoint) || (state.update))
  {
    state.Setpoint = fragdata.Setpoint;
  }
  if((state.Mode != fragdata.Mode) || (state.update))
  {
    state.Mode = fragdata.Mode;
  }
  if((state.Gate != fragdata.Gate) || (state.update))
  {
    state.Gate = fragdata.Gate;
  }
  state.update = false;
// Read the ADC values and convert
  val = ReadADCchannel(fragdata.RFlevelA);
  RFvoltageA = (1.0 - FILTER) * RFvoltageA + FILTER * val;
  val2 = ReadADCchannel(fragdata.RFlevelB);
  RFvoltageB = (1.0 - FILTER) * RFvoltageB + FILTER * val2;

  // The following logic is for arc detection and system shutdown
  if(holdOff <= 0)
  {
    if((val + val2) < ((RFvoltageA + RFvoltageB) * fragdata.arcSens))
    {
      if((fragdata.arcdDetEna) && ((RFvoltageA + RFvoltageB) > 500))
        fragdata.Enable = false;
    }
  }
  else holdOff--;
  val = ReadADCchannel(fragdata.DriveV);
  DriveVolt = (1.0 - FILTER) * DriveVolt + FILTER * val;
  val = ReadADCchannel(fragdata.DriveI);
  DriveCurrent = (1.0 - FILTER) * DriveCurrent + FILTER * val;
  if(DriveCurrent < 0) DriveCurrent = 0;
  Power = DriveVolt * DriveCurrent;
// Control functions
  RFdriver_tune();
  Control();
// Check limits and apply
  if(fragdata.Drive > fragdata.MaxDrive) fragdata.Drive = fragdata.MaxDrive;
}

void setup() 
{
  //setupD10pwm();
  //setDrive(0);
    // This disables the  drive
  pinMode(ENADRV, OUTPUT);
  digitalWrite(ENADRV, HIGH);
  // This disables the RF
  pinMode(ENAFRQ, OUTPUT);
  digitalWrite(ENAFRQ, HIGH);

  // Read the flash config contents and test the signature
  fragdata = flash_fragdata.read();
  if(fragdata.Signature != SIGNATURE) fragdata = Rev_1_fragdata;
  // Init serial communications
  SerialInit();
  Serial1.begin(19200); // port connected to MIPS box through fragmentor interface
  // Init the TWI interface
  Wire.begin();
  // Init the driver timer
  setupD10pwm();
  // Init the DIO
  // This disables the  drive
  pinMode(ENADRV, OUTPUT);
  digitalWrite(ENADRV, HIGH);
  // This disables the RF
  pinMode(ENAFRQ, OUTPUT);
  digitalWrite(ENAFRQ, HIGH);
  analogReadResolution(12);
  analogWriteResolution(12);
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Set the frequency
  setFrequency(fragdata.Freq);
  // Set the drive
  setDrive(fragdata.Drive);
  state.update = true; // force full update.
  // Setup the watchdog timer
  if(fragdata.watchDog) Watchdog.enable(4000);
}

void ReadAllSerial(void)
{
  ProcessSerial(false);
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serialTime = 0;
    serial = &Serial;
    PutCh(Serial.read());
  }
  if (Serial1.available() > 0)
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

// Auto tune algorithm, procedure is as follows:
// 1.) Set power tp 10%
// 2.) Set frequency to 1MHz
// 3.) Go down in frequency in 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 4.) Go up in frequency from 1MHzin 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 5.) Use the peak found in steps 3 and 4 and sweep in 10K steps using the same procedure in 3 and 4
// 6.) Use the peak found in step 5 and sweep in 1K steps using the same procedure in 3 and 4
// 7.) Done!
//
// Called from the main processing loop, this function does not block, uses a state machine to perform thge logic
// States
//  TUNE_SCAN_DOWN
//  TUNE_SCAN_UP

void RFdriver_tune(void)
{
   static int    TuneStep = 100000, TuneState;
   static float  Max, Current, Last;
   static int    FreqMax, Freq;
   static int    NumDown,Nth;
   static bool   arcDetSave;

   if(TuneRequest)
   {
     fragdata.Mode = false; // Make sure we are in open loop mode!
     // Set freq to 1MHz
     fragdata.Freq = 1000000;
     // Set drive to 10%
     fragdata.Enable = true;
     fragdata.Drive = 10;
     arcDetSave = fragdata.arcdDetEna;
     fragdata.arcdDetEna = false;
     Tuning = true;
     TuneStep = MAXSTEP;
     Freq = 1000000;
     Last = Max = 0;
     NumDown = -MaxNumDown;
     TuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     //TuneReport = true;
     if(TuneReport) serial->println("Tuning...");
     return;
   }
   if(RetuneRequest)
   {
     fragdata.Mode = false; // Make sure we are in open loop mode!
     // Set freq to current
     Freq = fragdata.Freq;
     Tuning = true;
     TuneStep = 1000;
     Last = Max = 0;
     NumDown = 0;
     RetuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     if(TuneReport) serial->println("Re-tuning...");
     return;
   }
   if(!Tuning) return;
   if(--Nth > 0) return;
   Nth = 20;
   // Here if the system is tuning
   Current = RFvoltageA + RFvoltageB;
   switch (TuneState)
   {
     case TUNE_SCAN_DOWN:
//        if(Current > Max)
//        {
//          Max = Current;
//          FreqMax = fragdata.Freq;
//        }
        if(Current <= (Last + 1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        fragdata.Freq -= TuneStep;
        if((NumDown >= MaxNumDown) || (fragdata.Freq < MinFreq))
        {
          TuneState = TUNE_SCAN_UP;
          fragdata.Freq = Freq;
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        if(Current > Max)  // Move this code here to force tune system to not use the limit value
        {
          Max = Current;
          FreqMax = fragdata.Freq;
        }
        if(Current <= (Last + 1)) NumDown++;
        break;
     case TUNE_SCAN_UP:
        if(Current > Max)
        {
          Max = Current;
          FreqMax = fragdata.Freq;
        }
        if(Current <= (Last +1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        fragdata.Freq += TuneStep;
        if((NumDown >= MaxNumDown) || (fragdata.Freq > fragdata.MaxFreq))
        {
          // Here we have found the peak for this step size, this
          // process repeats until step size is 1KHz
          Freq = FreqMax;
          if(Freq < MinFreq) Freq = MinFreq;
          if(Freq > fragdata.MaxFreq) Freq = fragdata.MaxFreq;
          fragdata.Freq = Freq;
          if(TuneStep == 1000)
          {
            // If here we are done!
            Tuning = false;
            if(TuneReport && !SerialMute)
            {
              serial->print("Auto tune complete, frequency = ");
              serial->println(Freq);
            }
            TuneReport = false;
            fragdata.arcdDetEna = arcDetSave;
            return;
          }
          else 
          {
            if(TuneStep == MAXSTEP) TuneStep = 10000;
            else TuneStep /= 10;
          }
          TuneState = TUNE_SCAN_DOWN;
          NumDown = 0;
        }
        break;
     default:
        break;
   }
   Last = Current;
}

void loop() 
{
  ProcessSerial();
  control.run();
  Watchdog.reset();
  if(fragdata.serialWatchDog) if(serialTime > 10000) NVIC_SystemReset(); 
}

//
// Host command functions
//

void SaveSettings(void)
{
  fragdata.Signature = SIGNATURE;
  flash_fragdata.write(fragdata);
  SendACK;
}

void RestoreSettings(void)
{
  static FragData fd;
  
  // Read the flash config contents and test the signature
  fd = flash_fragdata.read();
  if(fd.Signature == SIGNATURE) fragdata = fd;
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
}

void FormatFLASH(void)
{
  flash_fragdata.write(Rev_1_fragdata);  
  SendACK;
}

void ReadADC(int chan)
{
  SendACKonly;
  serial->println(GetADCvalue(chan, 20));
}

void AutoTune(void)
{
  SendACK;
  TuneRequest = true;
  TuneReport = false;
}

void AutoTuneReport(void)
{
  SendACK;
  TuneRequest = true;
  TuneReport = true;
}

void Debug(int i)
{
}

void bootloader(void)
{
  __disable_irq();
 	//THESE MUST MATCH THE BOOTLOADER
	#define DOUBLE_TAP_MAGIC 			      0xf01669efUL
	#define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)

	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DOUBLE_TAP_MAGIC;
	//NVMCTRL->ADDR.reg  = APP_START;
	//NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;
	
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}

// Functions supporting Circuit Python file system (CPFS). This is used to save
// the configuration and calibration data to the SPI flash filesystem.
// Provides non volitial storage of setup data.
bool FSsetup(void)
{
  // Init external flash
  if (!flash.begin()) serial->println("Error, failed to initialize flash chip!");
  else
  {
    serial->println("Flash chip initalized!");
    if(!fatfs.begin(&flash)) serial->println("Failed to mount filesystem!");
    else
    {
      serial->println("Mounted filesystem!");
      return true;
    }
  }
  return false;
}
void saveDefaults(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_WRITE | O_CREAT))==0) serial->println("Can't create default.dat!");
  else
  {
    size_t num = file.write((void *)&fragdata,sizeof(FragData));
    file.close();
    serial->print("default.dat written, number of bytes = ");
    serial->println(num);
  }
}
void loadDefaults(void)
{
  FragData h;

  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_READ))==0) serial->println("Can't open default.dat!");
  else
  {
    size_t num = file.read((void *)&h,sizeof(FragData));
    file.close();
    if((num != sizeof(FragData)) || (h.Signature != SIGNATURE))
    {
      serial->println("Error reading default.dat file!");
      return;
    }
    fragdata = h;
    serial->print("default.dat read, number of bytes = ");
    serial->println(num);
  }
}
void saveCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_WRITE | O_CREAT))==0) serial->println("Can't create cal.dat!");
  else
  {
    size_t num = file.write((void *)&fragdata.RFlevelA,sizeof(ADCchan));
    num += file.write((void *)&fragdata.RFlevelB,sizeof(ADCchan));
    num += file.write((void *)&fragdata.DriveV,sizeof(ADCchan));
    num += file.write((void *)&fragdata.DriveI,sizeof(ADCchan));
    file.close();
    serial->print("cal.dat written, number of bytes = ");
    serial->println(num);
  }
}
void loadCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_READ))==0) serial->println("Can't open cal.dat!");
  else
  {
    size_t num = file.read((void *)&fragdata.RFlevelA,sizeof(ADCchan));
    num += file.read((void *)&fragdata.RFlevelB,sizeof(ADCchan));
    num += file.read((void *)&fragdata.DriveV,sizeof(ADCchan));
    num += file.read((void *)&fragdata.DriveI,sizeof(ADCchan));
    file.close();
    serial->print("cal.dat read, number of bytes = ");
    serial->println(num);
  }
}
// End of CPFS
