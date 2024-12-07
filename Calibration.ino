#include "Calibration.h"

void CalibrateLoop(void)
{
  ProcessSerial(false);
  control.run();
  Watchdog.reset();
}

int CalibrateRFlevelPoint(ADCchan *adcchan, float *Vpp)
{
  char   *Token;
  String sToken;
  float  drive;

  // Set drive level
  serial->print("Enter drive level percentage: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  sToken = Token;
  serial->println(Token);
  drive = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
  setDrive(drive);
  // Enter measured RF level
  serial->print("Enter RF level, Vpp: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  sToken = Token;
  serial->println(Token);
  *Vpp = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
  if(adcchan != NULL) return GetADCvalue(adcchan->Chan, 20);
  return 0; 
}

void CalibrateRFlevel(ADCchan *adcchan)
{
  float  val1,val2,m,b;
  int    adcV1, adcV2;

  // Calibration point 1
  adcV1 = CalibrateRFlevelPoint(adcchan, &val1);
  // Calibration point 2
  adcV2 = CalibrateRFlevelPoint(adcchan, &val2);
  if(adcchan == NULL) return;
  m = (float)(adcV2-adcV1) / (val2-val1);
  b = (float)adcV1 - val1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  adcchan->m = m;
  adcchan->b = b;
  setDrive(fragdata.Drive);
}

void CalibrateRFA(void)
{
  serial->println("Calibrate RF level channel A.");
  CalibrateRFlevel(&fragdata.RFlevelA);
}

void CalibrateRFB(void)
{
  serial->println("Calibrate RF level channel B.");
  CalibrateRFlevel(&fragdata.RFlevelB);
}

void CalibrateCurrent(void)
{
  serial->println("Calibrate current sensor.");
  // Set first drive level and ask for the current value
  fragdata.Drive  = UserInputFloat((char *)"Enter drive level 1 : ", CalibrateLoop);
  float cur1 = UserInputFloat((char *)"Enter current, amps : ", CalibrateLoop);
  int   adc1 = GetADCvalue(fragdata.DriveI.Chan,20);
  // Set second drive level and ask for the current value
  fragdata.Drive = UserInputFloat((char *)"Enter drive level 2 : ", CalibrateLoop);
  float cur2 = UserInputFloat((char *)"Enter current, amps : ", CalibrateLoop);
  int   adc2 = GetADCvalue(fragdata.DriveI.Chan,20);
  // Calculate the calibration parameters and apply.
  // counts = value * m + b
  // adc1 = cur1 * m + b
  // adc2 = cur2 * m + b
  // adc1 - adc2 = (cur1 - cur2) * m
  // m = (adc1 - adc2) / (cur1 - cur2)
  // b = adc2 - cur2 * m
  fragdata.Drive = 10;
  fragdata.DriveI.m = (float)(adc1 - adc2) / (cur1 - cur2);
  fragdata.DriveI.b = (float)adc2 - cur2 * fragdata.DriveI.m;
  serial->print("m = "); serial->println(fragdata.DriveI.m); 
  serial->print("b = "); serial->println(fragdata.DriveI.b); 
}


