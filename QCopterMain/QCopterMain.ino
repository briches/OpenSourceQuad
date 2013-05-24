 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013


    -----------------------------------------------------------------------*/
    
#include <Control.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>

Control QCopter;
double rate = 0;
void setup()
{
  Serial.begin(115200);
  QCopter.initSensor();
  QCopter.Settings.g_threshold = 0.10; // 0.10 m/s^2 threshold for noise
  QCopter.Settings.d_threshold = 0.5; // 0.5 d/s threshold for noise
}
void loop()
{
  QCopter.updateData_State();
  rate = 1/((QCopter.Data_State.t_current - QCopter.Data_State.t_previous)/1000000);
  Serial.println((int)rate);
}
