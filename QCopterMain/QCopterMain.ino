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

void setup()
{
  Serial.begin(9600);
  QCopter.initSensor();
}
  
void loop()
{
  QCopter.updateData_State();
  Serial.println(QCopter.Data_State.heading);
}
