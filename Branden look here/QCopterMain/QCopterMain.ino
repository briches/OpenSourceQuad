

#include <Control.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>

Control QCopter;

void setup()
{
  QCopter.initSensor();
}
  
void loop()
{
  QCopter.updateData_State();
  
}
  
  
