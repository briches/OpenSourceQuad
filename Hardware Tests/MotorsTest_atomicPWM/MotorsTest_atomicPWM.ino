// Compile for mega2560 only
#include "OSQ_Motors.h"

OSQ_MotorControl myMotors;

void setup()
{
  Serial.begin(115200);
  delay(5000);
  
  myMotors.calibrateESC();
  myMotors.startMotors();
  
}

void pulseTest()
{
  
  for(int i =0; i<16;i++)
  {
    
    delay(200);
    motorSpeeds[0] = (MAX_COMMAND+MIN_COMMAND)/2;
    motorSpeeds[1] = (MAX_COMMAND+MIN_COMMAND)/2;
    motorSpeeds[2] = (MAX_COMMAND+MIN_COMMAND)/2;
    motorSpeeds[3] = (MAX_COMMAND+MIN_COMMAND)/2;
    writeMotors();
    
    delay(200);
    motorSpeeds[0] = MIN_COMMAND;
    motorSpeeds[1] = MIN_COMMAND;
    motorSpeeds[2] = MIN_COMMAND;
    motorSpeeds[3] = MIN_COMMAND;
    writeMotors();
    
    
  }
}

void loop()
{
  Serial.println("********************Begin Motor test********************");
  
  int control_var = 150;
  motorSpeeds[0] = 1250 + control_var;
  motorSpeeds[3] = 1250 - control_var;
  writeMotors();
  
  
}
