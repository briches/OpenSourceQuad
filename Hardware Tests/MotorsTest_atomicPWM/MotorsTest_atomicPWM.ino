// Compile for mega2560 only
#include "OSQ_Motors.h"

OSQ_MotorControl myMotors;

void setup()
{
  Serial.begin(115200);
  
  myMotors.calibrateESC();
  myMotors.startMotors();
  
}

void individualTest()
{
  for(int i = 0; i < 4; i++)
  {
    // Test each motor individually. 
    // bring each motor up to 1200, then back to MIN_COMMAND
    for (int thrust = MIN_COMMAND; thrust < MAX_COMMAND; thrust += 10)
    {
      motorSpeeds[i] = thrust;
      writeMotors();
      delay(200);
    }
    
    // Test each motor individually. 
    for (int thrust = MAX_COMMAND; thrust > MIN_COMMAND; thrust -= 10)
    {
      motorSpeeds[i] = thrust;
      writeMotors();
      delay(200);
    }
    delay(100);
  }   
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
  Serial.println("Individual test:");
  individualTest();
  
  Serial.println("Pulse test:");
  pulseTest();
  
}
