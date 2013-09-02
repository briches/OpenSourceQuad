// Compile for mega2560 only
#include <OSQ_Motors.h>

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
    // bring each motor up to 1200, then back to 1000
    for (int thrust = 1000; thrust < 1200; thrust += 10)
    {
      motorSpeeds[i] = thrust;
      writeMotors();
      delay(200);
    }
    
    // Test each motor individually. 
    for (int thrust = 1200; thrust > 1000; thrust -= 10)
    {
      motorSpeeds[i] = thrust;
      writeMotors();
      delay(200);
    }
    delay(1000);
  }   
}

void pulseTest()
{
  int jitters = 0;
  while(jitters < 21)
  {
    motorSpeeds[0] = 1100;
    motorSpeeds[1] = 1100;
    motorSpeeds[2] = 1100;
    motorSpeeds[3] = 1100;
    
    writeMotors();
    
    motorSpeeds[0] = 1200;
    motorSpeeds[1] = 1200;
    motorSpeeds[2] = 1200;
    motorSpeeds[3] = 1200;
    
    writeMotors();
    
    motorSpeeds[0] = 1000;
    motorSpeeds[1] = 1000;
    motorSpeeds[2] = 1000;
    motorSpeeds[3] = 1000;
    
    writeMotors();
    
    delay(10);
    jitters++;
  }
}

void loop()
{
  Serial.println("********************Begin Motor test********************");
  Serial.println("Individual test:");
  individualTest();
  individualTest();
  
  Serial.println("Pulse test:");
  pulseTest();
  
}
