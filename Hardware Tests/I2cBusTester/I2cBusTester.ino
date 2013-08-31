#include <I2C.h>

void setup() {
  Serial.begin(19200);
  I2c.begin();
  I2c.timeOut(1000);
  I2c.setSpeed(0);
  I2c.pullup(1);
  I2c.scan();
  I2c.end();
}

void loop() {
  // put your main code here, to run repeatedly: 
    
}
