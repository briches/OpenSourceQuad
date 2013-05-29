#include <Servo.h>

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

int x;
unsigned long time; 

void setup()
{
  Motor1.attach(11);
  Motor2.attach(10);
  Motor3.attach(9);
  Motor4.attach(6);
}

void loop()
{
  time = millis();
  if ((time < 10000) && (x ==0))
  {
    for(x = 0; x <= 50; x += 5)
    {
      Motor1.write(x);
      Motor2.write(x);
      Motor3.write(x);
      Motor4.write(x);
      delay(1000);
    }
  }
  else if ( (time >= 10000) && (x == 50) )
  {
    for(x = 50; x >= 0; x -= 5)
    {
      Motor1.write(x);
      Motor2.write(x);
      Motor3.write(x);
      Motor4.write(x);
      delay(250);
    }
  }
}
