#include <Servo.h>

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

int x;

void setup()
{
  Serial.begin(9600);
  Motor1.attach(2);
  Motor2.attach(3);
  Motor3.attach(4);
  Motor4.attach(5);
}

void loop()
{
if (x == 0)
{
  for(x = 1200; x <= 2000  ; x += 5)
  {
    Motor1.write(x);
    Motor2.write(x);
    Motor3.write(x);
    Motor4.write(x);
    delay(50);
  }
}
 if (millis() >= 10000)
 {
   Motor1.detach();
   Motor2.detach();
   Motor3.detach();
   Motor4.detach();
 }
}
