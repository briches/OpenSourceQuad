#include <SoftwareSerial.h>
#define upperLimit 100
#define lowerLimit 0

SoftwareSerial XBee(10, 11);
double filter[10];
double sum;

void setup()
{
  Serial.begin(115200);
  XBee.begin(115200);
}

void loop()
{
  // Read potentiometer SEND
  int SEND = analogRead(3);
  SEND = map(SEND,0, 1023, upperLimit, lowerLimit);
  
  
  // Filter out 
  sum = 0;
  for(int i = 9; i>0; i--)
  {
    filter[i] = filter[i-1];
    sum += filter[i];
  }
  filter[0] = SEND;
  sum += SEND;
  SEND = sum/10;
  
  // Send the data to the respective serial ports.
  Serial.print((uint8_t)SEND); 
  Serial.print(" ");
  Serial.println(XBee.write((uint8_t)SEND));
  
  delay(100);
}
