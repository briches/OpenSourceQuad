#include <Servo.h>

Servo Motor1;
Servo Motor2;

String incomingString;

void setup()
{
  Motor1.attach(9);
  Motor2.attach(10);
  
  Serial.begin(9600);
  
  Serial.println("Initializing");
}

void loop()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch != 10)
    {
      Serial.print("Received: ");
      Serial.println(ch, DEC);
      incomingString += ch;
    }
    else // Recieved newline
    {       
      int val = incomingString.toInt();
      
      Serial.print("Interger Value: ");
      Serial.println(val);
      
      if (val > -1 && val < 181)
      {
        Serial.println("Value is between 0 and 180");
        // Write to Servo
        Motor1.write(val);
        Motor2.write(val);
      }
      else
      {
        Serial.println("Value is NOT between 0 and 180");
        Serial.println("Error with the input");
      }
      incomingString = "";
      
    }
  }
}
        
        
