#include "OSQ_NoWire.h"
#include <SoftwareSerial.h>

#define LED0  (2)
#define LED1  (3)
#define LED2  (4)
#define LED3  (9)
#define LED4  (10)
#define LED5  (11)

NoWire  receiver;

void writeAllDigital(int state)
{
  digitalWrite(LED0, state);
  digitalWrite(LED1, state);
  digitalWrite(LED2, state);
  digitalWrite(LED3, state);
  digitalWrite(LED4, state);
  digitalWrite(LED5, state);
}

void writeAllAnalog(int state)
{
  analogWrite(LED0, state);
  analogWrite(LED1, state);
  analogWrite(LED2, state);
  analogWrite(LED3, state);
  analogWrite(LED4, state);
  analogWrite(LED5, state);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  writeAllDigital(LOW);
  receiver.start();
}

void loop()
{
  switch(receiver.ScanForMessages())
  {
    case err:
      Serial.println("Message not received");
      break;
      
    case off:
      Serial.println("Message received: off");
      writeAllDigital(LOW);
      break;
      
    case on:
      Serial.println("Message received: on");
      writeAllDigital(HIGH);
      break;
      
    case LED0:
      Serial.println("Message received: LED0");
      analogWrite(LED0, receiver.userMessage[MSG_SIZE-1]);
      break;
      
    case LED1:
      Serial.println("Message received: LED1");
      analogWrite(LED1, receiver.userMessage[MSG_SIZE-1]);
      break;
      
      
    case LED2:
      Serial.println("Message received: LED2");
      analogWrite(LED2, receiver.userMessage[MSG_SIZE-1]);
      break;
    
      
    case LED3:
      Serial.println("Message received: LED3");
      analogWrite(LED3, receiver.userMessage[MSG_SIZE-1]);
      break;
    
      
    case LED4:
      Serial.println("Message received: LED4");
      analogWrite(LED4, receiver.userMessage[MSG_SIZE-1]);
      break;
      
      
    case LED5:
      Serial.println("Message received: LED5");
      analogWrite(LED5, receiver.userMessage[MSG_SIZE-1]);
      break;
      
    default:
      Serial.println("Message not received");
      break;
  }
  delay(100);
}
