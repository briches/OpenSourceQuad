#include "OSQ_NoWire.h"
#include <SoftwareSerial.h>

#define LED0  (3)
#define LED1  (5)
#define LED2  (6)
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
                break;

        case 0:
                Serial.println("Message received: off");
                writeAllDigital(LOW);
                break;

        case 1:
                Serial.println("Message received: on");
                writeAllDigital(HIGH);
                break;

        case 2:
                Serial.println("Message received: LED0");
                analogWrite(LED0, receiver.newMessage[MSG_SIZE-1]);
                break;

        case 3:
                Serial.println("Message received: LED1");
                analogWrite(LED1, receiver.newMessage[MSG_SIZE-1]);
                break;


        case 4:
                Serial.println("Message received: LED2");
                analogWrite(LED2, receiver.newMessage[MSG_SIZE-1]);
                break;


        case 5:
                Serial.println("Message received: LED3");
                analogWrite(LED3, receiver.newMessage[MSG_SIZE-1]);
                break;


        case 6:
                Serial.println("Message received: LED4");
                analogWrite(LED4, receiver.newMessage[MSG_SIZE-1]);
                break;


        case 7:
                Serial.println("Message received: LED5");
                analogWrite(LED5, receiver.newMessage[MSG_SIZE-1]);
                break;

        default:
                Serial.println("Message not received");
                break;
        }
}

