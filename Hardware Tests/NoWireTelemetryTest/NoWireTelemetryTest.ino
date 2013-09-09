#include "OSQ_NoWire.h"
#include <SoftwareSerial.h>

NoWire  receiver;  // Class
double P, I, D;

void setup()
{
        Serial.begin(19200);
        receiver.start();
}

void loop()
{        
        
        
        switch(receiver.ScanForMessages())
        {
        case err:
                break;

        case 0:
                Serial.println("Message received: disarm");
                break;

        case 1:
                Serial.println("Message received: setP");
                P = 65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3];
                Serial.println("P gain now set to :");
                Serial.println(P);
                Serial.println();
                break;
                
        case 2:
                Serial.println("Message received: setI");
                I = 65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3];
                Serial.println("I gain now set to :");
                Serial.println(I);
                Serial.println();
                break;

        case 3:
                Serial.println("Message received: setD");
                D = 65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3];
                Serial.println("D gain now set to :");
                Serial.println(D);
                Serial.println();
                break;

        default:
                Serial.println("Message not received");
                break;
        }
}

