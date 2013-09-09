#include <SoftwareSerial.h>
#include "OSQ_NoWire.h"
#define upperLimit 100
#define lowerLimit 0

SoftwareSerial XBee(6, 7);
int filterP[10];
int filterI[10];
int filterD[10];
uint8_t packet[5] = {0,0,0,0,0};

void setup()
{
        Serial.begin(19200);
        XBee.begin(19200);
        Serial.println("*===========================PID_Send===========================*");
        pinMode(12, INPUT);
}



int readPGain()
{
        static int filterLocationP = 0;
        int P = 0;
        
        filterP[filterLocationP] = analogRead(5);
        
        filterLocationP++;
        if(filterLocationP == 10) filterLocationP = 0;
        
        for(int i = 0; i< 10; i++)
        {
                P += filterP[i];
        }
        
        return (P/10) >> 2;
}

int readIGain()
{
        static int filterLocationI = 0;
        int I = 0;
        
        filterI[filterLocationI] = analogRead(4);
        
        filterLocationI++;
        if(filterLocationI == 10) filterLocationI = 0;
        
        for(int i = 0; i< 10; i++)
        {
                I += filterI[i];
        }
        
        return (I/10) >> 2;
}

int readDGain()
{
        static int filterLocationD = 0;
        int D = 0;
        
        filterD[filterLocationD] = analogRead(3);
        
        filterLocationD++;
        if(filterLocationD == 10) filterLocationD = 0;
        
        for(int i = 0; i< 10; i++)
        {
                D += filterD[i];
        }
        
        return (D/10) >> 2;
}

void printData(int P, int I, int D)
{
        static long int printTime = 0;
        if((millis() - printTime) > 1000)
        {
                Serial.print("P: ");
                Serial.print(P);
                Serial.print("  I: ");
                Serial.print(I);
                Serial.print("  D: ");
                Serial.println(D);
                Serial.println();
                printTime = millis();
        }
}

bool checkWriteButton(int x)
{
        static long int buttonTime = 0;
        if(millis() - buttonTime > 500)
        {
                buttonTime = millis();
                return (x == HIGH);
                
        }
        else return false;
}

void sendPacket()
{
        XBee.write(packet[0]);
        XBee.write(packet[1]);
        XBee.write(packet[2]);
        XBee.write(packet[3]);
        XBee.write(packet[4]);
}

void loop()
{
        boolean tuneP = true;
        boolean tuneI = true;
        boolean tuneD = true;
        
        int P = readPGain() - 1;
        int I = readIGain() - 1;
        int D = readDGain() - 1;
        printData(P, I, D);
        
        bool send = checkWriteButton(digitalRead(12));
        if(send && tuneP) 
        {
                Serial.println("Sending P gain wirelessly!!");
                packet[0] |= B11111111;
                packet[1] = 0x01;        // Message code
                packet[2] = 0x00;
                packet[3] = (char)(P >> 8);
                packet[4] = (char)((P << 8) >> 8);
                sendPacket();
                Serial.print("Converted P Gain : "); 
                Serial.println(65536 * packet[DATA1] + 256 * packet[DATA2] + packet[DATA3]);
                Serial.println();
        }
        
        if(send && tuneI) 
        {
                Serial.println("Sending I gain wirelessly!!");
                packet[0] |= B11111111;
                packet[1] = 0x02;        // Message code
                packet[2] = 0x00;
                packet[3] = (uint8_t)(I >> 8);
                packet[4] = (uint8_t)((I << 8) >> 8);
                sendPacket();
                Serial.print("Converted I Gain : "); 
                Serial.println(65536 * packet[DATA1] + 256 * packet[DATA2] + packet[DATA3]);
                Serial.println();
        }
        
        if(send && tuneD) 
        {
                Serial.println("Sending D gain wirelessly!!");
                packet[0] |= B11111111;
                packet[1] = 0x03;        // Message code
                packet[2] = 0x00;
                packet[3] = (uint8_t)(D >> 8);
                packet[4] = (uint8_t)((D << 8) >> 8);
                sendPacket();
                Serial.print("Converted D Gain : "); 
                Serial.println(65536 * packet[DATA1] + 256 * packet[DATA2] + packet[DATA3]);
                Serial.println();
        }

}
