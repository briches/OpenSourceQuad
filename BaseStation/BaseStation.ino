#include <SoftwareSerial.h>
#include "OSQ_NoWire.h"
#include "BaseStation.h"

SoftwareSerial XBee(12, 13);
int filterP[10];
int filterI[10];
int filterD[10];
int P, I, D;
uint8_t packet[5] = {0,0,0,0,0};

boolean selectionMade = false;


int readPGain()
{
        static int filterLocationP = 0;
        P = 0;
        
        for(int i = 0; i < 10; i++)
        {
                filterP[filterLocationP] = analogRead(5);
                
                filterLocationP++;
                if(filterLocationP == 10) filterLocationP = 0;
        }
        
        for(int i = 0; i< 10; i++)
        {
                P += filterP[i];
        }
        
        return (P/10) >> 2;
}

int readIGain()
{
        static int filterLocationI = 0;
        I = 0;
        
        for(int i = 0; i < 10; i++)
        {
                filterI[filterLocationI] = analogRead(4);
                
                filterLocationI++;
                if(filterLocationI == 10) filterLocationI = 0;
        }
        
        for(int i = 0; i< 10; i++)
        {
                I += filterI[i];
        }
        
        return (I/10) >> 2;
}

int readDGain()
{
        static int filterLocationD = 0;
        D = 0;
        
        for(int i = 0; i < 10; i++)
        {
                filterD[filterLocationD] = analogRead(3);
                delay(10);
                
                filterLocationD++;
                if(filterLocationD == 10) filterLocationD = 0;
        }
        
        for(int i = 0; i< 10; i++)
        {
                D += filterD[i];
        }
        
        return (D/10) >> 2;
}



void sendPacket()
{
        XBee.write(packet[0]);
        XBee.write(packet[1]);
        XBee.write(packet[2]);
        XBee.write(packet[3]);
        XBee.write(packet[4]);
}

void setup()
{
        Serial.begin(19200);
        XBee.begin(19200);
        Serial.println();
        pinMode(12, INPUT);
        pinMode(11, INPUT);
        pinMode(10, INPUT);
        pinMode(9, INPUT);
        pinMode(8, INPUT);
        pinMode(7, INPUT);
        pinMode(6, INPUT);
        pinMode(5, INPUT);
        pinMode(4, INPUT);
        pinMode(3, INPUT);
        pinMode(2, INPUT);
        
        P = readPGain() - 1;
        I = readIGain() - 1;
        D = readDGain() - 1;
}

void executeInstruction(int selection)
{
        if(selection == 1)
        {
                // Start
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x02;
                packet[DATA1] = 0x33;
                packet[DATA2] = 0x33;
                packet[DATA3] = 0x33;
                sendPacket();
                Serial.println("Sending start instruction!");
        }
        if(selection == 2)
        {
                // Disarm 
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x00;
                packet[DATA1] = 0x33;
                packet[DATA2] = 0x33;
                packet[DATA3] = 0x33;
                sendPacket();
                Serial.println("Sending disarm instruction!");
        }
        if(selection == 3)
        {
                // Autoland
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x01;
                packet[DATA1] = 0x33;
                packet[DATA2] = 0x33;
                packet[DATA3] = 0x33;
                sendPacket();
                Serial.println("Sending autoland instruction!");
        }
        if(selection == 4)
        {
                // Broadcast data
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x03;
                packet[DATA1] = 0x33;
                packet[DATA2] = 0x33;
                packet[DATA3] = 0x33;
                sendPacket();
                Serial.println("Sending broadcast data instruction!");
        }
        if(selection == 5)
        {
                // Send reset angle command
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x10;
                packet[DATA1] = 0x33;
                packet[DATA2] = 0x33;
                packet[DATA3] = 0x33;
                sendPacket();
                Serial.println("Sending broadcast data instruction!");
                
        }
        if(selection == 7)
        {
                // setAngleP
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x0C;
                packet[DATA1] = 0x00;
                packet[DATA2] = (uint8_t)(P >> 8);
                packet[DATA3] = (uint8_t)((P << 8) >> 8);
                sendPacket();
                Serial.println("Sending set P gain instruction!");
        }
        if(selection == 8)
        {
                // setAngleI
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x0D;
                packet[DATA1] = 0x00;
                packet[DATA2] = (uint8_t)(I >> 8);
                packet[DATA3] = (uint8_t)((I << 8) >> 8);
                sendPacket();
                Serial.println("Sending set I gain instruction!");
        }
        if(selection == 9)
        {
                // setAngleD
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x0E;
                packet[DATA1] = 0x00;
                packet[DATA2] = (uint8_t)(D >> 8);
                packet[DATA3] = (uint8_t)((D << 8) >> 8);
                sendPacket();
                Serial.println("Sending set D gain instruction!");
        }
}

void loop()
{
        
        static long int selectionTimer = 0;
        Serial.println("*********************** BaseStation Menu ******************");
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println("Menu: ");
        Serial.print("1 :    Start (Arm)");
        Serial.print("  2 :        Disarm");
        Serial.println("   3 :    Autoland");
        Serial.println();
        Serial.print("4 : Broadcast Data");
        Serial.print("  5 :              ");
        Serial.println("   6 :               ");
        Serial.println();
        Serial.print("7 : Set P attitude");
        Serial.print("  8 : Set I attitude");
        Serial.println("  9 : Set D attitude");
        Serial.println();
        Serial.println("* :               ");
        
        Serial.println();
        Serial.println();
        Serial.println();
        
        Serial.println("Current PID gains: ");
        printData(P, I, D);

        
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println("***********************************************************");
        Serial.println("Select an option from the menu: ");
        
        while(!selectionMade)
        {
                if(millis() - selectionTimer > 500)
                {
                        int selection = scanButtonInput();
                        if((selection > 0) && (selection != 11))
                        {
                                selectionMade = true;
                                selectionTimer = millis();
                                Serial.println("Please wait 500 ms");
                                delay(500);
                                pinMode(8, INPUT);
                                pinMode(11,INPUT);
                                
                                executeInstruction(selection); // The meat and potatoes
                                
                        }
                        
                        else if(selection == 11)
                        {
                                P = readPGain() - 1;
                                I = readIGain() - 1;
                                D = readDGain() - 1;
                                Serial.println();
                                printData(P, I, D);
                                Serial.println();
                        }
                        
                        pinMode(8, INPUT);
                        pinMode(11,INPUT);
                }
        }
        selectionMade = false;
        Serial.println("***********************************************************");
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println("***********************************************************");
}
