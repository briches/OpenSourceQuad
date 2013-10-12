#include <SoftwareSerial.h>
#include "OSQ_NoWire.h"
#include "BaseStation.h"

int filterP[10];
int filterI[10];
int filterD[10];
int P, I, D;
uint8_t packet[5] = {0,0,0,0,0};

boolean selectionMade = false;

double rateKP = 0;

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
        Serial3.write(packet[0]);
        Serial3.write(packet[1]);
        Serial3.write(packet[2]);
        Serial3.write(packet[3]);
        Serial3.write(packet[4]);
}

void setup()
{
        Serial.begin(115200);
        Serial3.begin(19200);
        Serial.println();
        
        pinMode(30, OUTPUT);
        pinMode(31, OUTPUT);
        pinMode(32, OUTPUT);
        pinMode(33, OUTPUT);
        pinMode(34, OUTPUT);
        pinMode(35, OUTPUT);
        pinMode(36, OUTPUT);
        pinMode(37, OUTPUT);
        pinMode(38, OUTPUT);
        pinMode(39, OUTPUT);
        pinMode(40, OUTPUT);
        pinMode(41, OUTPUT);
        
        
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
                rateKP += 0.1;
                packet[FIRSTBYTE] = 0xFF;
                packet[M_ID] = 0x0C;
                packet[DATA1] = 0x00;
                packet[DATA2] = (uint8_t)(P >> 8);
                packet[DATA3] = (uint8_t)((P << 8) >> 8);
                sendPacket();
                Serial.println("Sending set P gain instruction!");
                Serial.print("RateKP set to: ");
                Serial.println(rateKP);
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
        Serial.print("  5 : Reset angles/PID ");
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
                        if((selection > 0) && (selection != 100))
                        {
                                selectionMade = true;
                                selectionTimer = millis();
                                Serial.println("Please wait 500 ms");
                                delay(500);
                                
                                executeInstruction(selection); // The meat and potatoes
                                
                        }
                        
                        else if(selection == 100)
                        {
                                P = readPGain() - 1;
                                I = readIGain() - 1;
                                D = readDGain() - 1;
                                Serial.println();
                                printData(P, I, D);
                                Serial.println();
                        }
                        
                }
        }
        selectionMade = false;
        Serial.println("***********************************************************");
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println("***********************************************************");
}
