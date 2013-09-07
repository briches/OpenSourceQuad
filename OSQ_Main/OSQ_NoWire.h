/*======================================================================
 	NoWire library
 	OpenSourceQuad
 	-------------------------------------------------------------------*/
/*================================================================================
 
 	Author		: Brandon Riches
 	Date		: August 2013
 	License		: GNU Public License
 
 	This library is designed to allow easy communication between two
 	microcontrollers through wireless modemXBs. It should abstract away the serial
 	communication part of the interation, and leave just the message send/recieve
 	functionality easily accessible by the user.
 
 	Copyright (C) 2013  Brandon Riches
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 	-----------------------------------------------------------------------------*/

#ifndef OSQ_NOWIRE_H_INCLUDED
#define OSQ_NOWIRE_H_INCLUDED

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SoftwareSerial.h>

/*================================================================================
 	modemXB settings
 	-----------------------------------------------------------------------------*/
#define BAUD		(19200)			// Baud rate of modemXBs
#define RX_PIN		7			// Change this to the pin RX is connected to
#define TX_PIN		8			// Change this to the pin TX is connected to
#define	PAN_ID		(1234)			// Network ID for communication

/*================================================================================
 	Communication settings
 	-----------------------------------------------------------------------------*/
#define MSG_SIZE	(5)			// Number of bytes in each message
                                                // | Start_Char | Message ID | Data * 3 |
#define START_CHAR	(0xFF)			// Signifies start of message
#define timeoutMicros   (15000)                 // Number of microseconds to wait for more data once a message is started


/*================================================================================
 	Message content
 	-----------------------------------------------------------------------------*/
// Create your messages here.
enum messages  // Customize these.
{

        off = 0,	//0x00
        on = 1,	        //0x01
        LED0 = 2,	//0x02
        LED1 = 3,
        LED2 = 4,
        LED3 = 5,
        LED4 = 6,
        LED5 = 7,
        err = -1, // Must have this one
        count = 8  // Set this to the number of messages you used, NOT including "err"
};

enum {  FIRSTBYTE, 
        M_ID, 
        DATA1, 
        DATA2, 
        DATA3};

/*================================================================================
 	Internal Message Data Type
 	-----------------------------------------------------------------------------*/
static SoftwareSerial modemXB(7, 8);
class NoWire
{
public:
        NoWire();
        int ScanForMessages();
        bool	start();

        uint8_t newMessage[MSG_SIZE];
        long timestamp;

private:
        bool	msgCurrentLoc[MSG_SIZE];
        bool 	startMessage(void);
        bool	msgInProgress(bool loc[MSG_SIZE]);
        int	findNextByte(bool loc[MSG_SIZE]);
        void 	deleteMessage(void);   
        void    timeout(void);        
};

NoWire :: NoWire() {
};

int NoWire :: ScanForMessages()
{
        timeout(); // Check for timeout.
        
        if(modemXB.available() > 0)
        {
                timestamp = micros();

                // Message in progress, receive next bytes
                if(msgInProgress(msgCurrentLoc))	// MSG in progress, find spot and fill it.
                {
                        int nextByte = findNextByte(msgCurrentLoc);
                        Serial.print("location bool array: ");
                        for(int i = 0; i < MSG_SIZE; i++)
                        {
                                Serial.print(" ");
                                Serial.print(msgCurrentLoc[i]);
                        }
                        Serial.print(" ");
                        Serial.print("Next location: ");
                        Serial.println(nextByte);

                        newMessage[nextByte] = modemXB.read();
                        msgCurrentLoc[nextByte] = true;

                        // Oops, dropped some bytes somehow
                        if(newMessage[FIRSTBYTE] != START_CHAR)
                        {
                                deleteMessage();
                                return err;
                        }
                        if(newMessage[nextByte] == START_CHAR)
                        {
                                newMessage[nextByte]--;
                        }

                        if ( (nextByte+1) == MSG_SIZE) // No more bytes to receive
                        {
                                Serial.print("Message : ");
                                Serial.print(newMessage[FIRSTBYTE],HEX);
                                Serial.print(" ");
                                Serial.print(newMessage[M_ID], HEX);
                                Serial.print(" ");
                                Serial.print(newMessage[DATA1], HEX);
                                Serial.print(" ");
                                Serial.print(newMessage[DATA2], HEX);
                                Serial.print(" ");
                                Serial.print(newMessage[DATA3], HEX);
                                Serial.println(" ");
                                
                                for(int i = 0; i<MSG_SIZE; i++)
                                {
                                        msgCurrentLoc[i] = false;
                                }
                                
                                Serial.print("Returned value: ");
                                Serial.println(newMessage[M_ID], HEX);
                                Serial.print("time: "); 
                                Serial.println(micros()-timestamp);
                                
                                return  newMessage[M_ID];	// Full message read.
                        }
                        return err;
                }
                // Message not in progress, check if we recieved a start char yet
                if(!msgInProgress(msgCurrentLoc))
                {
                        msgCurrentLoc[FIRSTBYTE] = startMessage();
                        return err;	// Message may or may not now be in progress, but either way, it's not finished.
                }
        } 
        return err; // Full message not yet recieved;
};

void  NoWire :: timeout(void)
{
        if((micros() - timestamp) > timeoutMicros)
        {
                deleteMessage();
                Serial.println("Timeout occured");
        }
}

void NoWire :: deleteMessage()
{
        for(int i = 0; i<MSG_SIZE; i++)
        {
                newMessage[i] = 0;
                msgCurrentLoc[i] = false;
        }
};

int NoWire :: findNextByte(bool loc[MSG_SIZE])
{
        int result = 0;
        for(int i = 0; i<MSG_SIZE; i++)
        {
                if(loc[i] == 1) result++;
        }
        return result;
};

bool NoWire :: msgInProgress(bool loc[MSG_SIZE])
{
        int result = 0;
        for(int i = 0; i< MSG_SIZE; i++)
        {
                result += loc[i];
        }
        return (result != 0);		// Returns true if result isnt zero, so a message is in progress.
};

bool NoWire :: startMessage()
{
        uint8_t x = modemXB.read();

        if(x == START_CHAR) 
        {
                deleteMessage();
                newMessage[FIRSTBYTE] = x;
                msgCurrentLoc[FIRSTBYTE] = true;
        }
        return msgCurrentLoc[FIRSTBYTE];
};

bool NoWire :: start()
{
        modemXB.begin(BAUD);
        return 1;
};

#endif // OSQ_NOWIRE_H_INCLUDED

