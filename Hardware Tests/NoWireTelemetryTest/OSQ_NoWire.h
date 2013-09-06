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
#define RX_PIN		7				// Change this to the pin RX is connected to
#define TX_PIN		8				// Change this to the pin TX is connected to
#define	PAN_ID		(1234)			// Network ID for communication

/*================================================================================
	Communication settings
	-----------------------------------------------------------------------------*/
#define MSG_SIZE	(5)				// Number of bytes in each message
									// | Start_Char | Message ID | Data * 3 |
#define START_CHAR	(0xFF)			// Signifies start of message


/*================================================================================
	Message content
	-----------------------------------------------------------------------------*/
// Create your messages here.
enum messages  // Customize these.
{

	off,		//0x00
	on,			//0x01
	LED0,		//0x02
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
	err = -1, // Must have this one
	count = 8  // Set this to the number of messages you used, NOT including "err"
};

enum {START, M_ID, DATA1, DATA2, DATA3};

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
		uint8_t userMessage[MSG_SIZE];



	private:



		bool	msgCurrentLoc[MSG_SIZE];

		bool 	startMessage(void);
		bool	msgInProgress(bool loc[MSG_SIZE]);
		int		findNextByte(bool loc[MSG_SIZE]);
		void 	deleteMessage(void);
		void	archiveMessage(void);
                messages parseMsgType();
};

NoWire :: NoWire() {};

int NoWire :: ScanForMessages()
{
	if(modemXB.available() > 0)
	{

		// Message in progress, receive next bytes
		if(msgInProgress(msgCurrentLoc))	// MSG in progress, find spot and fill it.
		{
			int nextByte = findNextByte(msgCurrentLoc);

			if (nextByte < MSG_SIZE)	// We still have bytes to receive
			{
				newMessage[nextByte] = modemXB.read();
				Serial.println(newMessage[nextByte], HEX);
				msgCurrentLoc[nextByte] = true;

				if(newMessage[nextByte] == START_CHAR) // Oops, dropped some bytes somehow
				{
					deleteMessage();
				}
			}
			else if (nextByte = MSG_SIZE) // No more bytes to receive
			{
				archiveMessage();
				return  userMessage[M_ID];	// Full message read.
			}
			return err;
					// Goddamnit I love this line.
		}

		// Message not in progress, check if we recieved a start char yet
		if(!msgInProgress(msgCurrentLoc))
		{
			msgCurrentLoc[START] = startMessage();
			return err;	// Message may or may not now be in progress, but either way, it's not finished.
		}




	} else return err; // Full message not yet recieved;
};

messages NoWire :: parseMsgType()
{
  messages type;
    for(int i = 0; i < count; i++)
  return type;
};

void NoWire :: archiveMessage()
{
	userMessage[START] = newMessage[START];
        userMessage[M_ID]  = newMessage[M_ID];
        userMessage[DATA1] = newMessage[DATA1];
        userMessage[DATA2] = newMessage[DATA2];
        userMessage[DATA3] = newMessage[DATA3];

	for(int i = 0; i<MSG_SIZE; i++)
	{
		newMessage[i] = 0;
		msgCurrentLoc[i] = false;
	}

}

void NoWire :: deleteMessage()
{
	for(int i = 0; i<MSG_SIZE; i++)
	{
		newMessage[i] = 0;
		msgCurrentLoc[i] = false;
	}
}

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
	newMessage[START] = modemXB.read();
	if(newMessage[START] == START_CHAR) msgCurrentLoc[START] = true;
	return msgCurrentLoc[START];
};

bool NoWire :: start()
{
	modemXB.begin(BAUD);
	return 1;
};

#endif // OSQ_NOWIRE_H_INCLUDED
