/**=====================================================================
	NoWire library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library is designed to allow easy communication between two
	microcontrollers through wireless modems. It should abstract away the serial
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

/*================================================================================
	Modem settings
	-----------------------------------------------------------------------------*/
#define BAUD		(115200)		// Baud rate of modems
#define RX_PIN		(41)			// Connect the RX pin of the modem to this pin
#define TX_PIN		(43)			// Connect the TX pin of the modem to this pin
#define	PAN_ID		(3300)			// Network ID for communication
#define MSG_SIZE	(4)				// Number of characters in each message
#define START_CHAR	('!')			// Signifies start of message
#define END_CHAR	('\n')			// Signifies end of message

/*================================================================================
	Message content and priority
	-----------------------------------------------------------------------------*/
// Create your messages here.
// Content should be TITLE_c, and priority should be TITLE_p
#define STOP_c		("STOP")	// Define your content like this
#define STOP_p		(1)		// Define your priorities like this

/*================================================================================
	Message Code List
	-----------------------------------------------------------------------------*/
// Define your possible messages here to have priorities automatically assigned
typedef struct {

} noWire_Codes;


/*================================================================================
	Internal Message Data Type
	-----------------------------------------------------------------------------*/


class NoWire
{


};

#endif // OSQ_NOWIRE_H_INCLUDED
