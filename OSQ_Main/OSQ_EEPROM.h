/*=====================================================================
 	OSQ_EEPROM
 	OpenSourceQuad
 	-------------------------------------------------------------------*/
/*================================================================================

 	Author		: Brandon Riches and Andrew Coulthard
 	Date		: August 2013
 	License		: GNU Public License

 	This library is designed to abstract away some read/write/check operations on
 	the on-board EEPROM

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

#ifndef OSQ_EEPROM_H_INCLUDED
#define OSQ_EEPROM_H_INCLUDED

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define SOFTWARE_VERSION_MINOR (85)
#define SOFTWARE_VERSION_MAJOR (0)


// Each block will be 256 bytes
#define MAX_ADDRESS (4096)

/* CONFIG BLOCK */
#define software_version_addr (0x000) 	// 0x000 L, 0x001 H
#define flight_number_addr (0x002)	// 0x002 L, 0x003 H

/* DATA BLOCK 1*/
#define previous_flight_times_addr (0x100)	// Stores 10 previous flight times, type double, in seconds. Most recent at lower address.
#define	flight_start_location_addr (0x10A)	// Stores 30 double values for lat/lon/alt, most recent at lower address.


/* DATA BLOCK 2*/
#define _0_04_Hz_waypoints_addr (0x200)	// Stores 320 seconds worth of waypoints (lat/long), with 16 separate waypoints.

/* DATA BLOCK 3*/
#define pid_coefficient_data_addr (0x300) // Stores saved pid coefficients (2 bytes each) in the following order: xAlt, yAlt, zAlt, xAng, yAng, zAng.



unsigned char EEPROM_read8(unsigned int uiAddress)
{
	while(EECR && (1<<EEPE));	// Wait for completion of previous write

	EEARH = (uiAddress>>8); // Set up address register
        EEARL = uiAddress;

	EECR |= (1<<EERE);

	return EEDR;
};

void EEPROM_write8(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR && (1<<EEPE));	// Wait for completion of previous write

	EEARH = (uiAddress>>8);	// Set up eeprom write
        EEARL = uiAddress;

	EEDR = ucData;		// Set up data register

	EECR |= (1<<EEMPE);

	EECR |= (1<<EEPE);	// Start
};

void writeConfigBlock()
{
	unsigned char flightNumberH = EEPROM_read8(flight_number_addr + 1);
        unsigned char flightNumberL = EEPROM_read8(flight_number_addr);

	EEPROM_write8(software_version_addr, SOFTWARE_VERSION_MINOR);
	EEPROM_write8(software_version_addr+1, SOFTWARE_VERSION_MAJOR);

        if(flightNumberL == 0xFF)
        {
                flightNumberH += 1;
                flightNumberL = 0;
        }

	EEPROM_write8(flight_number_addr + 1,  flightNumberH);
	EEPROM_write8(flight_number_addr, flightNumberL+1);
};

void EEPROMwritePIDCoefficients(int selection, unsigned int x1, unsigned int x2)
{
        selection *= 2;
        EEPROM_write8(pid_coefficient_data_addr + selection, x1);
        EEPROM_write8(pid_coefficient_data_addr + selection + 1, x2);
};

double EEPROMreadPIDCoefficients(int selection)
{
        return( 256 * EEPROM_read8(pid_coefficient_data_addr + 2 * selection ) + EEPROM_read8 (pid_coefficient_data_addr + 1 + 2 * selection ) );
};



#endif // OSQ_EEPROM_H_INCLUDED
