/*=====================================================================
	OSQ_BMP085 library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library interfaces with the BMP085 pressure sensor to return temperature
	calibrated altitude measurements.

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
#ifndef OSQ_BMP085_H_INCLUDED
#define OSQ_BMP085_H_INCLUDED

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <math.h>
#include <Wire.h>

#define BMP085_ADDR	(0x77)

class BMP085
{
	public:

		BMP085();

		void updatePTA(void); // Main function

		void readEEPROM();
		void setSLP(double inchesMercury);
		void setOSS(uint8_t OSS);
		void printEEPROM();

		float temperature;
		float altitude;
		int conversionStep;

		// EEPROM calibration values
		int16_t 		AC1;
		int16_t			AC2;
		int16_t			AC3;
		uint16_t 		AC4;
		uint16_t 		AC5;
		uint16_t		AC6;
		int16_t			_B1;
		int16_t			_B2;
		int16_t			MB;
		int16_t			MC;
		int16_t			MD;

		// Calculated calibration values
		int32_t X1, X2, X3;
		int32_t _B3, _B5, _B6;
		uint32_t _B4, _B7;
		int32_t p;

		int32_t			UT;
		int32_t			UP;


	private:


		void write8(byte reg, byte value);
		uint8_t read8(byte reg);

		uint16_t read16(byte addr);
};

#endif // OSQ_BMP085_H_INCLUDED
