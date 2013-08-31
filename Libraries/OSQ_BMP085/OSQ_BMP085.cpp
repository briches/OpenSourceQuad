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


#include "OSQ_BMP085.h"

static uint8_t _oversample_setting_  	= 0x00;
static double  SEA_LEVEL_PRESSURE		= 101325.0;
static bool _EEPROM_read_				= false;

BMP085 :: BMP085(void)
{
	conversionStep = 0;
};

void BMP085 :: setSLP(double inchesMercury)
{
	SEA_LEVEL_PRESSURE = inchesMercury * 3386.389;
};

void BMP085 :: setOSS(uint8_t OSS)
{
	_oversample_setting_ = OSS;
}


void BMP085 :: updatePTA(void)
{
	if (!_EEPROM_read_)
	{
		readEEPROM();
	}
	// Call at maximum 39 Hz
	if(conversionStep == 2)
	{
		// Read unconverted pressure
		UP = read16(0xF6);
		UP <<= 8;
		UP |= read8(0xF8);
		UP >>= (8-_oversample_setting_);

		// Calculate true pressure and temperature
		// Do temperature calcs
		X1 = (UT-(int32_t)(AC6))*((int32_t)(AC5))/pow(2,15);
		X2 = ((int32_t)MC * pow(2,11)) / (X1+(int32_t)MD);
		_B5 = X1 + X2;
		temperature = ((int32_t)(_B5) + 8) / pow(2,4);
		temperature /= 10;

		// Do pressure calcs
		_B6 = _B5 - 4000;
		X1 = ((int32_t) _B2 * ( (_B6 * _B6) >>12 )) >> 11;
		X2 = ((int32_t)AC2 * _B6) >> 11;
		X3 = X1 + X2;
		_B3 = ((((int32_t)AC1 * 4 + X3) << _oversample_setting_) + 2) / 4;

		X1 = ((int32_t)AC3 * _B6) >> 13;
		X2 = ((int32_t)_B1 * ((_B6 * _B6) >> 12)) >> 16;
		X3 = ((X1 + X2) + 2) >> 2;
		_B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
		_B7 = ((uint32_t)UP - _B3) * (uint32_t)( 50000UL >> _oversample_setting_ );

		if (_B7 < 0x80000000)
		{
			p = (_B7 * 2) / _B4;
		}
		else
		{
			p = (_B7 / _B4) * 2;
		}

		X1 = (p >> 8) * (p >> 8);
		X1 = (X1 * 3038) >> 16;
		X2 = (-7357 * p) >> 16;

		p = p + ((X1 + X2 + (int32_t)3791) >> 4);

		// Calculate altitude from pressure
		altitude = 44330 * (1.00 - pow(p/SEA_LEVEL_PRESSURE, 1.0/5.255));

		// Set zero to start conversion on this call
		conversionStep = 0;
	}

	if(conversionStep == 1)
	{
		// Read unconverted temp
		UT = read16(0xF6);

		// Request pressure conversion with OSS
		write8(0xF4, 0x34 + (_oversample_setting_ << 6));

		conversionStep++;
	}
	if(conversionStep == 0)
	{
		// Request temperature conversion
		write8(0xF4, 0x2E);
		conversionStep++;
	}


};

void BMP085 :: readEEPROM()
{
	Wire.begin();

	if (read8(0xD0) != 0x55)
	{
		Serial.println("Read did not work");
	}

	AC1 = read16(0xAA);
	AC2 = read16(0xAC);
	AC3 = read16(0xAE);
	AC4 = read16(0xB0);
	AC5 = read16(0xB2);
	AC6 = read16(0xB4);
	_B1 = read16(0xB6);
	_B2 = read16(0xB8);
	MB = read16(0xBA);
	MC = read16(0xBC);
	MD = read16(0xBE);

	if(	(AC1 == 0xFFFF) || (AC2 == 0xFFFF) || (AC3 == 0xFFFF) || (AC4 == 0xFFFF) || (AC5 == 0xFFFF) || (AC6 == 0xFFFF) || (_B1 == 0xFFFF) || (_B2 == 0xFFFF) || (MB == 0xFFFF) || (MC == 0xFFFF) || (MD == 0xFFFF) || (AC1 == 0x0) || (AC2 == 0x0) || (AC3 == 0x0) || (AC4 == 0x0) || (AC5 == 0x0) || (AC6 == 0x0) || (_B1 == 0x0) || (_B2 == 0x0) || (MB == 0x0) || (MC == 0x0) || (MD == 0x0))
	{
		Serial.println("Unable to read EEPROM!! Check your connections!");
		Serial.println("	- Is SDA connected to A4 and SCL to A5?");
		Serial.println("	- Are +5V and GND connected?");
	}
	else
	{
		_EEPROM_read_ = true;
	}

};

void BMP085 :: printEEPROM()
{
	if (!_EEPROM_read_)
	{
		readEEPROM();
	}
	Serial.print("AC1: ");
	Serial.println(AC1);

	Serial.print("AC2: ");
	Serial.println(AC2);

	Serial.print("AC3: ");
	Serial.println(AC3);

	Serial.print("AC4: ");
	Serial.println(AC4);

	Serial.print("AC5: ");
	Serial.println(AC5);

	Serial.print("AC6: ");
	Serial.println(AC6);

	Serial.print("B1: ");
	Serial.println(_B1);

	Serial.print("B2: ");
	Serial.println(_B2);

	Serial.print("MB: ");
	Serial.println(MB);

	Serial.print("MC: ");
	Serial.println(MC);

	Serial.print("MD: ");
	Serial.println(MD);
};

uint16_t BMP085 :: read16(byte addr)
{
	uint16_t value;

	Wire.beginTransmission(BMP085_ADDR);
	#if (ARDUINO >= 100)
		Wire.write(addr);
	#else
		Wire.send(addr);
	#endif
	Wire.endTransmission();

	Wire.beginTransmission(BMP085_ADDR);
	Wire.requestFrom(BMP085_ADDR, 2);
	#if (ARDUINO >= 100)
		value = Wire.read(); // receive DATA
		value <<= 8;
		value |= Wire.read(); // receive DATA
	#else
		value = Wire.receive(); // receive DATA
		value <<= 8;
		value |= Wire.receive(); // receive DATA
	#endif
	Wire.endTransmission(); // end transmission

	return value;
};


void BMP085 :: write8(byte reg, byte value)
{
	Wire.beginTransmission(BMP085_ADDR);
	#if ARDUINO >= 100
		Wire.write((uint8_t)reg);
		Wire.write((uint8_t)value);
	#else
		Wire.send(reg);
		Wire.send(value);
	#endif
	Wire.endTransmission();
};

uint8_t BMP085 :: read8(byte reg)
{
	uint8_t value;

	Wire.beginTransmission(BMP085_ADDR);
	#if ARDUINO >= 100
		Wire.write((uint8_t)reg);
	#else
		Wire.send(reg);
	#endif
		Wire.endTransmission();
		Wire.requestFrom((byte)BMP085_ADDR, (byte)1);
	#if ARDUINO >= 100
		value = Wire.read();
	#else
		value = Wire.receive();
	#endif
	Wire.endTransmission();

	return value;
};
