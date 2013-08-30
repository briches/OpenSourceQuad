//******************************************************************************
    /*//*//*   FreeQuad   *//*//*
    Sensor Library
    Author: Brandon Yue and Brandon Riches

    Replacement for the Adafruit LSM303 library. Uses Wire.
    Uses the same format as the Adafruit_LSM303 library. I assume the
    code is used under the license provided by Adafruit Industries.

    /***************************************************************************
	  This is a library for the LSM303 Accelerometer and magnentometer/compass

	  Designed specifically to work with the Adafruit LSM303DLHC Breakout

	  These displays use I2C to communicate, 2 pins are required to interface.

	  Adafruit invests time and resources providing this open source code,
	  please support Adafruit andopen-source hardware by purchasing products
	  from Adafruit!

	  Written by Kevin Townsend for Adafruit Industries.
	  BSD license, all text above must be included in any redistribution
	 *************************************************************************//*
/******************************************************************************/


#ifndef SENSORLIB_H_INCLUDED
#define SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Print.h>
#else
#include "WProgram.h"
#endif

#include <FQ_QuadGlobalDefined.h>
#include <Wire.h>


/* Unified sensor driver for the accelerometer */
class SENSORLIB_accel
{
	public:

		SENSORLIB_accel(int32_t sensorID = -1);
		bool begin(void);
		void getEvent(sensors_event_t*);

	private:

		int32_t	_sensorID;

		lsm303AccelData	_accelData;		// Last read accelerometer data here

		void write8(byte address, byte reg, byte value);
		byte read8(byte address, byte reg);
		void read(void);
};

/* Unified sensor driver for the magnetometer */
class SENSORLIB_mag
{
	public:


		SENSORLIB_mag(int32_t sensorID = -1);
		bool begin(void);
		void setMagGain(lsm303MagGain gain);
		void getEvent(sensors_event_t*);

	private:

		int32_t	_sensorID;

		lsm303MagGain   _magGain;
		lsm303MagData   _magData;     // Last read magnetometer data will be available here

		void write8(byte address, byte reg, byte value);
		byte read8(byte address, byte reg);
		void read(void);
};
#endif
