/**
    Sensor Library
    Author: Brandon Yue and Brandon Riches

    Replacement for the Adafruit LSM303 library. Uses Wire.
**/

#ifndef SENSORLIB_H_INCLUDED
#define SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Print.h>
#else
#include "WProgram.h"
#endif

#include <QuadGlobalDefined.h>
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
