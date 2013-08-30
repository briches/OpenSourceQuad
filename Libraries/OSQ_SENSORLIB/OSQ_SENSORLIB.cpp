/*=====================================================================
	OSQ_QuadGlobalDefined
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library manages the I2C interface with certain sensors, allowing them to
	easily communicate with the microcontroller.

	Copyright (C) 2013  Brandon Riches

	NOTE: Most of the methods and code in this library was originally designed by
	Kevin Townsend for Adafruit Industries. The code was released under the BSD license

	The following is the original license information:

	 This is a library for the LSM303 Accelerometer and magnentometer/compass

	  Designed specifically to work with the Adafruit LSM303DLHC Breakout

	  These displays use I2C to communicate, 2 pins are required to interface.

	  Adafruit invests time and resources providing this open source code,
	  please support Adafruit andopen-source hardware by purchasing products
	  from Adafruit!

	  Written by Kevin Townsend for Adafruit Industries.
	  BSD license, all text above must be included in any redistribution

	-----------------------------------------------------------------------------*/



#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include <OSQ_SENSORLIB.h>
#include <OSQ_QuadGlobalDefined.h>

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _GYRO_CONVERT_			 = SI_CONVERT_250;
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

/***************************************************************************
 GYRO
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new  class
*/
/**************************************************************************/
SENSORLIB_gyro::SENSORLIB_gyro(int32_t sensorID) {
  _sensorID = sensorID;
}


/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void SENSORLIB_gyro::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
};

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte SENSORLIB_gyro::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif
  Wire.endTransmission();

  return value;
};

/**************************************************************************/
/*!
    @brief  Reads the raw data from the gyro sensor
*/
/**************************************************************************/
void SENSORLIB_gyro::read()
{
  // Read the accelerometer
  Wire.beginTransmission((byte)GYRO_ADDR);
  #if ARDUINO >= 100
    Wire.write(GYRO_XOUT_H | 0x80);
  #else
    Wire.send(GYRO_XOUT_H | 0x80);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)GYRO_ADDR, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  #if ARDUINO >= 100
    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();
  #else
    uint8_t xhi = Wire.receive();
    uint8_t xlo = Wire.receive();
    uint8_t yhi = Wire.receive();
    uint8_t ylo = Wire.receive();
    uint8_t zhi = Wire.receive();
    uint8_t zlo = Wire.receive();
  #endif

  // Shift values to create properly formed integer (low byte first)
  _gyroData.x = ~( (xlo | (xhi << 8)) - 1);
  _gyroData.y = ~( (ylo | (yhi << 8)) - 1);
  _gyroData.z = ~( (zlo | (zhi << 8)) - 1);

};

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool SENSORLIB_gyro::begin()
{
	// Enable I2C
	Wire.begin();

	byte dDLPF_;
	byte statusCheck = 0x01;

	if(DLPF <= 6)
	{
		dDLPF_ = DLPF;
	}  // 0 to 6 (sets bandwidth of DLPF and sample rate)
	else if (DLPF >6)
	{
		dDLPF_ = 6;
	}    // 0 to 6 (sets bandwidth of DLPF and sample rate)


	byte FS_DLPF = byte(d_ScaleRange);
	FS_DLPF = (FS_DLPF << 3) | byte(dDLPF_);
	Serial.println(FS_DLPF, BIN);

	// Enable the gyro for aux use
	Serial.println("debug 1");
	write8(GYRO_ADDR, DLPF_FS_SYNC, FS_DLPF);

	Serial.println("debug 2");
	write8(GYRO_ADDR, PWR_MGM, statusCheck);

	Serial.println("debug 3");
	write8(GYRO_ADDR, USER_CTRL, B00100000);

	return true;
};

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
void SENSORLIB_gyro::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = 3;
  event->type      = 1;
  event->timestamp = 0;
  event->gyro.x = _gyroData.x * _GYRO_CONVERT_;
  event->gyro.y = _gyroData.y * _GYRO_CONVERT_;
  event->gyro.z = _gyroData.z * _GYRO_CONVERT_;
};

/***************************************************************************
 ACCELEROMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
SENSORLIB_accel::SENSORLIB_accel(int32_t sensorID) {
  _sensorID = sensorID;
}


/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void SENSORLIB_accel::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
};

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte SENSORLIB_accel::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif
  Wire.endTransmission();

  return value;
};

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void SENSORLIB_accel::read()
{
  // Read the accelerometer
  Wire.beginTransmission((byte)ACCEL_ADDR);
  #if ARDUINO >= 100
    Wire.write(ACCEL_OUT_X_L_A | 0x80);
  #else
    Wire.send(ACCEL_OUT_X_L_A | 0x80);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)ACCEL_ADDR, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  #if ARDUINO >= 100
    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();
  #else
    uint8_t xlo = Wire.receive();
    uint8_t xhi = Wire.receive();
    uint8_t ylo = Wire.receive();
    uint8_t yhi = Wire.receive();
    uint8_t zlo = Wire.receive();
    uint8_t zhi = Wire.receive();
  #endif

  // Shift values to create properly formed integer (low byte first)
  _accelData.x = (xlo | (xhi << 8)) >> 4;
  _accelData.y = (ylo | (yhi << 8)) >> 4;
  _accelData.z = (zlo | (zhi << 8)) >> 4;

};

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool SENSORLIB_accel::begin()
{
  // Enable I2C
  Wire.begin();

  // Enable the accelerometer
  write8(ACCEL_ADDR, ACCEL_CTRL_REG1_A, 0x27);

  return true;
};

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
void SENSORLIB_accel::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = 2;
  event->type      = 1;
  event->timestamp = 0;
  event->acceleration.x = _accelData.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = _accelData.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = _accelData.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * AZ_SCALE;
};

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
SENSORLIB_mag::SENSORLIB_mag(int32_t sensorID) {
  _sensorID = sensorID;
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void SENSORLIB_mag::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
};

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte SENSORLIB_mag::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif
  Wire.endTransmission();

  return value;
};

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void SENSORLIB_mag::read()
{
  // Read the magnetometer
  Wire.beginTransmission((byte)MAG_ADDR);
  #if ARDUINO >= 100
    Wire.write(MAG_OUT_X_H_M);
  #else
    Wire.send(MAG_OUT_X_H_M);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)MAG_ADDR, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Note high before low (different than accel)
  #if ARDUINO >= 100
    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();
  #else
    uint8_t xhi = Wire.receive();
    uint8_t xlo = Wire.receive();
    uint8_t zhi = Wire.receive();
    uint8_t zlo = Wire.receive();
    uint8_t yhi = Wire.receive();
    uint8_t ylo = Wire.receive();
  #endif

  // Shift values to create properly formed integer (low byte first)
  _magData.x = (xlo | (xhi << 8));
  _magData.y = (ylo | (yhi << 8));
  _magData.z = (zlo | (zhi << 8));

  // TODO: Calculate orientation
  _magData.orientation = 0.0;
};


/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool SENSORLIB_mag::begin()
{
  // Enable I2C
  Wire.begin();

  // Enable the magnetometer
  write8(MAG_ADDR, MAG_MR_REG_M, 0x00);

  // Set the gain to a known level
  setMagGain(LSM303_MAGGAIN_1_3);

  return true;
};

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void SENSORLIB_mag::setMagGain(lsm303MagGain gain)
{
  write8(MAG_ADDR, MAG_CRB_REG_M, gain);

  _magGain = gain;

  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      _lsm303Mag_Gauss_LSB_XY = 1100;
      _lsm303Mag_Gauss_LSB_Z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      _lsm303Mag_Gauss_LSB_XY = 855;
      _lsm303Mag_Gauss_LSB_Z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      _lsm303Mag_Gauss_LSB_XY = 670;
      _lsm303Mag_Gauss_LSB_Z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      _lsm303Mag_Gauss_LSB_XY = 450;
      _lsm303Mag_Gauss_LSB_Z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      _lsm303Mag_Gauss_LSB_XY = 400;
      _lsm303Mag_Gauss_LSB_Z  = 255;
      break;
    case LSM303_MAGGAIN_5_6:
      _lsm303Mag_Gauss_LSB_XY = 330;
      _lsm303Mag_Gauss_LSB_Z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      _lsm303Mag_Gauss_LSB_XY = 230;
      _lsm303Mag_Gauss_LSB_Z  = 205;
      break;
  }
};

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
void SENSORLIB_mag::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = 1;
  event->type      = 2;
  event->timestamp = micros();
  event->magnetic.x = _magData.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = _magData.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = _magData.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
};
