/*=====================================================================
	OSQ_IMU
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library manages the I2C interface with certain sensors, allowing them to
	easily communicate with the micro-controller.

	Copyright (C) 2013  Brandon Riches

	NOTE: Most of the methods and code in this library was originally designed by
	Kevin Townsend for Adafruit Industries. The code was released under the BSD license

	The following is the original license information:

	 This is a library for the LSM303 Accelerometer and magnetometer/compass

	  Designed specifically to work with the Adafruit LSM303DLHC Breakout

	  Adafruit invests time and resources providing this open source code,
	  please support Adafruit and open-source hardware by purchasing products
	  from Adafruit!

	  Written by Kevin Townsend for Adafruit Industries.
	  BSD license, all text above must be included in any redistribution

	-----------------------------------------------------------------------------*/


#ifndef OSQ_SENSORLIB_H_INCLUDED
#define OSQ_SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Print.h>
#else
#include "WProgram.h"
#endif


#include <Wire.h>
#include <limits.h>


/*=========================================================================
    I2C or TWI addresses
    -----------------------------------------------------------------------*/

#define GYRO_ADDR   (0xD2 >> 1)
#define ACCEL_ADDR  (0x31 >> 1)
#define MAG_ADDR  (0x3C >> 1)

/*=========================================================================
    CHIP ID
    -----------------------------------------------------------------------*/
#define LSM303_ID   (0b11010100)

/***************************************************************************
/////////////////////////////////////////////////////////////////////////////
// L3G4200D
/////////////////////////////////////////////////////////////////////////////
****************************************************************************/

/*=========================================================================
    L3G4200D Gyro Registers
    -----------------------------------------------------------------------*/
#define WHO_AM_I   	 (0x0)
#define CTRL_REG1        (0x20)
#define CTRL_REG2        (0x21)
#define CTRL_REG3        (0x22)
#define CTRL_REG4        (0x23)
#define CTRL_REG5        (0x24)
#define REFERENCE        (0x25)
#define OUT_TEMP         (0x26)
#define STATUS_REG       (0x27)
#define OUT_X_L          (0x28)
#define OUT_X_L          (0x29)
#define OUT_Y_L          (0x2A)
#define OUT_Y_L          (0x2B)
#define OUT_Z_L          (0x2C)
#define OUT_Z_L          (0x2D)
#define FIFO_CTRL_REG    (0x2E)
#define FIFO_SRC_REG     (0x2F)
#define INT1_CFG         (0x30)
#define INT1_SRC         (0x31)

/*=========================================================================
     L3G4200D Gyro Range settings and conversions
    -----------------------------------------------------------------------*/
#define FULL_SCALE_RANGE_250  	(0x0)
#define FULL_SCALE_RANGE_500  	(0x1)
#define FULL_SCALE_RANGE_1000  	(0x2)
#define FULL_SCALE_RANGE_2000  	(0x3)

#define SI_CONVERT_250  (0.0113)
#define SI_CONVERT_500  (0.01524) // WIP
#define SI_CONVERT_1000 (0.015259)
#define SI_CONVERT_2000 (0.030518)

/***************************************************************************
/////////////////////////////////////////////////////////////////////////////
// LSM303
/////////////////////////////////////////////////////////////////////////////
****************************************************************************/

/*=========================================================================
	Conversion factors
    -----------------------------------------------------------------------*/

#define AX_SCALE  	(0.12939)
#define AY_SCALE  	(0.15449)
#define AZ_SCALE  	(0.9036550534)

#define MXY_SCALE  	(1055)
#define MZ_SCALE    	(950)

#define SENSORS_GRAVITY_STANDARD 	(9.81369388)
#define SENSORS_GAUSS_TO_MICROTESLA       (100)

/*=========================================================================
    Accelerometer Register Addresses
    -----------------------------------------------------------------------*/
												// DEFAULT    TYPE
#define ACCEL_CTRL_REG1_A          (0x20)  	// 00000111   rw
#define ACCEL_CTRL_REG2_A          (0x21)  	// 00000000   rw
#define ACCEL_CTRL_REG3_A          (0x22)   	// 00000000   rw
#define ACCEL_CTRL_REG4_A          (0x23)   	// 00000000   rw
#define ACCEL_CTRL_REG5_A          (0x24)   	// 00000000   rw
#define ACCEL_REFERENCE_A          (0x26)   	// 00000000   r
#define ACCEL_STATUS_REG_A         (0x27)   	// 00000000   r
#define ACCEL_OUT_X_L_A            (0x28)
#define ACCEL_OUT_X_H_A            (0x29)
#define ACCEL_OUT_Y_L_A            (0x2A)
#define ACCEL_OUT_Y_H_A            (0x2B)
#define ACCEL_OUT_Z_L_A            (0x2C)
#define ACCEL_OUT_Z_H_A            (0x2D)
#define ACCEL_INT1_CFG_A           (0x30)
#define ACCEL_INT1_SOURCE_A        (0x31)
#define ACCEL_INT1_THS_A           (0x32)
#define ACCEL_INT1_DURATION_A      (0x33)
#define ACCEL_INT2_CFG_A           (0x34)
#define ACCEL_INT2_SOURCE_A        (0x35)
#define ACCEL_INT2_THS_A           (0x36)
#define ACCEL_INT2_DURATION_A      (0x37)

/*=========================================================================
    Accelerometer Register Settings
    -----------------------------------------------------------------------*/
// Sets the ODR configuration and the low-pass cut-off frequencies
#define ACCEL_ODR_50_37		(0x00)
#define ACCEL_ODR_100_74	(0x01)
#define ACCEL_ODR_400_292	(0x02)
#define ACCEL_ODR_1000_780	(0x03)

// Sets the maximum sensing range and sensitivity
#define ACCEL_FULL_SCALE_2g	(0x00)
#define ACCEL_FULL_SCALE_4g	(0x01)
#define ACCEL_FULL_SCALE_8g	(0x02)

/*=========================================================================
    Magnetometer Register Addresses
    -----------------------------------------------------------------------*/
#define MAG_CRA_REG_M              (0x00)
#define MAG_CRB_REG_M              (0x01)
#define MAG_MR_REG_M               (0x02)
#define MAG_OUT_X_H_M              (0x03)
#define MAG_OUT_X_L_M              (0x04)
#define MAG_OUT_Z_H_M              (0x05)
#define MAG_OUT_Z_L_M              (0x06)
#define MAG_OUT_Y_H_M              (0x07)
#define MAG_OUT_Y_L_M              (0x08)
#define MAG_SR_REG_Mg              (0x09)
#define MAG_IRA_REG_M              (0x0A)
#define MAG_IRB_REG_M              (0x0B)
#define MAG_IRC_REG_M              (0x0C)

/*=========================================================================
    Magnetometer Register Settings
    -----------------------------------------------------------------------*/
// Sets the rate at which data is written to the three output registers
#define MAG_ODR_0_75	(0x00)
#define MAG_ODR_1_50	(0x01)
#define MAG_ODR_3_00	(0x02)
#define MAG_ODR_7_50	(0x03)
#define MAG_ODR_15_0	(0x04)
#define MAG_ODR_30_0	(0x05)
#define MAG_ODR_75_0	(0x06)

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
typedef enum
{
  LSM303_MAGGAIN_1_3     = 0x20,  // +/- 1.3
  LSM303_MAGGAIN_1_9     = 0x40,  // +/- 1.9
  LSM303_MAGGAIN_2_5     = 0x60,  // +/- 2.5
  LSM303_MAGGAIN_4_0     = 0x80,  // +/- 4.0
  LSM303_MAGGAIN_4_7     = 0xA0,  // +/- 4.7
  LSM303_MAGGAIN_5_6     = 0xC0,  // +/- 5.6
  LSM303_MAGGAIN_8_1     = 0xE0   // +/- 8.1
} lsm303MagGain;
/*=========================================================================
    INTERNAL VECTOR DATA TYPE
    -----------------------------------------------------------------------*/
/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        /* Orientation sensors */
        struct {
            float azimuth;    /**< Angle between the magnetic north direction and the Y axis, around the Z axis (0<=azimuth<360).  0=North, 90=East, 180=South, 270=West */
            float pitch;      /**< Rotation around X axis (-180<=pitch<=180), with positive values when the z-axis moves toward the y-axis. */
            float roll;       /**< Rotation around Y axis (-90<=roll<=90), with positive values when the x-axis moves towards the z-axis. */
        };
    };
    int8_t status;
    uint8_t reserved[3];
} sensors_vec_t;

/*=========================================================================
    INTERNAL SENSOR EVENT DATA TYPE
    -----------------------------------------------------------------------*/
/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
    int32_t version;                          /**< must be sizeof(struct sensors_event_t) */
    int32_t sensor_id;                        /**< unique sensor identifier */
    int32_t type;                             /**< sensor type */
    int32_t reserved0;                        /**< reserved */
    int32_t timestamp;                        /**< time is in milliseconds */
    union
    {
        float          data[4];
        sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
        sensors_vec_t   orientation;          /**< orientation values are in degrees */
        sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
        float          temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float          distance;             /**< distance in centimeters */
        float          light;                /**< light in SI lux units */
        float          pressure;             /**< pressure in hectopascal (hPa) */
        float          relative_humidity;    /**< relative humidity in percent */
        float          current;              /**< current in milliamps (mA) */
        float          voltage;              /**< voltage in volts (V) */
    };
} sensors_event_t;

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct lsm303MagData_s
{
	float x;
	float y;
	float z;
  float orientation;
} lsm303MagData;

/*=========================================================================
    INTERNAL ACCELERATION DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct lsm303AccelData_s
{
  float x;
  float y;
  float z;
} lsm303AccelData;

/*=========================================================================
    INTERNAL GYRO DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct L3G4200D_data_s
{
  float x;
  float y;
  float z;
} L3G4200D_data;


/*===============================================
	Device settings
	-----------------------------------------------------------------------*/
// Gyro Settings
const unsigned char dataRate = 0 << 6;
const unsigned char bandwidth = 0 << 4;
const unsigned char FS_Sel = 0;


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

		byte write8(byte address, byte reg, byte value);
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

class SENSORLIB_gyro
{
	public:

		SENSORLIB_gyro(int32_t sensorID = -1);
		bool begin(void);
		void getEvent(sensors_event_t*);

	private:

		int32_t _sensorID;

		L3G4200D_data	_gyroData;	// Last read gyro data will be available here

		byte write8(byte address, byte reg, byte value);
		byte read8(byte address, byte reg);
		void read(void);
};

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _GYRO_CONVERT_  = SI_CONVERT_250;
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
byte SENSORLIB_gyro::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  byte error = Wire.endTransmission();
  return error;
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
    Wire.write(OUT_X_L | (1<<7));
  #else
    Wire.send(OUT_X_L);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)GYRO_ADDR, (byte)6);

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
  _gyroData.x = ~( (xlo | (xhi << 8)) - 1);
  _gyroData.y = ~( (ylo | (yhi << 8)) - 1);
  _gyroData.z = ~( (zlo | (zhi << 8)) - 1);

};

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setup the HW
*/
/**************************************************************************/
bool SENSORLIB_gyro::begin()
{
	// Enable I2C
	Wire.begin();
        unsigned char ctrl1 = dataRate | bandwidth | B1111;
        unsigned char ctrl2 = B0000; // HP Filter
        unsigned char ctrl4 = 0xC0 | (FS_Sel << 4) | B010;
        unsigned char ctrl5 = B00000; // HP Filt en
        
        write8(GYRO_ADDR, CTRL_REG1, ctrl1);
        write8(GYRO_ADDR, CTRL_REG2, ctrl2);
        write8(GYRO_ADDR, CTRL_REG4, ctrl4);
        write8(GYRO_ADDR, CTRL_REG5, ctrl5);

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
byte SENSORLIB_accel::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  return Wire.endTransmission();
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
  write8(ACCEL_ADDR, ACCEL_CTRL_REG1_A, 0x37);
  
  // Enable the internal filter ** May not work **
  write8(ACCEL_ADDR, ACCEL_CTRL_REG2_A, 0x0);
  
  // Block data update, FS Selection
  write8(ACCEL_ADDR, ACCEL_CTRL_REG4_A, 0x80);

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

// HERE
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

SENSORLIB_accel   	accel;
SENSORLIB_mag	        mag;
SENSORLIB_gyro          gyro;

#endif


