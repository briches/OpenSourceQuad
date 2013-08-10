/********************************
Library for use with the Osepp Gyro we have





Brandon
*********************************/


#ifndef OSEPPGYRO_H_INCLUDED
#define OSEPPGYRO_H_INCLUDED



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <I2c.h>
#include <QuadGlobalDefined.h>


extern "C" void gyroISR(void) __attribute__ ((signal));

class OseppGyro {


public:
  friend void gyroISR(void); //make friend so bttnPressISR can access private var keyhit

  OseppGyro();

/***********************************************************
 *
 * setI2CAddr
 *
 * set the i2c address of the MMA8453 to a new value, such as 0x1D
 *
 ***********************************************************/
    void setI2CAddr(int address);

/***********************************************************
 *
 * dataMode
 *
 * set the device to return raw data values
 *
 * note that all this device can do is this anyways, so we'll
 * use this function to change the FULL_SCALE_RANGE as well as
 * configure the internal digital low pass filter
 *
 ***********************************************************/
    void dataMode(int dScaleRange,int dDLPF);

/***********************************************************
 *
 * x
 *
 * returns the x axis value
 *
 ***********************************************************/
    int x(){ return x_; }

/***********************************************************
 *
 * y
 *
 * returns the y axis value
 *
 ***********************************************************/
    int y() { return y_; }

/***********************************************************
 *
 * z
 *
 * returns the z axis value
 *
 ***********************************************************/
    int z(){ return z_; }


/***********************************************************
 *
 * update
 *
 * update data values, or clear interrupts. Use at start of loop()
 *
 ***********************************************************/
    void update();


/***********************************************************
 *
 * regRead
 *
 *
 ***********************************************************/
    void regRead(byte reg, byte *buf, byte count = 1);

/***********************************************************
 *
 * regWrite
 *
 *
 ***********************************************************/
    void regWrite(byte reg, byte val);



private:

	void xyz(int& x,int& y, int& z);

	int x_,y_,z_;

	byte I2CAddr;

	byte dDLPF_;
	int dScaleRange_;
	boolean dataMode_;

	volatile boolean ISRFlag;
	static OseppGyro* pOseppGyro; //ptr to OseppGyro class for the ISR

};

#endif // OSEPPGYRO_H_INCLUDED
