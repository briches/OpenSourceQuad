/********************************
Library for use with the Osepp Gyro we have

source file



Brandon
*********************************/


#include "OseppGyro.h"
OseppGyro* OseppGyro::pOseppGyro = 0;

OseppGyro::OseppGyro()
{
	pOseppGyro = this;
	dataMode_ = false;
	ISRFlag = false;
	I2CAddr = 0x69; //The i2C address of the OseppGyro. Corresponds to the DPT switch on the device
	dScaleRange_ = 0;  //250d/s range

}

/***********************************************************
 *
 * setI2CAddr
 *
 *
 *
 ***********************************************************/
void OseppGyro::setI2CAddr(int address)
{
	I2CAddr = address; I2CAddr;
}


/***********************************************************
 *
 * update
 *
 *
 *
 ***********************************************************/
void OseppGyro::update()
{
	if(dataMode_)
	{
		xyz(x_,y_,z_);
	}

}

/*************************************************************
*
* xyz
*
* Get gyro readings (x, y, z)
* 16 bit data is used
*
* This function also convers 2's complement number to
* signed integer result.
*
*
*
*
*************************************************************/
void OseppGyro::xyz(int& x, int& y, int& z)
{

    byte buf[6];


    I2c.read(I2CAddr, GYRO_XOUT_H, 6, buf);
    x = buf[0] << 8 | buf[1];
    y = buf[2] << 8 | buf[3];
    z = buf[4] << 8 | buf[5];

    x = ~(x-1);
    y = ~(y-1);
    z = ~(z-1);

}


/***********************************************************
 *
 * dataMode
 *
 *
 *
 ***********************************************************/
void OseppGyro::dataMode(int dScaleRange,int dDLPF)
{
    dScaleRange_ = dScaleRange;
    byte x;

    if(dDLPF <= 6){dDLPF_ = dDLPF;}  // 0 to 6 (sets bandwidth of DLPF and sample rate)
    else if (dDLPF >6){dDLPF_ = 6;}    // 0 to 6 (sets bandwidth of DLPF and sample rate)
	dataMode_ = true;

	byte statusCheck = 0x1;

	byte FS_DLPF; // Combined byte to go into the DLPF_FS_SYNC reg

	//setup i2c
	I2c.begin();
	I2c.timeOut(1000);
	I2c.scan();

	if( dScaleRange_ <= 375){ dScaleRange_ = FULL_SCALE_RANGE_250; } //0-375 = 250d/s
	else if( dScaleRange_ <= 750){ dScaleRange_ = FULL_SCALE_RANGE_500; } //375 - 750 = 500d/s
	else if( dScaleRange_ <= 1500){ dScaleRange_ = FULL_SCALE_RANGE_1000; }// 750-1500 = 1000d/s
	else if( dScaleRange_ > 1500) { dScaleRange_ = FULL_SCALE_RANGE_2000; } //boundary


    FS_DLPF = byte(dScaleRange_);
    FS_DLPF = (FS_DLPF << 3) | byte(dDLPF_);


	I2c.read(I2CAddr, WHO_AM_I, 1, &x);

	Serial.println("debug 1");
	I2c.write(I2CAddr,DLPF_FS_SYNC, FS_DLPF);

	Serial.println("debug 2");
    //active Mode
    I2c.write(I2CAddr, PWR_MGM, statusCheck); // Sets standby off, activates all gyros, sets clock to x-gyro PLL

	Serial.println("debug 3");


}


/***********************************************************
 *
 * regRead
 *
 *
 *
 ***********************************************************/
void OseppGyro::regRead(byte reg, byte *buf, byte count)
{

   I2c.read(I2CAddr, reg, count, buf);
}

/***********************************************************
 *
 * regWrite
 *
 *
 *
 ***********************************************************/
void OseppGyro::regWrite(byte reg, byte val)
{

  I2c.write(I2CAddr, reg, val);
}


/***********************************************************
 *
 * gyroISR
 *
 *
 *
 ***********************************************************/
void gyroISR(void){
	OseppGyro::pOseppGyro->ISRFlag = true;
}




















