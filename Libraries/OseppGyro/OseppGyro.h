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

#define WHATTHEFUCK		(0x00)

const byte WHO_AM_I = 0x0;
const byte PRODUCT_ID = 0x1;

const byte X_OFFS_USRH = 0xC;// (R/W) User offset of H byte of X gyro (2's complement)
const byte X_OFFS_USRL = 0xD; // (R/W) User offset of L byte of X gyro (2's complement)

const byte Y_OFFS_USRH = 0xE; // (R/W) User offset of H byte of Y gyro (2's complement)
const byte Y_OFFS_USRL = 0xF; // (R/W) User offset of L byte of Y gyro (2's complement)

const byte Z_OFFS_USRH = 0x10; // (R/W) User offset of H byte of Z gyro (2's complement)
const byte Z_OFFS_USRL = 0x11; // (R/W) User offset of L byte of Z gyro (2's complement)

const byte FIFO_EN = 0x12; // (R/W)
const byte AUX_VDDIO = 0x13; //(R/W)
const byte AUX_SLV_ADDR = 0x14; // (R/W)

const byte SMPLRT_DIV = 0x15; // (R/W) Sample rate divider, divides analog sample rate
const byte DLPF_FS_SYNC = 0x16; // {R/W) EXT_SYNC_SET     FS_SEL      DLPF_CFG || Default is 0
                                // EXT_SYNC_SET - bit 7, bit 6, bit 5
                                // FS_SEL - Allows setting the full-scale range of the gyro sensors (bit 4, bit 3)
                                // 0 = 250d/s, 1= 500d/s, 2 = 1000d/s, 3 = 2000d/s
                                // DLPF_CFG (b2,1,0) - Sets the digital low pass filter config. Also determines the internal analog sampling rate

                                    //DLPF_CFG Low Pass Filter Bandwidth Analog Sample Rate
                                    //0         256Hz                            8kHz
                                    //1         188Hz                            1kHz
                                    //2         98Hz                             1kHz
                                    //3         42Hz                             1kHz
                                    //4         20Hz                             1kHz
                                    //5         10Hz                             1kHz
                                    //6         5Hz                              1kHz
const byte INT_CFG = 0x17; // (R/W) Configures interrupt operation
const byte AUX_ADDR = 0x18; // (R/W)
const byte INT_STATUS = 0x1A; // (R) Interrupt status

const byte TEMP_OUT_H = 0x1B; // (R)
const byte TEMP_OUT_L = 0x1C; // (R)
const byte GYRO_XOUT_H = 0x1D; // (R) 16 bit x gyro data (2's complement)
const byte GYRO_XOUT_L = 0x1E;
const byte GYRO_YOUT_H = 0x1F; // (R) 16 bit y gyro data (2's complement)
const byte GYRO_YOUT_L = 0x20;
const byte GYRO_ZOUT_H = 0x21; // (R) 16 bit z gyro data (2's complement)
const byte GYRO_ZOUT_L = 0x22;
const byte AUX_XOUT_H = 0x23;
const byte AUX_XOUT_L = 0x24;
const byte AUX_YOUT_H = 0x25;
const byte AUX_YOUT_L = 0x26;
const byte AUX_ZOUT_H = 0x27;
const byte AUX_ZOUT_L = 0x28;

const byte FIFO_COUNTH = 0x3A;
const byte FIFO_COUNTL = 0x3B;
const byte FIFO_R = 0x3C;

const byte USER_CTRL = 0x3D; // (R/W) Enable/Disable various modes on the sensor
const byte PWR_MGM = 0x3E; // (R/W) manage the power control, select clock source ( can be internal oscillator, external oscillator, x,y,z gyro oscillator)
                            // Parameters:
                                //H_RESET (b7) Reset device and internal registers to the power-up-default settings
                                //SLEEP (b6) Enable low power sleep mode
                                //STBY_XG (b5) Put gyro X in standby mode (1=standby, 0=normal)
                                //STBY_YG (b4) Put gyro Y in standby mode (1=standby, 0=normal)
                                //STBY_ZG (b3) Put gyro Z in standby mode (1=standby, 0=normal)
                                //CLK_SEL (b2,1,0) Select device clock source
                                    //CLK_SEL Clock Source
                                    //0 Internal oscillator  - default value
                                    //1 PLL with X Gyro reference   - recommended to use one of the PLLs!!!!
                                    //2 PLL with Y Gyro reference
                                    //3 PLL with Z Gyro reference
                                    //4 PLL with external 32.768kHz reference
                                    //5 PLL with external 19.2MHz reference
                                    //6 Reserved
                                    //7 Stop clock and synchronous reset clock state

const byte FULL_SCALE_RANGE_250 = 0x0;
const byte FULL_SCALE_RANGE_500 = 0x1;
const byte FULL_SCALE_RANGE_1000 = 0x2;
const byte FULL_SCALE_RANGE_2000 = 0x3;

const float SI_CONVERT_250 = 0.0076511;
const float SI_CONVERT_500 = 0.01524; // WIP
const float SI_CONVERT_1000 = 0.015259;
const float SI_CONVERT_2000 = 0.030518;


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
