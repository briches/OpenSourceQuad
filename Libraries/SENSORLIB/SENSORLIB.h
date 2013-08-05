/**
    Sensor Library
    Author: Brandon Yue and Brandon Riches

    Replacement for the Adafruit LSM303 library. Uses I2C.
**/

#ifndef SENSORLIB_H_INCLUDED
#define SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <I2c.h>

#define GRAV_STANDARD 	(9.80665)

/*=========================================================================
     Network I2c Addresses
    -----------------------------------------------------------------------*/
#define ACCEL_ADDR  	(0x19)
#define MAG_ADDR_W   	(0x3C >> 1)
#define MAG_ADDR_R    	(0x3C >> 1)

#define AX_SCALE  		(0.15696)
#define AY_SCALE  		(0.15449)
#define AZ_SCALE  		(0.14115)

#define MXY_SCALE  		(1055)
#define MZ_SCALE    	(950)

/*=========================================================================
    Accelerometer Register Addresses
    -----------------------------------------------------------------------*/
											// DEFAULT    TYPE
#define ACCEL_CTRL_REG1_A           (0x20)  // 00000111   rw
#define ACCEL_CTRL_REG2_A          	(0x21)    // 00000000   rw
#define ACCEL_CTRL_REG3_A          	(0x22)    // 00000000   rw
#define ACCEL_CTRL_REG4_A          	(0x23)    // 00000000   rw
#define ACCEL_CTRL_REG5_A          	(0x24)   // 00000000   rw
#define ACCEL_CTRL_REG6_A          	(0x25)    // 00000000   rw
#define ACCEL_REFERENCE_A          	(0x26)    // 00000000   r
#define ACCEL_STATUS_REG_A         	(0x27)    // 00000000   r
#define ACCEL_OUT_X_L_A            	(0x28)
#define ACCEL_OUT_X_H_A            	(0x29)
#define ACCEL_OUT_Y_L_A            	(0x2A)
#define ACCEL_OUT_Y_H_A            	(0x2B)
#define ACCEL_OUT_Z_L_A            	(0x2C)
#define ACCEL_OUT_Z_H_A            	(0x2D)
#define ACCEL_FIFO_CTRL_REG_A     	(0x2E)
#define ACCEL_FIFO_SRC_REG_A        (0x2F)
#define ACCEL_INT1_CFG_A           	(0x30)
#define ACCEL_INT1_SOURCE_A         (0x31)
#define ACCEL_INT1_THS_A           	(0x32)
#define ACCEL_INT1_DURATION_A    	(0x33)
#define ACCEL_INT2_CFG_A           	(0x34)
#define ACCEL_INT2_SOURCE_A        	(0x35)
#define ACCEL_INT2_THS_A           	(0x36)
#define ACCEL_INT2_DURATION_A    	(0x37)
#define ACCEL_CLICK_CFG_A          	(0x38)
#define ACCEL_CLICK_SRC_A          	(0x39)
#define ACCEL_CLICK_THS_A          	(0x3A)
#define ACCEL_TIME_LIMIT_A         	(0x3B)
#define ACCEL_TIME_LATENCY_A       	(0x3C)
#define ACCEL_TIME_WINDOW_A    		(0x3D)

/*=========================================================================
    Accelerometer Register Settings
    -----------------------------------------------------------------------*/
// Sets the ODR configuration and the low-pass cut-off frequencies
#define ACCEL_ODR_50_37				(0x00)
#define ACCEL_ODR_100_74			(0x01)
#define ACCEL_ODR_400_292			(0x02)
#define ACCEL_ODR_1000_780			(0x03)

// Sets the maximum sensing range and sensitivity
#define ACCEL_FULL_SCALE_2g			(0x00)
#define ACCEL_FULL_SCALE_4g			(0x01)
#define ACCEL_FULL_SCALE_8g			(0x02)

/*=========================================================================
    Magnetometer Register Addresses
    -----------------------------------------------------------------------*/
#define MAG_CRA_REG_M              	(0x00)
#define MAG_CRB_REG_M              	(0x01)
#define MAG_MR_REG_M               	(0x02)
#define MAG_OUT_X_H_M              	(0x03)
#define MAG_OUT_X_L_M              	(0x04)
#define MAG_OUT_Z_H_M              	(0x05)
#define MAG_OUT_Z_L_M              	(0x06)
#define MAG_OUT_Y_H_M              	(0x07)
#define MAG_OUT_Y_L_M              	(0x08)
#define MAG_SR_REG_Mg              	(0x09)
#define MAG_IRA_REG_M              	(0x0A)
#define MAG_IRB_REG_M              	(0x0B)
#define MAG_IRC_REG_M              	(0x0C)
#define MAG_TEMP_OUT_H_M           	(0x31)
#define MAG_TEMP_OUT_L_M           	(0x32)

/*=========================================================================
    Magnetometer Register Settings
    -----------------------------------------------------------------------*/
// Sets the rate at which data is written to the three output registers
#define MAG_ODR_0_75				(0x00)
#define MAG_ODR_1_50				(0x01)
#define MAG_ODR_3_00				(0x02)
#define MAG_ODR_7_50				(0x03)
#define MAG_ODR_15_0				(0x04)
#define MAG_ODR_30_0				(0x05)
#define MAG_ODR_75_0				(0x06)

// Sets the gain for the device
#define MAG_GAIN_1_3				(0x01)
#define MAG_GAIN_1_9				(0x02)
#define MAG_GAIN_2_5				(0x03)
#define MAG_GAIN_4_0				(0x04)
#define MAG_GAIN_4_7				(0x05)
#define MAG_GAIN_5_6				(0x06)
#define MAG_GAIN_8_1				(0x07)

// LSB/Gauss conversion factor
#define MAG_GAIN_LSB_xy_1_3			(1055)
#define MAG_GAIN_LSB_z_1_3			(950)

#define MAG_GAIN_LSB_xy_1_9			(795)
#define MAG_GAIN_LSB_z_1_9			(710)

#define MAG_GAIN_LSB_xy_2_5			(635)
#define MAG_GAIN_LSB_z_2_5			(570)

#define MAG_GAIN_LSB_xy_4_0			(430)
#define MAG_GAIN_LSB_z_4_0			(385)

#define MAG_GAIN_LSB_xy_4_7			(375)
#define MAG_GAIN_LSB_z_4_7			(335)

#define MAG_GAIN_LSB_xy_5_6			(320)
#define MAG_GAIN_LSB_z_5_6			(285)

#define MAG_GAIN_LSB_xy_8_1			(230)
#define MAG_GAIN_LSB_z_8_1			(205)




class SENSORLIB
{
public:

	// Constructor
    SENSORLIB();

    // Initializes the sensors
    void Easy_Start(byte ACCEL_ODR, byte ACCEL_FS, byte MAG_ODR, byte MAG_GAIN);

    // Fetches the newest data from the sensors
    void update();

    // Closes the I2C bus (not necessary)
    void Cleanup();

	// Returns the double stored in ax_data
	double ax();

	// Returns the double stored in ay_data
	double ay();

	// Returns the double stored in az_data
	double az();

	double ax_data;
	double ay_data;
	double az_data;

	double mx();
	double my();
	double mz();

	double mx_data;
	double my_data;
	double mz_data;
};
#endif
