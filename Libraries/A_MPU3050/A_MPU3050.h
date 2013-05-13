/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef A_MPU3050_H_INCLUDED
#define A_MPU3050_H_INCLUDED

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define MPU3050_ADDRESS          (0x68)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {
        MPU3050_REGISTER_WHO_AM_I = 0x0;
        MPU3050_REGISTER_PRODUCT_ID = 0x1;
        MPU3050_REGISTER_X_OFFS_USRH = 0xC; // (R/W) User offset of H byte of X gyro (2's complement)
        MPU3050_REGISTER_X_OFFS_USRL = 0xD; // (R/W) User offset of L byte of X gyro (2's complement)
        MPU3050_REGISTER_Y_OFFS_USRH = 0xE;
        MPU3050_REGISTER_Y_OFFS_USRL = 0xF;
        MPU3050_REGISTER_Z_OFFS_USRH = 0x10;
        MPU3050_REGISTER_Z_OFFS_USRL = 0x11;
        MPU3050_REGISTER_FIFO_EN = 0x12;
        MPU3050_REGISTER_AUX_VDDIO = 0x13;
        MPU3050_REGISTER_AUX_SLV_ADDR = 0x14;
        MPU3050_REGISTER_SMPLRT_DIV = 0x15;
        MPU3050_REGISTER_DLPF_FS_SYNC = 0x16;
        MPU3050_REGISTER_INT_CFG = 0x17;
        MPU3050_REGISTER_AUX_ADDR = 0x18;
        MPU3050_REGISTER_INT_STATUS = 0x1A;
        MPU3050_REGISTER_TEMP_OUT_H = 0x1B; // (R)
        MPU3050_REGISTER_TEMP_OUT_L = 0x1C; // (R)
        MPU3050_REGISTER_GYRO_XOUT_H = 0x1D;
        MPU3050_REGISTER_GYRO_XOUT_L = 0x1E;
        MPU3050_REGISTER_GYRO_YOUT_H = 0x1F;
        MPU3050_REGISTER_GYRO_YOUT_L = 0x20;
        MPU3050_REGISTER_GYRO_ZOUT_H = 0x21;
        MPU3050_REGISTER_GYRO_ZOUT_L = 0x22;
        MPU3050_REGISTER_AUX_XOUT_H = 0x23;
        MPU3050_REGISTER_AUX_XOUT_L = 0x24;
        MPU3050_REGISTER_AUX_YOUT_H = 0x25;
        MPU3050_REGISTER_AUX_YOUT_L = 0x26;
        MPU3050_REGISTER_AUX_ZOUT_H = 0x27;
        MPU3050_REGISTER_AUX_ZOUT_L = 0x28;
        MPU3050_REGISTER_FIFO_COUNTH = 0x3A;
        MPU3050_REGISTER_FIFO_COUNTL = 0x3B;
        MPU3050_REGISTER_FIFO_R = 0x3C;
        MPU3050_REGISTER_USER_CTRL = 0x3D; // (R/W) Enable/Disable various modes on the sensor
        MPU3050_REGISTER_PWR_MGM = 0x3E;
    } mpu3050_Registers_t;



#endif // A_MPU3050_H_INCLUDED
