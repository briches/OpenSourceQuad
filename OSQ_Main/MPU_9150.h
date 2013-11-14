// First attempts at deciphering the 9150
// Pending sanity.

#ifndef MPU_9150_H_INCLUDED
#define MPU_9150_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Print.h>
#else
#include "WProgram.h"
#endif


#include <Wire.h>
#include <Limits.h> // MAX_INT, etc.


/*=========================================================================
 I2C 7bit address
 -----------------------------------------------------------------------*/
byte MPU_9150_Addr = 104;

/*=========================================================================
 MPU Registers
 -----------------------------------------------------------------------*/

/** Accel and gyro **/
#define MPU_9150_SMPLRT_DIV        	(0x19)
#define MPU_9150_CONFIG            	(0x1A)
#define MPU_9150_GYRO_CONFIG       	(0x1B)
#define MPU_9150_ACCEL_CONFIG      	(0x1C)

#define MPU_9150_FIFO_EN		   	(0x23)

#define MPU_9150_I2C_MST_CTRL	   	(0x24)

#define MPU_9150_I2C_SLV0_ADDR	   	(0x25)
#define MPU_9150_I2C_SLV0_REG		(0x26)
#define MPU_9150_I2C_SLV0_CTRL		(0x27)

#define MPU_9150_I2C_SLV1_ADDR		(0x28)
#define MPU_9150_I2C_SLV1_REG		(0x29)
#define MPU_9150_I2C_SLV1_CTRL		(0x2A)

#define MPU_9150_I2C_SLV2_ADDR		(0x2B)
#define MPU_9150_I2C_SLV2_REG		(0x2C)
#define MPU_9150_I2C_SLV2_CTRL		(0x2D)

#define MPU_9150_I2C_SLV3_ADDR		(0x2E)
#define MPU_9150_I2C_SLV3_REG		(0x2F)
#define MPU_9150_I2C_SLV3_CTRL		(0x30)

#define MPU_9150_I2C_SLV4_ADDR		(0x31)
#define MPU_9150_I2C_SLV4_REG		(0x32)
#define MPU_9150_I2C_SLV4_DO		(0x33)
#define MPU_9150_I2C_SLV4_CTRL		(0x34)
#define MPU_9150_I2C_SLV4_DI		(0x35)

#define MPU_9150_I2C_MST_STATUS		(0x36)
#define MPU_9150_INT_PIN_CFG       	(0x37)
#define MPU_9150_INT_ENABLE			(0x38)
#define MPU_9150_INT_STATUS			(0x3A)

#define MPU_9150_ACCEL_XOUT_H		(0x3B)
#define MPU_9150_ACCEL_XOUT_L		(0x3C)
#define MPU_9150_ACCEL_YOUT_H		(0x3D)
#define MPU_9150_ACCEL_YOUT_L		(0x3E)
#define MPU_9150_ACCEL_ZOUT_H		(0x3F)
#define MPU_9150_ACCEL_ZOUT_L		(0x40)

#define MPU_9150_TEMP_OUT_H			(0x41)
#define MPU_9150_TEMP_OUT_L			(0x42)

#define MPU_9150_GYRO_XOUT_H		(0x43)
#define MPU_9150_GYRO_XOUT_L		(0x44)
#define MPU_9150_GYRO_YOUT_H		(0x45)
#define MPU_9150_GYRO_YOUT_L		(0x46)
#define MPU_9150_GYRO_ZOUT_H		(0x47)
#define MPU_9150_GYRO_ZOUT_L		(0x48)

#define MPU_9150_EXT_SENS_DATA_00	(0x49)
#define MPU_9150_EXT_SENS_DATA_01	(0x4A)
#define MPU_9150_EXT_SENS_DATA_02	(0x4B)
#define MPU_9150_EXT_SENS_DATA_03	(0x4C)
#define MPU_9150_EXT_SENS_DATA_04	(0x4D)
#define MPU_9150_EXT_SENS_DATA_05	(0x4E)
#define MPU_9150_EXT_SENS_DATA_06	(0x4F)
#define MPU_9150_EXT_SENS_DATA_07	(0x50)
#define MPU_9150_EXT_SENS_DATA_08	(0x51)
#define MPU_9150_EXT_SENS_DATA_09	(0x52)
#define MPU_9150_EXT_SENS_DATA_10	(0x53)
#define MPU_9150_EXT_SENS_DATA_11	(0x54)
#define MPU_9150_EXT_SENS_DATA_12	(0x55)
#define MPU_9150_EXT_SENS_DATA_13	(0x56)
#define MPU_9150_EXT_SENS_DATA_14	(0x57)
#define MPU_9150_EXT_SENS_DATA_15	(0x58)
#define MPU_9150_EXT_SENS_DATA_16	(0x59)
#define MPU_9150_EXT_SENS_DATA_17	(0x5A)
#define MPU_9150_EXT_SENS_DATA_18	(0x5B)
#define MPU_9150_EXT_SENS_DATA_19	(0x5C)
#define MPU_9150_EXT_SENS_DATA_20	(0x5D)
#define MPU_9150_EXT_SENS_DATA_21	(0x5E)
#define MPU_9150_EXT_SENS_DATA_22	(0x5F)
#define MPU_9150_EXT_SENS_DATA_23	(0x60)

#define MPU_9150_I2C_SLV0_DO		(0x63)
#define MPU_9150_I2C_SLV0_D1		(0x64)
#define MPU_9150_I2C_SLV0_D2		(0x65)
#define MPU_9150_I2C_SLV0_D3		(0x66)

#define MPU_9150_I2C_MST_DELAY_CTRL	(0x67)
#define MPU_9150_SIGNAL_PATH_RESET	(0x68)

#define MPU_9150_USER_CTRL         	(0x6A)
#define MPU_9150_PWR_MGMT_1        	(0x6B)
#define MPU_9150_PWR_MGMT_2			(0x6C)

#define MPU_9150_FIFO_COUNTH		(0x72)
#define MPU_9150_FIFO_COUNTL		(0x73)
#define MPU_9150_FIFO_R_W			(0x74)

#define MPU_9150_WHO_AM_I          	(0x75)

/** Magnetometer **/
#define MPU_9150_MAG_WIA			(0x00) // Device ID
#define MPU_9150_MAG_INFO			(0x01)
#define MPU_9150_MAG_ST1			(0x02)

#define MPU_9150_MAG_HXL			(0x03)
#define MPU_9150_MAG_HXH			(0x04)
#define MPU_9150_MAG_HYL			(0x05)
#define MPU_9150_MAG_HYH			(0x06)
#define MPU_9150_MAG_HZL			(0x07)
#define MPU_9150_MAG_HZH			(0x08)

#define MPU_9150_MAG_ST2			(0x09)
#define MPU_9150_MAG_CNTL			(0x0A)
#define MPU_9150_MAG_I2CDIS			(0x0F)

#define MPU_9150_MAG_ASAX			(0x10) // Read in fuse mode only
#define MPU_9150_MAG_ASAY			(0x11) // " "
#define MPU_9150_MAG_ASAZ			(0x12) // " "


/*=========================================================================
 Class and functions
 -----------------------------------------------------------------------*/
class MPU_9150_c
{
public:

        //Constructor
        MPU_9150_c(void);

        // Scan the I2C bus for devices
        byte scanBus(void);

        // Initialize the MPU
        byte initMPU(void);


        float returnData(void);

private:

        byte write8(byte I2Caddress, byte reg, byte value);
        byte read8(byte I2Caddress, byte reg);
};

MPU_9150_c :: MPU_9150_c()
{};

byte MPU_9150_c::scanBus()
{
        byte error, address;
        int nDevices;

        Serial.println("Scanning...");

        nDevices = 0;
        for(address = 1; address < 127; address++ )
        {
                Wire.beginTransmission(address);
                error = Wire.endTransmission();

                if (error == 0)
                {
                        Serial.print("I2C device found at address decimal ");
                        if (address<16) Serial.print("0");
                        Serial.print(address);
                        Serial.print(", hex 0x");
                        Serial.print(address, HEX);
                        Serial.println("  !");

                        nDevices++;
                }
                else if (error==4)
                {
                        Serial.print("Unknow error at address 0x");
                        if (address<16)
                                Serial.print("0");
                        Serial.println(address);
                }
        }
        if (nDevices == 0)
                Serial.println("No I2C devices found\n");
        else
                Serial.println("done");
        return nDevices;
};

/*=========================================================================
 begin()        - Sets up the device to return normal sensor data
 -----------------------------------------------------------------------*/
byte MPU_9150_c::initMPU()
{
        Serial.println("Beginning setup");
        byte address = 0x68;
        byte error;
        Serial.print("The address I'm testing is: 0x");
        Serial.println(address, HEX);

        // Move out of sleep mode and set the clock source to Gyro x PLL
        error = write8(MPU_9150_Addr, MPU_9150_PWR_MGMT_1, 0x01);

        // Disable I2C Passthrough
        error = write8(MPU_9150_Addr, MPU_9150_INT_PIN_CFG, 0x00);

        // Disable I2C Master (for now?)
        error = write8(MPU_9150_Addr, MPU_9150_USER_CTRL, 0x00);

        // Gyro Config
        error = write8(MPU_9150_Addr, MPU_9150_GYRO_CONFIG, 0x00);

        // Accel Config
        error = write8(MPU_9150_Addr, MPU_9150_ACCEL_CONFIG, 0x00);

        // Sample rate divider
        error = write8(MPU_9150_Addr, MPU_9150_SMPLRT_DIV, 0x00);


        return read8(MPU_9150_Addr, MPU_9150_WHO_AM_I);

};

/*=========================================================================
 write8(uint8_t I2Caddress, uint8_t reg, uint8_t value)
 - Write 8 bits to a specified register. Internal Function
 -----------------------------------------------------------------------*/
byte MPU_9150_c :: write8(byte I2Caddress, byte reg, byte value)
{
        byte error = 0x01;
        Wire.beginTransmission(I2Caddress);
	#if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
	#else
        Wire.send(reg);
        Wire.send(value);
	#endif
        error = Wire.endTransmission();
        return error;
};

/*=========================================================================
 read8(uint8_t I2Caddress, uint8_t reg)
 - Read 8 bits from a specified register. Internal Function
 -----------------------------------------------------------------------*/
byte MPU_9150_c :: read8(byte I2Caddress, byte reg)
{
        byte value;

        Wire.beginTransmission(I2Caddress);
	#if ARDUINO >= 100
        Wire.write(reg);
	#else
        Wire.send(reg);
	#endif
        Wire.endTransmission();

        Wire.requestFrom(I2Caddress, (byte)1);
	#if ARDUINO >= 100
        value = Wire.read();
	#else
        value = Wire.receive();
	#endif
        Wire.endTransmission();

        return value;
};

#endif // MPU_9150_H_INCLUDED

