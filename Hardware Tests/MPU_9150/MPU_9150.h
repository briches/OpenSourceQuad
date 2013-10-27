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

#define MPU_9150_SMPLRT_DIV        (0x19)
#define MPU_9150_CONFIG            (0x1A)
#define MPU_9150_GYRO_CONFIG       (0x1B)
#define MPU_9150_ACCEL_CONFIG      (0x1C)

#define MPU_9150_INT_PIN_CFG       (0x37)

#define MPU_9150_USER_CTRL         (0x6A)
#define MPU_9150_PWR_MGMT_1        (0x6B)

#define MPU_9150_WHO_AM_I          (0x75)


/*=========================================================================
 Class and functions
 -----------------------------------------------------------------------*/
class MPU_9150_c
{
public:

        //Constructor 
        MPU_9150_c(void);
        byte scanBus(void);
        byte initMPU(void);

        float returnData(void);

private:

        byte write8(byte I2Caddress, byte reg, byte value);
        byte read8(byte I2Caddress, byte reg);
};

MPU_9150_c :: MPU_9150_c() 
{
};

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
        
        delay(100);
        
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        Serial.print("Error = ");
        Serial.println(error);

        // Move out of sleep mode and set the clock source to Gyro x PLL
        error = write8(MPU_9150_Addr, MPU_9150_PWR_MGMT_1, 0x01);
//        Serial.print("Error = ");
//        Serial.println(error);

        // Disable I2C Passthrough
        error = write8(MPU_9150_Addr, MPU_9150_INT_PIN_CFG, 0x00);
//        Serial.print("Error = ");
//        Serial.println(error);

        // Disable I2C Master (for now?)
        error = write8(MPU_9150_Addr, MPU_9150_USER_CTRL, 0x00);
//        Serial.print("Error = ");
//        Serial.println(error);

        // Gyro Config
        error = write8(MPU_9150_Addr, MPU_9150_GYRO_CONFIG, 0x00);
//        Serial.print("Error = ");
//        Serial.println(error);

        // Accel Config
        error = write8(MPU_9150_Addr, MPU_9150_ACCEL_CONFIG, 0x00);
//        Serial.print("Error = ");
//        Serial.println(error);

        // Sample rate divider
        error = write8(MPU_9150_Addr, MPU_9150_SMPLRT_DIV, 0x00);
//        Serial.print("Error = ");
//        Serial.println(error);
        

        return (byte)read8(MPU_9150_Addr, MPU_9150_WHO_AM_I);

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

