/**
    Sensor Library
    Author: Brandon Yue

    Replacement for the Adafruit LSM303 library. Uses I2C.
**/

#include "SENSORLIB.h"
#include <I2C.h>
#include <cmath>

const uint8_t ACCEL_ADDR = 0x19;
const uint8_t MAG_ADDR   = 0x1E;

const double X_SCALE = 0.15696;
const double Y_SCALE = 0.15449;
const double Z_SCALE = 0.14115;


namespace CONTROL_BYTES
{
    const uint8_t   	CONTROL_REG_1   = 0x20,
								CONTROL_REG_2   = 0x21,
								CONTROL_REG_3   = 0x22,
								CONTROL_REG_4   = 0x23,
								CONTROL_REG_5   = 0x24,

								CRA_REG_M   = 0x00,
								CRB_REG_M   = 0x01,

								ACCEL_OUT_X1_REG= 0x28;

    const uint8_t   CONTROL1        = B01111,
								CONTROL2        = 0x00,
								CONTROL3        = 0x00,
								CONTROL4        = 0x00,
								CONTROL5        = 0x00,

								MAGCTRLA        = 0x00,
								MAGCTRLB        = 0x01;
};

I2C MyI2C;

using namespace CONTROL_BYTES;

SENSORLIB::SENSORLIB(){}

void SENSORLIB::Easy_Start()
{
        MyI2C.begin();

        ///ACCEL OPTIONS
        //CONTROL REG 1:    DATA RATES + SENSOR ENABLES
        //[n/a] [n/a] [n/a] [DATA RATE 1] [DATA RATE 2] [X] [Y] [Z]
        //DEFAULT 0x03
        MyI2C.write(ACCEL_ADDR, CONTROL_REG_1, CONTROL1);

        //CONTROL REG 2:    HIGH PASS FILTER
        //
        //DEFAULT 0x00
        MyI2C.write(ACCEL_ADDR, CONTROL_REG_2, CONTROL2);

        //CONTROL REG 3:    INTERRUPTS
        //
        //DEFAULT 0x00
        MyI2C.write(ACCEL_ADDR, CONTROL_REG_3, CONTROL3);

        //CONTROL REG 4:    BLOCK UPDATE PARAMS, SCALE SELECTION, SELF TEST
        //
        //DEFAULT 0x00
        MyI2C.write(ACCEL_ADDR, CONTROL_REG_4, CONTROL4);

        //CONTROL REG 5:    SLEEP-TO-WAKE
        //
        //DEFAULT 0x00
        MyI2C.write(ACCEL_ADDR, CONTROL_REG_5, CONTROL5);

        ///MAG OPTIONS
        //MAG CONTROL A
        //
        //DEFAULT 0x00
        MyI2C.write(MAG_ADDR, CRA_REG_M, MAGCTRLA);

        //MAG CONTROL B
        //
        //DEFAULT 0x01
        MyI2C.write(MAG_ADDR, CRB_REG_M, MAGCTRLB);
};





/**
    Read_Accel
    Reads the accelerometer data from the LSM303 registers. Converts the bytes
    to floating point numbers automaticlly.
    Takes 1 Argument:
        - DataArray[]: Array of AT LEAST 3 ELEMENTS which stores the result floats.
*/
void SENSORLIB::Read_Accel()
{
    uint8_t ByteBuffer [6] = {};

    for (int i = 0; i < 6; i++)
    {
        I2c.read(0x19, 0x28 + i, 1, ByteBuffer + i);
    }

    signed char x = (ByteBuffer[1]),
                y = (ByteBuffer[3]),
                z = (ByteBuffer[5]);

    x_data = (x * X_SCALE);
    y_data = (y * Y_SCALE);
    z_data = (z * Z_SCALE);
};

double SENSORLIB::x()
{
	return x_data;
};

double SENSORLIB::y()
{
	return y_data;
};

double SENSORLIB::z()
{
	return z_data;
};

void SENSORLIB::Cleanup()
{
    MyI2C.end();
};
