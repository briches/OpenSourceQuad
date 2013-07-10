/**
    Sensor Library
    Author: Brandon Yue and MEEEEEEEEEEE

    Replacement for the Adafruit LSM303 library. Uses I2C.
**/

#include "SENSORLIB.h"
#include <I2c.h>
#include <math.h>

const uint8_t ACCEL_ADDR = 0x19;
const uint8_t MAG_ADDR_W   = 0x3C;				// Use for WRITE operations
const uint8_t MAG_ADDR_R    = 0x3D;				// Use for READ operations

const double AX_SCALE = 0.15696;
const double AY_SCALE = 0.15449;
const double AZ_SCALE = 0.14115;

const double M_SCALE  = 0.635076;



namespace CONTROL_BYTES
{
	// Register address
    const uint8_t   	CONTROL_REG_1   = 0x20,
								CONTROL_REG_2   = 0x21,
								CONTROL_REG_3   = 0x22,
								CONTROL_REG_4   = 0x23,
								CONTROL_REG_5   = 0x24,

								CRA_REG_M  	    = 0x00,
								CRB_REG_M  	    = 0x01,

								MR_REG_M 			= 0x02,

								ACCEL_OUT_X1_REG= 0x28;

	// Data content
    const uint8_t    CONTROL1        = B00101111,
								CONTROL2        = B00000000,
								CONTROL3        = B00000000,
								CONTROL4        = B00000000,
								CONTROL5        = B00000000,

								MAGCTRLA        = B00011000,
								MAGCTRLB        = B00100000,

								MR_REG		= B00000000,

								OUT_XH_M		 = 0x03;				// H - L - H - L - H - L
};

using namespace CONTROL_BYTES;

SENSORLIB::SENSORLIB(){}

void SENSORLIB::Easy_Start()
{
		byte	before[1],
					after[1];

        I2c.begin();

        ///ACCEL OPTIONS
        //CONTROL REG 1:    DATA RATES + SENSOR ENABLES
        //[n/a] [n/a] [n/a] [DATA RATE 1] [DATA RATE 2] [X] [Y] [Z]
        //DEFAULT 0x03
        I2c.read(ACCEL_ADDR, CONTROL_REG_1, 1, before);
        I2c.write(ACCEL_ADDR, CONTROL_REG_1, CONTROL1);
        I2c.read(ACCEL_ADDR, CONTROL_REG_1, 1, after);

        Serial.println(" ");
        Serial.print("ACCEL CONTROL_REG_1:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        //CONTROL REG 2:    HIGH PASS FILTER
        //
        //DEFAULT 0x00
        I2c.read(ACCEL_ADDR, CONTROL_REG_2, 1, before);
        I2c.write(ACCEL_ADDR, CONTROL_REG_2, CONTROL2);
        I2c.read(ACCEL_ADDR, CONTROL_REG_2, 1, after);

        Serial.print("ACCEL CONTROL_REG_2:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        //CONTROL REG 3:    INTERRUPTS
        //
        //DEFAULT 0x00
        I2c.read(ACCEL_ADDR, CONTROL_REG_3, 1, before);
        I2c.write(ACCEL_ADDR, CONTROL_REG_3, CONTROL3);
		I2c.read(ACCEL_ADDR, CONTROL_REG_3, 1, after);

        Serial.print("ACCEL CONTROL_REG_3:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        //CONTROL REG 4:    BLOCK UPDATE PARAMS, SCALE SELECTION, SELF TEST
        //
        //DEFAULT 0x00
        I2c.read(ACCEL_ADDR, CONTROL_REG_4, 1, before);
        I2c.write(ACCEL_ADDR, CONTROL_REG_4, CONTROL4);
        I2c.read(ACCEL_ADDR, CONTROL_REG_4, 1, before);

        Serial.print("ACCEL CONTROL_REG_4:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        //CONTROL REG 5:    SLEEP-TO-WAKE
        //
        //DEFAULT 0x00
		I2c.read(ACCEL_ADDR, CONTROL_REG_5, 1, before);
        I2c.write(ACCEL_ADDR, CONTROL_REG_5, CONTROL5);
		I2c.read(ACCEL_ADDR, CONTROL_REG_5, 1, after);

		Serial.print("ACCEL CONTROL_REG_5:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        ///MAG OPTIONS
        //MAG CONTROL A
        //
        //DEFAULT 0x00
        I2c.read(MAG_ADDR_R, CRA_REG_M, 1, before);
        I2c.write(MAG_ADDR_W, CRA_REG_M, MAGCTRLA);
		I2c.read(MAG_ADDR_R, CRA_REG_M, 1, after);

		Serial.print("CRA_REG_M:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        //MAG CONTROL B
        //
        //DEFAULT 0x01
        I2c.read(MAG_ADDR_R, CRB_REG_M, 1, before);
        I2c.write(MAG_ADDR_W, CRB_REG_M, MAGCTRLB);
        I2c.read(MAG_ADDR_R, CRB_REG_M, 1, after);

		Serial.print("CRB_REG_M:		\n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

        //MR_REG
        //
        I2c.read(MAG_ADDR_R, MR_REG_M, 1, before);
        I2c.write(MAG_ADDR_W, MR_REG_M, MR_REG);
        I2c.read(MAG_ADDR_R, MR_REG_M, 1, after);

  		Serial.print("MR_REG_M:	    \n	Prior to write: ");
        Serial.print(before[0], BIN);
        Serial.print(" After write: ");
        Serial.println(after[0],BIN);

};





/**
    update()

    Reads the accelerometer data from the LSM303 registers. Converts the bytes
    to floating point numbers automatically.

    Reads the magnetometer data from the LSM303 registers. Converts the two byte
    values to milligauss floating point values.

    Stores the read values into the members "x_data, y_data, z_data"
*/
void SENSORLIB::update()
{
	/// Read Accelerometer data
	// Data is initially stored in this byte buffer
    byte ByteBuffer[6];

	// Read all 6 registers.

	//Method 1
    for (int i = 0; i < 6; i++)
    {
        I2c.read(ACCEL_ADDR, ACCEL_OUT_X1_REG + i, 1, &ByteBuffer[i]);
    }

	// The HIGH registers for each value. for some reason, the low ones are empty.
    signed char    x = (ByteBuffer[1]),
					y = (ByteBuffer[3]),
					z = (ByteBuffer[5]);

	// Convert the byte to m/s^2 double
    ax_data = (x * AX_SCALE);
    ay_data = (y * AY_SCALE);
    az_data = (z * AZ_SCALE);

    /// Read Magnetometer Data
    // Read the raw data from the registers
	for (int i = 0; i < 6; i++)
    {
        I2c.read(MAG_ADDR_R, OUT_XH_M + i, 1, &ByteBuffer[i]);
    }

	// Concatenate the two bytes
	x = ((ByteBuffer[0] << 8) | ByteBuffer[1]);
	y = ((ByteBuffer[2] << 8) | ByteBuffer[3]);
	z = ((ByteBuffer[4] << 8) | ByteBuffer[5]);

	// Convert the byte to gauss
    mx_data = (x * M_SCALE);
    my_data = (y * M_SCALE);
    mz_data = (z * M_SCALE);
};


/**
		double ax();
		- 	Call to return the most recent read value of ax
 */
double SENSORLIB::ax()
{
	return ax_data;
};

// Call to return the most recent value of ay
double SENSORLIB::ay()
{
	return ay_data;
};

// Call to return the most recent value of az
double SENSORLIB::az()
{
	return az_data;
};

// Call to return the most recent value of mx
double SENSORLIB::mx()
{
	return mx_data;
};

// Call to return the most recent value of my
double SENSORLIB::my()
{
	return my_data;
};

// Call to return the most recent value of mz
double SENSORLIB::mz()
{
	return mz_data;
};

void SENSORLIB::Cleanup()
{
    I2c.end();
};
