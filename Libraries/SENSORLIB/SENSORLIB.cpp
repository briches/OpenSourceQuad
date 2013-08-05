/**
    Sensor Library
    Author: Brandon Yue and Brandon Riches

    Replacement for the Adafruit LSM303 library. Uses I2C.
**/

#include <SENSORLIB.h>
#include <I2c.h>
#include <math.h>

SENSORLIB::SENSORLIB() {};

void SENSORLIB::Easy_Start(byte ACCEL_ODR, byte ACCEL_FS, byte MAG_ODR, byte MAG_GAIN)
{
        I2c.begin();

        ///ACCEL OPTIONS
        //CONTROL REG 1:    DATA RATES + SENSOR ENABLES
        //[n/a] [n/a] [n/a] [DATA RATE 1] [DATA RATE 2] [X] [Y] [Z]
        // Set the accelerometer ODR
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG1_A, (ACCEL_ODR << 3));

        //CONTROL REG 4:    BLOCK UPDATE PARAMS, SCALE SELECTION, SELF TEST
        //
        // Set the full-scale selection + sensitivity
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG4_A, (ACCEL_FS << 4));

        ///MAG OPTIONS
        //MAG CONTROL A
        // DATA RATE
        // DEFAULT 0x00
        I2c.write(MAG_ADDR_W, MAG_CRA_REG_M, (MAG_ODR << 2));

        //MAG CONTROL B
        //GAIN SETTINGS
        //DEFAULT 0x01
        I2c.write(MAG_ADDR_W, MAG_CRB_REG_M, (MAG_GAIN << 5));


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

    uint8_t		xhi,
					xlo,
					yhi,
					ylo,
					zhi,
					zlo;

	// Read all 6 registers.

	//Method 1
    for (int i = 0; i < 6; i++)
    {
        I2c.read(ACCEL_ADDR, ACCEL_OUT_X_L_A + i, 1, &ByteBuffer[i]);
    }

	// The HIGH registers for each value. for some reason, the low ones are empty.
    xlo = (ByteBuffer[0]);
	xhi = (ByteBuffer[1]);
	ylo = (ByteBuffer[2]);
	yhi = (ByteBuffer[3]);
	zlo = (ByteBuffer[4]);
	zhi = (ByteBuffer[5]);

	ax_data = (xlo | (xhi << 8)) >> 4;
	ay_data = (ylo | (yhi << 8)) >> 4;
	az_data = (zlo | (zhi << 8)) >> 4;


	// Convert the byte to m/s^2 double
    ax_data *= AX_SCALE;
    ay_data *= AY_SCALE;
    az_data *= AZ_SCALE;

    /// Read Magnetometer Data
    // Read the raw data from the registers
	I2c.read(MAG_ADDR_R, MAG_OUT_X_H_M , 1, &xhi);
	I2c.read(MAG_ADDR_R, MAG_OUT_X_L_M , 1, &xlo);
	I2c.read(MAG_ADDR_R, MAG_OUT_Y_H_M , 1, &yhi);
	I2c.read(MAG_ADDR_R, MAG_OUT_Y_L_M , 1, &ylo);
	I2c.read(MAG_ADDR_R, MAG_OUT_Z_H_M , 1, &zhi);
	I2c.read(MAG_ADDR_R, MAG_OUT_Z_L_M , 1, &zlo);

	// Concatenate the two bytes
	uint8_t x = (xlo | (xhi << 8));
	uint8_t y = (ylo | (yhi << 8));
	uint8_t z = (zlo | (zhi << 8));


	// Convert the byte to gauss
    mx_data = x * MAG_GAIN_LSB_xy_1_3 * GRAV_STANDARD;
    my_data = y * MAG_GAIN_LSB_xy_1_3 * GRAV_STANDARD;
    mz_data = z * MAG_GAIN_LSB_z_1_3 * GRAV_STANDARD;

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
