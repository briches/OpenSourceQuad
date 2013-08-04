/**
    Sensor Library
    Author: Brandon Yue and MEEEEEEEEEEE

    Replacement for the Adafruit LSM303 library. Uses I2C.
**/

#include <SENSORLIB.h>
#include <I2c.h>
#include <math.h>

SENSORLIB::SENSORLIB() {};

void SENSORLIB::Easy_Start()
{
		uint8_t	before,
						after;

        I2c.begin();

        ///ACCEL OPTIONS
        //CONTROL REG 1:    DATA RATES + SENSOR ENABLES
        //[n/a] [n/a] [n/a] [DATA RATE 1] [DATA RATE 2] [X] [Y] [Z]
        //DEFAULT 0x03
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG1_A, 1, &before);
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG1_A, 0x2F);
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG1_A, 1, &after);

        Serial.println(" ");
        Serial.print("ACCEL_CTRL_REG1_A:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);

        //CONTROL REG 2:    HIGH PASS FILTER
        //
        //DEFAULT 0x00
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG2_A, 1, &before);
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG2_A, 0x00);
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG2_A, 1, &after);

        Serial.print("ACCEL_CTRL_REG2_A,:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);

        //CONTROL REG 3:    INTERRUPTS
        //
        //DEFAULT 0x00
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG3_A, 1, &before);
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG3_A, 0x00);
		I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG3_A, 1, &after);

        Serial.print("ACCEL_CTRL_REG3_A,:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after, BIN);

        //CONTROL REG 4:    BLOCK UPDATE PARAMS, SCALE SELECTION, SELF TEST
        //
        //DEFAULT 0x00
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG4_A, 1, &before);
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG4_A, 0x00);
        I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG4_A, 1, &before);

        Serial.print("ACCEL_CTRL_REG4_A:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);

        //CONTROL REG 5:    SLEEP-TO-WAKE
        //
        //DEFAULT 0x00
		I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG5_A, 1, &before);
        I2c.write(ACCEL_ADDR, ACCEL_CTRL_REG5_A, 0x00);
		I2c.read(ACCEL_ADDR, ACCEL_CTRL_REG5_A, 1, &after);

		Serial.print("ACCEL_CTRL_REG5_A:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);

        ///MAG OPTIONS
        //MR_REG
        //
        I2c.read(MAG_ADDR_R, MAG_MR_REG_M, 1, &before);
        I2c.write(MAG_ADDR_W, MAG_MR_REG_M, 0x00);
        I2c.read(MAG_ADDR_R, MAG_MR_REG_M, 1, &after);

  		Serial.print("MAG_MR_REG_M:	    \n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);

        //MAG CONTROL A
        // DATA RATE
        // DEFAULT 0x00
        I2c.read(MAG_ADDR_R, MAG_CRA_REG_M, 1, &before);
        I2c.write(MAG_ADDR_W, MAG_CRA_REG_M, 0x18);
        I2c.read(MAG_ADDR_R, MAG_CRA_REG_M, 1, &after);

		Serial.print("MAG_CRA_REG_M:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);

        //MAG CONTROL B
        //GAIN SETTINGS
        //DEFAULT 0x01
        I2c.read(MAG_ADDR_R, MAG_CRB_REG_M, 1, &before);
        I2c.write(MAG_ADDR_W, MAG_CRB_REG_M, 0x20);
        I2c.read(MAG_ADDR_R, MAG_CRB_REG_M, 1, &after);

		Serial.print("MAG_CRB_REG_M:		\n	Prior to write: ");
        Serial.print(before, BIN);
        Serial.print(" After write: ");
        Serial.println(after,BIN);



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

	I2c.write(ACCEL_ADDR, ACCEL_OUT_X_L_A, 1, 0x80)

    uint8_t	xhi,
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

	Serial.print(xlo);
    Serial.print(" ");
    Serial.print(xhi);
    Serial.println(" ");

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
    mx_data = x * MXY_SCALE * GRAV_STANDARD;
    my_data = y * MXY_SCALE * GRAV_STANDARD;
    mz_data = z * MZ_SCALE * GRAV_STANDARD;

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
