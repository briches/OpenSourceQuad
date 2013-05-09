Name	: OseppGyro Library
Authors : Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date	: May 7th, 2013
Version : 0.1
Notes	: Arduino Library for use with the Osepp MPU3000/MPU3050 gyro via I2C 	
	  interfacing. Format of the library sourced from Noah Shibley, Michael 	  Grant, NoMi Design Ltd. http://n0m1.com




Dependancies:
	I2C library:
 	http://dsscircuits.com/articles/arduino-i2c-master-library


List of Functions:
-------------------------------------------------------------------------------
Function: setI2CAddr
Description: Sets the I2C address of the device to a new value
Parameters: (int) address, commonly 0x68 or 0x69
-------------------------------------------------------------------------------
Function: dataMode
Description: Configures the devices full scale range/sensitivity, and 	     configures the internal digital low pass filter.
Parameters: (int) dScaleRange: Configures the degree/second range:		(250,500,1000,2000)
	          (int) dDLPF: Digital Low Pass Filter mode (0,1,2,3,4,5,6)
-------------------------------------------------------------------------------
Function: x
Description: Returns the sensor value stored by the device for rotation about the x-axis.
Parameters: none
-------------------------------------------------------------------------------
Function: y
Description: Returns the sensor value stored by the device for rotation about the y-axis.
Parameters: none
-------------------------------------------------------------------------------
Function: z
Description: Returns the sensor value stored by the device for rotation about the z-axis.
Parameters: none
-------------------------------------------------------------------------------
Function: update
Description: updates data values, and clears interrupts. Use at start of loop()
Parameters: none
-------------------------------------------------------------------------------
Function: regRead
Description: Reads the value stored in one of the device's data registers.
Parameters: (byte) reg: the register to read from (consult data sheet)
	          (byte) *buf: the bufffer to store the read data into
	          (byte) count: number of bytes to read (usually one)
-------------------------------------------------------------------------------
Function: regWrite
Description: Writes a value to one of the device's data registers.
Parameters: (byte) reg: the register to write to
	          (byte) val: the value to write to the register



