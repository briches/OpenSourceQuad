//*****************************************************/
/*//*//*   FreeQuad   *//*//*

Library designed to manage, my_update, and Control the
state of quadcopter

Author  : Brandon Riches
Date     :	June 2013


******************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <FQ_Kinematics.h>
#include <FQ_Quadcopter.h>
#include <Wire.h>
#include <I2c.h>




void getInitialOffsets(struct kinematicData kinematics,
						SENSORLIB_accel accel,
						SENSORLIB_mag mag,
						OseppGyro gyro)
{
	/**************************************************************************/
		//! @brief Gets the initial offsets in both sensors to accomodate board mount
    /**************************************************************************/
	ERROR_LED(2);										// Warning LED
    int offset_counter = 10;                       // # of data sets to consider when finding offsets
    int counter = 1;
    double acceldata[3];
    double gyrodata[3];

    while(counter <= offset_counter)
    {
    	sensors_event_t accel_event;
        accel.getEvent(&accel_event);                            // my_updates the accelerometer registers
        acceldata[0] = accel_event.acceleration.x;
        acceldata[1] = accel_event.acceleration.y;
        acceldata[2] = accel_event.acceleration.z;

        gyro.update();                             // my_updates the gyro output registers
        gyrodata[0] = gyro.x();
        gyrodata[1] = gyro.y();
        gyrodata[2] = gyro.z();

        kinematics.io_ax = (kinematics.io_ax + acceldata[0] ); // Sum
        kinematics.io_ay = (kinematics.io_ay + acceldata[1] );
        kinematics.io_az = (kinematics.io_az + acceldata[2] );
        kinematics.io_wx = (kinematics.io_wx + gyrodata[0] );
        kinematics.io_wy = (kinematics.io_wy + gyrodata[1] );
        kinematics.io_wz = (kinematics.io_wz + gyrodata[2] );

		if ((kinematics.io_ax==0)&&(kinematics.io_ay == 0)&&(kinematics.io_az == 0))
		{
			ERROR_LED(3);						// Critical error, accelerometer is NOT working
		}
        counter = counter + 1 ;

        delay(1);
    }

    kinematics.io_ax /= offset_counter;
	kinematics.io_ay /= offset_counter;
	kinematics.io_az /= offset_counter;
	kinematics.io_wx /= offset_counter;
	kinematics.io_wy /= offset_counter;
	kinematics.io_wz /= offset_counter;

	Serial.println(" ");
    Serial.println(kinematics.io_ax);
    Serial.println(kinematics.io_ay);
    Serial.println(kinematics.io_az);
    Serial.println(kinematics.io_wx);
    Serial.println(kinematics.io_wy);
    Serial.println(kinematics.io_wz);

    ERROR_LED(1); 					// Success LED
};

bool initSensor(SENSORLIB_accel accel,
				SENSORLIB_mag mag,
				OseppGyro gyro,
				struct kinematicData kinematics)
{
	/**************************************************************************/
		//! @brief Initializes the various sensors and instruments
	/**************************************************************************/
	ERROR_LED(2);												// Warning LED

	byte x=0x0;
	Serial.println("Intializing gyro...    ");                      		// See OseppGyro.h

	Serial.println("Set I2c address.");
    gyro.setI2CAddr(Gyro_Address);                // Set the I2C address in OseppGyro class

	Serial.println("Set ScaleRange and DLPF settings");
    gyro.dataMode(d_ScaleRange, DLPF);       // Set the dataMode in the OseppGyro Class

	Serial.println("Set USER_CTRL register");
    while(x != B100000)
    {
        gyro.regRead(USER_CTRL, &x, 1);                            // See the data sheet for the MPU3050 gyro
        gyro.regWrite(USER_CTRL, B00100000);             // http://invensense.com/mems/gyro/documents/RM-MPU-3000A.pdf
        if(millis() >= 3000)
        {
        	Serial.println("Err: Unable to write to USER_CTRL. ");
        	break;
        }
    }

	// Same as the gyro initialization, but the accel isnt an ass
	accel.begin();
	mag.begin();


	Serial.print("Getting initial offset values...    ");
    getInitialOffsets(kinematics,
						accel,
						mag,
						gyro);

    return true;
};


void mainProcess(	double pitchPID_out,
					double rollPID_out,
					SENSORLIB_accel accel,
					SENSORLIB_mag	mag,
					OseppGyro	gyro,
					kinematicData kinematics,
					fourthOrderData fourthOrderXAXIS,
					fourthOrderData fourthOrderYAXIS,
					fourthOrderData fourthOrderZAXIS,
					FQ_MotorControl MotorControl)
{
	/**************************************************************************/
		//! @brief Checks elapsed time and executes various tasks such as running PID controllers
	/**************************************************************************/
	int 	priority = 0;

	double 	time 	= micros();
	double 	time1 	= time - tpoll1;
	double 	time2 	= time - tpoll2;
	double 	time3 	= time - tpoll3;
	double 	time4 	= time - tpoll4;

	// Check the priority. If the time elapsed is less than the interrupt period for a
	// certain device, it doesn't need to be updated.
	// The updates and calculations for those devices are wrapped in if statements that check
	// the value of priority, and thus determine if data is available.
	if (time1 >= _100HzPoll) /*microseconds*/
	{
		// Accel, gyro. Update roll and pitch measurements
		priority = 1;

	}
	if (time2 >=  _75HzPoll)
	{
		// Mag, USRF.
		priority = 2;

	}
	if (time3 >= _20HzPoll)
	{
		priority = 3;								// Apply error function output to motor speeds


	}

	if (time4 >= _10HzPoll)
	{
		priority = 4;								// GPS update


	}

	/**!					@Poll type one 				*/
	// Code in this block executes if any of the poll conditions are satisfied
	if (priority == 1)
	{

		kinematicEvent(	0,
						kinematics,
						accel,
						mag,
						gyro,
						fourthOrderXAXIS,
						fourthOrderYAXIS,
						fourthOrderZAXIS);

		// Read the time that poll1 was last executed.
		tpoll1 = micros();
	}


/**! 				@Poll type two - 75 Hz			*/
	// Code in this block executes if the conditions for priority == 2 are satisfied.
	if (priority == 2)
	{
		// Update kinematics including magnetometer
		kinematicEvent(	1,
						kinematics,
						accel,
						mag,
						gyro,
						fourthOrderXAXIS,
						fourthOrderYAXIS,
						fourthOrderZAXIS);

		// Read the time that poll2 was last executed
		tpoll2 = micros();
	}


	/**! 				@Poll type three - 20 Hz			*/
	// Code in this block executes if the conditions for priority == 2 are satisfied.
	if (priority == 3)
	{
		double x = 0;
		// Motor Logic
		MotorControl.updateMotors(pitchPID_out,  rollPID_out, x, x);

		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}

	/**! 				@Poll type four - 10 Hz				*/
	// Code in this block executes if the conditions for priority == 2 are satisfied.
	if (priority == 4)
	{
		/// Update GPS data using RMC


		/// Read battery voltage
		// TODO:
//		vbatt = (double)analogRead(15);
//		vbatt *= 11.1;
//		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}


};


void ERROR_LED(int LED_SEL)
{
	switch(LED_SEL)
	{
		case 1:
			// Green LED. Should indicate successes
			digitalWrite(GREEN_LED, HIGH);
			digitalWrite(RED_LED, LOW);
			digitalWrite(YELLOW_LED, LOW);
			break;

		case 3:
			//Red LED. Indicates critical errors
			digitalWrite(RED_LED, HIGH);
			digitalWrite(GREEN_LED, LOW);
			digitalWrite(YELLOW_LED, LOW);
			while(1);	// Stop everything
			break;

		case 2:
			// Yellow LED. Indicates warnings
			digitalWrite(RED_LED, LOW);
			digitalWrite(GREEN_LED, LOW);
			digitalWrite(YELLOW_LED, HIGH);
	}
};
