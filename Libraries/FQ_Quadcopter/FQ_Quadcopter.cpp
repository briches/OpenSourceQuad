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




void getInitialOffsets(struct kinematicData *kinematics,
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

        //TODO: Possibly add in a z-offset.

        kinematics->io_ax = (kinematics->io_ax + acceldata[0] ); // Sum
        kinematics->io_ay = (kinematics->io_ay + acceldata[1] );
        kinematics->io_az = (kinematics->io_az + acceldata[2] );
        kinematics->io_wx = (kinematics->io_wx + gyrodata[0] );
        kinematics->io_wy = (kinematics->io_wy + gyrodata[1] );
        kinematics->io_wz = (kinematics->io_wz + gyrodata[2] );

		if ((kinematics->io_ax==0)&&(kinematics->io_ay == 0)&&(kinematics->io_az == 0))
		{
			ERROR_LED(3);						// Critical error, accelerometer is NOT working
		}
        counter = counter + 1 ;

        delay(1);
    }

    kinematics->io_ax /= offset_counter;
	kinematics->io_ay /= offset_counter;
	kinematics->io_az /= offset_counter;
	kinematics->io_wx /= offset_counter;
	kinematics->io_wy /= offset_counter;
	kinematics->io_wz /= offset_counter;

	Serial.println(" ");
    Serial.println(kinematics->io_ax);
    Serial.println(kinematics->io_ay);
    Serial.println(kinematics->io_az);
    Serial.println(kinematics->io_wx);
    Serial.println(kinematics->io_wy);
    Serial.println(kinematics->io_wz);

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

bool initMotors(int speed,
				struct motorControl *motors)
{
	/**************************************************************************/
		//! @brief Initializes the motors, code from Andrew's script
	/**************************************************************************/
	unsigned long t1, t2;
	double elaps;

	t1 = micros();
	Serial.println("Initializing motors...");

    motors.motor1s = MIN_PULSE_WIDTH;            	// Initialize all motor speeds to 0% speed
    motors.motor2s = MIN_PULSE_WIDTH;          	// This might be useful, later
    motors.motor3s = MIN_PULSE_WIDTH;
    motors.motor4s = MIN_PULSE_WIDTH;

    motor1.attach(MOTOR1PIN);                      	// Attach motor 1 to D2
    motor2.attach(MOTOR2PIN);                      	// Attach motor 2 to D3
    motor3.attach(MOTOR3PIN);                       // Attach motor 3 to D4
    motor4.attach(MOTOR4PIN);                       // Attach motor 4 to D5

    // initializes motor1
    speed = map(speed, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	Serial.println(speed);
    for(int m1s = MIN_PULSE_WIDTH; m1s <= speed; m1s += 5)
    {
    	// Change the input to the function to a value to SERVO lib understands

		motor1.write(m1s);
		motor2.write(m1s);
		motor3.write(m1s);
		motor4.write(m1s);

		motors.motor1s = m1s;
		motors.motor2s = m1s;
		motors.motor3s = m1s;
		motors.motor4s = m1s;
		delay(10);
		Serial.print(motors.motor1s);
		Serial.println("%");
    }

	t2 = micros();
	elaps = (t2 - t1)/1000;
	Serial.print("Done!  Elapsed time (ms): ");
	Serial.println(elaps,4);
	Serial.println("");

    return true;
};


void mainProcess(	double aPID_out,
					double bPID_out,
					SENSORLIB_accel accel,
					SENSORLIB_mag	mag,
					OseppGyro	gyro,
					kinematicData kinematics,
					fourthOrderData fourthOrderXAXIS,
					fourthOrderData fourthOrderYAXIS,
					fourthOrderData fourthOrderZAXIS)
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
		// Motor Logic
		updateMotors(aPID_out,  bPID_out);

		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}

	/**! 				@Poll type four - 10 Hz				*/
	// Code in this block executes if the conditions for priority == 2 are satisfied.
	if (priority == 4)
	{
		/// Update GPS data using RMC


		/// Read battery voltage
		vbatt = (double)analogRead(15);
		vbatt *= 11;
		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}


};

bool updateMotors(double aPID_out, double bPID_out)
{

		int myMinSpeed = 1300;
		int myMaxSpeed = 1350;

		motor1s += aPID_out;
		motor4s -= aPID_out;

		motor2s += bPID_out;
		motor3s -= bPID_out;

		motor1s = constrain(motor1s, myMinSpeed, myMaxSpeed);
		motor2s = constrain(motor2s, myMinSpeed, myMaxSpeed);
		motor3s = constrain(motor3s, myMinSpeed, myMaxSpeed);
		motor4s = constrain(motor4s, myMinSpeed, myMaxSpeed);


		motor1.write(motor1s);
		motor2.write(motor2s);
		motor3.write(motor3s);
		motor4.write(motor4s);

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
