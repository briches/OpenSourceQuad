/******************************************************
Library designed to manage, update, and Control the
state of quadcopter

Author  : Brandon Riches
Date     :	June 2013

Updated for compatability with main polling loop and GPS interrupts


******************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Quadcopter.h>
#include <SENSORLIB.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <math.h>
#include <Servo.h>
#include <PID_v1.h>


/**************************************************************************/
/*!
    @brief Gets the initial offsets in both sensors to accomodate starting
*/
/**************************************************************************/
void Quadcopter :: get_Initial_Offsets()
{
    int offset_counter = 10;                       // # of data sets to consider when finding offsets
    int counter = 1;
    double acceldata[3];
    double gyrodata[3];

    while(counter <= offset_counter)
    {
        accel.update();                            // Updates the accelerometer registers
        acceldata[0] = accel.x();
        acceldata[1] = accel.y();
        acceldata[2] = accel.z();

        gyro.update();                             // Updates the gyro output registers
        gyrodata[0] = gyro.x();
        gyrodata[1] = gyro.y();
        gyrodata[2] = gyro.z();

        io_ax = (io_ax + acceldata[0] ); // Sum
        io_ay = (io_ay + acceldata[1] );
        io_az = (io_az + acceldata[2] );
        io_wx = (io_wx + gyrodata[0] );
        io_wy = (io_wy + gyrodata[1] );
        io_wz = (io_wz + gyrodata[2] );

        counter = counter + 1 ;
    }

    io_ax /= offset_counter;
	io_ay /= offset_counter;
	io_az /= offset_counter + 256;
	io_wx /= offset_counter;
	io_wy /= offset_counter;
	io_wz /= offset_counter;

    Serial.println(io_ax);
    Serial.println(io_ay);
    Serial.println(io_az);
    Serial.println(io_wx);
    Serial.println(io_wy);
    Serial.println(io_wz);
};

/**************************************************************************/
/*!
    @brief Converts the raw data from sensors to SI units
*/
/**************************************************************************/
void Quadcopter :: SI_convert()
{
  //Convert accelerometer readouts to m/s^2
    switch(g_ScaleRange) {
        case FULL_SCALE_RANGE_2g:
            ax = ax * SI_CONVERT_2g;
            ay = ay * SI_CONVERT_2g;
            az = az * SI_CONVERT_2g;
            break;

        case FULL_SCALE_RANGE_4g:
            ax = ax * SI_CONVERT_4g;
            ay = ay * SI_CONVERT_4g;
            az = az * SI_CONVERT_4g;
            break;

        case FULL_SCALE_RANGE_8g:
            ax = ax * SI_CONVERT_8g;
            ay = ay * SI_CONVERT_8g;
            az = az * SI_CONVERT_8g;
            break;
    }
    // Convert gyro readouts to degrees/s
    switch(d_ScaleRange) {

        case FULL_SCALE_RANGE_250:
            wx = wx * SI_CONVERT_250;
            wy = wy * SI_CONVERT_250;
            wz = wz * SI_CONVERT_250;
            break;

        case FULL_SCALE_RANGE_500:
            wx = wx * SI_CONVERT_500;
            wy = wy * SI_CONVERT_500;
            wz = wz * SI_CONVERT_500;
            break;

        case FULL_SCALE_RANGE_1000:
            wx = wx * SI_CONVERT_1000;
            wy = wy * SI_CONVERT_1000;
            wz = wz * SI_CONVERT_1000;
            break;

        case FULL_SCALE_RANGE_2000:
            wx = wx * SI_CONVERT_2000;
            wy = wy * SI_CONVERT_2000;
            wz = wz * SI_CONVERT_2000;
            break;
    }
};

/**************************************************************************/
/*!
    @brief Initializes the various sensors and instruments
*/
/**************************************************************************/
bool Quadcopter :: initSensor()
{
	byte x=0x0;

	Serial.print("Int gyro  ");                      		// See OseppGyro.h

    gyro.setI2CAddr(Gyro_Address);                // Set the I2C address in OseppGyro class

    gyro.dataMode(d_ScaleRange, DLPF);       // Set the dataMode in the OseppGyro Class

    while(x != B100000)
    {
        gyro.regRead(USER_CTRL,&x);                            // See the data sheet for the MPU3050 gyro
        gyro.regWrite(USER_CTRL,B00100000);             // http://invensense.com/mems/gyro/documents/RM-MPU-3000A.pdf
    }
    Serial.println("Done!");

	// Same as the gyro initialization, but the accel isnt an ass
    accelmag.Easy_Start();

    get_Initial_Offsets();                                     // Initial offsets private function in Control class

    return true;
};

/**************************************************************************/
/*!
    @brief Initializes the motors, code from Andrew's script
*/
/**************************************************************************/
bool Quadcopter::initMotors()
{
    motor1s = 0;                         // Initialize all motor speeds to 0
    motor2s = 0;                         // This might be useful, later
    motor3s = 0;
    motor4s = 0;

    motor1.attach(11);                      // Attach motor 1 to D11
    motor2.attach(10);                      // Attach motor 2 to D10
    motor3.attach(9);                       // Attach motor 3 to D9
    motor4.attach(6);                       // Attach motor 4 to D8

    // initializes motor1
    for(motor1s = 0; motor1s < 60; motor1s += 1)
    {
      motor1.write(motor1s);
      motor2.write(motor1s);
      motor3.write(motor1s);
      motor4.write(motor1s);

      motor2s = motor1s;
      motor3s = motor1s;
      motor4s = motor1s;

      delay(50);
    }

    return true;
}


/**************************************************************************/
/*!
    @brief Checks elapsed time and executes various tasks such as running PID controllers
*/
/**************************************************************************/
void Quadcopter::update()
{
	double alpha_accel, alpha_gyro, beta_accel, beta_gyro;
	int poll_type;
	unsigned long time;
	float gcoeff = 0.9;

	// Check the type of interrupt. If the time elapsed is less than the interrupt latency for a
	// certain device, it doesn't need to be updated.
	// The updates and calculations for those devices are wrapped in if statements that check
	// the value of poll_type, and thus determine if data is available.
	if (micros() - tpoll1 >= 10000) /*microseconds*/
	{
		poll_type = 1;								// Only essential updates are needed.
																// Update PID controllers based on accel/gyro/mag data,
																// and motor logic
	}
	/* Important note about this interrupt: This interrupt may or may not be necessary, but
	is here for future proofing */
	if (micros() - tpoll2 >=  poll2_interrupt)
	{
		poll_type = 2;								// Essential updates + USRF updates needed
																// Update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}
	if (micros() - tpoll3 >= poll3_interrupt)
	{
		poll_type = 3;								// Essential updates + USRF + GPS updates needed
																// Update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}

	/**!					@Poll type one 				*/
	// Code in this block executes if any of the poll conditions are satisfied
	if (poll_type == 1 || poll_type == 2 || poll_type ==3 )
	{
		accel.update();                                                 // Update the registers storing data INSIDE the sensors
		gyro.update();

		ax = accel.x() - io_ax;                         // Store Raw values from the sensor registers
		ay = accel.y() - io_ay;                         // and removes the initial offsets
		az = accel.z() - io_az;
		wx = gyro.y() - io_wy;
		wy = gyro.x() - io_wx;
		wz = gyro.z() - io_wz;

		SI_convert();                                           // Convert to SI units from raw data type

		mov_avg();					// Very important step!
												// Runs a moving average with a circular buffer

		time = (micros() - tpoll1)/1000; // This is the elapsed time since poll1 ran

		if (time <= 300000)			// For the begining of the program; It takes some time to get
		{											// started and we dont want a massive integration to start
			time = 0;
		}
		alpha_gyro += wy * time/1000;		// Time integration of wy gets rotation about y
		beta_gyro += wx * time/1000;			// Time integration of wx gets rotation about x
		Serial.print(alpha_gyro); Serial.print(" ");

		alpha_accel = atan2(ax, az) *180/Pi ; // Arctan of the two values returns the angle,
		Serial.println(alpha_accel);
		beta_accel = atan2(ay, az) *180/Pi;   // in rads, and convert to degrees

		alpha = gcoeff * alpha_gyro +   (1-gcoeff)*alpha_accel;
		beta = gcoeff * beta_gyro +  (1-gcoeff)*beta_accel;

		tpoll1 = micros();						// Ready for the next poll
	}

	/**! 				@Poll type two 				*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type == 2)
	{
		tpoll2 = micros();
	}

	/**! 				@Poll type three 				*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type == 3)
	{
		tpoll3 = micros();
	}


// Chebyshev IIR low pass filter formula
// incl. here, maybe

};

bool Quadcopter::updateMotors(double aPID_out, double bPID_out)
{
	motor1s -= aPID_out;
	motor4s -= aPID_out;

	motor2s += aPID_out;
	motor3s += aPID_out;

	motor4s -= bPID_out;
	motor3s -= bPID_out;

	motor2s += bPID_out;
	motor1s += bPID_out;

	motor1.write(motor1s);
	motor2.write(motor2s);
	motor3.write(motor3s);
	motor4.write(motor4s);
}

void Quadcopter :: mov_avg()
{
		// Updates the prev_data by bumping up each data set

		for(int set = 9; set >= 0; set --)					// Data set number
		{
			for (int var = 0; var <= 9; var++)				// Variable
			{
				prev_data[set][var] = prev_data[set-1][var];
			}
		}

		// Add new data sets to the first spot in the buffer
		prev_data[0][0] = ax;
		prev_data[0][1] = ay;
		prev_data[0][2] = az;
		prev_data[0][3] = wx;
		prev_data[0][4] = wy;
		prev_data[0][5] = wz;
		prev_data[0][6] = mx;
		prev_data[0][7] = my;
		prev_data[0][8] = mz;
		prev_data[0][9] = elev;

		// Buffer is updated, now a moving average can be calculated,  If the buffer size has to be increased then we can do that.
		ax = 0;
		ay = 0;
		az = 0;
		wx = 0;
		wy = 0;
		wz = 0;
		mx = 0;
		my = 0;
		mz = 0;
		elev = 0; // ready for average calculation

		double foo[10];

		for(int var = 0; var <= 9; var++)				// Each variable
		{
			for (int set= 0; set<= 9; set++) 			// Each set
			{
				foo[var] += prev_data[set][var];
			}
		}

		ax = foo[0];
		ay = foo[1];
		az = foo[2];
		wx = foo[3];
		wy = foo[4];
		wz = foo[5];
		mx = foo[6];
		my = foo[7];
		mz = foo[8];
		elev = foo[9];

		ax /= 10;						// Finish off the calculation
		ay /= 10;
		az /= 10;
		wx /= 10;
		wy /= 10;
		wz /= 10;
		mx /= 10;
		my /= 10;
		mz /= 10;
		elev /= 10;
};


// This function for debugging purposes
//
///**************************************************************************/
///*!
//    @brief Sets the motors to a new speed
//*/
///**************************************************************************/
//void Control::setMotorSpeed(int motor, int speed)
//{
//	int m1d;		// Directions that motor speed needs to increment
//	int m2d;		// +1 for increase, -1 for decrease
//	int m3d;
//	int m4d;
//
//	switch (motor)
//	{
//		/// MOTOR /
//		case 1:
//
//			if (speed !=  MotorSpeeds.motor1s)										// Change required, not the same
//			{
//				if (speed > MotorSpeeds.motor1s)										// Need to go up
//				{
//					for (int x = MotorSpeeds.motor1s; x <= speed; x += 1)	// Ease into the new speed
//					{
//						motor1.write(x);
//						MotorSpeeds.motor1s = x;
//						delay(25);
//					}
//				}
//				else																					// Need to decrease speed
//				{
//					for (int x = MotorSpeeds.motor1s; x >= speed; x -= 1)	// Ease into the new speed
//					{
//						motor1.write(x);
//						MotorSpeeds.motor1s = x;
//						delay(25);
//					}
//				}
//			}
//			break;
//
//			//MOTOR 2
//			case 2:
//
//			if (speed !=  MotorSpeeds.motor2s)										// Change required, not the same
//			{
//				if (speed > MotorSpeeds.motor2s)										// Need to go up
//				{
//					for (int x = MotorSpeeds.motor2s; x <= speed; x += 1)	// Ease into the new speed
//					{
//						motor2.write(x);
//						MotorSpeeds.motor2s = x;
//						delay(25);
//					}
//				}
//				else																					// Need to decrease speed
//				{
//					for (int x = MotorSpeeds.motor2s; x >= speed; x -= 1)	// Ease into the new speed
//					{
//						motor2.write(x);
//						MotorSpeeds.motor2s = x;
//						delay(25);
//					}
//				}
//			}
//			break;
//			//MOTOR 3
//			case 3:
//
//			if (speed !=  MotorSpeeds.motor3s)										// Change required, not the same
//			{
//				if (speed > MotorSpeeds.motor3s)										// Need to go up
//				{
//					for (int x = MotorSpeeds.motor3s; x <= speed; x += 1)	// Ease into the new speed
//					{
//						motor3.write(x);
//						MotorSpeeds.motor3s = x;
//						delay(25);
//					}
//				}
//				else																					// Need to decrease speed
//				{
//					for (int x = MotorSpeeds.motor3s; x >= speed; x -= 1)	// Ease into the new speed
//					{
//						motor3.write(x);
//						MotorSpeeds.motor3s = x;
//						delay(25);
//					}
//				}
//			}
//			break;
//
//			// MOTOR 4
//			case 4:
//
//			if (speed !=  MotorSpeeds.motor4s)										// Change required, not the same
//			{
//				if (speed > MotorSpeeds.motor4s)										// Need to go up
//				{
//					for (int x = MotorSpeeds.motor4s; x <= speed; x += 1)	// Ease into the new speed
//					{
//						motor4.write(x);
//						MotorSpeeds.motor4s = x;
//						delay(25);
//					}
//				}
//				else																					// Need to decrease speed
//				{
//					for (int x = MotorSpeeds.motor4s; x >= speed; x -= 1)	// Ease into the new speed
//					{
//						motor4.write(x);
//						MotorSpeeds.motor4s = x;
//						delay(25);
//					}
//				}
//			}
//			break;
//
//			// ALL MOTORS REQUIRE CHANGE
//			case 5:
//
//			if  ((speed > MotorSpeeds.motor1s )&& (speed != MotorSpeeds.motor1s)) {m1d = 1;}
//			else {m1d = -1;}
//
//			if  ((speed > MotorSpeeds.motor2s )&& (speed != MotorSpeeds.motor2s)) {m2d = 1;}
//			else {m2d = -1;}
//
//			if  ((speed > MotorSpeeds.motor3s )&& (speed != MotorSpeeds.motor3s)) {m3d = 1;}
//			else {m3d= -1;}
//
//			if  ((speed > MotorSpeeds.motor4s )&& (speed != MotorSpeeds.motor4s)) {m4d = 1;}
//			else {m4d= -1;}
//
//			while ( (MotorSpeeds.motor1s != speed)
//				  ||  (MotorSpeeds.motor2s != speed)
//				  ||  (MotorSpeeds.motor3s != speed)
//				  ||  (MotorSpeeds.motor4s != speed) )
//				  {
//						if (MotorSpeeds.motor1s != speed )
//						{
//							MotorSpeeds.motor1s += m1d;
//							motor1.write(MotorSpeeds.motor1s);
//						}
//						if (MotorSpeeds.motor2s != speed )
//						{
//							MotorSpeeds.motor2s += m2d;
//							motor2.write(MotorSpeeds.motor2s);
//						}
//						if (MotorSpeeds.motor3s != speed )
//						{
//							MotorSpeeds.motor3s += m3d;
//							motor3.write(MotorSpeeds.motor3s);
//						}
//						if (MotorSpeeds.motor4s != speed )
//						{
//							MotorSpeeds.motor4s += m4d;
//							motor4.write(MotorSpeeds.motor4s);
//						}
//
//						delay(25);
//				  }
//
//
//			break;
//	}
//}

