/******************************************************
Library designed to manage, my_update, and Control the
state of quadcopter

Author  : Brandon Riches
Date     :	June 2013

my_updated for compatability with main polling loop and GPS interrupts


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



void Quadcopter :: get_Initial_Offsets()
{
	/**************************************************************************/
		//! @brief Gets the initial offsets in both sensors to accomodate starting non-level
    /**************************************************************************/
	ERROR_LED(2);										// Warning LED
    int offset_counter = 10;                       // # of data sets to consider when finding offsets
    int counter = 1;
    double acceldata[3];
    double gyrodata[3];

    while(counter <= offset_counter)
    {
        accelmag.update();                            // my_updates the accelerometer registers
        acceldata[0] = accelmag.ax();
        acceldata[1] = accelmag.ay();
        acceldata[2] = accelmag.az();

        gyro.update();                             // my_updates the gyro output registers
        gyrodata[0] = gyro.x();
        gyrodata[1] = gyro.y();
        gyrodata[2] = gyro.z();

        //TODO: Possibly add in a z-offset.

        io_ax = (io_ax + acceldata[0] ); // Sum
        io_ay = (io_ay + acceldata[1] );
        io_az = (io_az + acceldata[2] );
        io_wx = (io_wx + gyrodata[0] );
        io_wy = (io_wy + gyrodata[1] );
        io_wz = (io_wz + gyrodata[2] );

		if ((io_ax==0)&&(io_ay == 0)&&(io_az == 0))
		{
			ERROR_LED(3);						// Critical error, accelerometer is NOT working
		}
        counter = counter + 1 ;
    }

    io_ax /= offset_counter;
	io_ay /= offset_counter;
	io_az /= offset_counter;
	io_wx /= offset_counter;
	io_wy /= offset_counter;
	io_wz /= offset_counter;

	io_wz -= 9.81;

	Serial.println(" ");
    Serial.println(io_ax);
    Serial.println(io_ay);
    Serial.println(io_az);
    Serial.println(io_wx);
    Serial.println(io_wy);
    Serial.println(io_wz);
    ERROR_LED(1); 					// Success LED
};

void Quadcopter :: SI_convert()
{
	/**************************************************************************/
		//! @brief Converts the raw data from sensors to SI units
    /**************************************************************************/
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

bool Quadcopter :: initSensor()
{
	/**************************************************************************/
		//! @brief Initializes the various sensors and instruments
	/**************************************************************************/
	ERROR_LED(2);												// Warning LED

	byte x=0x0;
	unsigned long t1 = micros();
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

    unsigned long t2 = micros();
    double elaps = (t2 - t1);
    elaps /= 1000;
    Serial.print("Done!  Elapsed time (ms): ");
    Serial.println(elaps,4);
    Serial.println("");

	t1 = micros();
	Serial.print("Initializing accel and magnetometer...	");
	// Same as the gyro initialization, but the accel isnt an ass
    accelmag.Easy_Start(ACCEL_ODR_100_74, ACCEL_FULL_SCALE_2g, MAG_ODR_75_0, MAG_GAIN_1_3);

    t2 = micros();
    elaps = (t2 - t1);
    elaps /= 1000;
	Serial.print("Done!  Elapsed time (ms): ");
	Serial.println(elaps,4);
	Serial.println("");

	t1 = micros();
	Serial.print("Getting initial offset values...    ");
    get_Initial_Offsets();                                     // Initial offsets private function in Control class
    t2 = micros();
    elaps = (t2 - t1);
    elaps/=1000;
	Serial.print("Done!  Elapsed time (ms): ");
	Serial.println(elaps,4);
	Serial.println("");

    return true;
};


bool Quadcopter::initMotors(int speed)
{
	/**************************************************************************/
		//! @brief Initializes the motors, code from Andrew's script
	/**************************************************************************/
	unsigned long t1, t2;
	double elaps;

	t1 = micros();
	Serial.println("Initializing motors...");

    motor1s = MIN_PULSE_WIDTH;                         // Initialize all motor speeds to 0% speed
    motor2s = MIN_PULSE_WIDTH;                         // This might be useful, later
    motor3s = MIN_PULSE_WIDTH;
    motor4s = MIN_PULSE_WIDTH;

    motor1.attach(2);                      // Attach motor 1 to D2
    motor2.attach(3);                      // Attach motor 2 to D3
    motor3.attach(4);                       // Attach motor 3 to D4
    motor4.attach(5);                       // Attach motor 4 to D5

    // initializes motor1
    int m1s = map(speed, 0, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

    // TODO: Map percentage to mus range of servo lib.
    for(m1s = 0; m1s <= speed; m1s += 1)
    {
    	// Change the input to the function to a value to SERVO lib understands

		motor1.write(m1s);
		motor2.write(m1s);
		motor3.write(m1s);
		motor4.write(m1s);

		motor1s = m1s;
		motor2s = m1s;
		motor3s = m1s;
		motor4s = m1s;
		delay(50);
		Serial.print(motor1s);
		Serial.println("%");
    }

	t2 = micros();
	elaps = (t2 - t1)/1000;
	Serial.print("Done!  Elapsed time (ms): ");
	Serial.println(elaps,4);
	Serial.println("");

    return true;
};



void Quadcopter::update(double aPID_out, double bPID_out)
{
	/**************************************************************************/
		//! @brief Checks elapsed time and executes various tasks such as running PID controllers
	/**************************************************************************/
	double 	alpha_accel,
					beta_accel,
					heading_mag;
	int poll_type = 0;
	double   time,
					t_convert = 1000000,
					time1,
					time2,
					time3;
	double 	a_gcoeff = 0,
					h_gcoeff = 0.99;

	time = micros();

	time1 = time - tpoll1;
	time2 = time - tpoll2;
	time3 = time - tpoll3;

	// Check the type of interrupt. If the time elapsed is less than the interrupt latency for a
	// certain device, it doesn't need to be my_updated.
	// The my_updates and calculations for those devices are wrapped in if statements that check
	// the value of poll_type, and thus determine if data is available.
	if (time1 >= 10000) /*microseconds*/
	{
		poll_type = 1;								// Only essential my_updates are needed.
																// my_update PID controllers based on accel/gyro/mag data,

	}
	/* Important note about this interrupt: This interrupt may or may not be necessary, but
	is here for future proofing */
	if (time2 >=  poll2_interrupt)
	{
		poll_type = 2;								// Essential my_updates + USRF my_updates needed
																// my_update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}
	if (time3 >= poll3_interrupt)
	{
		poll_type = 3;								// Essential my_updates + USRF + GPS my_updates needed
																// my_update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}

	/**!					@Poll type one 				*/
	// Code in this block executes if any of the poll conditions are satisfied
	if (poll_type == 1 || poll_type == 2 || poll_type ==3 )
	{
		// my_update the registers storing data INSIDE the sensors
		accelmag.update();
		gyro.update();

		/// Store Raw values from the sensor registers, and remove initial offsets
		ax = accelmag.ax() - io_ax;
		ay = accelmag.ay() - io_ay;
		az = (accelmag.az() - io_az);
		mx = accelmag.mx();
		my = accelmag.my();
		mz = accelmag.mz();
		wx = -(gyro.x()- io_wx) - time*GYRO_DRIFT_RATE_X;
		wy = -(gyro.y() - io_wy) - time*GYRO_DRIFT_RATE_Y;
		wz = gyro.z() - io_wz -time*GYRO_DRIFT_RATE_Z;

		/// Convert to SI units from raw data type in gyro data
		SI_convert();

		/// Move the new data into the array to prepare for the LPF
		NEW_DATA[my_update][0] = ax;
		NEW_DATA[my_update][1] = ay;
		NEW_DATA[my_update][2] = az;
		NEW_DATA[my_update][3] = wx;
		NEW_DATA[my_update][4] = wy;
		NEW_DATA[my_update][5] = wz;
		NEW_DATA[my_update][6] = mx;
		NEW_DATA[my_update][7] = my;
		NEW_DATA[my_update][8] = mz;
		NEW_DATA[my_update][9] = elev;

		/// Runs a Chebyshev 4th order filter with a circular buffer
		// Working on a filter with better frequency response now.
		// Soon, the Chebyshev 4th order LPF will be implemented.
		IIRF(NEW_DATA, FILTERED_DATA, my_update);

		ax = FILTERED_DATA[my_update][0];
		ay = FILTERED_DATA[my_update][1];
		az = FILTERED_DATA[my_update][2];
		wx = FILTERED_DATA[my_update][3];
		wy = FILTERED_DATA[my_update][4];
		wz = FILTERED_DATA[my_update][5];
		mx = FILTERED_DATA[my_update][6];
		my = FILTERED_DATA[my_update][7];
		mz = FILTERED_DATA[my_update][8];
		elev = FILTERED_DATA[my_update][9];

		my_update++;
		if (my_update > 4)
		{
			my_update = 0;
		}


		time = (micros() - tpoll1)/t_convert; // This is the elapsed time since poll1 ran

		/// my_update pitch and roll
		// Time integration of wy gets rotation about y
		alpha_gyro += wy * time;
		beta_gyro += wx * time;

		// This is my feeble first attempt to deal with NaN errors resulting from the vector calculations
		// done below. Situations were arising where ax, ay, az were all > 0 and az was > 9.81. This screwed
		// up the trig, giving the acos of a value > 1.
		if (az >= 9.81)
		{
			double foobar = pow(9.81,2) - pow(ax,2) - pow(ay,2);
			az = sqrt(foobar);
		}

		// Beta is analogous to ROLL, or rotating about the X-AXIS
		beta_accel = asin(ay/9.81)*180/Pi;

		// Alpha is analogous to PITCH, or rotating about the Y-AXIS
		alpha_accel = acos( az / ( 9.81 * cos(  asin(ay/9.81)  ) ) )* 180/Pi;

		// Check the quadrant the angle is in. Switch the sign if necessary
		if (ax < 0)
		{
			alpha_accel = -alpha_accel;
		}

		// Still getting NaNs, so this is a brutish way of fixing it.
		// Seems to only happen at extreme angles or right at the begining or runtime,
		// so likely it wont be a problem
		if (isnan(alpha_accel))
		{
			alpha_accel = 0;
		}
		if (isnan(beta_accel))
		{
			beta_accel = 0;
		}


		/// Complementary filter
		// Since both sensors have the ability to calculate angle, we take the advantages both sensors have
		// into account:
		// Gyro: Less prone to noise from mechanical oscillation, but responds slower and drifts over time
		// Accel: Prone to noise from mechanical oscillation, but responds quickly and doesnt drift.
		alpha = a_gcoeff * alpha_gyro +   (1-a_gcoeff)*alpha_accel;
		beta = a_gcoeff * beta_gyro +  (1-a_gcoeff)*beta_accel;

		/// my_update heading
		// Get the heading (in degrees) from the magnetometer.
		heading_mag = ((atan2(my,mx))*180)/Pi;

		// Normalize to 360 degrees
		 if (heading_mag < 0)
		  {
			heading_mag += 360;
		  }

		// Integrate wz to get the heading from the gyro
		heading_gyro += wz*0.01;

		// Normalize to 360 degrees
		 if (heading_gyro < 0)
		  {
			heading_gyro += 360;
		  }

		// Complementary filter the compass heading and the gyro heading
		heading = h_gcoeff * heading_gyro + (1-h_gcoeff)*heading_mag;

		/// Read the time that poll1 was last executed.
		tpoll1 = micros();
	}

	/**! 				@Poll type two 				*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type == 2)
	{
		/// Motor Logic takes place at a lower frequency.
		// ESCs need several periods of signal to properly read motor speed,
		// thus placing a upper limit on the frequency with which the ESCs can
		// be my_updated.
		updateMotors(aPID_out,  bPID_out);

		/// Read the time that poll2 was last executed
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

		int myMinSpeed = MIN_PULSE_WIDTH;
		int myMaxSpeed = MAX_PULSE_WIDTH;

		aPID_out = -aPID_out;
		bPID_out = -bPID_out;

		motor1s +=  aPID_out;
		motor4s -= aPID_out;

		motor2s += bPID_out;
		motor3s -= bPID_out;

		// Check if motors are going to be set to smaller than the minimum.
		if (motor1s <= myMinSpeed)
		{
			motor1s = myMinSpeed;
		}
		if (motor2s <= myMinSpeed)
		{
			motor2s = myMinSpeed;
		}
		if (motor3s <= myMinSpeed)
		{
			motor3s = myMinSpeed;
		}
		if (motor4s <= myMinSpeed)
		{
			motor4s = myMinSpeed;
		}

		// Check if motors are going to be set to higher than the maximum
		if(motor1s >= myMaxSpeed)
		{
			motor1s = myMaxSpeed;
		}
		if(motor2s >= myMaxSpeed)
		{
			motor2s = myMaxSpeed;
		}
		if(motor3s >= myMaxSpeed)
		{
			motor3s = myMaxSpeed;
		}
		if(motor4s >= myMaxSpeed)
		{
			motor4s = myMaxSpeed;
		}


		motor1.write(motor1s);
		motor2.write(motor2s);
		motor3.write(motor3s);
		motor4.write(motor4s);

};

void Quadcopter :: IIRF(double NEW_DATA[][10], double FILTERED_DATA[][10], int my_update)
{
	int index1 = (my_update + 1)- (((my_update + 0)/4)*5);
	int index2 = (my_update + 2)- (((my_update + 1)/4)*5);
	int index3 = (my_update + 3)- (((my_update + 2)/4)*5);
	int index4 = (my_update + 4)- (((my_update + 3)/4)*5);

	for (int i = 0; i < 10; i++)
	{
		FILTERED_DATA [my_update][i] =
		(1/_a0) * ((_b0 * NEW_DATA[my_update][i]) +
        (_b1 * NEW_DATA     [index1][i]) + (_b2 * NEW_DATA     [index2][i]) + (_b3 * NEW_DATA     [index3][i]) + (_b4 * NEW_DATA     [index4][i]) -
        (_a1 * FILTERED_DATA[index1][i]) - (_a2 * FILTERED_DATA[index2][i]) - (_a3 * FILTERED_DATA[index3][i]) - (_a4 * FILTERED_DATA[index4][i]) );
	}
};


void Quadcopter::ERROR_LED(int LED_SEL)
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
