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
	ERROR_LED(2);										// Warning LED
    int offset_counter = 10;                       // # of data sets to consider when finding offsets
    int counter = 1;
    double acceldata[3];
    double gyrodata[3];

    while(counter <= offset_counter)
    {
        accelmag.update();                            // Updates the accelerometer registers
        acceldata[0] = accelmag.ax();
        acceldata[1] = accelmag.ay();
        acceldata[2] = accelmag.az();

        gyro.update();                             // Updates the gyro output registers
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

	io_az -= 256;

	Serial.println(" ");
    Serial.println(io_ax);
    Serial.println(io_ay);
    Serial.println(io_az);
    Serial.println(io_wx);
    Serial.println(io_wy);
    Serial.println(io_wz);
    ERROR_LED(1); 					// Success LED
};

/**************************************************************************/
/*!
    @brief Converts the raw data from sensors to SI units
*/
/**************************************************************************/
void Quadcopter :: SI_convert()
{
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
    accelmag.Easy_Start();

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

/**************************************************************************/
/*!
    @brief Initializes the motors, code from Andrew's script
*/
/**************************************************************************/
bool Quadcopter::initMotors(int speed)
{
	unsigned long t1, t2;
	double elaps;

	t1 = micros();
	Serial.println("Initializing motors...");

    motor1s = 0;                         // Initialize all motor speeds to 0% speed
    motor2s = 0;                         // This might be useful, later
    motor3s = 0;
    motor4s = 0;

    motor1.attach(2);                      // Attach motor 1 to D2
    motor2.attach(3);                      // Attach motor 2 to D3
    motor3.attach(4);                       // Attach motor 3 to D4
    motor4.attach(5);                       // Attach motor 4 to D5

    // initializes motor1
    int m1s = map(speed, 0, 100, 0, 180);

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


/**************************************************************************/
/*!
    @brief Checks elapsed time and executes various tasks such as running PID controllers
*/
/**************************************************************************/
void Quadcopter::update(double aPID_out, double bPID_out)
{
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
	// certain device, it doesn't need to be updated.
	// The updates and calculations for those devices are wrapped in if statements that check
	// the value of poll_type, and thus determine if data is available.
	if (time1 >= 10000) /*microseconds*/
	{
		poll_type = 1;								// Only essential updates are needed.
																// Update PID controllers based on accel/gyro/mag data,

	}
	/* Important note about this interrupt: This interrupt may or may not be necessary, but
	is here for future proofing */
	if (time2 >=  poll2_interrupt)
	{
		poll_type = 2;								// Essential updates + USRF updates needed
																// Update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}
	if (time3 >= poll3_interrupt)
	{
		poll_type = 3;								// Essential updates + USRF + GPS updates needed
																// Update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}

	/**!					@Poll type one 				*/
	// Code in this block executes if any of the poll conditions are satisfied
	if (poll_type == 1 || poll_type == 2 || poll_type ==3 )
	{
		// Update the registers storing data INSIDE the sensors
		accelmag.update();
		gyro.update();

		// Store Raw values from the sensor registers, and remove initial offsets
		ax = accelmag.ax() - io_ax;
		ay = accelmag.ay() - io_ay;
		az = (accelmag.az() - io_az)+256;
		mx = accelmag.mx();
		my = accelmag.my();
		mz = accelmag.mz();
		wx = -(gyro.x()- io_wx);
		wy = -(gyro.y() - io_wy);
		wz = gyro.z() - io_wz;

		// Convert to SI units from raw data type in gyro data
		SI_convert();

		// Runs a moving average with a circular buffer
		mov_avg();


		time = (micros() - tpoll1)/t_convert; // This is the elapsed time since poll1 ran

		/// Update pitch and roll
		// Time integration of wy gets rotation about y
		alpha_gyro += wy * time;
		beta_gyro += wx * time;

		// Arctan of the two values returns the angle,

		if (az >= 9.81)
		{
			double foobar = pow(9.81,2) - pow(ax,2) - pow(ay,2);
			az = sqrt(foobar);
		}

		beta_accel = asin(ay/9.81)*180/Pi; // Beta is analogous to ROLL, or rotating about the x-axis

		alpha_accel = -acos( az / ( 9.81 * cos(  asin(ay/9.81)  ) ) )* 180/Pi; // Alpha is analogous to PITCH, or rotating about the y-axis


		// Complementary filter
		alpha = a_gcoeff * alpha_gyro +   (1-a_gcoeff)*alpha_accel;
		beta = a_gcoeff * beta_gyro +  (1-a_gcoeff)*beta_accel;

		/// Update heading
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
		// be updated.
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

		int myMinSpeed = 60;
		int myMaxSpeed = 140;

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

void Quadcopter :: mov_avg()
{
    if (overwrite < DATA_POINTS && overwrite > -1)
    {
        //movave[10][10];
        /*
        0   1   2   3   4   5   6   7   8   9
        ax  ay  az  wx  wy  wz  mx  my  mz  ele
        */

        double ave[10] = {};
        double total = 0;

        movave[0][overwrite] = ax;   //ax
        movave[1][overwrite] = ay;   //ay
        movave[2][overwrite] = az;   //az
        movave[3][overwrite] = wx;   //wx
        movave[4][overwrite] = wy;   //wy
        movave[5][overwrite] = wz;   //wz
        movave[6][overwrite] = mx;   //mx
        movave[7][overwrite] = my;   //my
        movave[8][overwrite] = mz;   //mz
        movave[9][overwrite] = elev; //ele

        for (int i = 0; i < 10; i++)
        {
            total = 0;
            for (int n = 0; n < DATA_POINTS; n++)
            {
                total += movave[i][n];
            }
            ave[i] = total / DATA_POINTS;
        }

        ax = ave[0];
        ay = ave[1];
        az = ave[2];
        wx = ave[3];
        wy = ave[4];
        wz = ave[5];
        mx = ave[6];
        my = ave[7];
        mz = ave[8];
        elev = ave[9];

        if (overwrite > DATA_POINTS)
        {
            overwrite = 0;
        }
        else
        {
            overwrite++;
        }
    }
    else
    {
        //ERROR!!!!!
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
