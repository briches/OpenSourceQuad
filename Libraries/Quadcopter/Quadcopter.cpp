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
#include <Wire.h>
#include <I2c.h>




void Quadcopter :: get_Initial_Offsets()
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

        delay(1);
    }

    io_ax /= offset_counter;
	io_ay /= offset_counter;
	io_az /= offset_counter;
	io_wx /= offset_counter;
	io_wy /= offset_counter;
	io_wz /= offset_counter;



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

	// Initialize the fourth order struct
	setupFourthOrder();

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
	accel.begin();
	mag.begin();

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

    motor1s = MIN_PULSE_WIDTH;            	// Initialize all motor speeds to 0% speed
    motor2s = MIN_PULSE_WIDTH;          	// This might be useful, later
    motor3s = MIN_PULSE_WIDTH;
    motor4s = MIN_PULSE_WIDTH;

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
	double 		alpha_accel,
					beta_accel,
					heading_mag;
	int 			poll_type = 0;
	double   		time,
					t_convert = 1000000,
					time1,
					time2,
					time3,
					time4;
	double 		a_gcoeff = 0.95,
					h_gcoeff = 0.99;
	sensors_event_t	accel_event,
					mag_event;

	time = micros();

	time1 = time - tpoll1;
	time2 = time - tpoll2;
	time3 = time - tpoll3;
	time4 = time - tpoll4;

	// Check the type of interrupt. If the time elapsed is less than the interrupt latency for a
	// certain device, it doesn't need to be updated.
	// The updates and calculations for those devices are wrapped in if statements that check
	// the value of poll_type, and thus determine if data is available.
	if (time1 >= _100HzPoll) /*microseconds*/
	{
		poll_type = 1;								// Only essential updates are needed.
													// update PID controllers based on accel/gyro data,

	}
	if (time2 >=  _75HzPoll)
	{
		poll_type = 2;								// USRF update


	}
	if (time3 >= _20HzPoll)
	{
		poll_type = 3;								// Apply error function output to motor speeds


	}

	if (time4 >= _10HzPoll)
	{
		poll_type = 4;								// GPS update


	}

	/**!					@Poll type one 				*/
	// Code in this block executes if any of the poll conditions are satisfied
	if (poll_type > 0)
	{
		// update the registers storing data INSIDE the sensors
		accel.getEvent(&accel_event);
		gyro.update();

		/// Store Raw values from the sensor registers, and remove initial offsets
		ax = accel_event.acceleration.x - io_ax;
		ay = accel_event.acceleration.y - io_ay;
		az = accel_event.acceleration.z - io_az + SENSORS_GRAVITY_STANDARD;



		/**! 				@Poll type two - 75 Hz			*/
		// Code in this block executes if the conditions for poll_type == 2 are satisfied.
		if (poll_type > 1)
		{
			/// The fastest we can update the magnetometer is 75Hz
			mag.getEvent(&mag_event);

			mx[1] = mx[0];
			my[1] = my[0];
			mz[1] = mz[0];

			mx[0] = mag_event.magnetic.x;
			my[0] = mag_event.magnetic.y;
			mz[0] = mag_event.magnetic.z;

			// Read the time that poll2 was last executed
			tpoll2 = micros();
		}
		else
		{
			mx[0] = mx[1];
			my[0] = my[1];
			my[0] = my[1];
		}


		wx =  -(gyro.x() - io_wx);//- time * GYRO_DRIFT_RATE_X;
		wy =  -(gyro.y() - io_wy);//- time * GYRO_DRIFT_RATE_Y;
		wz = 	gyro.z() - io_wz;//- time * GYRO_DRIFT_RATE_Z;

		/// Convert to SI units from raw data type in gyro data
		SI_convert();

		/// Runs a Chebyshev 4th order filter
		ax = computeFourthOrder(ax, &fourthOrderXAXIS);
		ay = computeFourthOrder(ay, &fourthOrderYAXIS);
		az = computeFourthOrder(az, &fourthOrderZAXIS);

		time = (micros() - tpoll1)/t_convert; // This is the elapsed time since poll1 ran

		/// my_update pitch and roll
		// Time integration of wy gets rotation about y
		alpha_gyro += wy * time;
		beta_gyro += wx * time;

		// This is my feeble first attempt to deal with NaN errors resulting from the vector calculations
		// done below. Situations were arising where ax, ay, az were all > 0 and az was > 9.81. This screwed
		// up the trig, giving the acos of a value > 1.
		if (az >= SENSORS_GRAVITY_STANDARD)
		{
			double foobar = pow(SENSORS_GRAVITY_STANDARD,2) - pow(ax,2) - pow(ay,2);
			az = sqrt(foobar);
		}

		// Beta is analogous to ROLL, or rotating about the X-AXIS
		beta_accel = asin(ay/SENSORS_GRAVITY_STANDARD)*180/Pi;

		// Alpha is analogous to PITCH, or rotating about the Y-AXIS
		alpha_accel = acos( az / ( SENSORS_GRAVITY_STANDARD * cos(  asin(ay/SENSORS_GRAVITY_STANDARD)  ) ) )* 180/Pi;

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
		alpha 	= a_gcoeff * alpha_gyro +   (1-a_gcoeff) *	alpha_accel;
		beta 	= a_gcoeff * beta_gyro 	+  	(1-a_gcoeff) *	beta_accel;

		/// my_update heading
		// Get the heading (in degrees) from the magnetometer.
		heading_mag = ((atan2(my[0],mx[0]))*180)/Pi;

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

		// Read the time that poll1 was last executed.
		tpoll1 = micros();
	}


	/**! 				@Poll type three - 20 Hz			*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type > 2)
	{
		/// Motor Logic takes place at a lower frequency.
		// ESCs need several periods of signal to properly read motor speed,
		// thus placing a upper limit on the frequency with which the ESCs can
		// be my_updated.

		updateMotors(aPID_out,  bPID_out);

		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}

	/**! 				@Poll type four - 10 Hz				*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type > 3)
	{
		/// Update GPS data using RMC


		/// Read battery voltage
		vbatt = (double)analogRead(15);
		vbatt *= 11;
		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}


};

float Quadcopter::computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters)
{
  // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567

  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809

  float output;

  output = _b0 * currentInput                +
           _b1 * filterParameters->inputTm1  +
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;

  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;

  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;

  return output;
};


void Quadcopter::setupFourthOrder()
{
  fourthOrderXAXIS.inputTm1 = 0.0;
  fourthOrderXAXIS.inputTm2 = 0.0;
  fourthOrderXAXIS.inputTm3 = 0.0;
  fourthOrderXAXIS.inputTm4 = 0.0;

  fourthOrderXAXIS.outputTm1 = 0.0;
  fourthOrderXAXIS.outputTm2 = 0.0;
  fourthOrderXAXIS.outputTm3 = 0.0;
  fourthOrderXAXIS.outputTm4 = 0.0;

  //////////
  fourthOrderYAXIS.inputTm1 = 0.0;
  fourthOrderYAXIS.inputTm2 = 0.0;
  fourthOrderYAXIS.inputTm3 = 0.0;
  fourthOrderYAXIS.inputTm4 = 0.0;

  fourthOrderYAXIS.outputTm1 = 0.0;
  fourthOrderYAXIS.outputTm2 = 0.0;
  fourthOrderYAXIS.outputTm3 = 0.0;
  fourthOrderYAXIS.outputTm4 = 0.0;

  //////////
  fourthOrderZAXIS.inputTm1 = -9.8065;
  fourthOrderZAXIS.inputTm2 = -9.8065;
  fourthOrderZAXIS.inputTm3 = -9.8065;
  fourthOrderZAXIS.inputTm4 = -9.8065;

  fourthOrderZAXIS.outputTm1 = -9.8065;
  fourthOrderZAXIS.outputTm2 = -9.8065;
  fourthOrderZAXIS.outputTm3 = -9.8065;
  fourthOrderZAXIS.outputTm4 = -9.8065;
};

bool Quadcopter::updateMotors(double aPID_out, double bPID_out)
{

		int myMinSpeed = 1200;
		int myMaxSpeed = 1500;

		aPID_out = -aPID_out;
		bPID_out = -bPID_out;

		motor1s +=  aPID_out;
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
