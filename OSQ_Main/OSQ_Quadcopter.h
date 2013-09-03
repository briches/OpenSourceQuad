/*=====================================================================
	OSQ_Quadcopter library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library is designed to abstract away some of the craft management functions
	from the main file (OSQ_Main.ino)

	Copyright (C) 2013  Brandon Riches

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	-----------------------------------------------------------------------------*/
#ifndef OSQ_QUADCOPTER_H_INCLUDED
#define OSQ_QUADCOPTER_H_INCLUDED


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif



#include <I2C.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

#include "OSQ_BMP085.h"

/*=========================================================================
    General IO pins
    -----------------------------------------------------------------------*/
#define GREEN_LED 	27
#define RED_LED 	25
#define YELLOW_LED 	23


/*=========================================================================
    Polling Rates.
    -----------------------------------------------------------------------*/

#define _100HzPoll 		(10000)				// us Period of 100 Hz poll
#define	_50HzPoll		(20000)				// us Period of 75 Hz poll
#define _20HzPoll 		(50000)				// us Period of 50 Hz poll
#define _1HzPoll		(1000000)			// us Period of 10 Hz poll

/*===============================================
Time keeping for polling and interrupts
-----------------------------------------------------------------------*/
static double tpoll1;
static double tpoll2;
static double tpoll3;
static double tpoll4;

/***************************************************************************
 *! @FUNCTIONS
 ***************************************************************************/

void ERROR_LED(int LED_SEL);

void getInitialOffsets( struct kinematicData *kinematics,
						SENSORLIB_accel accel,
						SENSORLIB_mag mag,
						SENSORLIB_gyro gyro)
{
	/**************************************************************************/
		//! @brief Gets the initial offsets in both sensors to accomodate board mount
    /**************************************************************************/
	ERROR_LED(2);										// Warning LED
    int offset_counter = 100;                       // # of data sets to consider when finding offsets
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

        sensors_event_t gyro_event;
        gyro.getEvent(&gyro_event);
        gyrodata[0] = gyro_event.gyro.x;
        gyrodata[1] = gyro_event.gyro.y;
        gyrodata[2] = gyro_event.gyro.z;

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
				SENSORLIB_gyro gyro,
				struct kinematicData *kinematics)
{
	/**************************************************************************/
		//! @brief Initializes the various sensors and instruments
	/**************************************************************************/
	ERROR_LED(2);												// Warning LED

	// Same as the gyro initialization, but the accel isnt an ass
	gyro.begin();
	accel.begin();
	mag.begin();


    getInitialOffsets(kinematics,
						accel,
						mag,
						gyro);

    return true;
};


void mainProcess(	double pitchPID_out,
					double rollPID_out,
					class SENSORLIB_accel *accel,
					class SENSORLIB_mag	*mag,
					class SENSORLIB_gyro *gyro,
                                        class BMP085    *barometer,
					struct kinematicData *kinematics,
					struct fourthOrderData *fourthOrderXAXIS,
					struct fourthOrderData *fourthOrderYAXIS,
					struct fourthOrderData *fourthOrderZAXIS,
					class OSQ_MotorControl *motorControl)
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
	if (time2 >=  _50HzPoll)
	{
		// Mag, USRF.
		priority = 2;

	}
	if (time3 >= _20HzPoll)
	{
		priority = 3;								// Apply error function output to motor speeds


	}

	if (time4 >= _1HzPoll)
	{
		priority = 4;								// GPS update


	}


	/**!					@Poll type one 				*/
	// Code in this block executes if any of the poll conditions are satisfied
	if ((priority == 1) || (priority == 2) || (priority == 3) || (priority == 4) )
	{

		kinematicEvent(	0,
						kinematics,
						accel,
						mag,
						gyro,
						fourthOrderXAXIS,
						fourthOrderYAXIS,
						fourthOrderZAXIS);

		double x = 0;
		// Motor Logic
		motorControl->updateMotors(pitchPID_out,  rollPID_out, x ,  x);

		// Read the time that poll1 was last executed.
		tpoll1 = micros();
	}


	/**! 				@Poll type two - 75 Hz			*/
	// Code in this block executes if the conditions for priority == 2 are satisfied.
	if ((priority == 1) || (priority == 2))
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
	if ((priority == 4) || (priority == 3))
	{
                barometer->updatePTA();

		// Read the time that poll3 was last executed
		tpoll3 = micros();
	}

	/**! 				@Poll type four - 10 Hz				*/
	// Code in this block executes if the conditions for priority == 2 are satisfied.
	if (priority == 4)
	{	
		/// Read battery voltage
		// TODO:
		tpoll4 = micros();
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


#endif // OSQ_QUADCOPTER_H_INCLUDED
