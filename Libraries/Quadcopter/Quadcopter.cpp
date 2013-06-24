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
#include <MMA8453_n0m1.h>
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
void get_Initial_Offsets()
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

        QV.io_ax = (QV.io_ax + acceldata[0] ); // Sum
        QV.io_ay = (QV.io_ay + acceldata[1] );
        QV.io_az = (QV.io_az + acceldata[2] );
        QV.io_wx = (QV.io_wx + gyrodata[0] );
        QV.io_wy = (QV.io_wy + gyrodata[1] );
        QV.io_wz = (QV.io_wz + gyrodata[2] );

        counter = counter + 1 ;
    }

    QV.io_ax /= offset_counter;
	QV.io_ay /= offset_counter;
	QV.io_az /= offset_counter;
	QV.io_wx /= offset_counter;
	QV.io_wy /= offset_counter;
	QV.io_wz /= offset_counter;

    Serial.println(QV.io_ax);
    Serial.println(QV.io_ay);
    Serial.println(QV.io_az);
    Serial.println(QV.io_wx);
    Serial.println(QV.io_wy);
    Serial.println(QV.io_wz);
};

/**************************************************************************/
/*!
    @brief Converts the raw data from sensors to SI units
*/
/**************************************************************************/
void SI_convert()
{
  //Convert accelerometer readouts to m/s^2
    switch(g_ScaleRange) {
        case FULL_SCALE_RANGE_2g:
            QV.ax = QV.ax * SI_CONVERT_2g;
            QV.ay = QV.ay * SI_CONVERT_2g;
            QV.az = QV.az * SI_CONVERT_2g;
            break;

        case FULL_SCALE_RANGE_4g:
            QV.ax = QV.ax * SI_CONVERT_4g;
            QV.ay = QV.ay * SI_CONVERT_4g;
            QV.az = QV.az * SI_CONVERT_4g;
            break;

        case FULL_SCALE_RANGE_8g:
            QV.ax = QV.ax * SI_CONVERT_8g;
            QV.ay = QV.ay * SI_CONVERT_8g;
            QV.az = QV.az * SI_CONVERT_8g;
            break;
    }
    // Convert gyro readouts to degrees/s
    switch(d_ScaleRange) {

        case FULL_SCALE_RANGE_250:
            QV.wx = QV.wx * SI_CONVERT_250;
            QV.wy = QV.wy * SI_CONVERT_250;
            QV.wz = QV.wz * SI_CONVERT_250;
            break;

        case FULL_SCALE_RANGE_500:
            QV.wx = QV.wx * SI_CONVERT_500;
            QV.wy = QV.wy * SI_CONVERT_500;
            QV.wz = QV.wz * SI_CONVERT_500;
            break;

        case FULL_SCALE_RANGE_1000:
            QV.wx = QV.wx * SI_CONVERT_1000;
            QV.wy = QV.wy * SI_CONVERT_1000;
            QV.wz = QV.wz * SI_CONVERT_1000;
            break;

        case FULL_SCALE_RANGE_2000:
            QV.wx = QV.wx * SI_CONVERT_2000;
            QV.wy = QV.wy * SI_CONVERT_2000;
            QV.wz = QV.wz * SI_CONVERT_2000;
            break;
    }
};

/**************************************************************************/
/*!
    @brief Initializes the various sensors and instruments
*/
/**************************************************************************/
bool initSensor()
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

    Serial.print("Init accel ");                    // Same as the gyro initialization, but the accel isnt an ass
    accel.setI2CAddr(Accel_Address);                           // See the data sheet for the MMA8452Q Accelerometer registers
    accel.dataMode(HighDef, g_ScaleRange);   // http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8452Q.pdf?fpsp=1
	Serial.println("Done!");

	Serial.print("get init offs ");
    get_Initial_Offsets();                                     // Initial offsets private function in Control class
    Serial.println("Done!");

    return true;
};

/**************************************************************************/
/*!
    @brief Initializes the motors, code from Andrew's script
*/
/**************************************************************************/
bool initMotors()
{
    QV.motor1s = 0;                         // Initialize all motor speeds to 0
    QV.motor2s = 0;                         // This might be useful, later
    QV.motor3s = 0;
    QV.motor4s = 0;

    motor1.attach(11);                      // Attach motor 1 to D11
    motor2.attach(10);                      // Attach motor 2 to D10
    motor3.attach(9);                       // Attach motor 3 to D9
    motor4.attach(6);                       // Attach motor 4 to D8

    // initializes motor1
    for(QV.motor1s = 0; QV.motor1s < 60; QV.motor1s += 1)
    {
      motor1.write(QV.motor1s);
      motor2.write(QV.motor1s);
      motor3.write(QV.motor1s);
      motor4.write(QV.motor1s);

      QV.motor2s = QV.motor1s;
      QV.motor3s = QV.motor1s;
      QV.motor4s = QV.motor1s;

      delay(50);
    }

    return true;
}


/**************************************************************************/
/*!
    @brief Checks elapsed time and executes various tasks such as running PID controllers
*/
/**************************************************************************/
void update()
{
	int poll_type;

	// Check the type of interrupt. If the time elapsed is less than the interrupt latency for a
	// certain device, it doesn't need to be updated.
	// The updates and calculations for those devices are wrapped in if statements that check
	// the value of poll_type, and thus determine if data is available.
	if (micros() - QV.tpoll1 >= 10000) /*microseconds*/
	{
		poll_type = 1;								// Only essential updates are needed.
																// Update PID controllers based on accel/gyro/mag data,
																// and motor logic
	}
	/* Important note about this interrupt: This interrupt may or may not be necessary, but
	is here for future proofing */
	if (micros() - QV.tpoll2 >=  poll2_interrupt)
	{
		poll_type = 2;								// Essential updates + USRF updates needed
																// Update PID contollers based on accel/gyro/mag data,
																// motor logic, other sensors that fit this interrupt as decided.
	}
	if (micros() - QV.tpoll3 >= poll3_interrupt)
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

		QV.ax = accel.x() - QV.io_ax;                         // Store Raw values from the sensor registers
		QV.ay = accel.y() - QV.io_ay;                         // and removes the initial offsets
		QV.az = accel.z() - QV.io_az;
		QV.wx = gyro.y() - QV.io_wy;
		QV.wy = gyro.x() - QV.io_wx;
		QV.wz = gyro.z() - QV.io_wz;

		SI_convert();                                           // Convert to SI units from raw data type

		if(fabs(QV.ax) < g_threshold) {QV.ax = 0;}     // Check if the data is less than the threshold
		if(fabs(QV.ay) < g_threshold) {QV.ay = 0;}
		if(fabs(QV.az) < g_threshold) {QV.az = 0;}
		if(fabs(QV.wx) < d_threshold) {QV.wx = 0;}
		if(fabs(QV.wy) < d_threshold) {QV.wy = 0;}
		if(fabs(QV.wz) < d_threshold) {QV.wz = 0;}

		mov_avg();

		QV.alpha = atan2(QV.ax, QV.az) *180/Pi ; // Arctan of the two values returns the angle,
		QV.beta = atan2(QV.ay, QV.az) *180/Pi;   // in rads, and convert to degrees

	}

	/**! 				@Poll type two 				*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type == 2)
	{
		QV.tpoll2 = micros();
	}

	/**! 				@Poll type three 				*/
	// Code in this block executes if the conditions for poll_type == 2 are satisfied.
	if (poll_type == 3)
	{
		QV.tpoll3 = micros();
	}


// Chebyshev IIR low pass filter formula
// incl. here, maybe

};


void mov_avg()
{
			// Updates the prev_data by bumping up each data set
		for (int i = 35; i <= 41; i ++)
		{
			QV.prev_data[i+7] = QV.prev_data[i];
		}
		for (int i = 28; i <=34; i++)
		{
			QV.prev_data[i+7] = QV.prev_data[i];
		}
		for (int i = 21; i <=27; i++)
		{
			QV.prev_data[i+7] = QV.prev_data[i];
		}
		for (int i = 14; i <=20; i++)
		{
			QV.prev_data[i+7] = QV.prev_data[i];
		}
		for (int i = 7; i <=13; i++)
		{
			QV.prev_data[i+7] = QV.prev_data[i];
		}
		for (int i = 0; i <=6; i++)
		{
			QV.prev_data[i+7] = QV.prev_data[i];
		}

		// Add new data sets to the first spot in the buffer
		QV.prev_data[0] = QV.ax;
		QV.prev_data[1] = QV.ay;
		QV.prev_data[2] = QV.az;
		QV.prev_data[3] = QV.wx;
		QV.prev_data[4] = QV.wy;
		QV.prev_data[5] = QV.wz;

		// Buffer is updated, now a moving average can be calculated,  If the buffer size has to be increased then we can do that.
		QV.ax = 0; QV.ay = 0; QV.az = 0; QV.wx = 0; QV.wy = 0; QV.wz = 0; QV.elev = 0; // ready for average calculation

		for (int i = 0; i <= 42; i += 7) 	{ QV.ax += QV.prev_data[i];}
		for (int i = 1; i <= 43; i += 7) 	{ QV.ay += QV.prev_data[i];}
		for (int i = 2; i <= 44; i += 7) 	{ QV.az += QV.prev_data[i];}
		for (int i = 3; i <= 45; i += 7) 	{ QV.wx += QV.prev_data[i];}
		for (int i = 4; i <= 46; i += 7) 	{ QV.wy += QV.prev_data[i];}
		for (int i = 5; i <= 47; i += 7) 	{ QV.wz += QV.prev_data[i];}
		for (int i = 6; i <= 48; i += 7) 	{ QV.elev += QV.prev_data[i];}

		QV.ax /= 7;						// Finish off the calculation
		QV.ay /= 7;
		QV.az /= 7;
		QV.wx /= 7;
		QV.wy /= 7;
		QV.wz /= 7;
		QV.elev /= 7;
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

