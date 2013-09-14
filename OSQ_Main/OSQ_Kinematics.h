/*=====================================================================
 	OSQ_Kinematics library
 	OpenSourceQuad
 	-------------------------------------------------------------------*/
/*================================================================================
 
 	Author		: Brandon Riches
 	Date		: August 2013
 	License		: GNU Public License
 
 	This library implements sensor measuments to update the measured state
 	of the craft.
 
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

#ifndef OSQ_KINEMATICS_H_INCLUDED
#define OSQ_KINEMATICS_H_INCLUDED

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "OSQ_SENSORLIB.h"
#include "OSQ_Motors.h"
#include <Wire.h>
#include <I2C.h>

bool startup = true;

#define Pi  			(3.14159265359F)	// Its pi.

#define ORDER 4
// Filter uses n = 4, r = 60, Wc = 12.5/50


struct fourthOrderData
{
        double  inputTm1,  inputTm2,  inputTm3,  inputTm4;
        double outputTm1, outputTm2, outputTm3, outputTm4;
};

fourthOrderData fourthOrderXAXIS,
fourthOrderYAXIS,
fourthOrderZAXIS;

/*=========================================================================
 Kinematics Data Type
 -----------------------------------------------------------------------*/
struct kinematicData
{
        double pitch,
        roll,
        yaw,
        phi,        // used for USRF altitude calcs

        altitude,

        io_ax,
        io_ay,
        io_az,
        io_wx,
        io_wy,
        io_wz,

        pitch_gyro,
        roll_gyro,
        yaw_gyro,

        yaw_mag;

        unsigned long timestamp;
};

kinematicData	  		kinematics;

#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)

double computeFourthOrder(double currentInput, struct fourthOrderData *filterParameters);
double normalize(double x, double y, double z);
void nan_quad_Check(double num1, double num2, double num3);
double complementary(double mynum, int select, double coeff);

// Kinematic events include yaw, pitch and roll calculations
// as well as altitudes
void kinematicEvent(int eventType, class SENSORLIB_accel *accel, class SENSORLIB_mag *mag, class SENSORLIB_gyro *gyro)
{

        sensors_event_t	accel_event, mag_event, gyro_event;

        double 	elapsed_time = 0,t_convert = 1000000;

        double 	pitch_accel, roll_accel, pitchRollCoeff = 0.9, yawCoeff = 0.9;


        if(eventType == 1)
        {
                mag->getEvent(&mag_event);
                
                double mx = mag_event.magnetic.z;
                double my = -(mag_event.magnetic.y);
                double mz = mag_event.magnetic.x;
                
                double alpha = -kinematics.pitch * Pi/180;
                double beta = -kinematics.roll * Pi/180;
                
                
                kinematics.yaw_mag = atan2(cos(beta)*my + sin(beta)*mz, cos(alpha)*mx + sin(alpha)*sin(beta)*my - sin(alpha)*cos(beta)*mz) * 180/Pi;
                
                if(startup)
                {
                        kinematics.yaw_gyro = kinematics.yaw_mag;        // Setup heading
                        startup = false;
                }
        }

        if(eventType == 0)
        {
                accel->getEvent(&accel_event);
                gyro->getEvent(&gyro_event);

                /// Store Raw values from the sensor registers, and remove initial offsets
                double ax = accel_event.acceleration.z - kinematics.io_az;
                double ay = -(accel_event.acceleration.y - kinematics.io_ay);
                double az = accel_event.acceleration.x - kinematics.io_ax + SENSORS_GRAVITY_STANDARD;
                
                double wx =  gyro_event.gyro.z - kinematics.io_wz;
                double wy =  gyro_event.gyro.x - kinematics.io_wx;
                double wz =  gyro_event.gyro.y - kinematics.io_wy; 
                
                // Compute a Chebyshev 4th order filter
                ax = computeFourthOrder(ax, &fourthOrderXAXIS);
                ay = computeFourthOrder(ay, &fourthOrderYAXIS);
                az = computeFourthOrder(az, &fourthOrderZAXIS);

                double norm = normalize(ax, ay, az);
                ax /= norm;
                ay /= norm;
                az /= norm;

                elapsed_time = (micros() - kinematics.timestamp) / t_convert;
                kinematics.timestamp = micros();

                kinematics.pitch_gyro    += wy * elapsed_time;
                kinematics.roll_gyro     += wx * elapsed_time;
                kinematics.yaw_gyro	    += wz * elapsed_time;

                kinematics.phi = atan2( sqrt(ax*ax + ay*ay), az) * 180 / Pi;
                pitch_accel = atan2( ax, sqrt(ay*ay + az*az)) * 180 / Pi;
                roll_accel = atan2( ay, sqrt(az*az + az*az)) * 180 / Pi;

                // Remove pesky NaNs that seem to occur around 0.
                // Check the quadrant of vector
                nan_quad_Check(pitch_accel, roll_accel, kinematics.yaw_mag);

                kinematics.pitch = complementary(pitch_accel, 0, pitchRollCoeff);
                kinematics.roll  = complementary(roll_accel, 1, pitchRollCoeff);
                kinematics.yaw   = complementary(kinematics.yaw_mag, 2, yawCoeff);
                }
};

double complementary(double mynum, int select, double coeff)
{
        if (select == 0)
        {
                return coeff * kinematics.pitch_gyro 	+ (1 - coeff) * mynum;
        }
        if (select == 1)
        {
                return coeff * kinematics.roll_gyro 	+ (1 - coeff) * mynum;
        }
        if (select == 2)
        {
                return coeff * kinematics.yaw_gyro	+ (1 - coeff) * mynum;
        }
};

double computeFourthOrder(	double currentInput,
struct fourthOrderData *filterParameters)
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

        double output;

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

void setupFourthOrder()
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

double normalize(double x, double y, double z)
{
        return sqrt( x * x + y * y + z * z );
};

void nan_quad_Check(double num1, double num2, double num3)
{
        double *pnum1 = &num1;
        double *pnum2 = &num2;
        double *pnum3 = &num3;

        if(isnan(num1))
        {
                *pnum1 = 0;
        }
        if(isnan(num2))
        {
                *pnum2 = 0;
        }
        // Check quadrants
        if (num2 < 0)
        {
                *pnum1 = - num1;
        }
        if (num2 < 0)
        {
                *pnum2 	= - num2;
        }
        if (num3 < 0)
        {
                *pnum3 += 360;
        }
};


#endif // KINEMATICS_H_INCLUDED

