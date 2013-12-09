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
#include <SD.h>

bool startup = true;

#define Pi        (3.14159265359F)	// Its pi.

#define ORDER 4
// Filter uses n = 4, r = 60, Wc = 12.5/50

/*=========================================================================
 Function declarations
 -----------------------------------------------------------------------*/
double computeFourthOrder(double currentInput, struct fourthOrderData *filterParameters);
void setupFourthOrder();
void complementaryFilter(double ax, double  ay, double  az,  double wx, double  wy,  double wz, double elapsedTime, struct kinematicData *kinData);


/*=========================================================================
 Filter Data Type
 -----------------------------------------------------------------------*/
struct fourthOrderData
{
        double  inputTm1,  inputTm2,  inputTm3,  inputTm4;
        double outputTm1, outputTm2, outputTm3, outputTm4;
};

fourthOrderData fourthOrderXAXIS, fourthOrderYAXIS, fourthOrderZAXIS;

/*=========================================================================
 Kinematics Data Type
 -----------------------------------------------------------------------*/
struct kinematicData
{
        double pitch,
        roll,
        yaw,
        phi,        // used for USRF altitude calcs
        
        pitchRate,
        rollRate,
        yawRate,
        
        lastPitch,
        lastRoll,
        lastYaw,
        
        altitude,

        io_ax,
        io_ay,
        io_az,
        io_wx,
        io_wy,
        io_wz,

        yaw_mag;

        unsigned long timestamp;
};

kinematicData	  kinematics;

#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)

// Kinematic events include yaw, pitch and roll calculations
// as well as altitudes
void kinematicEvent(int eventType, class SENSORLIB_accel *accel, class SENSORLIB_mag *mag, class SENSORLIB_gyro *gyro)
{

        sensors_event_t	accel_event, mag_event, gyro_event;

        double 	elapsedTime = 0, t_convert = 1000000;

        double 	pitch_accel, roll_accel, pitchRollCoeff = 0.5, yawCoeff = 0.9;


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
                        kinematics.yaw = kinematics.yaw_mag;        // Setup heading
                        startup = false;
                }
        }

        if(eventType == 0)
        {
                accel->getEvent(&accel_event);
                gyro->getEvent(&gyro_event);
                
                kinematics.lastPitch = kinematics.pitch;
                kinematics.lastRoll = kinematics.roll;
                kinematics.lastYaw = kinematics.yaw;

                /// Store Raw values from the sensor registers, and remove initial offsets
                double ax = accel_event.acceleration.z - kinematics.io_az;
                double ay = -(accel_event.acceleration.y - kinematics.io_ay);
                double az = accel_event.acceleration.x - kinematics.io_ax + SENSORS_GRAVITY_STANDARD;

                double wx =  (gyro_event.gyro.z - kinematics.io_wz);
                double wy =  gyro_event.gyro.x - kinematics.io_wx;
                double wz =  gyro_event.gyro.y - kinematics.io_wy;

                // Compute a Chebyshev 4th order filter
                ax = computeFourthOrder(ax, &fourthOrderXAXIS);
                ay = computeFourthOrder(ay, &fourthOrderYAXIS);
                az = computeFourthOrder(az, &fourthOrderZAXIS);
                
                elapsedTime = (micros() - kinematics.timestamp) / t_convert;
                kinematics.timestamp = micros();

                kinematics.phi = atan2( sqrt(ax*ax + ay*ay), az) * 180 / Pi;
                
                // Complementary filter
                // Calculates current angles, corrects a bit for gyro drift, if any.
                complementaryFilter(ax, ay, az, wx, wy, wz, elapsedTime, &kinematics);
                
                // Calculate time derivative of attitudes
                kinematics.pitchRate = wy;
                kinematics.rollRate = wx;
                kinematics.yawRate = wz;
                
        }
};

void complementaryFilter(double ax, double  ay, double  az,  double wx, double  wy,  double wz, double elapsedTime, struct kinematicData *kinData)
{
        // Filter parameter
        double beta = 1;
        
        // Gyroscope 
        kinData->pitch += wy * elapsedTime;
        kinData->roll += wx * elapsedTime;
        kinData->yaw += wz * elapsedTime;
        
        // Magnetometer complementary
        kinData->yaw = kinData->yaw * 0.95 + kinData->yaw_mag * 0.05;
        
        // Compensate for gyro drift, if the accel isnt completely garbage
        double magnitudeApprox = sqrt(abs(ax)*abs(ax) + abs(ay)*abs(ay) + abs(az)*abs(az));
        if(magnitudeApprox > 9.31 && magnitudeApprox < 10.51)
        {
                double pitchAcc = atan2(ax, az) * 180/ Pi;
                kinData->pitch = kinData->pitch * beta + (1-beta) * pitchAcc;
                
                double rollAcc = atan2(ay, az) * 180 / Pi;
                kinData->roll = kinData->roll * 0.98 + 0.02 * rollAcc;
        }
};

double computeFourthOrder(double currentInput, struct fourthOrderData *filterParameters)
{
        // cheby2(4,60,12.5/50)
//#define _b0  0.001893594048567
//#define _b1 -0.002220262954039
//#define _b2  0.003389066536478
//#define _b3 -0.002220262954039
//#define _b4  0.001893594048567
//
//#define _a1 -3.362256889209355
//#define _a2  4.282608240117919
//#define _a3 -2.444765517272841
//#define _a4  0.527149895089809

// Cheby2(4,60,5/50);
#define _b0  0.001066578484441
#define _b1 -0.003520583754742
#define _b2  0.004979264107821
#define _b3 -0.003520583754742
#define _b4  0.001066578484441

#define _a1 -3.754902351869990
#define _a2  5.294313666885127
#define _a3 -3.321856314439796
#define _a4  0.782516252991879

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
        fourthOrderZAXIS.inputTm1 = 9.8065;
        fourthOrderZAXIS.inputTm2 = 9.8065;
        fourthOrderZAXIS.inputTm3 = 9.8065;
        fourthOrderZAXIS.inputTm4 = 9.8065;

        fourthOrderZAXIS.outputTm1 = 9.8065;
        fourthOrderZAXIS.outputTm2 = 9.8065;
        fourthOrderZAXIS.outputTm3 = 9.8065;
        fourthOrderZAXIS.outputTm4 = 9.8065;
};


#endif // KINEMATICS_H_INCLUDED


