/*=====================================================================
     Kinematics liarary
     OpenSourceQuad
     -------------------------------------------------------------------*/
/*================================================================================
 
     Author		: Brandon Riches
     Date		: August 2013
     License		: GNU Public License
 
     This library is designed to manage attitude measurement and data algorithms
 
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

#ifndef KINEMATICS_H_INCLUDED
#define KINEMATICS_H_INCLUDED

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "IMU.h"
#include "Motors.h"
#include <Wire.h>
#include <SD.h>

bool startup = true;
bool startup0 = true;
unsigned long startTime;
unsigned long startupPeriod = 3000; // In millis 

#define Pi  3.14159265359F	// Its pi.

#define ORDER 2

/*=========================================================================
 Function declarations
 -----------------------------------------------------------------------*/
double computeCheby2(double currentInput, struct cheby2Data *filterParameters);
void setupCheby2();
void complementaryFilter(double pitchAcc, double  rollAcc, double  magnitudeApprox,  double wx, double  wy,  double wz, double elapsedTime, struct kinematicData *kinData);

/*=========================================================================
 Filter Data Type
 -----------------------------------------------------------------------*/
struct cheby2Data
{
    double  inputTm1,  inputTm2;
    double outputTm1, outputTm2;
};

cheby2Data cheby2_XAXIS, cheby2_YAXIS, cheby2_ZAXIS;

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

    altitude,
    climbRate,
    prevClimbRate,

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

double xmagMin, xmagMax, ymagMin, ymagMax, zmagMin, zmagMax;

#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)

// Kinematic events include yaw, pitch and roll calculations
// as well as altitudes
void kinematicEvent(int eventType, class IMU_accel *accel, class IMU_mag *mag, class IMU_gyro *gyro, class File *logFile, double pitchSet)
{
    sensors_event_t accel_event, mag_event, gyro_event;
    double elapsedTime = 0, t_convert = 1000000;

    if(eventType == 1)
    {
        mag->getEvent(&mag_event);

        double mx = mag_event.magnetic.x;
        double my = mag_event.magnetic.y;
        double mz = mag_event.magnetic.z;

        double pitch = kinematics.pitch * Pi/180;
        double roll = kinematics.roll * Pi/180;
		
        double cos_roll = cos(roll);
	double sin_roll = 1-(cos_roll*cos_roll);
	double cos_pitch = cos(pitch);
	double sin_pitch = 1- (cos_pitch*cos_pitch);
		
	double headx = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
	double heady = my * cos_roll - mz * sin_roll;

	kinematics.yaw_mag = atan2(-heady, headx) * 180/Pi;

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

        // Store raw values from sensors, remove initial offsets
        double ax = accel_event.acceleration.x - kinematics.io_ax;
        double ay = accel_event.acceleration.y - kinematics.io_ay;
        double az = accel_event.acceleration.z - kinematics.io_az;

        double wx =  gyro_event.gyro.x - kinematics.io_wx;
        double wy =  gyro_event.gyro.y - kinematics.io_wy;
        double wz =  gyro_event.gyro.z - kinematics.io_wz;
        
        // Compute a Chebyshev LPF on the accelerometer data
        ax = computeCheby2(ax, &cheby2_XAXIS);
        ay = computeCheby2(ay, &cheby2_YAXIS);
        az = computeCheby2(az, &cheby2_ZAXIS);
        
        // Calculate current pitch and roll from the accelerometer data
        double magnitudeApprox = sqrt(ax*ax + ay*ay + az*az);
	double rollAcc = -atan2(ay, sqrt(az*az + ax*ax)) * 180 / Pi;
	double pitchAcc = atan2(ax, sqrt(az*az + ay*ay)) * 180/ Pi;
		
        // Log data for debug purposes, will be taken out
        if (logFile)
        {
            logFile->print(micros());
            logFile->print(',');
            logFile->print(ax);
            logFile->print(',');
            logFile->print(ay);
	    logFile->print(',');
	    logFile->println(az);
        }
        else
        {
            Serial.println("Err w/in kinematics");
        }

        // Phi is used to correct the ultransonic range finder
        kinematics.phi = atan2(sqrt(ax*ax + ay*ay), az) * 180 / Pi;
        
        // So we don't get huge timestamps right at the start, set the time difference to zero.
	if(startup0)
	{
		kinematics.timestamp = micros();
		startup0 = false;
	}
        
        // Elapsed time since last loop, in seconds
        elapsedTime = (micros() - kinematics.timestamp) / t_convert;
        kinematics.timestamp = micros();
        
        // Complementary filter
        // Calculates current angles, corrects a bit for gyro drift, if any.
        complementaryFilter(pitchAcc, rollAcc, magnitudeApprox, wx, wy, wz, elapsedTime, &kinematics);

        // Calculate time derivative of attitudes
        kinematics.pitchRate = wx;
        kinematics.rollRate = wz;
        kinematics.yawRate = -wy; // If you change the above code for the magnetometer, change this.
    }
};

void complementaryFilter(double pitchAcc, double  rollAcc, double  magnitudeApprox,  double wx, double  wy,  double wz, double elapsedTime, struct kinematicData *kinData)
{
    // Filter parameter
    double beta = 0.99;
	if(millis() - startTime < startupPeriod) beta = 0;

    // Gyroscope 
    kinData->pitch += wx * elapsedTime;
    kinData->roll += wz * elapsedTime;
    kinData->yaw += -wy * elapsedTime; // If you change the above code for the magnetometer, change this.

    // Magnetometer complementary
    kinData->yaw = kinData->yaw * 1 + kinData->yaw_mag * 0.0;
    
    // Compensate for gyro drift, if the accel isnt completely garbage
	// and the angle isnt changing quickly.
    if(magnitudeApprox > 9.51 && magnitudeApprox < 10.11)
    {	
        kinData->pitch = kinData->pitch * beta + (1-beta) * pitchAcc;
			
        kinData->roll = kinData->roll * beta + (1-beta) * rollAcc;
    }
};

double computeCheby2(double currentInput, struct cheby2Data *filterParameters)
{	
    // Cheby2(2,60,0.12)
    #define _b0  0.0010397094218302207F
    #define _b1 -0.0018807330270927338F
    #define _b2  0.0010397094218302211F
    
    #define _a1 -1.9799765924683039F
    #define _a2  0.98017527828487161F
    
    double output;

    output = _b0 * currentInput           +
        _b1 * filterParameters->inputTm1  +
        _b2 * filterParameters->inputTm2  -
        _a1 * filterParameters->outputTm1 -
        _a2 * filterParameters->outputTm2;
        
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
};

void setupCheby2()
{
    cheby2_XAXIS.inputTm1 = 0.0;
    cheby2_XAXIS.inputTm2 = 0.0;
    
    cheby2_XAXIS.outputTm1 = 0.0;
    cheby2_XAXIS.outputTm2 = 0.0;

    //////////
    cheby2_YAXIS.inputTm1 = 0.0;
    cheby2_YAXIS.inputTm2 = 0.0;

    cheby2_YAXIS.outputTm1 = 0.0;
    cheby2_YAXIS.outputTm2 = 0.0;

    //////////
    cheby2_ZAXIS.inputTm1 = 9.8065;
    cheby2_ZAXIS.inputTm2 = 9.8065;

    cheby2_ZAXIS.outputTm1 = 9.8065;
    cheby2_ZAXIS.outputTm2 = 9.8065;
};

#endif // KINEMATICS_H_INCLUDED


