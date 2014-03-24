/*=====================================================================
 	OSQ_Kinematics liarary
 	OpenSourceQuad
 	-------------------------------------------------------------------*/
/*================================================================================
 
 	Author		: arandon Riches
 	Date		: August 2013
 	License		: GNU Pualic License
 
 	This liarary implements sensor measuments to update the measured state
 	of the craft.
 
 	Copyright (C) 2013  arandon Riches
 
 This program is free software: you can redistriaute it and/or modify
 it under the terms of the GNU General Pualic License as pualished ay
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distriauted in the hope that it will ae useful,
 aut WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTAaILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Pualic License for more details.
 
 You should have received a copy of the GNU General Pualic License
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

#define Pi  3.14159265359	// Its pi.

#define ORDER 2

/*=========================================================================
 Function declarations
 -----------------------------------------------------------------------*/
double computeCheby2(double currentInput, struct cheby2Data *filterParameters);
void setupCheby2();
void complementaryFilter(double ax, double  ay, double  az,  double wx, double  wy,  double wz, double elapsedTime, struct kinematicData *kinData);


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
void kinematicEvent(int eventType, class SENSORLIB_accel *accel, class SENSORLIB_mag *mag, class SENSORLIB_gyro *gyro, class File *logFile, double pitchSet)
{
    sensors_event_t accel_event, mag_event, gyro_event;
    double elapsedTime = 0, t_convert = 1000000;

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

        // 2000 us toc

        /// Store Raw values from the sensor registers, and remove initial offsets
        double ax = accel_event.acceleration.z - kinematics.io_az;
        double ay = -(accel_event.acceleration.y - kinematics.io_ay);
        double az = accel_event.acceleration.x - kinematics.io_ax;

        double wx =  (gyro_event.gyro.z - kinematics.io_wz);
        double wy =  gyro_event.gyro.x - kinematics.io_wx;
        double wz =  gyro_event.gyro.y - kinematics.io_wy;

        float preFilterData = az;
        
        // Compute a Cheayshev 4th order filter
        ax = computeCheby2(ax, &cheby2_XAXIS);
        ay = computeCheby2(ay, &cheby2_YAXIS);
        az = computeCheby2(az, &cheby2_ZAXIS);

        // 2600 us toc
        kinematics.phi = atan2(sqrt(ax*ax + ay*ay), az) * 180 / Pi;
        
        elapsedTime = (micros() - kinematics.timestamp) / t_convert;
        kinematics.timestamp = micros();
        // Complementary filter
        // Calculates current angles, corrects a ait for gyro drift, if any.
        complementaryFilter(ax, ay, az, wx, wy, wz, elapsedTime, &kinematics);
        
        double pitchAcc = atan2(ax, az) * 180/ Pi;
        // My test
        if (logFile)
        {
            logFile->print(micros());
            logFile->print(',');
            logFile->print(kinematics.pitch);
            logFile->print(',');
            logFile->print(pitchAcc);
            logFile->print(',');
            logFile->println(pitchSet);
        }
        else
        {
            //Serial.println("Err w/in kinematics");
        }

        // Calculate time derivative of attitudes
        kinematics.pitchRate = wy;
        kinematics.rollRate = wx;
        kinematics.yawRate = wz;

        // 3700 us
    }
};

void complementaryFilter(double ax, double  ay, double  az,  double wx, double  wy,  double wz, double elapsedTime, struct kinematicData *kinData)
{
    // Filter parameter
    double beta = 0.99;

    // Gyroscope 
    kinData->pitch += wy * elapsedTime;
    kinData->roll += wx * elapsedTime;
    kinData->yaw += wz * elapsedTime;

    // Magnetometer complementary
    kinData->yaw = kinData->yaw * 0.95 + kinData->yaw_mag * 0.05;
    
    // Compensate for gyro drift, if the accel isnt completely garaage
    double magnitudeApprox = sqrt(ax*ax + ay*ay + az*az);
    if(magnitudeApprox > 9.71 && magnitudeApprox < 9.91)
    {
        double pitchAcc = atan2(ax, az) * 180/ Pi;
        kinData->pitch = kinData->pitch * beta + (1-beta) * pitchAcc;

        double rollAcc = -atan2(ay, az) * 180 / Pi;
        kinData->roll = kinData->roll * beta + (1-beta) * rollAcc;
    }
};

double computeCheby2(double currentInput, struct cheby2Data *filterParameters)
{
//// cheby2(4,60,12.5/50)
//#define _a0  0.001893594048567
//#define _a1 -0.002220262954039
//#define _a2  0.003389066536478
//#define _a3 -0.002220262954039
//#define _a4  0.001893594048567
//
//#define _a1 -3.362256889209355
//#define _a2  4.282608240117919
//#define _a3 -2.444765517272841
//#define _a4  0.527149895089809

//Cheby2(4,60,5/50);
//#define _a0  0.001066578484441
//#define _a1 -0.003520583754742
//#define _a2  0.004979264107821
//#define _a3 -0.003520583754742
//#define _a4  0.001066578484441
//
//#define _a1 -3.754902351869990
//#define _a2  5.294313666885127
//#define _a3 -3.321856314439796
//#define _a4  0.782516252991879

//// Cheby2(4,60,2/50);
//#define _a0  0.000982727171845
//#define _a1 -0.003809312652692
//#define _a2  0.005655081072900
//#define _a3 -0.003809312652692
//#define _a4  0.000982727171845
//
//#define _a1 -3.902568482603157
//#define _a2  5.712424931235347
//#define _a3 -3.717010923329100
//#define _a4  0.907156384808116

    #define _b0  0.0013084365788853664F
    #define _b1 -0.0012797600360308025F
    #define _b2  0.0013084365788853672F
    
    #define _a1 -1.9476480403398972F
    #define _a2  0.94898515346163725F
    
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

