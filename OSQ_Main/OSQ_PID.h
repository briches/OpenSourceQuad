/*=====================================================================
	OSQ_PID library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library implements the standard form PID controller

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
#ifndef OSQ_PID_H_INCLUDED
#define OSQ_PID_H_INCLUDED


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Limits.h>

#define SINGLE_PID
//#define NESTED_PID

#ifdef NESTED_PID
        double SET_ATT_KP = 6;                        // TODO:
        double SET_ATT_KI = 0;
        
        double RATE_ATT_KP = 2.0;
        double RATE_ATT_KI = 0.0;
        double RATE_ATT_KD = 0.0;
        
        double altitudekP = 0;
        double altitudekI = 0;
#endif

#ifdef SINGLE_PID
        double ATT_KP = 5;
        double ATT_KI = 0.1;
        double ATT_KD = 2;
#endif

#define altitudekI (0)
#define altitudekP (0)
#define altitudekD (0)

struct RATE_PID_t
{
	unsigned long lastTimestamp;
	double RATE_KP, RATE_KI, RATE_KD;

        double lastError;

	double windupGuard; // When set point changes by a lot, the i-term may get really large.
	double integratedError;
	double target;

};

struct SET_PID_t
{
	unsigned long lastTimestamp;
	double SET_KP, SET_KI;

	double windupGuard; // When set point changes by a lot, the i-term may get really large.
	double integratedError;
	double target;
	double desiredRate;
        double motorOutput;

        // Rate pid controller nested in the target controller, makes it easy to set targets
        RATE_PID_t RATE_PID;         
};

SET_PID_t pitchPID;
SET_PID_t rollPID;
SET_PID_t yawPID;
SET_PID_t altitudePID;


enum {lower, upper};



void calculateRATE_PID(struct SET_PID_t *PID, double measuredRate)
{
        double dt = (micros() - PID->RATE_PID.lastTimestamp)/1000000.;
        
        double error = measuredRate - PID->RATE_PID.target;
        
        PID->RATE_PID.integratedError += PID->RATE_PID.RATE_KI * error * dt;
        PID->RATE_PID.integratedError = constrain(PID->RATE_PID.integratedError, -PID->RATE_PID.windupGuard, PID->RATE_PID.windupGuard);
        
        PID->motorOutput = PID->RATE_PID.RATE_KP * error;
        PID->motorOutput += PID->RATE_PID.integratedError;
        PID->motorOutput += (error  - PID->RATE_PID.lastError) * PID->RATE_PID.RATE_KD / dt;
        
        PID->RATE_PID.lastTimestamp = micros();
};

void calculateSET_PID(struct SET_PID_t *PID, double measuredAttitude)
{
        // Outputs a target RATE for the inner RATE PID controller to achieve.
	double dt = (micros() - PID->lastTimestamp)/1000000.;

        // Error signal
	double error = measuredAttitude - PID->target;	

        // Integrate
	PID->integratedError += PID->SET_KI * error * dt; 

	PID->integratedError = constrain(PID->integratedError,-PID->windupGuard, PID->windupGuard);

	PID->desiredRate = PID-> SET_KP * error; // P
	PID->desiredRate += PID-> integratedError; // P + I + D


	PID->lastTimestamp = micros();

        PID->RATE_PID.target = PID->desiredRate;
};

void initializePID(struct SET_PID_t *PID, double kP, double kI, double RATE_KP, double RATE_KI, double RATE_KD)
{
	PID->SET_KP = kP;
	PID->SET_KI = kI;
        PID->RATE_PID.RATE_KP = RATE_KP;
        PID->RATE_PID.RATE_KI = RATE_KI;
        PID->RATE_PID.RATE_KD = RATE_KD;
	PID->target = 0;
	PID->desiredRate = 0;
	PID->lastTimestamp = 0;
	PID->windupGuard = LONG_MAX;
	PID->integratedError = 0;

};

void rollPitchPID(struct SET_PID_t *pPID, struct SET_PID_t *rPID, double measuredPitch, double measuredRoll, double measuredPitchRate, double measuredRollRate)
{
        #ifdef SINGLE_PID
                calculateSET_PID(&pitchPID, kinematics.pitch);
                calculateSET_PID(&rollPID, kinematics.roll);
                
                pPID->motorOutput = pPID->desiredRate;
                rPID->motorOutput = rPID->desiredRate;
        #endif
        
        #ifdef NESTED_PID
                calculateSET_PID(&pitchPID, kinematics.pitch);
                calculateSET_PID(&rollPID, kinematics.roll);
                
                calculateRATE_PID(&pitchPID,  kinematics.ratePITCH);        
                calculateRATE_PID(&rollPID,  kinematics.rateROLL);
        #endif
};

#endif // OSQ_PID_H_INCLUDED

