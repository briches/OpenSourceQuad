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

// TODO:
#ifdef NESTED_PID
        double SET_ATT_KP = 6;
        double SET_ATT_KI = 0;
        double SET_ATT_KD = 0;

        double RATE_ATT_KP = 2.0;
        double RATE_ATT_KI = 0.0;
        double RATE_ATT_KD = 0.0;

#endif

#ifdef SINGLE_PID 
        double ATT_KP = 0.48099;
        double ATT_KI = 0.070461*2;
        double ATT_KD = 0.43744;
#endif

double altitudekP = 0;
double altitudekI = 0;
double altitudekD = 0;

enum{pitch, roll, yaw, altitude};
enum{lower, upper};

struct PID_Gain
{

	double setP, setI, setD;

	#ifdef NESTED_PID
		double rateP, rateI, rateD;
	#endif

}PID_GAINS[6];

typedef struct PID_Manager_t
{
	unsigned long lastTimestamp;
	int ID;


	// Set variables
	double windupGuard;
	double setIntegratedError;
	double setLastError;
	double setTarget;
	double output;

	#ifdef NESTED_PID
	// Rate variables
		double rateIntegratedError;
		double rateLastError;
	#endif

	// Constructor
	PID_Manager_t(int selection);
};

PID_Manager_t :: PID_Manager_t(int selection)
{
	ID = selection;
};

PID_Manager_t pitchPID(pitch);
PID_Manager_t rollPID(roll);
PID_Manager_t yawPID(yaw);
PID_Manager_t altitudePID(altitude);

double calculatePID(struct PID_Manager_t *PID, double measuredValue, double measuredRate)
{
	// Timestep, seconds
	double dt = (micros() - PID->lastTimestamp)/1000000.;



	#ifdef SINGLE_PID
		double error = measuredValue - PID->setTarget;

		// Integral response
		PID->setIntegratedError += PID_GAINS[PID->ID].setI * error * dt;
		PID->setIntegratedError = constrain(PID->setIntegratedError, -PID->windupGuard, PID->windupGuard);

                // Proportional Response
                double pTerm = error * PID_GAINS[PID->ID].setP;
                
                // Derivative response.
                double dTerm = (error - PID->setLastError) * PID_GAINS[PID->ID].setD / dt;
                
		PID->output = PID->setIntegratedError + pTerm + dTerm;

		PID->lastTimestamp = micros();
                PID->setLastError = error;

		return PID->output;
	#endif

	#ifdef NESTED_PID
		double error = measuredAttitude - PID->setTarget;

		// Exterior PID set loop
		PID->setIntegratedError += PID_GAINS[PID->ID].setI * error * dt;

		PID->setIntegratedError = constrain(PID->setIntegratedError, -PID->windupGuard, PID->windupGuard);

		PID->output = 	PID->setIntegratedError +
						error * PID_GAINS[PID->ID].setP + // P
						(error - PID->setLastError) * PID_GAINS[PID->ID].setD / dt; // D

		// Interior PID rate loop
		double rateError = measuredRate - PID->output;

		PID->rateIntegratedError += PID_GAINS[PID->ID].rateI * rateError * dt;

		PID->output = 	PID->rateIntegratedError +
						rateError * PID_GAINS[PID->ID].rateP +
						(rateError - PID->rateLastError) * PID_GAINS[PID->ID].setD / dt;

		PID->lastTimestamp = micros();
                PID->setLastError = error;
                PID->rateLastError = error;

		return PID->output;
	#endif
};

void initializePID(struct PID_Manager_t *PID)
{
	PID->windupGuard = INT_MAX;
	PID->setIntegratedError = 0;
	PID->setTarget = 0;
	PID->output = 0;
	PID->setLastError = 0;

	if(PID->ID < altitude)
	{
	#ifdef SINGLE_PID
		PID_GAINS[PID->ID].setP = ATT_KP;
		PID_GAINS[PID->ID].setI = ATT_KI;
		PID_GAINS[PID->ID].setD = ATT_KD;
	#endif

	#ifdef NESTED_PID
		PID_GAINS[PID->ID].setP = SET_ATT_KP;
		PID_GAINS[PID->ID].setI = SET_ATT_KI;
		PID_GAINS[PID->ID].setD = SET_ATT_KD;

		PID_GAINS[PID->ID].rateP = RATE_ATT_KP;
		PID_GAINS[PID->ID].rateI = RATE_ATT_KI;
		PID_GAINS[PID->ID].rateD = RATE_ATT_KD;

		PID->rateLastError = 0;
	#endif
	}
	else
	{
		PID_GAINS[PID->ID].setP = altitudekP;
		PID_GAINS[PID->ID].setI = altitudekI;
		PID_GAINS[PID->ID].setD = altitudekD;
	}

        Serial.println("PID init: ");
        Serial.print("ID: "); Serial.println(PID->ID);

};

#endif // OSQ_PID_H_INCLUDED

