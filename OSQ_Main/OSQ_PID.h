/*=====================================================================
	OSQ_PID library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

	This library interfaces with the BMP085 pressure sensor to return temperature
	calibrated altitude measurements.

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

#define anglekP 35                        // TODO:
#define anglekI 85
#define anglekD 30

#define altitudekP 0
#define altitudekI 0
#define altitudekD 0

struct PID_t
{
	unsigned long timestamp;
	double kP, kI, kD;

	double windupGuard; // When set point changes by a lot, the i-term may get really large.
	double lastError;
	double iTerm;
	double setpoint;
	double output;

	bool limitsSet;
	double limits[2];
};

PID_t pitchPID;
PID_t rollPID;
PID_t altitudePID;

enum {lower, upper};

void setPoint(struct PID_t *PID, double newSetpoint)
{
	PID->setpoint = newSetpoint;
};

void initializePID(struct PID_t *PID, double kP, double kI, double kD)
{
	PID->kP = kP;
	PID->kI = kI;
	PID->kD = kD;
	PID->setpoint = 0;
	PID->output = 0;
	PID->timestamp = 0;
	PID->windupGuard = LONG_MAX;
	PID->lastError = 0;
	PID->iTerm = 0;
	PID->limitsSet = false;
	PID->limits[lower] = 0;
        PID->limits[upper] = 0;

};

void setWindupGuard(struct PID_t *PID, double windupGuard)
{
	PID->windupGuard = windupGuard;
};

void checkWindupGuard(struct PID_t *PID)
{
	if(PID->iTerm > PID->windupGuard)
	{
		PID->iTerm = PID->windupGuard;
	}

	if(PID->iTerm < -PID->windupGuard)
	{
		PID->iTerm = -PID->windupGuard;
	}
};

void constrainToLimits(struct PID_t *PID)
{
	if(PID->output > PID->limits[upper])
	{
		PID->output = PID->limits[upper];
	}
	if(PID->output < PID->limits[lower])
	{
		PID->output = PID->limits[lower];
	}
};


void calculatePID(struct PID_t *PID, double input)
{
	double dt = (micros() - PID->timestamp)/1000000.;

	double error = input - PID->setpoint;	// Error signal

	PID->iTerm += PID->kI * error * dt; // Integrate

	checkWindupGuard(PID); //Check for integral windup

	PID->output = PID->kP * error + PID->kD * (PID->lastError - error) / dt; // P + D
	PID->output += PID->iTerm; // P + I + D

	if(PID->limitsSet) constrainToLimits(PID);	// Exceed limits?

	PID->timestamp = micros();
	PID->lastError = error;

};
#endif // OSQ_PID_H_INCLUDED
