/*=====================================================================
     OSQ_AltitudeProcessor
     OpenSourceQuad
     -------------------------------------------------------------------*/
/*================================================================================
 
     Author		: Brandon Riches
     Date		: August 2013
     License		: GNU Public License
 
     This library is designed to manage altitude measurement and control algorithms
 
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
#ifndef OSQ_ALTITUDEPROCESSOR_H_INCLUDED
#define OSQ_ALTITUDEPROCESSOR_H_INCLUDED

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif
#include <Limits.h>

#define Pi (3.14159265359F)	// Its pi.

// Keep track just for kicks
static double previousAltitude = 0;

/*=========================================================================
 Function declarations
 -----------------------------------------------------------------------*/
void computeCheby2(int64_t currentInput, struct altitudeSensor_t *sensor);
double getAccurateAltitude(double GPS, double baro, double USRF, double phi, int GPS_quality);
void checkRegion(struct altitudeSensor_t *sensor);

/*=========================================================================
 Altitude sensor data type
 Uses interger math for speed
 -----------------------------------------------------------------------*/
typedef struct altitudeSensor_t
{
    // Is the sensor in its useful range, does it need LPF
    bool active, useFilt;
    
    // Initial and current altitude estimates
    int64_t initial, current;
    
    // Upper and lower bounds of the sensors useful range
    int64_t usefulRange[2];
    
    // Filter paramters
    int64_t inputTm1, inputTm2, outputTm1, outputTm2;
    
    // Weight for the weighted average 
    int64_t confidence;
    
    altitudeSensor_t(int64_t acc, int64_t rangeL, int64_t rangeH, bool filt);
};
// Constructor
altitudeSensor_t :: altitudeSensor_t(int64_t acc, int64_t rangeL, int64_t rangeH, bool filt)
{
    this->confidence = acc;
    this->active = true;
    this->usefulRange[0] = rangeL;
    this->usefulRange[1] = rangeH;
  
    // Setup the filter
    this->inputTm1 = this->initial;
    this->inputTm2 = this->initial;
    this->outputTm1 = this->initial;
    this->outputTm2 = this->initial;
};

// Declaration of the altitude sensors
altitudeSensor_t baroSensor(1, LONG_MIN, LONG_MAX, true);
altitudeSensor_t GPSSensor(1, LONG_MIN, LONG_MAX, true);
altitudeSensor_t USRFSensor(8, 15000, 1000000, false); // 15 cm to 10 m

/*=========================================================================
 Returns an estimate of the current altitude 
 Uses interger math for speed
 -----------------------------------------------------------------------*/
double getAccurateAltitude(double GPS, double baro, double USRF, double phi, int GPS_quality)
{
    int64_t GPS_ = 100000*GPS;
    int64_t baro_ = 100000*baro;
    int64_t USRF_ = 100000*USRF;
    int64_t phi_ = 100000*phi;
    double sensorAltitude = 0; // Actual result
    int measurementCount = 0;
    
    // Check active regions
    checkRegion(&USRFSensor);
    checkRegion(&GPSSensor);
    checkRegion(&baroSensor);
    
    // Barometer
    if(baroSensor.active)
    {
        // Filter
        //computeCheby2(baro_, &baroSensor);
        baroSensor.current = baro_;
        // Add result into final estimate
        sensorAltitude += baroSensor.current - baroSensor.initial;
        measurementCount++;
    }
    else{/* Sadness */}
    
    // USRF
    if(USRFSensor.active)
    {
        sensorAltitude += USRF_;
        measurementCount++;
    }
    
    Serial.println(sensorAltitude);
    sensorAltitude /= measurementCount;
    // Convert to real altitude and return.
    return sensorAltitude/100000;
};

/*=========================================================================
 void checkRegion(struct altitudeSensor_t *sensor)
 Check if the sensor's output will be valid
 -----------------------------------------------------------------------*/
void checkRegion(struct altitudeSensor_t *sensor)
{
    if(previousAltitude < sensor->usefulRange[0] || previousAltitude > sensor->usefulRange[1])
    {
        sensor->active = false;
    }
    else sensor->active = true;
};

/*=========================================================================
 int64_t computeCheby2(int64_t currentInput, struct altitudeSensor_t *sensor)
 Filter the current sensor output
 -----------------------------------------------------------------------*/
void computeCheby2(int64_t currentInput, struct altitudeSensor_t *sensor)
{	
    // Cheby2(2,60,0.25)
    #define _b0  130L // Multiplied by 100 thousand
    #define _b1 -130L
    #define _b2  130L
    
    #define _a1 -194760L
    #define _a2  94900L
    
    int64_t output;

    output = _b0 * currentInput           +
        _b1 * sensor->inputTm1  +
        _b2 * sensor->inputTm2  -
        _a1 * sensor->outputTm1 -
        _a2 * sensor->outputTm2;
        
    sensor->inputTm2 = sensor->inputTm1;
    sensor->inputTm1 = currentInput;

    sensor->outputTm2 = sensor->outputTm1;
    sensor->outputTm1 = output;

    sensor->current = output;
};



#endif // OSQ_ALTITUDEPROCESSOR_H_INCLUDED


