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

static double targetAltitude = 0;
// Keep track just for kicks
static double latestAltitude = 0;
static double previousAltitude = 0;
bool inFlight, altitudeHold;

/*=========================================================================
 Function declarations
 -----------------------------------------------------------------------*/
long computeRunningAvg(long currentInput, struct altitudeSensor_t *sensor);
double getAccurateAltitude(double GPS, double baro, double USRF, double phi, int GPS_quality);
void checkRegion(struct altitudeSensor_t *sensor);
long weightedAvg(struct altitudeSensor_t *sensor1, struct altitudeSensor_t *sensor2, struct altitudeSensor_t *sensor3);

/*=========================================================================
 Altitude sensor data type
 Uses interger math for speed
 -----------------------------------------------------------------------*/
typedef struct altitudeSensor_t
{
    // Is the sensor in its useful range, does it need LPF
    bool active, useFilt;
    
    // Initial and current altitude estimates
    long initial, current;
    
    // Upper and lower bounds of the sensors useful range
    long usefulRange[2];
    
    // Filter paramters
    double filt[40];
    int spot;
    
    // Weight for the weighted average 
    long confidance;
    
    altitudeSensor_t(long acc, long rangeL, long rangeH, bool filt);
};
// Constructor
altitudeSensor_t :: altitudeSensor_t(long acc, long rangeL, long rangeH, bool filt)
{
    this->confidance = acc;
    this->active = true;
    this->usefulRange[0] = rangeL;
    this->usefulRange[1] = rangeH;
    this->spot = 0;        
};

// Declaration of the altitude sensors
altitudeSensor_t baroSensor(1, LONG_MIN, LONG_MAX, true);
altitudeSensor_t GPSSensor(1, LONG_MIN, LONG_MAX, true);
altitudeSensor_t USRFSensor(5, 100, 10000, false); // 40 cm to 10 m

/*=========================================================================
 Returns an estimate of the current altitude 
 Uses interger math for speed
 -----------------------------------------------------------------------*/
double getAccurateAltitude(double GPS, double baro, double USRF, double phi, int GPS_quality)
{
    long GPS_ = 1000*GPS;
    long baro_ = 1000*baro;
    long USRF_ = 1000*USRF;
    long phi_ = 1000*phi;
    double sensorAltitude = 0; // Actual result
    int measurementCount = 0;
    
    // Check active regions
    checkRegion(&USRFSensor);
    checkRegion(&GPSSensor);
    checkRegion(&baroSensor);
    
    ////// Heads up brandon //////
    // Todo
    GPSSensor.active = false; 
    
    //******************** Barometer
    if(baroSensor.active)
    {
        long avg = computeRunningAvg(baro_, &baroSensor);
        baroSensor.current = avg - baroSensor.initial;
    }
    else{/* Sadness */}
    
    // USRF
    if(!inFlight)
        USRFSensor.active = false;
    if(USRFSensor.active)
    {
        USRFSensor.current = USRF_;
    }
    
    // GPS
    if(GPSSensor.active)
    {
        GPSSensor.current = GPS_ - GPSSensor.initial;
    }
    
    // Convert to real altitude and return.
    previousAltitude = latestAltitude/1000.0;
    latestAltitude = (double)weightedAvg(&baroSensor, &USRFSensor, &GPSSensor);
    return latestAltitude/1000.0;
};

/*=========================================================================
 void weightedAvg(struct altitudeSensor_t *sensor1, 
                  struct altitudeSensor_t *sensor2, 
                  struct altitudeSensor_t *sensor3)
 Combine the sensor readings according to their confidance
 -----------------------------------------------------------------------*/
long weightedAvg(struct altitudeSensor_t *sensor1, struct altitudeSensor_t *sensor2, struct altitudeSensor_t *sensor3)
{
    long result, numerator, denominator;
    
    if(sensor1->active)
    {
        numerator += sensor1->confidance * sensor1->current;
        denominator += sensor1->confidance;
    }
    if(sensor2->active)
    {
        numerator += sensor2->confidance * sensor2->current;
        denominator += sensor2->confidance;
    }
    if(sensor3->active)
    {
        numerator += sensor3->confidance * sensor3->current;
        denominator += sensor3->confidance;
    }
    return numerator/denominator;
}

/*=========================================================================
 void checkRegion(struct altitudeSensor_t *sensor)
 Check if the sensor's output will be valid
 -----------------------------------------------------------------------*/
void checkRegion(struct altitudeSensor_t *sensor)
{
    if(latestAltitude < sensor->usefulRange[0] || latestAltitude > sensor->usefulRange[1])
    {
        sensor->active = false;
    }
    else sensor->active = true;
};

/*=========================================================================
 long computeRunningAvg(long currentInput, struct altitudeSensor_t *sensor)
 Filter the current sensor output with a simple running avg
 -----------------------------------------------------------------------*/
long computeRunningAvg(long currentInput, struct altitudeSensor_t *sensor)
{	
    double output = 0;
    
    sensor->filt[sensor->spot] = currentInput;
    sensor->spot++;
    if(sensor->spot >= 40) sensor->spot = 0;
    
    for(int i = 0; i<40; i++)
    {
        output += sensor->filt[i];
    }
    
    output = output/40.0;
    return (long)output;
};



#endif // OSQ_ALTITUDEPROCESSOR_H_INCLUDED


