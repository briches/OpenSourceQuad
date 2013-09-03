/*=====================================================================
 	OSQ_Quadcopter library
 	OpenSourceQuad
 	-------------------------------------------------------------------*/
/*================================================================================
 
 	Author		: Brandon Riches
 	Date		: August 2013
 	License		: GNU Public License
 
 	This library is designed to abstract away some of the craft management functions
 	from the main file (OSQ_Main.ino)
 
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

// Waiting on the Kalman for this one.
#ifndef OSQ_ALTITUDEPROCESSOR_H_INCLUDED
#define OSQ_ALTITUDEPROCESSOR_H_INCLUDED

#define Pi  	    (3.14159265359F)	// Its pi.

#define altKp       (0.0)
#define altKi       (0.0)
#define altKd       (0.0)



static boolean useUSRF = true;
static boolean isSetInitialAltitude = false;
static boolean altitudeHold  = false;
static boolean changedFromInitial = false;

static double barometerOffset = 0;
static double GPSOffset = 0;

static double initialAltitude = 0;    // From sea level;
static double previousAltitude = 0;
static double accurateAltitude = 0;
static double altitudeCovariance = 1;

boolean altitudeDebug = true;

void setInitialAltitude(double GPS, double baro);

double getAccurateAltitude(double GPS, double baro, double USRF, double phi, int GPS_quality)
{
    static boolean haveUsedGPS = false;
    double GPSCovar = 5;        // Assume covariance in GPS            // 1
    double baroCovar = 1;     // Assume covariance in barometer      // 2
    double USRFCovar = 0.1;     // Assume covariance in USRF           // 3
    
    if(altitudeDebug)
    {
        Serial.print(" Raw GPS: ");
        Serial.print(GPS);
        Serial.print(" Raw baro: ");
        Serial.print(baro);
        Serial.print(" Raw USRF: ");
        Serial.print(USRF);
    }
    
    if(isSetInitialAltitude)
    {
        if(accurateAltitude < 10)    // If lower than 10 m, we can probably trust the USRF
        {
            useUSRF = true;
        } 
        else {
            useUSRF = false;
        }
    
        // This is a super basic kalman filter.
        USRF *= cos(phi * Pi / 180);
        GPS -= initialAltitude;
        baro -= initialAltitude;
        
        if((accurateAltitude - previousAltitude) > altitudeCovariance) changedFromInitial = true;
        
        previousAltitude = accurateAltitude;
       
        if(GPS_quality == 1)
        {
            if(!changedFromInitial)
            {
                barometerOffset = initialAltitude - (initialAltitude + GPS) / 2;
                GPSOffset = initialAltitude - (initialAltitude + GPS) / 2;
                initialAltitude = (initialAltitude + GPS) / 2;
            }
            if(changedFromInitial && !haveUsedGPS)
            {
                GPSOffset = initialAltitude - GPS;
            }
           
            haveUsedGPS = true;
            
            GPS -= GPSOffset;
            baro -= barometerOffset;
            
            accurateAltitude = (accurateAltitude * GPSCovar + GPS * altitudeCovariance) / (altitudeCovariance + GPSCovar);
            altitudeCovariance = (altitudeCovariance * GPSCovar)/(altitudeCovariance + GPSCovar);
        }
        
        accurateAltitude = (accurateAltitude * baroCovar + baro * altitudeCovariance) / (altitudeCovariance + baroCovar);
        altitudeCovariance = (altitudeCovariance * baroCovar)/(altitudeCovariance + baroCovar);
        
        if(useUSRF)
        {
            accurateAltitude = (accurateAltitude * USRFCovar + USRF * altitudeCovariance) / (altitudeCovariance + USRFCovar);
            altitudeCovariance = (altitudeCovariance * USRFCovar)/(altitudeCovariance + USRFCovar);
        }
        
        altitudeCovariance += abs(accurateAltitude - previousAltitude);
        
        
        if(altitudeDebug)
        {
            Serial.print(" GPS: "); Serial.print(GPS);
            Serial.print(" Baro: "); Serial.print(baro);
            Serial.print(" USRF: "); Serial.print(USRF);
            Serial.print(" 'Accurate Altitude' : ");
            Serial.print(accurateAltitude);
            Serial.print(" Altitude covariance: ");
            Serial.println(altitudeCovariance);
        }
            
    } 
    else 
    {
        setInitialAltitude(GPS, baro);
    }
    
    return accurateAltitude;

};

void setInitialAltitude(double GPS, double baro)
{
    if(altitudeDebug) Serial.println("Checking initial altitude");
    if( baro != 0 )
    {
        initialAltitude = (baro);    // From sea level
        accurateAltitude = 0;    // From ground
        isSetInitialAltitude = true;
    }

};



#endif // OSQ_ALTITUDEPROCESSOR_H_INCLUDED

