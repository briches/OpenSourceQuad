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

#define USRF_POWER      (40)


static boolean altitudeHold  = false;

static boolean isSetInitialAltitudeBarometer = false;
static boolean isSetInitialAltitudeGPS = false;
static boolean changedFromInitial = false;

static double barometerOffset = 0;
static double GPSOffset = 0;

static double initialAltitudeBarometer = 0;    // From sea level;
static double initialAltitudeGPS = 0;    // From sea level;
static double previousAltitude = 0;
static double sensorAltitude = 0;
static double sensorCovariance = 1;

boolean altitudeDebug = true;

void setInitialAltitude(double GPS, double baro);
bool checkUSRF(double hieght);

double getAccurateAltitude(double GPS, double baro, double USRF, double phi, int GPS_quality)
{
        static int barometerSampleCount = 0;
        static boolean calibratedUSRF = false;
        double GPSCovar = 3;        // Assume covariance in GPS            // 1
        double baroCovar = 1;       // Assume covariance in barometer      // 2
        double USRFCovar = 0.1;     // Assume covariance in USRF           // 3

        previousAltitude = sensorAltitude;

        // This is a super basic kalman filter.
        // We only use each sensor if certain conditions are met
        // Barometer
        if(isSetInitialAltitudeBarometer)	// Wait at least 150 ms into the loop() to get barometer altitudes
        {
                baro -= initialAltitudeBarometer;
                sensorAltitude = baro;
                sensorCovariance = baroCovar;
        }

        else if(baro != 0)
        {
                initialAltitudeBarometer += baro;
                barometerSampleCount++;
                if(barometerSampleCount == 10)
                {
                        isSetInitialAltitudeBarometer = true;
                        initialAltitudeBarometer /= 10;
                }
        }

        // GPS
        if( (GPS_quality == 1) && (isSetInitialAltitudeGPS) )	// If the GPS is connected on 3axis
        {
                GPS -= initialAltitudeGPS;
                sensorAltitude = (sensorAltitude * GPSCovar + GPS * sensorCovariance) / (sensorCovariance + GPSCovar);
                sensorCovariance = (sensorCovariance * GPSCovar)/(sensorCovariance + GPSCovar);
        }
        else if(GPS_quality == 1)
        {
                initialAltitudeGPS = GPS;
                isSetInitialAltitudeGPS = true;
        }

        // USRF
        if(checkUSRF(10.0))	// If under 10 m, use the USRF.
        {
                USRF *= cos(phi * Pi / 180);
                sensorAltitude = (sensorAltitude * USRFCovar + USRF * sensorCovariance) / (sensorCovariance + USRFCovar);
                sensorCovariance = (sensorCovariance * USRFCovar)/(sensorCovariance + USRFCovar);
        }

        // Update covariance with "movement step"
        sensorCovariance += abs(previousAltitude - sensorAltitude);

        if(altitudeDebug)
        {
                Serial.print(" GPS: ");
                Serial.print(GPS);
                Serial.print(" Baro: ");
                Serial.print(baro);
                Serial.print(" USRF: ");
                Serial.print(USRF);
                Serial.print(" 'Accurate Altitude' : ");
                Serial.print(sensorAltitude);
                Serial.print(" Altitude covariance: ");
                Serial.println(sensorCovariance);
        }

        return sensorAltitude;

};

bool movedFromInitial(boolean changed)
{
        if(!changed)
        {
                if((sensorAltitude - previousAltitude) > sensorCovariance)
                        return true;
        }
        else return false;

};


bool checkUSRF(double hieght)
{
        return (sensorAltitude <= hieght);
};




#endif // OSQ_ALTITUDEPROCESSOR_H_INCLUDED


