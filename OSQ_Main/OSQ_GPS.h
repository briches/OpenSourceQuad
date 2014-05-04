/*=====================================================================
     OSQ_GPS
     OpenSourceQuad
     -------------------------------------------------------------------*/
/*================================================================================
 
     Author		: Brandon Riches
     Date		: August 2013
     License		: GNU Public License
 
     Interfaces with the optional onboard GPS module to provide LAT/LON data, altitude,
     and waypoints
     
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
#ifndef OSQ_GPS_H_INCLUDED
#define OSQ_GPS_H_INCLUDED

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

struct gpsdata_t
{
    int	fix, quality;	// Is fixed or no
    int    	satellites;		// number of sats
    double 	altitude,		// in m, apparently. this seems to be fucked
    angle,		// in deg, based off movement vector
    lat,		// in degrees, decimal minutes
    lon,		// degrees, decimal minutes
    spd;		// converted to m/s, direction of motion

};

gpsdata_t       GPSDATA;
SoftwareSerial  GPSSerial(13, 12); // TX, RX GPS pins
Adafruit_GPS    GPS(&GPSSerial);

/*=========================================================================
 checkGPS()
 Checks for a new NMEA sentence, and parses it
 -----------------------------------------------------------------------*/
void checkGPS()
{
    // Should be called like all the time, pretty much
    // Call it in loop
    if (GPS.newNMEAreceived())
    {
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence so we should just wait for another
    }

}

/*=========================================================================
 getGPS_Data()
 Places parsed data into the GPS data type
 -----------------------------------------------------------------------*/
void getGPS_Data()
{
    // Call in the 1Hz loop
    GPSDATA.fix = GPS.fix;
    GPSDATA.quality = (uint8_t)GPS.fixquality;
    GPSDATA.altitude = GPS.altitude;
    GPSDATA.satellites = (int8_t)GPS.satellites;
    GPSDATA.angle = GPS.angle;
    GPSDATA.lat = GPS.lat;
    GPSDATA.lon = GPS.lon;
    GPSDATA.spd = GPS.speed / 0.5144;		// Convert to m/s from knots
}

/*=========================================================================
 initGPS
 Starts the GPS by sending commands, and sets up the ISR
 -----------------------------------------------------------------------*/
void initGPS()
{
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A); // Enable OCR0A interrupt
}


/*=========================================================================
 SIGNAL(TIMER0_COMPA_vect)
 ISR for GPS
 -----------------------------------------------------------------------*/
SIGNAL(TIMER0_COMPA_vect)
{
    GPS.read();
}


#endif // OSQ_GPS_H_INCLUDED


