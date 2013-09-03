/*=====================================================================
	OSQ_GPS
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: August 2013
	License		: GNU Public License

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

typedef struct gpsdata_t
{
	int		fix, quality;	// Is fixed or no
	int    		satellites;		// number of sats
	double 		altitude,		// in m, apparently. this seems to be fucked
		        angle,			// in deg, based of movement
			lat,			// in degrees, decimal minutes
			lon,			// degrees, decimal minutes
			spd;			// converted to m/s, direction of motion

};


#endif // OSQ_GPS_H_INCLUDED

