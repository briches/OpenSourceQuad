/*=====================================================================
	OSQ_BMP085 library
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
/*
  Here's the (extremely difficult) circuit diagram
  
         Arduino
     /-------------\         BMP085
     |             |       /--------\
     |          +5V|-------| VIN    |
     |          GND|-------| GND    |
     |    (SDA) A4 |-------| SDA    |
     |    (SCL) A5 |-------| SCL    |
     |             |       \--------/
     \-------------/
      
*/
#include <Wire.h>
#include <OSQ_BMP085.h>


BMP085 barometer;


void setup()
{
  Serial.begin(19200);
  Serial.println("BMP085 Test:");
  
  // Call this function first
  barometer.readEEPROM();
  
  // This function is optional. Use for debugging purposes.
  barometer.printEEPROM();
  
  // Sea level pressure.
  // Look up daily. Give in inchesHg
  barometer.setSLP(30.094); 
  
  // Set the accuracy and frequency. 
  // Options are 0, 1, 2, 3
  barometer.setOSS(3); 
  
}

void loop()
{
  // This library needs a delay to allow ADC
  delay(50);
  
  // Call this function on every cycle
  barometer.updatePTA();
  
  Serial.print("Altitude (m): ");
  Serial.print(barometer.altitude);
  Serial.print(" Temperature (*C): ");
  Serial.println(barometer.temperature);
}
