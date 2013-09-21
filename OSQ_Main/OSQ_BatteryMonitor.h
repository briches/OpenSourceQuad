/*=====================================================================
 	OSQ_BatteryMonitor library
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
#ifndef OSQ_BATTERYMONITOR_H_INCLUDED
#define OSQ_BATTERYMONITOR_H_INCLUDED

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define nominalBatteryVoltage	(11.1)		// Nominal operating voltage
#define softAlarmVoltage		(10.5)		// Battery voltage low
#define	criticalAlarmVoltage	(10.0)		// Battery voltage critical
#define soundAlarmCount			(100)		// For critical alarm; counter increments when below
#define analogPositivePin		(5)		// Connect red wire to this pin
#define voltageConversion		(0.2475F)	// From voltage divider

bool	softAlarm = false;
bool	criticalAlarm = false;

bool    initialized = false;

struct batteryStats_st
{
        int alarmCount,
        arrayLocation;
        double  batteryVoltage,
        storageArray[10];
};

struct batteryStats_st batteryStats;

double filterVoltage()
{
        double result = 0;
        for(int i = 0; i< 10; i++)
        {
                result += batteryStats.storageArray[i];
        }

        batteryStats.arrayLocation++;
        if(batteryStats.arrayLocation == 10)
        {
                batteryStats.arrayLocation = 0;
        }

        return result/10;

};

void checkAlarm()
{
        // Check for soft alarm
        if(batteryStats.batteryVoltage <= softAlarmVoltage)
        {
                batteryStats.alarmCount += 1;
                if (batteryStats.alarmCount >= soundAlarmCount/10)
                {
                        softAlarm = true;
                }
        }
        else
        {
                batteryStats.alarmCount -= 1;
                if(batteryStats.alarmCount < 0)
                {
                        batteryStats.alarmCount = 0;
                }
                if (batteryStats.alarmCount <= soundAlarmCount/10)
                {
                        softAlarm = false;
                }
        }

        // Check for critical alarm
        if(batteryStats.batteryVoltage <= criticalAlarmVoltage)
        {
                batteryStats.alarmCount += 20;
                if (batteryStats.alarmCount >= soundAlarmCount)
                {
                        criticalAlarm = true;
                }
        }
        else
        {
                batteryStats.alarmCount -= 20;
                if(batteryStats.alarmCount < 0)
                {
                        batteryStats.alarmCount = 0;
                }
                if (batteryStats.alarmCount <= soundAlarmCount)
                {
                        criticalAlarm = false;
                }
        }
}

void monitorBatteryVoltage()
{
        if(!initialized)
        {
                for(int i = 0; i < 10; i++)
                {
                        batteryStats.storageArray[i] = nominalBatteryVoltage;
                }
                initialized = true;
        }

        batteryStats.storageArray[batteryStats.arrayLocation] = analogRead(analogPositivePin) * (5.0/1023.0) / voltageConversion; // Read voltage

        batteryStats.batteryVoltage = filterVoltage();	// Running average filter the voltage

        checkAlarm();	// Check for low voltage

};

#endif // OSQ_BATTERYMONITOR_H_INCLUDED



