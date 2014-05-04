/*=====================================================================
     OSQ_BatteryMonitor
     OpenSourceQuad
     -------------------------------------------------------------------*/
/*================================================================================
 
     Author		: Brandon Riches
     Date		: August 2013
     License		: GNU Public License
 
     Monitors battery voltage and battery alarms
 
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

#define nominalvoltage (3.7)		// Nominal operating voltage
#define softAlarmVoltage (3.5)		// Battery voltage low
#define	criticalAlarmVoltage (3.33)		// Battery voltage critical
#define soundAlarmCount	 (100)		// For critical alarm; counter increments when below
#define analogPositivePin (5)		// Connect red wire to this pin
#define voltageConversion (0.2475F)	// From voltage divider

bool softAlarm = false;
bool criticalAlarm = false;
bool initialized = false;

typedef struct battery_st
{
    int alarmCount[3], arrayLocation[3];
    double  voltage[3], storageArray[3][10];
};

struct battery_st battery;

double filterVoltage(int cell)
{
    double result = 0;
    for(int i = 0; i< 10; i++)
    {
        result += battery.storageArray[cell][i];
    }

    battery.arrayLocation[cell]++;
    if(battery.arrayLocation[cell] >= 10)
    {
        battery.arrayLocation[cell] = 0;
    }

    return result/10;
};

void checkAlarm(int cell)
{
    // Check for soft alarm
    if(battery.voltage[cell] <= softAlarmVoltage)
    {
        battery.alarmCount[cell] += 1;
        if (battery.alarmCount[cell] >= soundAlarmCount/10)
        {
            softAlarm = true;
        }
    }
    else
    {
        battery.alarmCount[cell] -= 1;
        if(battery.alarmCount[cell] < 0)
        {
            battery.alarmCount[cell] = 0;
        }
        if (battery.alarmCount[cell] <= soundAlarmCount/10)
        {
            softAlarm = false;
        }
    }

    // Check for critical alarm
    if(battery.voltage[cell] <= criticalAlarmVoltage)
    {
        battery.alarmCount[cell] += 20;
        if (battery.alarmCount[cell] >= soundAlarmCount)
        {
            criticalAlarm = true;
        }
    }
    else
    {
        battery.alarmCount[cell] -= 20;
        if(battery.alarmCount[cell] < 0)
        {
            battery.alarmCount[cell] = 0;
        }
        if (battery.alarmCount[cell] <= soundAlarmCount)
        {
            criticalAlarm = false;
        }
    }
}

void monitorVoltage()
{
    if(!initialized)
    {
        for(int i = 0; i < 10; i++)
        {
            battery.storageArray[0][i] = nominalvoltage;
            battery.storageArray[1][i] = nominalvoltage;
            battery.storageArray[2][i] = nominalvoltage;
        }
        initialized = true;
    }

    for(int cell = 0; cell < 3; cell++)
    {
        battery.storageArray[cell][battery.arrayLocation[cell]] = analogRead(cell + 1) * (5.0/1023.0); // Read voltage
        battery.voltage[cell] = filterVoltage(cell);	// Running average filter the voltage
        checkAlarm(cell);	// Check for low voltage
    }
};

#endif // OSQ_BATTERYMONITOR_H_INCLUDED


