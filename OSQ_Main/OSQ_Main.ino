/*=====================================================================
	OSQ_Main
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Brandon Riches
	Date		: September 2013
	License		: GNU Public License

	Main file for OSQ firmware

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
#include "OSQ_Kinematics.h"
#include "OSQ_SENSORLIB.h"
#include "OSQ_Quadcopter.h"
#include "OSQ_Motors.h"
#include "OSQ_BMP085.h"
#include "OSQ_NoWire.h"
#include "OSQ_GPS.h"
#include "OSQ_altitudeProcessor.h"
#include "OSQ_BatteryMonitor.h"
#include "OSQ_PID.h"

#include <RTClib.h>
#include <Adafruit_GPS.h>         
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SD.h>

/*=========================================================================
 Math related definitions
 -----------------------------------------------------------------------*/
#define Pi  		(3.14159265359F) // Its pi.

/*=========================================================================
 Sensor analog pins
 -----------------------------------------------------------------------*/
#define USRF_PIN 	(0x0)        

/*=========================================================================
 Battery Monitor
 -----------------------------------------------------------------------*/
#define NOMINAL_V	  11.1
#define SOFTWARE_VERSION  "V0.9.1"
uint32_t cycleCount;

/*=========================================================================
 Some classes. Most are at the bottom of their respective headers
 -----------------------------------------------------------------------*/
File                    logFile;
RTC_DS1307              rtc;


/*=========================================================================
 PID output variables and desired setpoints, and settings
 -----------------------------------------------------------------------*/

/*=========================================================================
 SD logging definitions
 -----------------------------------------------------------------------*/
// Hardware SS pin on the ATmega2560
#define chipSelect  (53)
char logFilename[] = "OSQ_Log.txt";


/*=========================================================================
 scanTelemetry()
 Watches for new commands sent via radio
 -----------------------------------------------------------------------*/
 
bool gotPID = false;
void scanTelemetry()
{
        switch(receiver.ScanForMessages())
        {
        case err:
                break;

        case disarm:
                Serial.println("Received DISARM command");
                motorControl.motorDISARM();
                break;
                
        case setAngleP:
                anglekP = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]);
                Serial.print("Received kP: ");
                Serial.println(anglekP);
                gotPID = true;
                break;
        
        case setAngleI:
               anglekI = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]);
               Serial.print("Received kI: ");
               Serial.println(anglekI);
               gotPID = true;
               break;
        
        case setAngleD:
               anglekD = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]);
               Serial.print("Received kD: ");
               Serial.println(anglekD);
               gotPID = true;
               break;
        }
}
/*=========================================================================
 processBatteryAlarms
 Watches for battery voltage alarms
 -----------------------------------------------------------------------*/
void processBatteryAlarms()
{
        if(softAlarm && !criticalAlarm)
        {
                // Auto Land
        }
        if(criticalAlarm)
        {
                motorControl.motorDISARM();
                while(true);
        }

}

/*=========================================================================
 PID_init()
 - Initializes the PID controllers
 -----------------------------------------------------------------------*/
void PID_init()                                          
{
        initializePID(&pitchPID, anglekP, anglekI, anglekD);
        initializePID(&rollPID, anglekP, anglekI, anglekD);
        initializePID(&altitudePID, altitudekP, altitudekI, altitudekD);
        
        setPoint(&pitchPID, 0);
        setPoint(&rollPID, 0);
        setPoint(&altitudePID, 1.5);
        
}
/*=========================================================================
 logData
 - Writes various data to the flight txt
 -----------------------------------------------------------------------*/
void logData()
{

        if (logFile)
        {
                logFile.print(micros());
                logFile.print(",");
                logFile.print(kinematics.pitch);
                logFile.print(",");
                logFile.print(kinematics.roll);
                logFile.print(",");
                logFile.print(pitchPID.output);
                logFile.print(",");
                logFile.println(rollPID.output);
        }
        else
        {
                //Serial.println("Error opening file!");
        }

}

/*=========================================================================
 logFileStart
 - Initializes a .txt on the uSD
 -----------------------------------------------------------------------*/
void logFileStart()
{
        DateTime now = rtc.now();

        rtc.now(); // Update the current date and time

        // Initialize SD card
        Serial.println("Initializing SD card");
        // Hardware SS pin
        pinMode(SS, OUTPUT);

        if (!SD.begin(chipSelect)) {
                Serial.println("initialization failed!");
                statusLED(-1);
                return;
        }

        // If the file exists, we want to delete it. 
        if (SD.exists(logFilename))
        {
                SD.remove(logFilename);
        }

        // Open the file for writing, here just for a title.
        logFile = SD.open(logFilename, FILE_WRITE);

        // if the file opened okay, write to it:
        if (logFile) {
                logFile.println("-----OpenSourceQuad-----");
                logFile.println();
                logFile.print("Software version: ");
                logFile.println(SOFTWARE_VERSION);
                logFile.print(now.year());
                logFile.print("/");
                logFile.print(now.month());
                logFile.print("/");
                logFile.print(now.day());
                logFile.print("  ");
                logFile.print(now.hour());
                logFile.print(":");
                logFile.print(now.minute());
                logFile.print(":");
                logFile.println(now.second());
                logFile.println("Runtime data: ");
        } 
        else {
                // if the file didn't open, print an error:
                Serial.println("error opening file");
        }

        logFile.close();

}

/*=========================================================================
 Main Setup
 -----------------------------------------------------------------------*/
void setup()
{ 
        Serial.begin(115200); 
        Serial.println();
        Wire.begin();

        // Initialize various LED outputs
        pinMode(GREEN_LED1, OUTPUT);
        pinMode(GREEN_LED2, OUTPUT);
        pinMode(GREEN_LED3, OUTPUT);
        pinMode(YELLOW_LED1, OUTPUT);
        pinMode(YELLOW_LED2, OUTPUT);
        pinMode(YELLOW_LED3, OUTPUT);
        digitalWrite(GREEN_LED1, LOW);
        digitalWrite(GREEN_LED2, LOW);
        digitalWrite(GREEN_LED3, LOW);
        digitalWrite(YELLOW_LED1, LOW);
        digitalWrite(YELLOW_LED2, LOW);
        digitalWrite(YELLOW_LED3, LOW);

        // Turn on the yellow LED to signify start of setup
        statusLED(4);

        Serial.println("-----OpenSourceQuad-----");
        Serial.println();
        Serial.print("Software version: ");
        Serial.println(SOFTWARE_VERSION);

        // Open a .txt file for data logging and debugging
        logFileStart();

        // Initialize the sensors.
        // Sensors include: 
        //   - Gyro (InvenSense MPU3050)
        //   - Accel/Magnetometer (LSM303)
        //   - USRF
        //   - GPS module (Adafruit Ultimate)
        //   - RTC Module
        Serial.println("Initializing Sensors");

        while(!initSensor(accel, mag,  gyro, &kinematics));

        barometer.readEEPROM();
        barometer.setSLP(29.908);
        barometer.setOSS(3);

        statusLED(5);
        
        // Start the radio
        receiver.start();
        
        // Receive PID coefficients from basestation
        gotPID = true;
        while(!gotPID)
        {
                scanTelemetry();
        }

        Serial.println("Initializing PID"); // Initialize PID
        PID_init();   

        // Arm and initialize motors
        Serial.println("Initializing ESCs");
        motorControl.calibrateESC();

        Serial.println("Initializing Motors");
        motorControl.startMotors();



        // Initialize the fourth order struct
        setupFourthOrder();

        Serial.println("Initializing Data Logging");
        logFile = SD.open(logFilename, FILE_WRITE);


        Serial.println("Initializing GPS");
        GPS.begin(9600);
        initGPS();

        Serial.println("Setup Complete");
        statusLED(1);    
}

/*===============================================
 Time keeping for polling
 -----------------------------------------------------------------------*/
double t_100Hz;
double t_70Hz;
double t_20Hz;
double t_10Hz;
double t_1Hz;

/*=========================================================================
 _100HzTask
 Process on a 100Hz clock
 -----------------------------------------------------------------------*/
void _100HzTask()
{
        kinematicEvent(0,&accel,&mag,&gyro);

        calculatePID(&pitchPID, kinematics.pitch);
        calculatePID(&rollPID, kinematics.roll);
        
        motorControl.updateMotors(pitchPID.output, rollPID.output, 0.0, altitudePID.output);

        t_100Hz = micros();
}

/*=========================================================================
 _70HzTask
 Process on a 70Hz clock
 -----------------------------------------------------------------------*/
void _70HzTask()
{
        kinematicEvent(1, &accel, &mag, &gyro);

        t_70Hz = micros();
}

/*=========================================================================
 _20HzTask
 Process on a 20Hz clock
 -----------------------------------------------------------------------*/
void _20HzTask()
{
        barometer.updatePTA(); 

        // Print data to the SD logFile, using some RTC data
        logData();

        t_20Hz = micros();
}

/*=========================================================================
 _10HzTask
 Process on a 10Hz clock
 -----------------------------------------------------------------------*/
void _10HzTask()
{
        // Integrate all 3 altitude sensor readings
        kinematics.altitude = getAccurateAltitude(  GPSDATA.altitude, barometer.altitude, analogRead(USRF_PIN)*0.01266762, kinematics.phi, GPSDATA.quality);
        calculatePID(&altitudePID, kinematics.altitude);
        
        checkGPS(); // Check for GPS data fully received, uses ISR

        t_10Hz = micros();
}

/*=========================================================================
 _1HzTask
 Process on a 1Hz clock
 -----------------------------------------------------------------------*/
void _1HzTask()
{
        monitorBatteryVoltage();
        //processBatteryAlarms();
        getGPS_Data();
        t_1Hz = micros();
}

uint32_t timer = micros();
/*=========================================================================
 MAIN CONTROL LOOP
 -----------------------------------------------------------------------*/
void loop()                          
{	
        if(micros() - t_100Hz >= _100HzPeriod)
        {
                statusLED(4);
                _100HzTask();
               statusLED(1);
        }

        if(micros() - t_70Hz >= _70HzPeriod)
        {
                statusLED(4);
                _70HzTask();
                statusLED(1);
        }

        if( (micros() - t_20Hz >= _20HzPeriod))
        {
                statusLED(4);
                _20HzTask();
                statusLED(1);
        }

        if((micros() - t_10Hz  >= _10HzPeriod))
        {
                statusLED(5);
                _10HzTask();
                statusLED(2);
        }


        if((micros() - t_1Hz  >= _1HzPeriod))
        {
                statusLED(6);
                _1HzTask();
                statusLED(3);
        }

        // Stop after some logging is done for debugging
        if (millis() >= 60000)
        {
                logFile.close();
                motorControl.motorDISARM();
                statusLED(-1);
                while(1);
        }
        
        
        scanTelemetry();

        // Track the number of elapsed cycles
        cycleCount++;
}
/**! @ END MAIN CONTROL LOOP. @ !**/


