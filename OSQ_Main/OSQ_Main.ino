/**========================================================================
 //*//**   OpenSourceQuad   *//**//**
 OSQ_Main.ino **/
/** ===============================================================================
 * 
 * 	Author	        : Brandon Riches
 * 	Date		: August 2013
 * 	License		: GNU Public License
 * 
 * 	This library interfaces with the BMP085 pressure sensor to return temperature
 * 	calibrated altitude measurements.
 * 
 * 	Copyright (C) 2013  Brandon Riches
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 	----------------------------------------------------------------------------*/
#include "OSQ_Kinematics.h"
#include "OSQ_SENSORLIB.h"
#include "OSQ_Quadcopter.h"
#include "OSQ_Motors.h"
#include "OSQ_BMP085.h"
#include "OSQ_NoWire.h"
#include "OSQ_GPS.h"
#include "OSQ_altitudeProcessor.h"
#include "OSQ_BatteryMonitor.h"

#include <RTClib.h>
#include <Adafruit_GPS.h>         
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <I2C.h>
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
double setPitch = 0, pitchPID_out;
double setRoll = 0, rollPID_out;

int PID_SampleTime = 10; // Sample time for PID controllers in ms

#define angleKp 35                        // TODO:
#define angleKi 85
#define angleKd 30
PID pitchPID(&kinematics.pitch,  &pitchPID_out,  &setPitch,   angleKp,  angleKi,  angleKd,  DIRECT);
PID rollPID(&kinematics.roll,   &rollPID_out,  &setRoll,   angleKp,  angleKi,  angleKd,  DIRECT);


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
void scanTelemetry()
{
        switch(receiver.ScanForMessages())
        {
        case err:
                break;

        case disarm:
                motorControl.motorDISARM();
                break;

                // Put messages here
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
        // Output limits on the PID controllers.
        // Both the alpha and the beta controller use these limits.
        // They represent the maximum absolute value that the PID equation could reach,
        // regardless of what the gain coefficients are. 
        int pitch_roll_PID_OutLims[] = {
                -100,100        };
        pitchPID.SetMode(AUTOMATIC);
        pitchPID.SetSampleTime(PID_SampleTime);	                 
        pitchPID.SetOutputLimits(pitch_roll_PID_OutLims[0],pitch_roll_PID_OutLims[1]);	
        rollPID.SetMode(AUTOMATIC);
        rollPID.SetSampleTime(PID_SampleTime);	               
        rollPID.SetOutputLimits(pitch_roll_PID_OutLims[0],pitch_roll_PID_OutLims[1]);

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
                logFile.print(pitchPID_out);
                logFile.print(",");
                logFile.println(rollPID_out);
        }
        else
        {
                Serial.println("Error opening file!");
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

        // Hardware SS pin must be output. 
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

        pitchPID.Compute();
        rollPID.Compute();

        motorControl.updateMotors(pitchPID_out, rollPID_out, 0.0, 0.0);

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
        scanTelemetry();

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
        processBatteryAlarms();

        getGPS_Data();
        
        
        

        t_1Hz = micros();
}

uint32_t timer = micros();
/*=========================================================================
 MAIN CONTROL LOOP
 -----------------------------------------------------------------------*/
void loop()                          
{		
        bool priorityFlag = false;
        if(t_100Hz - micros() >= _100HzPeriod)
        {
                statusLED(4);
                _100HzTask();
                priorityFlag = true;
               statusLED(1);
        }

        if(t_70Hz - micros()  >= _70HzPeriod)
        {
                statusLED(4);
                _70HzTask();
                statusLED(1);
        }

        if( (t_20Hz - micros()  >= _20HzPeriod) && !priorityFlag)
        {
                statusLED(4);
                _20HzTask();
                statusLED(1);
        }

        if((t_10Hz - micros()  >= _10HzPeriod) && !priorityFlag)
        {
                statusLED(4);
                _10HzTask();
                statusLED(1);
        }


        if((t_1Hz - micros()  >= _1HzPeriod) && !priorityFlag)
        {
                statusLED(4);
                _1HzTask();
                statusLED(1);
        }

        // Stop after some logging is done for debugging
        if (millis() >= 60000)
        {
                logFile.close();
                motorControl.motorDISARM();
                statusLED(-1);
                while(1);
        }

        // Track the number of elapsed cycles
        cycleCount++;
}
/**! @ END MAIN CONTROL LOOP. @ !**/


