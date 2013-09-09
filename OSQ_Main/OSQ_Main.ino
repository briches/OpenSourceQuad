/**========================================================================
 //*//**   OpenSourceQuad   *//**//**
 OSQ_Main.ino **/
/** ===============================================================================
 
 	Author	        : Brandon Riches
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
 Classes and important structures
 -----------------------------------------------------------------------*/
SENSORLIB_accel   	accel;
SENSORLIB_mag	        mag;
SENSORLIB_gyro          gyro;
BMP085                  barometer;
fourthOrderData   	fourthOrderXAXIS,
                        fourthOrderYAXIS,
                        fourthOrderZAXIS;
kinematicData	  	kinematics;
OSQ_MotorControl   	motorControl;
File                    logFile;
RTC_DS1307              rtc;
gpsdata_t               GPSDATA;
SoftwareSerial          GPSSerial(13, 12); // TX, RX GPS pins
Adafruit_GPS            GPS(&GPSSerial);
NoWire                  receiver;


/*=========================================================================
 PID output variables and desired setpoints, and settings
 -----------------------------------------------------------------------*/
double setPitch = 0, pitchPID_out;
double setRoll = 0, rollPID_out;

int PID_SampleTime = 10; // Sample time for PID controllers in ms

#define angleKp 35                        // TODO:
#define angleKi 85
#define angleKd 30
PID aPID(&kinematics.pitch,  &pitchPID_out,  &setPitch,   angleKp,  angleKi,  angleKd,  DIRECT);
PID bPID(&kinematics.roll,   &rollPID_out,  &setRoll,   angleKp,  angleKi,  angleKd,  DIRECT);


/*=========================================================================
 SD logging definitions
 -----------------------------------------------------------------------*/
// Hardware SS pin on the ATmega2560
#define chipSelect  (53)
char logFilename[] = "OSQ_Log.txt";

/*=========================================================================
 getTelemetryCommands()
 Watches for new commands sent via radio
 -----------------------------------------------------------------------*/
void getTelemetryCommands()
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
        char c = GPS.read();
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
        int pitch_roll_PID_OutLims[] = {-100,100};
        aPID.SetMode(AUTOMATIC);
        aPID.SetSampleTime(PID_SampleTime);	                 
        aPID.SetOutputLimits(pitch_roll_PID_OutLims[0],pitch_roll_PID_OutLims[1]);	
        bPID.SetMode(AUTOMATIC);
        bPID.SetSampleTime(PID_SampleTime);	               
        bPID.SetOutputLimits(pitch_roll_PID_OutLims[0],pitch_roll_PID_OutLims[1]);

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
                ERROR_LED(3);
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
 _100HzTask
 Process on a 100Hz clock
 -----------------------------------------------------------------------*/
 

/*=========================================================================
 Main Setup
 -----------------------------------------------------------------------*/
void setup()
{ 
        Serial.begin(115200); 
        Serial.println();

        // Join the I2C bus
        Wire.begin();

        // Start the rtc
        rtc.begin();
        
        // Start the radio
        receiver.start();

        // Initialize these pins for digital output.
        // They are used in the ERROR_LED function
        // Use ERROR_LED(1) for success,
        //     ERROR_LED(2) for warnings,
        //     ERROR_LED(3) for critical fail (has a while(1)).
        pinMode(GREEN_LED,   OUTPUT);
        pinMode(YELLOW_LED,  OUTPUT);
        pinMode(RED_LED,     OUTPUT);
        pinMode(USRF_POWER,  OUTPUT);
        digitalWrite(GREEN_LED,  LOW);
        digitalWrite(YELLOW_LED, LOW);
        digitalWrite(RED_LED,    LOW);
        digitalWrite(USRF_POWER, LOW);

        // Turn on the yellow LED to signify start of setup
        ERROR_LED(2);

        // Check that the RTC is running properly
        if (! rtc.isrunning()) {
                Serial.println("RTC is NOT running!");
                // following line sets the RTC to the date & time this sketch was compiled
                rtc.adjust(DateTime(__DATE__, __TIME__));
        }
        DateTime now = rtc.now();

        Serial.println("-----OpenSourceQuad-----");
        Serial.println();
        Serial.print("Software version: ");
        Serial.println(SOFTWARE_VERSION);
        Serial.print(now.year());
        Serial.print("/");
        Serial.print(now.month());
        Serial.print("/");
        Serial.print(now.day());
        Serial.print("  ");
        Serial.print(now.hour());
        Serial.print(":");
        Serial.print(now.minute());
        Serial.print(":");
        Serial.println(now.second());

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

        while(!initSensor(accel, 
        mag, 
        gyro,
        &kinematics));

        barometer.readEEPROM();
        barometer.setSLP(29.908);
        barometer.setOSS(3);

        ERROR_LED(2);

        // Initialize the PID controllers. This is a sub-function, below loop.
        Serial.println("Initializing PID");
        PID_init();   

        // Initialize motors. This turns the motors on, and sets them all to a speed
        // just below take-off speed.
        Serial.println("Initializing ESCs");
        motorControl.calibrateESC();

        Serial.println("Initializing Motors");
        motorControl.startMotors();



        // Initialize the fourth order struct
        setupFourthOrder(&fourthOrderXAXIS,
        &fourthOrderYAXIS,
        &fourthOrderZAXIS);

        Serial.println("Initializing Data Logging");
        logFile = SD.open(logFilename, FILE_WRITE);


        Serial.println("Initializing GPS");
        GPS.begin(9600);
        initGPS();

        Serial.println("Setup Complete");

        ERROR_LED(1);    
}

uint32_t GPS_Timer = millis();
/*=========================================================================
 MAIN CONTROL LOOP
 -----------------------------------------------------------------------*/
void loop()                          
{
        // This is the main runtime function
        mainProcess(    pitchPID_out, 
                        rollPID_out, 
                        &accel, 
                        &mag, 
                        &gyro,
                        &barometer,
                        &kinematics,
                        &fourthOrderXAXIS,
                        &fourthOrderYAXIS,
                        &fourthOrderZAXIS,
                        &motorControl );  

        // Check for GPS data, uses ISR
        checkGPS();
        
        // Integrate all 3 altitude sensor readings
        double altitude = getAccurateAltitude(  GPSDATA.altitude, 
                                                barometer.altitude, 
                                                analogRead(USRF_PIN)*0.01266762, 
                                                kinematics.phi, 
                                                GPSDATA.quality);
        // Scan for instructions                         
        getTelemetryCommands();

        
        if(millis() - GPS_Timer > 1000)
        {
                GPS_Timer = millis();
                
                monitorBatteryVoltage();
                
                processBatteryAlarms();

                getGPS_Data();
                
        }

        // Updates the PID controllers. They return new outputs based on current
        // and past data. These outputs are used to decide what the motor speeds should be set to.
        if (millis() > 3000)
        {
                aPID.Compute();
                bPID.Compute();
        }  
        // Print data to the SD logFile, using some RTC data
        logData();

        // Stop after some logging is done for debugging
        if (millis() >= 60000)
        {
                logFile.close();
                motorControl.motorDISARM();
                ERROR_LED(3);
        }

        // Track the number of elapsed cycles
        cycleCount++;
}
/**! @ END MAIN CONTROL LOOP. @ !**/
