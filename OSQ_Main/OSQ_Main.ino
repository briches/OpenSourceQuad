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
#include "OSQ_AltitudeProcessor.h"
#include "OSQ_BatteryMonitor.h"
#include "OSQ_PID.h"
#include "OSQ_EEPROM.h"
#include "OSQ_Kalman.h"

#include <Adafruit_GPS.h>         
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SD.h>

/** Program Specifications **/
int softwareVersionMajor;
int softwareVersionMinor;
unsigned int flightNumber;
uint32_t cycleCount;
bool receivedStartupCommand = false;

/** Debugging Options **/
//#define serialDebug        // <- Must be defined to use any of the other debuggers
//#define attitudeDebug     
//#define altitudeDebug
//#define rx_txDebug
//#define autoBroadcast
//#define motorsDebug
//#define sdDebug
//#define rollPIDdebug
//#define pitchPIDdebug

/** Math related definitions **/
#define Pi (3.14159265359F) // Its pi.

/** Sensor analog pins      **/
#define USRF_PIN (0x0)        


/** SD logging definitions **/
#define chipSelect  (53)
char logFilename[] = "OSQ_Log.txt";
File logFile;


/*=========================================================================
 scanTelemetry()
 Watches for new commands sent via radio
 -----------------------------------------------------------------------*/
bool gotPID = false;
void scanTelemetry()
{
        unsigned char packet[5] = {0xFF, 0x00, 0x00, 0x00, 0x00};
        int EEPROMselectionPID;
        
        switch(receiver.ScanForMessages())
        {
        case err:
                break;

        case disarm:
        
                #ifdef serialDebug
                        #ifdef rx_txDebug
                                Serial.println("Received DISARM command!");
                        #endif
                #endif
                
                motorControl.motorDISARM();
                logFile.close();
                while(1);
                break;
                
        case autoland:
        
                #ifdef serialDebug
                        #ifdef rx_txDebug
                                Serial.println("Received autoland command!");
                        #endif
                #endif
                
                // Autoland
                break;
                
        case start:
        
                #ifdef serialDebug
                        #ifdef rx_txDebug
                                Serial.println("Received start command!");
                        #endif
                #endif
                
                receivedStartupCommand = true;
                commandAllMotors(1300);
                break;
                
        case broadcastData:
        
                #ifdef serialDebug
                        #ifdef rx_txDebug
                                Serial.println("Received broadcast data command!");
                        #endif
                #endif
                // send packets
                packet[0] = 0xFF;
                packet[1] = (unsigned char)(millis() >> 24);
                packet[2] = (unsigned char)((millis() << 8)>>24);
                packet[3] = (unsigned char)((millis() << 16)>>24);
                packet[4] = (unsigned char)((millis() << 24)>>24);
                for(int i = 0; i < 5; i++)
                {
                        Serial3.print(packet[i]);
                }
                break;
                
        case setAngleP:
        
                EEPROMselectionPID = 3;
                
                #ifdef NESTED_PID
                        RATE_ATT_KP = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]);
                #endif
                
                #ifdef SINGLE_PID
                        ATT_KP = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]);
                #endif
                
                EEPROMwritePIDCoefficients(EEPROMselectionPID, receiver.newMessage[DATA2], receiver.newMessage[DATA3]);
                
                
                #ifdef serialDebug
                        #ifdef rx_txDebug
                                Serial.print("Received kP: ");
                                Serial.println(RATE_ATT_KP,3);
                                Serial.print("Wrote PID Value: ");
                                Serial.println(EEPROMreadPIDCoefficients(EEPROMselectionPID));
                        #endif
                #endif
                
                gotPID = true;
                break;
        
        case setAngleI:
              
               EEPROMselectionPID = 4;
               
               #ifdef NESTED_PID
                       RATE_ATT_KI = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]);           
               #endif  
               
               #ifdef SINGLE_PID
                       ATT_KI = (65536 * receiver.newMessage[DATA1] + 256 * receiver.newMessage[DATA2] + receiver.newMessage[DATA3]); 
               #endif
               
               EEPROMwritePIDCoefficients(EEPROMselectionPID, receiver.newMessage[DATA2], receiver.newMessage[DATA3]);
               
               #ifdef serialDebug
                       #ifdef rx_txDebug
                               Serial.print("Received kI: ");
                               Serial.println(RATE_ATT_KI, 3);
                               Serial.print("Wrote PID Value: ");
                               Serial.println(EEPROMreadPIDCoefficients(EEPROMselectionPID));
                       #endif
               #endif
               
               gotPID = true;
               break;
               
        case resetPitchRoll:
                // Reset the values we are holding onto.
                pitchPID.setIntegratedError = 0;
                rollPID.setIntegratedError = 0;
                
                
                #ifdef NESTED_PID
                        rollPID.rateIntegratedError = 0;
                        pitchPID.rateIntegratedError = 0;
                        kinematics.ratePITCH = 0;
                        kinematics.rateROLL = 0;
                #endif
                
                kinematics.pitch = 0;
                kinematics.roll = 0;
                kinematics.rateROLL = 0;
                
                #ifdef serialDebug
                        #ifdef rx_txDebug
                                Serial.println("Received reset Pitch and Roll command! ");                          
                        #endif
                #endif
                
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
        
        initializePID(&pitchPID);
        initializePID(&rollPID);
        initializePID(&yawPID);
        initializePID(&altitudePID);
        
        rollPID.windupGuard = 50; // TODO
        
        #ifdef serialDebug
                #ifdef NESTED_PID
                        Serial.println(SET_ATT_KP);
                        Serial.println(SET_ATT_KI);
                #endif
                #ifdef SINGLE_PID
                        Serial.println(ATT_KP);
                        Serial.println(ATT_KI);
                        Serial.println(ATT_KD);
                #endif
        #endif
        
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
                logFile.print(kinematics.roll);
                logFile.print(",");
                logFile.print(rollPID.output);
                logFile.print(",");
                logFile.print(motorSpeeds[motor2]);
                logFile.print(",");
                logFile.println(motorSpeeds[motor3]);
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
        #ifdef serialDebug
                #ifdef sdDebug
                        Serial.println("----Initializing .txt");
                #endif
        #endif
        
        pinMode(SS, OUTPUT);

        if (!SD.begin(chipSelect)) {
                
                #ifdef serialDebug
                        #ifdef sdDebug
                                Serial.println("--------Initialization failed!");
                        #endif
                #endif
                
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
                logFile.print("Date: ");
                logFile.println(__DATE__);
                logFile.print("Time: ");
                logFile.println(__TIME__);
                logFile.print("Software version: ");
                logFile.print(softwareVersionMajor);
                logFile.print(".");
                logFile.println(softwareVersionMinor);
                logFile.print("Flight Number: ");
                logFile.println(flightNumber);
                logFile.println("Runtime data: \n\n");
        } 
        else {
                // if the file didn't open, print an error:
                #ifdef serialDebug
                        #ifdef sdDebug
                                Serial.println("--------Error opening file!");
                        #endif
                #endif
        }

        logFile.close();

}

/*=========================================================================
 Main Setup
 -----------------------------------------------------------------------*/
void setup()
{ 
        delay(100); // Power supply
        
        #ifdef serialDebug
        Serial.begin(115200); 
        Serial.println();
        #endif
        
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
        
        /*****************************/
        /* Initialize EEPROM */
        /*****************************/
        
        statusLED(4);
        
        writeConfigBlock();
        softwareVersionMinor = EEPROM_read8(software_version_addr);
        softwareVersionMajor = EEPROM_read8(software_version_addr + 1);
        flightNumber = ((EEPROM_read8(flight_number_addr + 1))<<8) | EEPROM_read8(flight_number_addr);
        
        statusLED(1);
        
        /*****************************/
        /* Initialize Serial Monitor */
        /*****************************/
        
        statusLED(4);
        #ifdef serialDebug
                Serial.println("------------------------OpenSourceQuad------------------------");
                Serial.println();
                Serial.print("Software version: ");               
                Serial.print(softwareVersionMajor);
                Serial.print(".");          
                Serial.println(softwareVersionMinor);
                Serial.print("Flight number: ");
                Serial.println(flightNumber);
                Serial.println();
                Serial.println("--------------------------------------------------------------");
        #endif
        statusLED(1);
        

        /*****************************/
        /* Initialize data file. */
        /*****************************/
        
        statusLED(5);
        
        #ifdef serialDebug
                Serial.println("Initializing SD Card");
        #endif
        
        // Open a .txt file for data logging and debugging
        logFileStart();
        logFile = SD.open(logFilename, FILE_WRITE);
        
        statusLED(2);
        

        /*****************************/
        /* Initialize sensors. */
        /*****************************/
        
        #ifdef serialDebug
                Serial.println("Initializing Sensors");
        #endif
        
        statusLED(6);
        
        while(!initSensor(accel, mag,  gyro, &kinematics));
        
        barometer.readEEPROM();
        barometer.setSLP(29.908);
        barometer.setOSS(3);
         
        setupFourthOrder();       // Initialize the fourth order struct
        
        #ifdef serialDebug
                Serial.println("Initializing GPS");
        #endif
        
        GPS.begin(9600);
        initGPS();
        
        statusLED(3);
        
        
        
        /*****************************/
        /* Initialize telemetry */
        /*****************************/
        
        #ifdef serialDebug
                Serial.println("Initializing RX/TX");
        #endif
        
        statusLED(4);
        receiver.start();
        statusLED(1);

        
        /*****************************/
        /* Wait for start command and receive data */
        /*****************************/
        
        #ifdef serialDebug
                Serial.println("Waiting for start command"); 
        #endif
        
        statusLED(-1);
        long timer = micros();
        while(receivedStartupCommand == false) 
        {
                
                #ifdef serialDebug
                        if(micros() - timer > 1000000)
                        {
                                timer = micros();
                                Serial.println("Waiting for start command...");
                        }
                #endif
                scanTelemetry();
        }
        //Set the timestamp to now so angle doesnt fly out of control
        kinematics.timestamp = micros();
        statusLED(1);
        
        
        /*****************************/
        /* Initialize PID */
        /*****************************/
        #ifdef serialDebug
                Serial.println("Initializing PID controllers");
        #endif
        
        statusLED(4);
        PID_init();
        statusLED(1);
        
        
        /*****************************/
        /* Arm and initialize motors */
        /*****************************/
        statusLED(4);
        
        #ifdef serialDebug
                Serial.println("Initializing ESCs");
        #endif
        
        motorControl.calibrateESC();
        
        #ifdef serialDebug
                Serial.println("Initializing Motors");
        #endif
        
        motorControl.startMotors();

        
        #ifdef serialDebug
                Serial.println("Setup Complete");
        #endif
        
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
        
        //double pitchOut = calculatePID(&pitchPID, kinematics.pitch, kinematics.ratePITCH);
        double rollOut = calculatePID(&rollPID, kinematics.roll, kinematics.rateROLL);
        
        // TODO: add other PID calculatePID
        motorControl.updateMotors(0., rollOut, 0., 0.);
        
        // Print data to the SD logFile
        logData();
        
        t_100Hz = micros();
        
        #ifdef serialDebug        // Debug Section
        
                #ifdef rollPIDdebug
                        Serial.print("Roll: ");
                        Serial.print(kinematics.roll);
                        Serial.print(" motorOutput: ");
                        Serial.println(rollPID.output);
                        Serial.println("");
                #endif
                
                #ifdef pitchPIDdebug
                        Serial.print("Roll: ");
                        Serial.print(kinematics.pitch);
                        Serial.print(" motorOutput: ");
                        Serial.println(pitchPID.output);
                        Serial.println("");
                #endif
        
                #ifdef attitudeDebug
                        Serial.print(" Pitch: ");
                        Serial.print(kinematics.pitch);
                        Serial.print(" Roll: ");
                        Serial.print(kinematics.roll);
                        Serial.print(" Yaw: ");
                        Serial.println(kinematics.yaw);
                #endif
                
                #ifdef altitudeDebug
                        Serial.print(" Altitude: ");
                        Serial.print(kinematics.altitude);
                        Serial.print(" GPS: ");
                        Serial.print(GPSDATA.altitude);
                        Serial.print(" Barometer: ");
                        Serial.println(barometer.altitude);
                        
                        Serial.print(" altitudePIDo: ");
                        Serial.println(altitudePID.motorOutput);
                        Serial.println();
                #endif
                
                #ifdef motorsDebug
                        Serial.print(" Motor speeds: ");
                        for(int i = 0; i<4; i++)
                        {
                                Serial.print(motorSpeeds[i]);
                                Serial.print("    ");
                        }
                        Serial.println();
                #endif
        
        #endif
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

        t_20Hz = micros();
}

/*=========================================================================
 _10HzTask
 Process on a 10Hz clock
 -----------------------------------------------------------------------*/
void _10HzTask()
{
        kinematics.altitude = getAccurateAltitude(  GPSDATA.altitude, barometer.altitude, analogRead(USRF_PIN)*0.01266762, kinematics.phi, GPSDATA.quality);
        //calculateRATE_PID(&altitudePID,  measuredRate)
        //TODO: Make sure Altitude PID is working.
        
        //TODO: Make sure GPS is working fully
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
        
        #ifdef serialDebug
        
        #endif
}

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

        scanTelemetry();

        // Track the number of elapsed cycles
        cycleCount++;
}
/**! @ END MAIN CONTROL LOOP. @ !**/


