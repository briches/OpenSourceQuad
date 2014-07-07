/*=====================================================================
     osq_main
     OpenSourceQuad
     -------------------------------------------------------------------*/
/*================================================================================
 
     Author		: Brandon Riches
     Date		: August 2013
     License		: GNU Public License

     Main file for the OpenSourceQuad flight control firmware

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
#include "kinematics.h"
#include "IMU.h"
#include "quadcopter.h"
#include "motors.h"
#include "BMP085.h"
#include "nowire.h"
#include "GPS.h"
#include "altitudeprocessor.h"
#include "batterymanager.h"
#include "PID.h"
#include "EEPROM.h"

#include <Adafruit_GPS.h>         
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SD.h>
#include <OSQ_Queue.h>

/** Program Specifications **/
int softwareVersionMajor;
int softwareVersionMinor;
long int flightNumber;
uint64_t cycleCount;
bool receivedStartupCommand = false;

/*=========================================================================
 Serial debugger options
 -----------------------------------------------------------------------*/
#define serialDebug        // <- Must be defined to use any of the other debuggers
//#define attitudeDebug     
//#define altitudeDebug
#define rx_txDebug
//#define autoBroadcast
//#define motorsDebug
#define sdDebug
//#define rollPIDdebug
//#define pitchPIDdebug
//#define yawPIDdebug
//#define altPIDdebug
//#define batteryDebug
//#define GPSDebug

/*=========================================================================
 Hardware options
 -----------------------------------------------------------------------*/
//** Battery **/
#define LIPO_3s
//#define 4sLIPO

/*=========================================================================
 Attitude offset calibration
 -----------------------------------------------------------------------*/
// Comment this line to use previously calibrated measurements
//#define newSensorOffsets
#define usePreviousOffsets

/** Math related definitions **/
#define Pi (3.14159265359F) // Its pi.

/*=========================================================================
 Sensor analog pins 
 -----------------------------------------------------------------------*/
#define USRF_PIN (3)        

/*=========================================================================
 SD logging definitions 
 -----------------------------------------------------------------------*/
char logFilename[12];
File logFile;

/*=========================================================================
 scanTelemetry()
 Watches for new commands sent via radio
 -----------------------------------------------------------------------*/
bool gotPID = false;
void scanTelemetry()
{
    unsigned char packet[5] = {
        0xFF, 0x00, 0x00, 0x00, 0x00    };
    int EEPROMselectionPID;

    switch(receiver.ScanForMessages())
    {
    case err:
        {
        }
        break;

    case disarm:
        {
            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.println("Received DISARM command!");
                #endif
            #endif

            motorControl.motorDISARM();
            logFile.close();
            while(1);
        }
        break;

    case autoland:
        {
            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.println("Received auto-land command!");
                #endif
            #endif

            // Auto-land
        }
        break;

    case start:
        {
            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.println("Received start command!");
                #endif
            #endif

            receivedStartupCommand = true;
        }
        break;

    case broadcastData:
        {
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
                Serial1.print(packet[i]);
            }
        }
        break;

    case increaseOperatingPoint:
        {
            motorControl.operatingPoint += 1;
            
            #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received increase operating point: ");
                    Serial.println(motorControl.operatingPoint);
                #endif
            #endif
        }
        break;

    case decreaseOperatingPoint:
        {
            motorControl.operatingPoint -= 1;

            #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received decrease operating point: ");
                    Serial.println(motorControl.operatingPoint);
                #endif
            #endif
        }
        break;
		
    case increaseP:
	{
	    PID_GAINS[pitch].setP += 0.05;
	    PID_GAINS[roll].setP += 0.05;
			
	    #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received increase P: ");
                    Serial.println(PID_GAINS[pitch].setP);
                #endif
            #endif
	}
	break;

    case activateAltitudeHold:
	{
	    altitudeHold = true;
            initializePID(&altitudePID);
            targetAltitude = kinematics.altitude;
            
	    #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received turn on altitude hold");
                #endif
            #endif
	}
	break;
    
    case deactivateAltitudeHold:
	{
	    altitudeHold = false;
            initializePID(&altitudePID);

	    #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received turn off altitude hold");
                #endif
            #endif
	}
	break;
    
    case decreaseP:
	{
	    PID_GAINS[pitch].setP -= 0.05;
	    PID_GAINS[roll].setP -= 0.05;
			
	    #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received decrease P: ");
                    Serial.println(PID_GAINS[pitch].setP);
                #endif
            #endif
	}
	break;
		
    case increaseI:
	{
    	    PID_GAINS[pitch].setI += 0.05;
    	    PID_GAINS[roll].setI += 0.05;
    			
    	    #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received increase I : ");
                    Serial.println(PID_GAINS[pitch].setI);
                #endif
            #endif
	}
	break;
		
    case decreaseI:
        {
            PID_GAINS[pitch].setI -= 0.05;
            PID_GAINS[roll].setI -= 0.05;
    			
            #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received decrease I: ");
                    Serial.println(PID_GAINS[pitch].setI);
                #endif
            #endif
        }
        break;
	
    case increaseD:
        {
            PID_GAINS[pitch].setD += 0.05;
            PID_GAINS[roll].setD += 0.05;
			
            #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received increase D: ");
                    Serial.println(PID_GAINS[pitch].setD);
                #endif
            #endif
	}
	break;
		
    case decreaseD:
	{
	    PID_GAINS[pitch].setD -= 0.05;
	    PID_GAINS[roll].setD -= 0.05;
			
	    #ifdef serialDebug
                #ifdef rx_txDebug
                    Serial.print("Received decrease D: ");
                    Serial.println(PID_GAINS[pitch].setD);
                #endif
            #endif
	}
	break;

    case increasePitch:
        {
            double newSetpoint = incrementSetpoint(&pitchPID, 1);

            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.print("Received increase pitch: ");
                            Serial.println(newSetpoint);
                #endif
            #endif
        }
        break;


    case decreasePitch:
        {
            double newSetpoint = incrementSetpoint(&pitchPID, -1);

            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.print("Received decrease pitch: ");
                            Serial.println(newSetpoint);
                #endif
            #endif
        }
        break;

    case increaseRoll:
        {
            double newSetpoint = incrementSetpoint(&rollPID, 1);

            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.print("Received increase roll: ");
                            Serial.println(newSetpoint);
                #endif  
            #endif
        }
        break;

    case decreaseRoll:
        {
            double newSetpoint = incrementSetpoint(&rollPID, -1);

            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.print("Received decrease roll: ");
                            Serial.println(newSetpoint);
                #endif        
            #endif
        }
        break;

    case resetPitchRoll:
        {
            // Reset the values we are holding onto.
            pitchPID.setIntegratedError = 0;
            rollPID.setIntegratedError = 0;


            #ifdef NESTED_PID
                        rollPID.rateIntegratedError = 0;
                        pitchPID.rateIntegratedError = 0;
                        kinematics.pitchRate = 0;
                        kinematics.rollRate = 0;
            #endif

            kinematics.pitch = 0;
            kinematics.roll = 0;
            kinematics.rollRate = 0;

            #ifdef serialDebug
                #ifdef rx_txDebug
                            Serial.println("Received reset Pitch and Roll command! ");                          
                #endif
            #endif

            break;
        }
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
        // Log something
        logFile.print(millis());
        logFile.print(',');
        logFile.print(kinematics.altitude);
        logFile.print(',');
        logFile.print(targetAltitude);
        logFile.print(',');
        logFile.print(kinematics.climbRate);
        logFile.print(',');
        logFile.print(altitudePID.output);
        logFile.print(',');
        logFile.println(altitudePID.setpoint);
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

    if (!SD.begin(SS)) 
    {
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
		logFile.println("Starting PID values: ");
		#ifdef SINGLE_PID
		logFile.print("P: "); logFile.print(ATT_KP);
		logFile.print("I: "); logFile.print(ATT_KI);
		logFile.print("D: "); logFile.println(ATT_KD);
		#endif
		#ifdef NESTED_PID
		logFile.print("P: "); logFile.print(SET_ATT_KP);
		logFile.print(" I: "); logFile.print(SET_ATT_KI);
		logFile.print(" D: "); logFile.print(SET_ATT_KD);
		logFile.print(" Vel feedforward "); logFile.print(RATE_ATT_KP);
		logFile.print(" Acc "); logFile.print(RATE_ATT_KI);
		logFile.print(" Jerk "); logFile.println(RATE_ATT_KD);
		#endif
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
 barometerInit
 - Initializes the barometer
 -----------------------------------------------------------------------*/
long barometerInit()
{
    // Take 10 readings of altitude to calibrate
    long initialAltitude, result;
    int count;
    for(int i = 0; i<60; i++)
    {
        barometer.updatePTA();
        delay(35); // Delay 35 ms for ADC
        if(barometer.conversionStep == 1)
        {
            initialAltitude += 1000*barometer.altitude;
            count++;
        }
    }
    result = initialAltitude / count;
    return result;
}

/*=========================================================================
 -----------------------------------------------------------------------*/
/*=========================================================================
 ******************************* Main Setup *******************************
 -----------------------------------------------------------------------*/
 /*=========================================================================
 -----------------------------------------------------------------------*/
void setup()
{ 
    delay(100); // Power supply, and various chip boot times

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

    // Use the previously stored sensor offset values
    #ifdef usePreviousOffsets
        sensor_offs_t accelOffsets;
        sensor_offs_t gyroOffsets;
        
        readEEPROMOffsets(0, &accelOffsets);
        readEEPROMOffsets(1, &gyroOffsets);
        
        kinematics.io_ax = accelOffsets.x/1000.;
        kinematics.io_ay = accelOffsets.y/1000.;
        kinematics.io_az = accelOffsets.z/1000.;
        
        kinematics.io_wx = gyroOffsets.x/1000;
        kinematics.io_wy = gyroOffsets.y/1000;
        kinematics.io_wz = gyroOffsets.z/1000;
        
//        Serial.println(kinematics.io_ax);
//        Serial.println(kinematics.io_ay);
//        Serial.println(kinematics.io_az);
//        Serial.println(kinematics.io_wx);
//        Serial.println(kinematics.io_wy);
//        Serial.println(kinematics.io_wz);
    #endif
    statusLED(1);

    /*****************************/
    /* Initialize Serial Monitor */
    /*****************************/

    statusLED(4);
    #ifdef serialDebug
        Serial.println("\n------------------------OpenSourceQuad------------------------");
        Serial.println("--------------------------------------------------------------");
    #endif
    statusLED(1);

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
        heartbeat();
        scanTelemetry();
    }
    
    statusLED(1);
    
    /*****************************/
    /* Initialize data file. */
    /*****************************/

    #ifdef serialDebug
        Serial.println("Initializing SD Card");
    #endif
    
    flightNumber = 1337;
    ltoa(flightNumber,logFilename,10);
    strcat(logFilename,"log");
    strcat(logFilename,".txt");

    // Open a .txt file for data logging and debugging
    logFileStart();
    logFile = SD.open(logFilename, FILE_WRITE);


    /*****************************/
    /* Initialize sensors. */
    /*****************************/

    #ifdef serialDebug
        Serial.println("Initializing Sensors");
    #endif
    statusLED(6);

    while(!initSensor(accel, mag,  gyro, &kinematics));

    // Get new sensor offsets and write them to the EEPROM
    #ifdef newSensorOffsets
        sensor_offs_t accelOffsets;
        sensor_offs_t gyroOffsets;
        Serial.println("Getting new sensor offsets...");
        getInitialOffsets(&kinematics, accel, mag, gyro);
        
        accelOffsets.x = (long)(1000*kinematics.io_ax);
        accelOffsets.y = (long)(1000*kinematics.io_ay);
        accelOffsets.z = (long)(1000*kinematics.io_az);
        
        gyroOffsets.x = (long)(1000*kinematics.io_wx);
        gyroOffsets.y = (long)(1000*kinematics.io_wy);
        gyroOffsets.z = (long)(1000*kinematics.io_wz);
        
        Serial.println(kinematics.io_ax);
        Serial.println(kinematics.io_ay);
        Serial.println(kinematics.io_az);
        Serial.println(kinematics.io_wx);
        Serial.println(kinematics.io_wy);
        Serial.println(kinematics.io_wz);
        
        Serial.println("Writing accel offsets to E2PROM");
        writeEEPROMOffsets(0, &accelOffsets);
        Serial.println("Writing gyro offsets to E2PROM");
        writeEEPROMOffsets(1, &gyroOffsets);
    #endif
    
    /** Initialize barometric sensor **/
    barometer.readEEPROM();
    barometer.setSLP(30);
    barometer.setOSS(3);
    // Set the initial altitude
    baroSensor.initial = barometerInit() - 0.05 * 1000;
    
    for(int i = 0; i < 40; i++) baroSensor.filt[i] = baroSensor.initial;
    
    /** Initialize accelerometer filter **/
    setupCheby2();

    /** Initialize GPS **/
    #ifdef serialDebug
        Serial.println("Initializing GPS");
    #endif

    GPS.begin(9600);
    initGPS();

    statusLED(3);

    /*****************************/
    /* Write some EEPROM config data */
    /*****************************/
    //writeConfigBlock();

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
	
	
	//Set the timestamp to now so angle doesnt fly out of control
    startTime = millis();
    kinematics.timestamp = startTime;
    kinematics.pitch = 0;
    kinematics.roll = 0;

    statusLED(1);
}

/*=========================================================================
 Time keeping for polling
 -----------------------------------------------------------------------*/
double t_200Hz;
double t_70Hz;
double t_20Hz;
double t_10Hz;
double t_1Hz;

/*=========================================================================
 fastTask
 Process on a 200Hz clock
 -----------------------------------------------------------------------*/
void fastTask()
{
    kinematicEvent(0,&accel,&mag,&gyro, &logFile, pitchPID.setpoint);
    
    /* Startup time */
    if(millis() - startTime < startupPeriod)  // In OSQ_kinematics.h
    {
    	pitchPID.setpoint = kinematics.pitch;
	rollPID.setpoint = kinematics.roll; 
	yawPID.setpoint = kinematics.yaw;
        altitudePID.setpoint = 0;
    }

    // Run PID algorithm on attitude axes
    double rollOut = calculatePID(&rollPID, kinematics.roll, kinematics.rollRate);
    double pitchOut = calculatePID(&pitchPID, kinematics.pitch, kinematics.pitchRate);
    double yawOut = calculatePID(&yawPID, kinematics.yaw, kinematics.yawRate);
    
    /** Update motors with PID outputs **/
    motorControl.updateMotors(pitchOut, rollOut, yawOut, 0.0);

    t_200Hz = micros(); 

    #ifdef serialDebug        // Debug Section
        #ifdef rollPIDdebug
            Serial.print("Roll: ");
            Serial.print(kinematics.roll);
            Serial.print(" motorOutput: ");
            Serial.println(rollPID.output);
            Serial.println(); 
        #endif
        
        #ifdef pitchPIDdebug
            Serial.print("Pitch: ");
            Serial.print(kinematics.pitch);
            Serial.print(" Setpoint: ");
            Serial.print(pitchPID.setpoint);
            Serial.print(" motorOutput: ");
            Serial.println(pitchPID.output);
            Serial.println();
        #endif
		
	#ifdef yawPIDdebug
	    Serial.print("Yaw: ");
	    Serial.print(kinematics.yaw);
	    Serial.print(" Setpoint: ");
	    Serial.print(yawPID.setpoint);
	    Serial.print(" motor output: ");
	    Serial.println(yawPID.output);
    	    Serial.println();
	#endif
        
        #ifdef attitudeDebug
            Serial.print(" Pitch: ");
            Serial.print(kinematics.pitch);
            Serial.print(" Roll: ");
            Serial.print(kinematics.roll);
            Serial.print(" Yaw: ");
            Serial.println(kinematics.yaw);
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
    t_70Hz = micros();
}

/*=========================================================================
 _20HzTask
 Process on a 20Hz clock
 -----------------------------------------------------------------------*/
void _20HzTask()
{
    // Pending shipment of hardware
    barometer.updatePTA(); 

    kinematicEvent(1, &accel, &mag, &gyro, &logFile, rollPID.setpoint);

    // Print data to the SD logFile
    logData();

    t_20Hz = micros();
}

/*=========================================================================
 _10HzTask
 Process on a 10Hz clock
 -----------------------------------------------------------------------*/
void _10HzTask()
{
    double altUSRF = analogRead(USRF_PIN)*0.01266762;
    double maxAllowedChange;
    
    // Integrate the different sensor readings and calculate vertical rate, assuming
    // a perfect 10Hz loop.
    maxAllowedChange = 2 * kinematics.prevClimbRate / 0.1; // Twice the distance we would have moved previously.
    // Get the new altitude estimate based on all the available sensors
    kinematics.altitude = getAccurateAltitude(GPSDATA.altitude, barometer.altitude, altUSRF, kinematics.phi, GPSDATA.quality, maxAllowedChange);
    kinematics.prevClimbRate = kinematics.climbRate;
    // A quick 3 term running average on the climb rate. Should help to deal with crazy rangefinder readings
    kinematics.climbRate = ((kinematics.altitude - previousAltitude)/0.1 + kinematics.climbRate + kinematics.prevClimbRate)/3;
    
    
    if(kinematics.altitude > 0.5 && !inFlight || altitudeHold)
        inFlight = true;
    
    // Setpoint is target rate multiplied by a constant gain
    double climbGain = 0.5;
    altitudePID.setpoint = climbGain * (targetAltitude - kinematics.altitude);
    
    /// PID algorithm for altitude
    double altOut = 0;
    if(altitudeHold)
        altOut = calculatePID(&altitudePID, kinematics.climbRate, 0.);   
    else
        altOut = 0;
    
    // Update motors with altitude hold output
    motorControl.updateMotors(0., 0., 0., altOut);
    
    #ifdef serialDebug
        #ifdef altitudeDebug
            Serial.print("Alt: ");
            Serial.print(kinematics.altitude);
            Serial.print(" USRF: ");
            Serial.print(altUSRF);
            Serial.print(" Baro: ");
            Serial.print(barometer.altitude);
            Serial.print(" GPS: ");
            Serial.println(GPSDATA.altitude);
        #endif
        
        #ifdef altPIDdebug
            Serial.print("Alt: ");
            Serial.print(kinematics.altitude);
            Serial.print("Rate: ");
            Serial.print(kinematics.climbRate);
            Serial.print("Target: ");
            Serial.print(altitudePID.setpoint);
            Serial.print("PID out: ");
            Serial.println(altOut);
        #endif
    #endif

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
    monitorVoltage();
    processBatteryAlarms();
    getGPS_Data();
    t_1Hz = micros();

    #ifdef serialDebug
        #ifdef batteryDebug
            Serial.print("Voltages: ");
            Serial.print(battery.voltage[0]); Serial.print(",");
            Serial.print(battery.voltage[1]); Serial.print(",");
            Serial.print(battery.voltage[2]);
            Serial.print(" Alarms: ");
            Serial.print(" Soft = "); Serial.print(softAlarm);
            Serial.print(" Critical = "); Serial.println(criticalAlarm);
        #endif
        
        #ifdef GPSDebug
        
        #endif
    #endif
}

/*=========================================================================
 -----------------------------------------------------------------------*/
/*=========================================================================
 ******************************* Main Loop *******************************
 -----------------------------------------------------------------------*/
 /*=========================================================================
 -----------------------------------------------------------------------*/
void loop()                          
{	
    // Check if we have enough time to safely do non-critical processes
    static bool priority = true;

    if(priority)
    {
        statusLED(4);
        fastTask();
	priority = false;
        statusLED(1);
    }
    else priority = true;

    // 70 Hz process
    if(micros() - t_70Hz >= _70HzPeriod && !priority)
    {
        statusLED(4);
        _70HzTask(); // 2000 us
	priority = true;
        statusLED(1);
    }

    // 20 Hz process
    if(micros() - t_20Hz >= _20HzPeriod && !priority)
    {
        statusLED(4);
        _20HzTask();
	priority = true;
        statusLED(1);
    }

    // 10 Hz process
    if(micros() - t_10Hz  >= _10HzPeriod && !priority)
    {
        statusLED(5);
        _10HzTask();
	priority = true;
        statusLED(2);
    }

    // 1 Hz process
    if(micros() - t_1Hz  >= _1HzPeriod && !priority)
    {
        statusLED(6);
        _1HzTask();
	priority = true;
        statusLED(3);
    }

    scanTelemetry();
    heartbeat();

    // Track the number of elapsed cycles
    cycleCount++;
}
/**! @ END MAIN CONTROL LOOP. @ !**/


