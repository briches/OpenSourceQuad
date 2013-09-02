 /*=========================================================================
*//*//*   OpenSourceQuad   *//*//*
  OSQ_Main.ino
  Brandon Riches
With some help from: Branden Yue, Andrew Coulthard
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
#include "OSQ_Kinematics.h"
#include "OSQ_SENSORLIB.h"
#include "OSQ_Quadcopter.h"
#include "OSQ_Motors.h"
#include "OSQ_BMP085.h"
#include "OSQ_NoWire.h"

#include <RTClib.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>

/*=========================================================================
    Math related definitions
    -----------------------------------------------------------------------*/
#define Pi  			(3.14159265359F)			// Its pi.


/*=========================================================================
    Sensor analog pins
    -----------------------------------------------------------------------*/

#define VBATT_PIN 	(0x0)
#define USRF_pin 	(0x0)	

/*=========================================================================
    Battery Monitor
    -----------------------------------------------------------------------*/
#define NOMINAL_V	11.1
#define SOFTWARE_VERSION   "V0.9.1"
uint32_t cycleCount;

/*=========================================================================
    Classes and important structures
    -----------------------------------------------------------------------*/
SENSORLIB_accel   	accel;
SENSORLIB_mag	mag;
SENSORLIB_gyro       gyro;
fourthOrderData   	fourthOrderXAXIS,
			                fourthOrderYAXIS,
			                fourthOrderZAXIS;
kinematicData	  	kinematics;
OSQ_MotorControl   	motorControl;
File                                 logFile;
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
PID aPID(&kinematics.pitch,  &pitchPID_out,  &setPitch,   angleKp,  angleKi,  angleKd,  DIRECT);
PID bPID(&kinematics.roll,   &rollPID_out,  &setRoll,   angleKp,  angleKi,  angleKd,  DIRECT);


/*=========================================================================
    Function declarations
    -----------------------------------------------------------------------*/
void PID_init();           
void logFileStart();
void logData();

/*=========================================================================
    SD logging definitions
    -----------------------------------------------------------------------*/
// Hardware SS pin on the ATmega2560
#define chipSelect  (53)
char logFilename[] = "OSQ_Log.txt";

/*=========================================================================
    Main Setup
    -----------------------------------------------------------------------*/
void setup()
{ 
  // Initialize the main serial UART for output. 
  Serial.begin(115200); 
  
  // Join the I2C bus
  Wire.begin();
  
  // Start the rtc
  rtc.begin();
  
  Serial.println(" ");
  
  // Initialize these pins for digital output.
  // They are used in the ERROR_LED function
  // Use ERROR_LED(1) for success,
  //     ERROR_LED(2) for warnings,
  //     ERROR_LED(3) for critical fail (has a while(1)).
  pinMode(GREEN_LED,   OUTPUT);
  pinMode(YELLOW_LED,  OUTPUT);
  pinMode(RED_LED,     OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  
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
  
  Serial.println("Setup Complete");
  ERROR_LED(1);    
}

/*=========================================================================
    MAIN CONTROL LOOP
    -----------------------------------------------------------------------*/
void loop()                          
{
  // This is the main runtime function included in the Quadcopter class.
  //
  // It includes a number of different priorities, based on the time elapsed
  // since it was last called (This is because various sensors or events may
  // have different time intervals between them). 
  // 
  // In the most basic priority, the function gathers new magnetometer, 
  // accelerometer, and gryo data. It then runs a basic moving average on these
  // data to smooth them. Then, it uses a complementary filter to help obtain 
  // more accurate readings of angle
  // Everything's in here because I don't dev in Arduino IDE
  mainProcess(pitchPID_out, 
              rollPID_out, 
              &accel, 
              &mag, 
              &gyro,
              &kinematics,
              &fourthOrderXAXIS,
              &fourthOrderYAXIS,
              &fourthOrderZAXIS,
              &motorControl);  
        
  // Updates the PID controllers. They return new outputs based on current
  // and past data. These outputs are used to decide what the motor speeds should be set to.
  if (millis() > 4000)
  {
    aPID.Compute();
    bPID.Compute();
  }  
  // Print data to the SD logFile, using some RTC data
  logData();
  
  // Stop after some logging is done for debugging
  if (millis() >= 25000)
  {
    logFile.close();
    motorControl.motorDISARM();
    ERROR_LED(3);
  }
  
  Serial.println(kinematics.roll);
  // Track the number of elapsed cycles
  cycleCount++;
}
/**! @ END MAIN CONTROL LOOP. @ !**/

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
  Serial.print("Initializing SD card");
  
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
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
  
  logFile.close();

}






