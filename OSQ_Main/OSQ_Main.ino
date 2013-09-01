 /*=========================================================================
 
*//*//*   OpenSourceQuad   *//*//*
Name: OSQ_Main.ino
Authors: Brandon Riches
         With some help from: Branden Yue, Andrew Coulthard
Date: 2013


TODO:
1) Chebyshev filter. 
2) Fix gyro DC drift. (Measure over time, possibly)
3) Tune PID using ZieglerÃ¢â‚¬â€œNichols method
        
Copyright stuff from all included libraries that I didn't write

  I2C.h   - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  
  PID Library
  Brett Beauregard, br3ttb@gmail.com
  
  Adafruit_GPS
  
  Servo
  
  SoftwareSerial
  
  Wire
  
  SD
    -----------------------------------------------------------------------*/
#include <OSQ_QuadGlobalDefined.h>
#include <OSQ_Kinematics.h>
#include <OSQ_SENSORLIB.h>
#include <OSQ_Quadcopter.h>
#include <OSQ_Motors.h>

#include <RTClib.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>

#define SOFTWARE_VERSION   "V0.9.1"
uint32_t cycleCount;

/*=========================================================================
    Classes and important structures
    -----------------------------------------------------------------------*/
SENSORLIB_accel   	accel;
SENSORLIB_mag	  	mag;
SENSORLIB_gyro         	gyro;
fourthOrderData   	fourthOrderXAXIS,
			fourthOrderYAXIS,
			fourthOrderZAXIS;
kinematicData	  	kinematics;
OSQ_MotorControl   	motorControl;

File                    logFile;
RTC_DS1307              rtc;


/*=========================================================================
    PID output variables and desired setpoints, and settings
    -----------------------------------------------------------------------*/
/*=========================================================================
    State variables
    - The program didnt like having these in the class.
    -----------------------------------------------------------------------*/
// PID output for alpha and beta, PID setpoint vars for alpha and beta
double set_a, pitchPID_out; 		  
double set_b, rollPID_out;	

// Sample time for PID controllers in ms	      	
int PID_SampleTime = 10;

// Constructors for the PID controllers
// The 4th, 5th, and 6th args in the constructors are, respectively:
// - Proportional gain
// - Integral gain
// - Derivative gain
#define Kp 1.500    
#define Ki 2.0    
#define Kd -0.0070
PID aPID(&kinematics.pitch,  &pitchPID_out,  &set_a,   Kp,  Ki,  Kd,  DIRECT);
PID bPID(&kinematics.roll,   &rollPID_out,  &set_b,   Kp,  Ki,  Kd,  DIRECT);


/*=========================================================================
    Function declarations
    -----------------------------------------------------------------------*/
void PID_init();           
void logFileStart();
void getFilename(DateTime now);

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
  
  // Open a .txt file for data logging and debugging
  logFileStart();
    
  // Initialize the sensors.
  // Sensors include: 
  //   - Gyro (InvenSense MPU3050)
  //   - Accel/Magnetometer (LSM303)
  //   - USRF
  //   - GPS module (Adafruit Ultimate)
  //   - RTC Module
  while(!initSensor(accel, 
                    mag, 
                    gyro,
                    &kinematics));
  ERROR_LED(2);
  // Initialize the PID controllers. This is a sub-function, below loop.
  PID_init();   
  
  // Initialize motors. This turns the motors on, and sets them all to a speed
  // just below take-off speed.
  motorControl.calibrateESC();
  motorControl.startMotors();
  delay(50);
  
  // Set both the alpha and beta setpoints to 0. 
  set_a = 0;		               
  set_b = 0;  
  
  // Initialize the fourth order struct
  setupFourthOrder(&fourthOrderXAXIS,
                   &fourthOrderYAXIS,
                   &fourthOrderZAXIS);
                   
  logFile = SD.open(logFilename, FILE_WRITE);
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
  
  Serial.println(cycleCount);
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
  Serial.print("Initializing PID controllers...    ");
  aPID.SetMode(AUTOMATIC);
  aPID.SetSampleTime(PID_SampleTime);	                 
  aPID.SetOutputLimits(pitch_roll_PID_OutLims[0],pitch_roll_PID_OutLims[1]);	
  bPID.SetMode(AUTOMATIC);
  bPID.SetSampleTime(PID_SampleTime);	               
  bPID.SetOutputLimits(pitch_roll_PID_OutLims[0],pitch_roll_PID_OutLims[1]);
  
  Serial.println("Done! ");
  Serial.println();
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
//    logFile.print(motor1s);
//    logFile.print(",");
//    logFile.print(motor2s);
//    logFile.print(",");
//    logFile.print(motor3s);
//    logFile.print(",");
//    logFile.print(motor4s);
//    logFile.print(",");
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
  Serial.println(logFilename);
  DateTime now = rtc.now();
  
  rtc.now(); // Update the current date and time
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  
  // Hardware SS pin must be output. 
  pinMode(SS, OUTPUT);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    ERROR_LED(3);
    return;
  }
  Serial.println("initialization done.");
  
  // If the file exists, we want to delete it. 
  if (SD.exists(logFilename))
  {
    SD.remove(logFilename);
  }
  
  // Open the file for writing, here just for a title.
  logFile = SD.open(logFilename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (logFile) {
    Serial.print("Writing to file");
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
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
  
  logFile.close();

}






