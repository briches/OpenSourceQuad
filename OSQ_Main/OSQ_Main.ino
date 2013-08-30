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

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>

File logfile;

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
boolean XBee_read();
int XBee_send(String data);
void get_telemetry(double* set_a, double* set_b);
void logfileStart();


/*=========================================================================
    Main Setup
    -----------------------------------------------------------------------*/
void setup()
{ 
  // Initialize the main serial UART for output. 
  Serial.begin(19200); 
  
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
  
  // Open a .txt file for data logging and debugging
  logfileStart();
    
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
  
  // Initialize the PID controllers. This is a sub-function, below loop.
  PID_init();   
  
  // Initialize motors. This turns the motors on, and sets them all to a speed
  // just below take-off speed.
  // Enter a %, from 0 - 100
  ERROR_LED(2);
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
  // We print all the data to the SD logfile, for debugging purposes. 
  // Once a completely working build is finished, this may or may not be removed for
  // the sake of speed.
  logfile = SD.open("run_log.txt", FILE_WRITE);
  if (logfile)
  {
    logfile.print(micros());
    logfile.print(",");
    logfile.print(kinematics.pitch);
    logfile.print(",");
    logfile.print(kinematics.roll);
    logfile.print(",");
//    logfile.print(motor1s);
//    logfile.print(",");
//    logfile.print(motor2s);
//    logfile.print(",");
//    logfile.print(motor3s);
//    logfile.print(",");
//    logfile.print(motor4s);
//    logfile.print(",");
    logfile.print(pitchPID_out);
    logfile.print(",");
    logfile.println(rollPID_out);
  }
  logfile.close();
  
  // Close the file after sometime  of logging.
  if (millis() >= 60000)
  {
    motorControl.motorDISARM();
    ERROR_LED(3);
  }

 // Serial.println(motorControl.motorSpeeds.m1_DC);

  
}
/**! @ END MAIN CONTROL LOOP. @ !**/


/*=========================================================================
    logfileStart
    - Initializes a .txt on the uSD
    -----------------------------------------------------------------------*/
void logfileStart()
{
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
  if (SD.exists("run_log.txt"))
  {
    SD.remove("run_log.txt");
  }
  
  // Open the file for writing, here just for a title.
  logfile = SD.open("run_log.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (logfile) {
    Serial.print("Writing to run_log.txt...");
    logfile.println("Time,Ax,Ay,Az,Wx,Wy,Wz,Alpha,Beta,Motor 1,Motor 2,Motor 3,Motor 4,APID,BPID");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening run_log.txt");
  }
  
  logfile.close();

}


// Initializes the PID controllers.
// The function calls in here are pretty straightforwared, and have
// been explained already, essentially.
void PID_init()                                          
{
  // Output limits on the PID controllers.
  // Both the alpha and the beta controller use these limits.
  // They represent the maximum absolute value that the PID equation could reach,
  // regardless of what the gain coefficients are. 
  int PID_OutLims[] = {-100,100};
  Serial.print("Initializing PID controllers...    ");
  aPID.SetMode(AUTOMATIC);
  aPID.SetSampleTime(PID_SampleTime);	                 
  aPID.SetOutputLimits(PID_OutLims[0],PID_OutLims[1]);	
  bPID.SetMode(AUTOMATIC);
  bPID.SetSampleTime(PID_SampleTime);	               
  bPID.SetOutputLimits(PID_OutLims[0],PID_OutLims[1]);
  
  Serial.println("Done! ");
  Serial.println();
}




