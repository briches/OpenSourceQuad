 /*=========================================================================
 
*//*//*   OpenSourceQuad   *//*//*
Name: OSQ_PID_Tuner.ino
Authors: Brandon Riches
         With some help from: Branden Yue, Andrew Coulthard
Date: 2013

  Using the PID autotune library from Brett Beauregard to find reasonable coefficients
        
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

#include <PID_AutoTune_v0.h>

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
double Kp = 0;
double Ki = 0;
double Kd = 0;
PID aPID(&kinematics.pitch,  &pitchPID_out,  &set_a,   Kp,  Ki,  Kd,  DIRECT);
PID bPID(&kinematics.roll,   &rollPID_out,  &set_b,   Kp,  Ki,  Kd,  DIRECT);

PID_ATune aTune(&kinematics.roll, &rollPID_out);
SoftwareSerial mySerial(10, 11);

boolean tuning = true;

/*=========================================================================
    Function declarations
    -----------------------------------------------------------------------*/
void PID_init();           

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
  digitalWrite(RED_LED,   LOW);
  
  // Turn on the yellow LED to signify start of setup
  ERROR_LED(2);
  
  // Check that the RTC is running properly
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
  DateTime now = rtc.now();
    
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
  
  // Initialize the AutoTuner.
  aTune.SetNoiseBand(3);
  aTune.SetOutputStep(30);
  aTune.SetLookbackSec(1);
  aTune.SetControlType(1);
  
  mySerial.begin(115200);
  delay(10000);
  boolean receivedValue = false;
  delay(2000); // Wait for XBee to start up
  while (receivedValue == false)
  {
    if(mySerial.available())
    {
      Kp = mySerial.read();
      mySerial.write(Kp);
      delay(2000);
      break;
      
    }
  }

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
  if (millis() > 1000)
  {
    aPID.Compute();
    bPID.Compute();
    mySerial.print(rollPID_out);
    mySerial.print("   ");
    mySerial.println(motorControl.motorSpeeds.m1_DC);
    if(tuning)
      {
        byte val = (aTune.Runtime());
        if (val!=0)
        {
          tuning = false;
        }
        if(!tuning)
        { //we're done, set the tuning parameters
          double kp = aTune.GetKp();
          double ki = aTune.GetKi();
          double kd = aTune.GetKd();
          mySerial.print("kP: ");
          mySerial.print(kp);
          mySerial.print("kI: ");
          mySerial.print(ki);
          mySerial.print("lD: ");
          mySerial.println(kd);
          motorControl.motorDISARM();
          while(1);
        }
      }
  }  
  
  
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
  int pitch_roll_PID_OutLims[] = {-10000,10000};
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








