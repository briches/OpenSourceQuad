 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013

    Data is stored in the QV public array. Data fields are:
    See the Quadcopter library for fields
        
        
Copyright stuff from all included libraries that I didn't write

  I2C.h   - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  
  MMA8453_n0m1 Library
  Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com .
  
  PID Library
  Brett Beauregard, br3ttb@gmail.com

    -----------------------------------------------------------------------*/
#include <Quadcopter.h>
#include <PID_v1.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <Servo.h>
#include <math.h>
#include <SdFat.h>



/*=========================================================================
    PID output variables and desired setpoints, and settings
    -----------------------------------------------------------------------*/
/*=========================================================================
    State variables
    - The program didnt like having these in the class.
    -----------------------------------------------------------------------*/
double set_a, aPID_out; 		  // PID output for alpha
double set_b, bPID_out;		          // PID output for beta	
int PID_SampleTime = 10;                  // Sample time for PID controllers in ms
int PID_OutLims[] = {-10,10};


double heading;                            // Time integration of wz

/*=========================================================================
    Classes that need to be initialized
    -----------------------------------------------------------------------*/
Quadcopter Quadcopter;
PID aPID(&Quadcopter.alpha,  &aPID_out,  &set_a,   1,  2,  0,  DIRECT);
PID bPID(&Quadcopter.beta,   &bPID_out,  &set_b,   1,  2,  0,  DIRECT);
void PID_init();                           // Function declaration

void setup()
{
  Serial.begin(19200);                     // Later we will just take out all the "Serial" commands. No need, just write to file
  
  while(!Serial) {
    ;                                      // Wait for serial port to connect
  }
  
  Serial.println(" ");
  while(!Quadcopter.initSensor());     // See the control.cpp file for clarification
  PID_init();                          // Initialize PID controllers
  Quadcopter.initMotors();             // Turn on the motors.
  set_a = 0;		               // Setpoint for alpha set to 0. This can be changed later, for control
  set_b = 0;                           // Setpoint for beta set to 0.
}
void loop()                            // Main runtime loop
{
  Quadcopter.update();                 // See control.cpp for clarification
  aPID.Compute();
  bPID.Compute();
  //Serial.println(Quadcopter.alpha);
  Quadcopter.updateMotors(aPID_out, bPID_out);
  
}


void PID_init()                                          // Initializes the PID controllers
{
  Serial.print("InitPID....  ");
  aPID.SetMode(AUTOMATIC);
  aPID.SetSampleTime(PID_SampleTime);	                 // Set sample time
  aPID.SetOutputLimits(PID_OutLims[0],PID_OutLims[1]);	 // Sets the output limits to {-5,5}. Might be managable
  bPID.SetMode(AUTOMATIC);
  bPID.SetSampleTime(PID_SampleTime);	                 // Set sample time
  bPID.SetOutputLimits(PID_OutLims[0],PID_OutLims[1]);   // Output limits to {-5,5}
  Serial.println("D");
}


