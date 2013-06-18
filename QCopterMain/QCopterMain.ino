 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013
  
    Initialize the Control class by calling: Control QCopter;
    
    Data is stored in the QCopter.Data public array. Data fields are:
        double ax;                   // Basic sensor data
        double ay;
        double az;
        double wx;
        double wy;
        double wz;
        double t_current;            // Time @ call to updateDS();
        double t_previous;           // Time @ previous call to updateDS();
  
    Public members of class Control 
        double alpha;                // Angle between x and z
        double beta;                 // Angle between y and z
        double heading;              // Time integration of wz
        
        
        int motor1s;                 // Motor speed for all 4 motors
        int motor2s;
        int motor3s;
        int motor4s;
        
        
Copyright stuff from all included libraries that I didn't write

  I2C.h   - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  
  MMA8453_n0m1 Library
  Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com .
  
  PID Library
  Brett Beauregard, br3ttb@gmail.com

    -----------------------------------------------------------------------*/
    
#include <Control.h>
#include <PID_v1.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <Servo.h>
#include <math.h>


/*=========================================================================
    SD card variables and class instances
    -----------------------------------------------------------------------*
/*
#include <SD.h>
#include <stdio.h>
const int chipSelect = 10;  
File myFile;
String BUFFER1 = "";
String BUFFER2 = "";
String BUFFER3 = "";
String BUFFER4 = "";
int BUFFER_C1;
int BUFFER_C2;
int BUFFER_C3;
int BUFFER_C4;
int BUF_SEL = 1;
const int myBufLen = 12;
char tstring[15]; 
*/

/*=========================================================================
    PID gain constants
    -----------------------------------------------------------------------*/
double aKp = 1.5;                         // Proportional gain const for alpha
double aKi = 0.1;                         // Integral gain const for alpha
double aKd = 0;                           // Differential gain const for alpha

double bKp = 1.5;                         // Proportional gain const for beta
double bKi = 0.1;                         // Integral gain const for beta
double bKd = 0;                           // Differential gain const for beta

/*=========================================================================
    PID output variables and desired setpoints, and settings
    -----------------------------------------------------------------------*/
double aPID_out; 		          // PID output for alpha
double bPID_out;		          // PID output for beta
double des_a;				  // Setpoint for alpha
double des_b;				  // Setpoint for beta
int PID_SampleTime = 10;                  // Sample time for PID controllers in ms
int PID_OutLims[] = {-10,10};


/*=========================================================================
    Classes that need to be initialized
    -----------------------------------------------------------------------*/
Control QCopter;
PID aPID(&QCopter.alpha,  &aPID_out,  &des_a,   aKp,  aKi,  aKd,  DIRECT);
PID bPID(&QCopter.beta,   &bPID_out,  &des_b,   bKp,  bKi,  bKd,  DIRECT);

void PID_init();            // Function declaration

void setup()
{
  Serial.begin(19200);      // Later we will just take out all the "Serial" commands. No need, just write to file
  
  while(!Serial) {
    ;                       // Wait for serial port to connect
  }
  
  Serial.println(" ");
  QCopter.initSensor();     // See the control.cpp file for clarification
  PID_init();               // Initialize PID controllers
  QCopter.initMotors();     // Turn on the motors.
  des_a = 0;		    // Setpoint for alpha set to 0. This can be changed later, for control
  des_b = 0;                // Setpoint for beta set to 0.
}
void loop()                 // Main runtime loop
{
  QCopter.update();         // See control.cpp for clarification
  aPID.Compute();
  bPID.Compute();
  Serial.println(QCopter.alpha);
 
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


