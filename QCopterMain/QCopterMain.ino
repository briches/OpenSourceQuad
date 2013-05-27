 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013
  
    Initialize the Control class by calling: Control QCopter;
    
//    Data is stored in the QCopter.Data public array. Data fields are:
//        double ax;                   // Basic sensor data
//        double ay;
//        double az;
//        double wx;
//        double wy;
//        double wz;
//        double t_current;            // Time @ call to updateDS();
//        double t_previous;           // Time @ previous call to updateDS();
  
//    State info is store in the QCopter.State publix array. Fields are: 
//        double alpha;                // Angle between x and z
//        double beta;                 // Angle between y and z
//        double heading;              // Time integration of wz
//        int motor1s;                 // Motor speed for all 4 motors
//        int motor2s;
//        int motor3s;
//        int motor4s;
        
        
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

using namespace std;

/*=========================================================================
	PID gain constants
    -----------------------------------------------------------------------*/
double aKp = 0.2;                   // Proportional gain const for alpha
double aKi = 0.1;                     // Integral gain const for alpha
double aKd = 0;                     // Differential gain const for alpha

double bKp = 0.2;                   // Proportional gain const for beta
double bKi = 0.1;                     // Integratl gain const for beta
double bKd = 0;                     // Differential gain const for beta

/*=========================================================================
	PID outputs and desired setpoints
    -----------------------------------------------------------------------*/
double aPID_out; 		          // PID output for alpha
double bPID_out;				  // PID output for beta
double des_a;				  // Setpoint for alpha
double des_b;				  // Setpoint for beta

Control QCopter;
PID aPID(&QCopter.alpha,  &aPID_out,  &des_a,   aKp,  aKi,  aKd,  DIRECT);
PID bPID(&QCopter.beta,   &bPID_out,  &des_b,   bKp,  bKi,  bKd,  DIRECT);



void setup()
{
  Serial.begin(9600);          // Nothing depends on Serial comms, so set to max for speedy ops
  QCopter.initSensor();          // See the control.cpp file for clarification
  
  QCopter.Settings.g_threshold = 0.10; // 0.10 m/s^2 threshold for noise
  QCopter.Settings.d_threshold = 0.5; // 0.5 d/s threshold for noise
  
  aPID.SetMode(AUTOMATIC);
  aPID.SetSampleTime(10);	// Set sample time to 10 ms
  aPID.SetOutputLimits(-5,5);	// Sets the output limits to {-5,5}. Might be managable
  bPID.SetMode(AUTOMATIC);
  bPID.SetSampleTime(10);	// Set sample time to 10 ms
  bPID.SetOutputLimits(-5,5);	// Output limits to {-5,5}
      
  des_a = 0;		// Setpoint set to 0. This can be changed later, for control
  des_b = 0;
  
}
void loop()
{
  QCopter.update();    // See control.cpp for clarification
  aPID.Compute();
  bPID.Compute();
  Serial.print(QCopter.alpha);
  Serial.print(" PID: "); Serial.println(aPID_out);
  
  
}
