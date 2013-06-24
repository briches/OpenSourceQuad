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
  
  
  /*=========================================================================
      SD card variables and class instances
      -----------------------------------------------------------------------*
  
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
  PID aPID(&QV.alpha,  &aPID_out,  &des_a,   aKp,  aKi,  aKd,  DIRECT);
  PID bPID(&QV.beta,   &bPID_out,  &des_b,   bKp,  bKi,  bKd,  DIRECT);
  void PID_init();            // Function declaration
  
  void setup()
  {
    Serial.begin(19200);      // Later we will just take out all the "Serial" commands. No need, just write to file
    
    while(!Serial) {
      ;                       // Wait for serial port to connect
    }
    
    Serial.println(" ");
    initSensor();     // See the control.cpp file for clarification
    PID_init();               // Initialize PID controllers
    initMotors();     // Turn on the motors.
    des_a = 0;		    // Setpoint for alpha set to 0. This can be changed later, for control
    des_b = 0;                // Setpoint for beta set to 0.
  }
  void loop()                 // Main runtime loop
  {
    update();         // See control.cpp for clarification
    aPID.Compute();
    bPID.Compute();
    Serial.println(QV.alpha);
   
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
  

