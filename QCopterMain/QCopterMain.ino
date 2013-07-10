 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013


TO- DO:
1) Fix integration of gyro value. (data type?)
2) Complementary filter. 
3) Add PID outputs correctly to motors
        
Copyright stuff from all included libraries that I didn't write

  I2C.h   - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  
  MMA8453_n0m1 Library
  Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com .
  
  PID Library
  Brett Beauregard, br3ttb@gmail.com
    -----------------------------------------------------------------------*/
#include <SENSORLIB.h>
#include <Quadcopter.h>
#include <PID_v1.h>
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
// PID output for alpha and beta, PID setpoint vars for alpha and beta
double set_a, aPID_out; 		  
double set_b, bPID_out;	

// Sample time for PID controllers in ms	      	
int PID_SampleTime = 10;

// Output limits on the PID controllers.
// Both the alpha and the beta controller use these limits.
// They represent the maximum absolute value that the PID equation could reach,
// regardless of what the gain coefficients are. 
int PID_OutLims[] = {-10,10};


/*=========================================================================
    Classes that need to be initialized
    -----------------------------------------------------------------------*/
    
// This is the main class for the Quadcopter driver. 
// Contains most of the methods, variables and other information about state. 
// Refer to Quadcopter.h for info, or @ http://tinyurl.com/kl6uk87 . 
Quadcopter Quadcopter;

// Constructors for the PID controllers
// As it stands right now, there are only two. There will eventually be more, as we
// begin to implement more control on pathing and related controllable variables.
// The 4th, 5th, and 6th args in the constructors are, respectively:
// - Proportional gain
// - Integral gain
// - Derivative gain
PID aPID(&Quadcopter.alpha,  &aPID_out,  &set_a,   1,  2,  0,  DIRECT);
PID bPID(&Quadcopter.beta,   &bPID_out,  &set_b,   1,  2,  0,  DIRECT);

// Function declaration, where many of the PID initialization functions have been moved.
void PID_init();                           


/*=========================================================================
    Main Setup
    -----------------------------------------------------------------------*/
void setup()
{ 
  unsigned long t1, t2;
  double elaps;
  
  t1 = micros();
  
  // Initialize these pins (39,41,43) for digital output.
  // They are used in the ERROR_LED function
  // Use ERROR_LED(1) for success,
  //     ERROR_LED(2) for warnings,
  //     ERROR_LED(3) for critical fail (has a while(1)).
  pinMode(GREEN_LED,   OUTPUT);
  pinMode(YELLOW_LED,  OUTPUT);
  pinMode(RED_LED,     OUTPUT);
  
  
  // Initialize the main serial UART for output. 
  Serial.begin(115200); 
  
  // Wait for serial port to connect
  // This isn't essential, but we might as well.
  while(!Serial) {
    ;                                      
  }
  Serial.println(" ");
  
  // Initialize the sensors.
  // Sensors include: 
  //   - Gyro (InvenSense MPU3050)
  //   - Accel/Magnetometer (LSM303)
  while(!Quadcopter.initSensor());
  
  // Initialize the PID controllers. This is a sub-function, below loop.
  PID_init();                        
  
  // Initialize motors. This turns the motors on, and sets them all to a speed
  // just below take-off speed.
  Quadcopter.ERROR_LED(2);
  // Enter a %, from 0 - 100
  Quadcopter.initMotors(5);
  Quadcopter.ERROR_LED(1);  
  
  // Set both the alpha and beta setpoints to 0. 
  // This is just for starters, eventually, pathing will modify the
  // setpoints en-route to various locations. 
  set_a = 0;		               
  set_b = 0;  
  
  t2 = micros();
  elaps = (t2 - t1)/1000;
  Serial.print("Setup complete!  Total time (ms): ");
  Serial.println(elaps,4);
  Serial.println("");
}

/*=========================================================================
    MAIN CONTROL LOOP
    -----------------------------------------------------------------------*/
void loop()                          
{
  // This is the main data gathering function included in the Quadcopter class.
  //
  // It includes a number of different priorities, based on the time elapsed
  // since it was last called (This is because various sensors or events may
  // have different time intervals between them). 
  // In the most basic priority, the function gathers new magnetometer, 
  // accelerometer, and gryo data. It then runs a basic moving average on these
  // data to smooth them. Then, it uses a complementary filter to help obtain 
  // more accurate readings of angle.
  Quadcopter.update();  

  // Updates the PID controllers. They return new outputs based on current
  // and past data. These outputs are used to decide what the motor speeds should be set to.
  aPID.Compute();
  bPID.Compute();
  
  // Adds the PID outputs to the motors, based on their current configuration
  Quadcopter.updateMotors(aPID_out, bPID_out);
  
  
  
  /* Some debug printing. */
  
  
//  Serial.print("M1s: "); Serial.print(Quadcopter.motor1s);
//  Serial.print(" M2s: "); Serial.print(Quadcopter.motor2s);
//  Serial.print(" M3s: "); Serial.print(Quadcopter.motor3s);
//  Serial.print(" M4s: "); Serial.println(Quadcopter.motor4s);


}
// END MAIN CONTROL LOOP.


// Initializes the PID controllers.
// The function calls in here are pretty straightforwared, and have
// been explained already, essentially.
void PID_init()                                          
{
  unsigned long t1, t2;
  double elaps;
  t1 = micros();
  
  Serial.print("Initializing PID controllers...    ");
  aPID.SetMode(AUTOMATIC);
  aPID.SetSampleTime(PID_SampleTime);	                 
  aPID.SetOutputLimits(PID_OutLims[0],PID_OutLims[1]);	
  bPID.SetMode(AUTOMATIC);
  bPID.SetSampleTime(PID_SampleTime);	               
  bPID.SetOutputLimits(PID_OutLims[0],PID_OutLims[1]);
  
  t2 = micros();
  elaps = (t2-t1);
  Serial.print("Done! Elapsed time (us): ");
  Serial.println(elaps,6);
  Serial.println("");
}


