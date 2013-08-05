 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013


TODO:
1) Chebyshev filter. 
2) Fix gyro DC drift. (Measure over time, possibly)
3) Tune PID using Ziegler–Nichols method
  - http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method .
        
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
#include <SD.h>

File logfile;

// Hardware SS pin on the ATmega2560
const int chipSelect = 53;

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
int PID_OutLims[] = {-1000,1000};


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
#define Kp 1.4
#define Ki 0.014
#define Kd 0
PID aPID(&Quadcopter.alpha,  &aPID_out,  &set_a,   Kp,  Ki,  Kd,  DIRECT);
PID bPID(&Quadcopter.beta,   &bPID_out,  &set_b,   Kp,  Ki,  Kd,  DIRECT);

// Function declaration, where many of the PID initialization functions have been moved.
void PID_init();           

// Function declaration. This function 
void XBee_read();

// Radio transmitted instructions are stored in this character array.
char XBeeArray[80];

bool STOP_FLAG;
bool START_FLAG;


/*=========================================================================
    Main Setup
    -----------------------------------------------------------------------*/
void setup()
{ 
  unsigned long t1, t2;
  double elaps;
  t1 = micros();
  
  // Initialize the main serial UART for output. 
  Serial.begin(115200); 
  
  // Initialize the radio comms serial port for communication with the XBee radios
  //Serial1.begin(19200);
  
  Serial.println(" ");
  
  // Initialize these pins (39,41,43) for digital output.
  // They are used in the ERROR_LED function
  // Use ERROR_LED(1) for success,
  //     ERROR_LED(2) for warnings,
  //     ERROR_LED(3) for critical fail (has a while(1)).
  pinMode(GREEN_LED,   OUTPUT);
  pinMode(YELLOW_LED,  OUTPUT);
  pinMode(RED_LED,     OUTPUT);
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  
  // Hardware SS pin must be output. 
  pinMode(SS, OUTPUT);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    Quadcopter.ERROR_LED(3);
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
    
  // Initialize the sensors.
  // Sensors include: 
  //   - Gyro (InvenSense MPU3050)
  //   - Accel/Magnetometer (LSM303)
  //   - GPS module (Adafruit Ultimate)
  //   - RTC Module
  while(!Quadcopter.initSensor());
  
  // Initialize the PID controllers. This is a sub-function, below loop.
  PID_init();                        
  
  // Initialize motors. This turns the motors on, and sets them all to a speed
  // just below take-off speed.
  Quadcopter.ERROR_LED(2);
  // Enter a %, from 0 - 100
  Quadcopter.initMotors(20);
  while(millis() <= 5000);
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
  
  logfile.close();
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
   
  Quadcopter.update(aPID_out, bPID_out);  

  // Updates the PID controllers. They return new outputs based on current
  // and past data. These outputs are used to decide what the motor speeds should be set to.
  aPID.Compute();
  bPID.Compute();
  
  // We print all the data to the SD logfile, for debugging purposes. 
  // Once a completely working build is finished, this may or may not be removed for
  // the sake of speed.
  logfile = SD.open("run_log.txt", FILE_WRITE);
  if (logfile)
  {
    logfile.print(micros());
    logfile.print(",");
    logfile.print(Quadcopter.ax);
    logfile.print(",");
    logfile.print(Quadcopter.ay);
    logfile.print(",");
    logfile.print(Quadcopter.az);
    logfile.print(",");
    logfile.print(Quadcopter.wx);
    logfile.print(",");
    logfile.print(Quadcopter.wy);
    logfile.print(",");
    logfile.print(Quadcopter.wz);
    logfile.print(",");
    logfile.print(Quadcopter.alpha);
    logfile.print(",");
    logfile.print(Quadcopter.beta);
    logfile.print(",");
    logfile.print(Quadcopter.motor1s);
    logfile.print(",");
    logfile.print(Quadcopter.motor2s);
    logfile.print(",");
    logfile.print(Quadcopter.motor3s);
    logfile.print(",");
    logfile.print(Quadcopter.motor4s);
    logfile.print(",");
    logfile.print(aPID_out);
    logfile.print(",");
    logfile.println(bPID_out);
  }
  logfile.close();
  
  // Close the file after two minutes of logging.
  if (millis() >= 60000)
  {
    while(1);
  }
  
  //XBee_read();
  if (XBeeArray[0] == 'S' || XBeeArray[0] == 's')
  {
    Quadcopter.initMotors(10);
    while(1)
    {
      Quadcopter.ERROR_LED(2);
      XBee_read();
      if(XBeeArray[0] == 'g' || XBeeArray[0] == 'G')
      {
        Quadcopter.ERROR_LED(1);
        break;
        
      }
    }
  }
  
  
}
// END MAIN CONTROL LOOP.

/**

**/
void XBee_read()
{
  // Get the number of bytes available to read
  int bytes = Serial1.available();
  if (bytes > 0)
  {
    // Read the serial data from the modem into the array
    for(int i = 0; i < bytes; i++)
    {
    XBeeArray[i] = (char)Serial1.read();
    }
  }  
}


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


