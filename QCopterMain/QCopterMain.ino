 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013
  
    Initialize the Control class by calling: Control QCopter;
    
    Data is stored in the QCopter.DS public array. Data fields are:
        double ax;                   // Basic sensor data
        double ay;
        double az;
        double wx;
        double wy;
        double wz;
        double t_current;            // Time @ call to updateDS();
        double t_previous;           // Time @ previous call to updateDS();
        double alpha;                // Angle between x and z
        double beta;                 // Angle between y and z
        double heading;              // Time integration of wz
        double height;               // From Infrared Sensor; Accelerometry is too hard
        int motor1s;                 // Motor speed for all 4 motors
        int motor2s;
        int motor3s;
        int motor4s;


    -----------------------------------------------------------------------*/
    
#include <Control.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <Servo.h>

Control QCopter;

void setup()
{
  Serial.begin(115200);          // Nothing depends on Serial comms, so set to max for speedy ops
  QCopter.initSensor();          // See the control.cpp file for clarification
    
  QCopter.Settings.g_threshold = 0.10; // 0.10 m/s^2 threshold for noise
  QCopter.Settings.d_threshold = 0.5; // 0.5 d/s threshold for noise
}
void loop()
{
  int rate = 0;
  QCopter.update();    // See control.cpp for clarification
  rate = 1/((QCopter.DS.t_current - QCopter.DS.t_previous)/1000000);
  Serial.println(QCopter.DS.alpha);
}
