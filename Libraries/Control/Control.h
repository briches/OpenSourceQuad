/******************************************************
Library designed to manage, update, and Control the
state of quadcopter

Author  : Brandon Riches
Date    : May 2013


    *Intructions to use*
    1) Call initSensor() in setup().
    2) Call get_Initial_Offsets in setup().
    3) At least, call update() in loop().


******************************************************/
#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <math.h>
#include <Servo.h>

const double Pi = 3.14159;

/*=========================================================================
    I2C Addresses
    -----------------------------------------------------------------------*/
#define Accel_Address   (0x1D)
#define Gyro_Address    (0x69)


 /*=========================================================================
    Device settings (Options for sensors)
    -----------------------------------------------------------------------*/
const int d_ScaleRange = FULL_SCALE_RANGE_250; // x250,x500,x1000,x2000
const int g_ScaleRange = FULL_SCALE_RANGE_2g; // x2g,x4g,x8g
const int DLPF = 0; // 0,1,2,3,4,5,6,7 // See data sheet
const bool HighDef = true; // Is accel output 2byte or 1byte

const int g_threshold = 0.05; //Upper threshold for set zero from accel data
const int d_threshold = 1; //Upper threshold for set zero from gyro data


/*=========================================================================
    INTERNAL ACCELERATION DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct Quadcopter_DS_s
{

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

} Quadcopter_DS;

/*=========================================================================
    DEVICE SETTINGS STRUCTURE
    -----------------------------------------------------------------------*/
typedef struct Device_Settings_s
{
    int d_ScaleRange;
    int g_ScaleRange;
    int DLPF;
    bool HighDef;
    int g_threshold;
    int d_threshold;

} Device_Settings;

/*=========================================================================
    DEVICE SETTINGS STRUCTURE
    -----------------------------------------------------------------------*/
typedef struct Initial_Offsets_s
{
    double ax;                   // Offsets from initial position sensor data
    double ay;
    double az;
    double wx;
    double wy;
    double wz;

} Initial_Offsets;


/*=========================================================================
    CLASS DECLARATION
    -----------------------------------------------------------------------*/
class Control
{
    public:

        bool initSensor();                  // Initializes the two sensors
        bool initMotors();                  // Initializes the 4 motors
        void update();                      // Updates the structure

        Quadcopter_DS           DS;         // Creates the structure
        Device_Settings         Settings;   // Creates the structure

    private:

        Initial_Offsets         Offsets;    // Creates the structure

        MMA8453_n0m1            accel;      // Device classes
        OseppGyro               gyro;
        Servo                   motor1;
        Servo                   motor2;
        Servo                   motor3;
        Servo                   motor4;

        void getSettings(void);                 // Fill settings struct
        void SI_convert(void);                  // Convert to SI
        void get_Initial_Offsets(void);         // Gets the initial Offsets
};
#endif // CONTROL_H_INCLUDED
