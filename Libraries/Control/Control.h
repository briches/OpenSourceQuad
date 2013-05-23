/******************************************************
Library designed to manage, update, and Control the
state of quadcopter

Author  : Brandon Riches
Date    : May 2013


    *Intructions to use*
    1) Call initSensor() in setup().
    2) Call get_Initial_Offsets in setup().
    3) At least, call updateData_State() in loop().


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
const bool HighDef = true;
const int g_threshold = 10; //Upper threshold for set zero from accel data
const int d_threshold = 10;


/*=========================================================================
    INTERNAL ACCELERATION DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct Quadcopter_Data_State_s
{

    double ax;
    double ay;
    double az;
    double wx;
    double wy;
    double wz;
    double t_current;
    double t_previous;
    double alpha;
    double beta;
    double heading;
    double altitude;

} Quadcopter_Data_State;

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
    double ax;
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
        void updateData_State();            // Updates the structure
        Quadcopter_Data_State   Data_State; // Creates the structure

    private:

        Device_Settings         Settings;   // Creates the structure
        Initial_Offsets         Offsets;    // Creates the structure

        MMA8453_n0m1            accel;      // Constructs an instance
        OseppGyro               gyro;       // Constructs an instance

        void getSettings(void);                 // Fill settings struct
        void SI_convert(void);                  // Convert to SI
        void get_Initial_Offsets(void);         // Gets the initial Offsets


};
#endif // CONTROL_H_INCLUDED
