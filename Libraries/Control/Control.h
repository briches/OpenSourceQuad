/******************************************************
Library designed to manage, update, and control the
state of quadcopter


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

    float ax;
    float ay;
    float az;
    float wx;
    float wy;
    float wz;
    float timestamp;
    float alpha;
    float beta;
    float heading;
    float altitude;

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
    CLASS DECLARATION
    -----------------------------------------------------------------------*/
class control
{
    public:

        control();                          // Constructor
        bool initSensor();              // Initializes the two sensors
        void updateData_State();            // Updates the structure


    private:

        Quadcopter_Data_State   Data_State; // Creates the structure
        Device_Settings         Settings;   // Creates the structure
        void getSettings(void)
}
#endif // CONTROL_H_INCLUDED
