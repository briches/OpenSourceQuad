/******************************************************
Library designed to manage, update, and control the
state of quadcopter

*Intructions to use*
1) Call initSensor() in setup().


******************************************************/


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Control.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <I2C.h>


/**************************************************************************/
/*!
    @brief Reads the settings into the structure. Private!
*/
/**************************************************************************/
void control::getSettings()
{
    Settings.d_ScaleRange = d_ScaleRange;
    Settings.g_ScaleRange = g_ScaleRange;
    Settings.DLPF = DLPF;
    Settings.HighDef = HighDef;
    Settings.g_threshold = g_threshold;
    Settings.d_threshold = d_threshold;
}


/**************************************************************************/
/*!
    @brief Initializes the sensors, much like in the original setup
*/
/**************************************************************************/
bool control::initSensor()
{
    getSettings();

    MMA8453_n0m1 accel;
    OseppGyro gyro;

    Serial.println("Intializing gyro: ");
    gyro.setI2CAddr(Gyro_Address);
    gyro.dataMode(Settings.d_ScaleRange, Settings.DLPF);
    byte x=0x0;
    while(x != B100000)
    {
        gyro.regRead(USER_CTRL,&x);
        gyro.regWrite(USER_CTRL,B00100000);
    }
    Serial.println("Gyro init complete!")
    delayMicroseconds(10);

    Serial.println("Initializing accel: ");
    accel.setI2CAddr(Accel_Address);
    accel.dataMode(Settings.HighDef, Settings.gScaleRange);
    Serial.println("Accel init complete!")
    delayMicroseconds(10);

    return true;
}
