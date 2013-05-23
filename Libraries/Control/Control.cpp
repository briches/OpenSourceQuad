/******************************************************
Library designed to manage, update, and control the
state of quadcopter

Author  : Brandon Riches
Date    : May 2013


    *Intructions to use*
    1) Call initSensor() in setup().
    2) Call get_Initial_Offsets in setup().
    3) At least, call updateData_State() in loop().


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


/***************************************************************************
 CONTROL
 ***************************************************************************/
/***************************************************************************
 *! @PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief Reads the settings into the structure. Private!
*/
/**************************************************************************/
void Control::getSettings()
{
    Settings.d_ScaleRange = d_ScaleRange;
    Settings.g_ScaleRange = g_ScaleRange;
    Settings.DLPF = DLPF;
    Settings.HighDef = HighDef;
    Settings.g_threshold = g_threshold;
    Settings.d_threshold = d_threshold;
};

/**************************************************************************/
/*!
    @brief Gets the initial offsets in both sensors to accomodate starting
*/
/**************************************************************************/
void Control::get_Initial_Offsets()
{
    int offset_counter = 10;
    int counter = 1;
    float acceldata[3];
    float gyrodata[3];

    Serial.println("Getting baseline offsets...");

    while(counter <= offset_counter)
    {
        accel.update();  // Updates the accelerometer registers
        acceldata[0] = accel.x();
        acceldata[1] = accel.y();
        acceldata[2] = accel.z();
        gyro.update();   // Updates the gyro output registers
        gyrodata[0] = gyro.x();
        gyrodata[1] = gyro.y();
        gyrodata[2] = gyro.z();
        Offsets.ax = (Offsets.ax + acceldata[0] ); // Sum
        Offsets.ay = (Offsets.ay + acceldata[1] );
        Offsets.az = (Offsets.az + acceldata[2] );
        Offsets.wx = (Offsets.wx + gyrodata[0] );
        Offsets.wy = (Offsets.wy + gyrodata[1] );
        Offsets.wz = (Offsets.wz + gyrodata[2] );
        counter = counter + 1 ;

        delayMicroseconds(10);
    }

    Serial.println(" ");
    Offsets.ax = (Offsets.ax)/offset_counter;
    Serial.print("accelerometer x-offset: ");
    Serial.println(Offsets.ax);
    Offsets.ay = (Offsets.ay)/offset_counter;
    Serial.print("accelerometer y-offset: ");
    Serial.println(Offsets.ay);
    Offsets.az = ((Offsets.az)/offset_counter) + 256;
    Serial.print("accelerometer z-offset: ");
    Serial.println(Offsets.az);
    Offsets.wx = (Offsets.wx)/offset_counter;
    Serial.print("gyro x-offset: ");
    Serial.println(Offsets.wx);
    Offsets.wy = (Offsets.wy)/offset_counter;
    Serial.print("gyro y-offset: ");
    Serial.println(Offsets.wy);
    Offsets.wz = (Offsets.wz)/offset_counter;
    Serial.print("gyro z-offset: ");
    Serial.println(Offsets.wz);
    Serial.println(" ");
};

/**************************************************************************/
/*!
    @brief Converts the raw data from sensors to SI units
*/
/**************************************************************************/
void Control::SI_convert()
{
  //Convert accelerometer readouts to m/s^2
    switch(g_ScaleRange) {

        case FULL_SCALE_RANGE_2g:
            Data_State.ax = Data_State.ax * SI_CONVERT_2g;
            Data_State.ay = Data_State.ay * SI_CONVERT_2g;
            Data_State.az = Data_State.az * SI_CONVERT_2g;
            break;

        case FULL_SCALE_RANGE_4g:
            Data_State.ax = Data_State.ax * SI_CONVERT_4g;
            Data_State.ay = Data_State.ay * SI_CONVERT_4g;
            Data_State.az = Data_State.az * SI_CONVERT_4g;
            break;

        case FULL_SCALE_RANGE_8g:
            Data_State.ax = Data_State.ax * SI_CONVERT_8g;
            Data_State.ay = Data_State.ay * SI_CONVERT_8g;
            Data_State.az = Data_State.az * SI_CONVERT_8g;
            break;
    }
    // Convert gyro readouts to degrees/s
    switch(d_ScaleRange) {

        case FULL_SCALE_RANGE_250:
            Data_State.wx = Data_State.wx * SI_CONVERT_250;
            Data_State.wy = Data_State.wy * SI_CONVERT_250;
            Data_State.wz = Data_State.wz * SI_CONVERT_250;
            break;

        case FULL_SCALE_RANGE_500:
            Data_State.wx = Data_State.wx * SI_CONVERT_500;
            Data_State.wy = Data_State.wy * SI_CONVERT_500;
            Data_State.wz = Data_State.wz * SI_CONVERT_500;
            break;

        case FULL_SCALE_RANGE_1000:
            Data_State.wx = Data_State.wx * SI_CONVERT_1000;
            Data_State.wy = Data_State.wy * SI_CONVERT_1000;
            Data_State.wz = Data_State.wz * SI_CONVERT_1000;
            break;

        case FULL_SCALE_RANGE_2000:
            Data_State.wx = Data_State.wx * SI_CONVERT_2000;
            Data_State.wy = Data_State.wy * SI_CONVERT_2000;
            Data_State.wz = Data_State.wz * SI_CONVERT_2000;
            break;
    }
};


/***************************************************************************
 CONTROL
 ***************************************************************************/
/***************************************************************************
 *! @PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief Initializes the sensors, much like in the original setup
*/
/**************************************************************************/
bool Control::initSensor()
{
    getSettings();                                      // Settings struct

    Serial.println("Intializing gyro: ");
    gyro.setI2CAddr(Gyro_Address);
    gyro.dataMode(Settings.d_ScaleRange, Settings.DLPF);
    byte x=0x0;
    while(x != B100000)
    {
        gyro.regRead(USER_CTRL,&x);
        gyro.regWrite(USER_CTRL,B00100000);
    }
    Serial.println("Gyro init complete!");
    delayMicroseconds(10);

    Serial.println("Initializing accel: ");
    accel.setI2CAddr(Accel_Address);
    accel.dataMode(Settings.HighDef, Settings.g_ScaleRange);
    Serial.println("Accel init complete!");
    delayMicroseconds(10);

    get_Initial_Offsets();                              // Initial offsets

    return true;
};

/**************************************************************************/
/*!
    @brief Reads data from sensors
*/
/**************************************************************************/
void Control::updateData_State()
{
    accel.update();
    gyro.update();

    Data_State.ax = accel.x() - Offsets.ax;          // Store Raw values
    Data_State.ay = accel.y() - Offsets.ay;
    Data_State.az = accel.z() - Offsets.az;

    Data_State.wx = gyro.x() - Offsets.wx;
    Data_State.wy = gyro.y() - Offsets.wy;
    Data_State.wz = gyro.z() - Offsets.wz;

    SI_convert();                                    // Convert to SI units

};
