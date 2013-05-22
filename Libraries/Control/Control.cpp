/******************************************************
Library designed to manage, update, and control the
state of quadcopter

*Intructions to use*
1) Call initSensor() in setup().
2) Call get_Initial_Offsets


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

/**************************************************************************/
/*!
    @brief Gets the initial offsets in both sensors to accomodate starting
*/
/**************************************************************************/
void control:get_Initial_Offsets()
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
}



/**************************************************************************/
/*!
    @brief Reads data from sensors
*/
/**************************************************************************/
void control::updateData_State()
{

}
