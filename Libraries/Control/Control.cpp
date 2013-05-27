/******************************************************
Library designed to manage, update, and control the
state of quadcopter

Author  : Brandon Riches
Date    : May 2013


    *Intructions to use*
    1) Call initSensor() in setup().
    2) Call get_Initial_Offsets in setup().
    3) At the very least, call updateData() in loop().


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
#include <math.h>
#include <Servo.h>
#include <PID_v1.h>

using namespace std;

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
    Settings.d_ScaleRange = d_ScaleRange;          // This function is just for
    Settings.g_ScaleRange = g_ScaleRange;          // generality. Allows user
    Settings.DLPF = DLPF;                          // changes of settings.
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
    int offset_counter = 10;                       // # of data sets to consider when finding offsets
    int counter = 1;
    double acceldata[3];
    double gyrodata[3];

    Serial.println("Getting baseline offsets...");

    while(counter <= offset_counter)
    {
        accel.update();                            // Updates the accelerometer registers
        acceldata[0] = accel.x();
        acceldata[1] = accel.y();
        acceldata[2] = accel.z();
        gyro.update();                             // Updates the gyro output registers
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

        delayMicroseconds(10);                     // Delay to ensure accel/gyro can physically keep up
    }

    Serial.println(" ");
    Offsets.ax = (Offsets.ax)/offset_counter;      // Store the average of the readings into the Offsets struct
    Serial.print("accelerometer x-offset: ");
    Serial.println(Offsets.ax);
    Offsets.ay = (Offsets.ay)/offset_counter;
    Serial.print("accelerometer y-offset: ");
    Serial.println(Offsets.ay);
    Offsets.az = ((Offsets.az)/offset_counter) - 256;
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
            Data.ax = Data.ax * SI_CONVERT_2g;
            Data.ay = Data.ay * SI_CONVERT_2g;
            Data.az = Data.az * SI_CONVERT_2g;
            break;

        case FULL_SCALE_RANGE_4g:
            Data.ax = Data.ax * SI_CONVERT_4g;
            Data.ay = Data.ay * SI_CONVERT_4g;
            Data.az = Data.az * SI_CONVERT_4g;
            break;

        case FULL_SCALE_RANGE_8g:
            Data.ax = Data.ax * SI_CONVERT_8g;
            Data.ay = Data.ay * SI_CONVERT_8g;
            Data.az = Data.az * SI_CONVERT_8g;
            break;
    }
    // Convert gyro readouts to degrees/s
    switch(d_ScaleRange) {

        case FULL_SCALE_RANGE_250:
            Data.wx = Data.wx * SI_CONVERT_250;
            Data.wy = Data.wy * SI_CONVERT_250;
            Data.wz = Data.wz * SI_CONVERT_250;
            break;

        case FULL_SCALE_RANGE_500:
            Data.wx = Data.wx * SI_CONVERT_500;
            Data.wy = Data.wy * SI_CONVERT_500;
            Data.wz = Data.wz * SI_CONVERT_500;
            break;

        case FULL_SCALE_RANGE_1000:
            Data.wx = Data.wx * SI_CONVERT_1000;
            Data.wy = Data.wy * SI_CONVERT_1000;
            Data.wz = Data.wz * SI_CONVERT_1000;
            break;

        case FULL_SCALE_RANGE_2000:
            Data.wx = Data.wx * SI_CONVERT_2000;
            Data.wy = Data.wy * SI_CONVERT_2000;
            Data.wz = Data.wz * SI_CONVERT_2000;
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
    getSettings();                                             // Settings struct

    Serial.println("Intializing gyro: ");                      // See OseppGyro.h
    gyro.setI2CAddr(Gyro_Address);                             // Set the I2C address in OseppGyro class
    gyro.dataMode(Settings.d_ScaleRange, Settings.DLPF);       // Set the dataMode in the OseppGyro Class
    byte x=0x0;
    while(x != B100000)
    {
        gyro.regRead(USER_CTRL,&x);                            // See the data sheet for the MPU3050 gyro
        gyro.regWrite(USER_CTRL,B00100000);                    // http://invensense.com/mems/gyro/documents/RM-MPU-3000A.pdf
    }
    Serial.println("Gyro init complete!");
    delayMicroseconds(10);

    Serial.println("Initializing accel: ");                    // Same as the gyro initialization, but the accel isnt an ass
    accel.setI2CAddr(Accel_Address);                           // See the data sheet for the MMA8452Q Accelerometer registers
    accel.dataMode(Settings.HighDef, Settings.g_ScaleRange);   // http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8452Q.pdf?fpsp=1
    Serial.println("Accel init complete!");
    delayMicroseconds(10);

    get_Initial_Offsets();                                     // Initial offsets private function in Control class

    return true;
};

/**************************************************************************/
/*!
    @brief Initializes the motors, code from Andrew's script
*/
/**************************************************************************/
bool Control::initMotors()
{
    Speeds.motor1s = 0;                         // Initialize all motor speeds to 0
    Speeds.motor2s = 0;                         // This might be useful, later
    Speeds.motor3s = 0;
    Speeds.motor4s = 0;
    motor1.attach(11);                      // Attach motor 1 to D11
    motor2.attach(10);                      // Attach motor 2 to D10
    motor3.attach(9);                       // Attach motor 3 to D9
    motor4.attach(6);                       // Attach motor 4 to D8

    // initializes motor1
    for(Speeds.motor1s = 0; Speeds.motor1s < 40; Speeds.motor1s += 5)
    {
      motor1.write(Speeds.motor1s);
      delay(250);
    }

    // initializes motor2
    for(Speeds.motor2s = 0; Speeds.motor2s < 40; Speeds.motor2s += 5)
    {
      motor2.write(Speeds.motor2s);
      delay(250);
    }

    // initializes motor3
    for(Speeds.motor3s = 0; Speeds.motor3s < 40; Speeds.motor3s += 5)
    {
      motor3.write(Speeds.motor3s);
      delay(250);
    }

    // initializes motor4
    for(Speeds.motor4s = 0; Speeds.motor4s < 40; Speeds.motor4s += 5)
    {
      motor4.write(Speeds.motor4s);
      delay(250);
    }

    return true;
}


/**************************************************************************/
/*!
    @brief Reads data from sensors
*/
/**************************************************************************/
void Control::update()
{
    accel.update();                                                 // Update the registers storing data INSIDE the sensors
    gyro.update();

    Data.ax = accel.x() - Offsets.ax;                         // Store Raw values from the sensor registers
    Data.ay = accel.y() - Offsets.ay;                         // and removes the initial offsets
    Data.az = accel.z() - Offsets.az;
    Data.wx = gyro.x() - Offsets.wx;
    Data.wy = gyro.y() - Offsets.wy;
    Data.wz = gyro.z() - Offsets.wz;

    SI_convert();                                           // Convert to SI units from raw data type

    if(fabs(Data.ax) < Settings.g_threshold) {Data.ax = 0;}     // Check if the data is less than the threshold
    if(fabs(Data.ay) < Settings.g_threshold) {Data.ay = 0;}
    if(fabs(Data.az) < Settings.g_threshold) {Data.az = 0;}
    if(fabs(Data.wx) < Settings.d_threshold) {Data.wx = 0;}
    if(fabs(Data.wy) < Settings.d_threshold) {Data.wy = 0;}
    if(fabs(Data.wz) < Settings.d_threshold) {Data.wz = 0;}

    Data.t_previous = Data.t_current;                   // Update the timestamps
    Data.t_current = micros();
    double time = (Data.t_current - Data.t_previous) / (1000000); // Converts time from microseconds to seconds
    heading = heading + Data.wz * (time);// Integrates wz to find the angular displacement

    alpha = atan2(Data.ax, Data.az) *180/Pi ; // Arctan of the two values returns the angle,
    beta = atan2(Data.ay, Data.az) *180/Pi;   // in rads, and convert to degrees.
};
