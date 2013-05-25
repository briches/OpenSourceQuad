/******************************************************
Library designed to manage, update, and control the
state of quadcopter

Author  : Brandon Riches
Date    : May 2013


    *Intructions to use*
    1) Call initSensor() in setup().
    2) Call get_Initial_Offsets in setup().
    3) At the very least, call updateData_State() in loop().


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
    if (Data_State.height >= 400) {Data_State.height = 10;}
    else if( (Data_State.height <= 400) && (Data_State.height > 290)) {Data_State.height = 15;}
    else if( (Data_State.height <= 290) && (Data_State.height > 230)) {Data_State.height = 20;}
    else if( (Data_State.height <= 230) && (Data_State.height > 190)) {Data_State.height = 25;}
    else if( (Data_State.height <= 190) && (Data_State.height > 170)) {Data_State.height = 30;}
    else if( (Data_State.height <= 170) && (Data_State.height > 150)) {Data_State.height = 35;}
    else if( (Data_State.height <= 150) && (Data_State.height > 130)) {Data_State.height = 40;}
    else if( (Data_State.height <= 130) && (Data_State.height > 118)) {Data_State.height = 45;}
    else if( (Data_State.height <= 118) && (Data_State.height > 107)) {Data_State.height = 50;}
    else if( (Data_State.height <= 107) && (Data_State.height > 97)) {Data_State.height = 55;}
    else if( (Data_State.height <= 97) && (Data_State.height > 91)) {Data_State.height = 60;}
    else if( (Data_State.height <= 91) && (Data_State.height > 81)) {Data_State.height = 65;}
    else if( (Data_State.height <= 81) && (Data_State.height > 75)) {Data_State.height = 70;}
    else if( (Data_State.height <= 75) && (Data_State.height > 71)) {Data_State.height = 75;}
    else if( (Data_State.height <= 71) && (Data_State.height > 69)) {Data_State.height = 80;}
    else if( (Data_State.height <= 69) && (Data_State.height > 67)) {Data_State.height = 85;}
    else if( (Data_State.height <= 67) && (Data_State.height > 65)) {Data_State.height = 90;}
    else if( (Data_State.height <= 65) && (Data_State.height > 64)) {Data_State.height = 95;}
    else if( (Data_State.height <= 64) && (Data_State.height > 63)) {Data_State.height = 100;}
    else if( (Data_State.height <= 63) && (Data_State.height > 62)) {Data_State.height = 105;}
    else {Data_State.height = 999;}
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
    @brief Reads data from sensors
*/
/**************************************************************************/
void Control::updateData_State()
{
    accel.update();                                                 // Update the registers storing data INSIDE the sensors
    gyro.update();

    Data_State.ax = accel.x() - Offsets.ax;                         // Store Raw values from the sensor registers
    Data_State.ay = accel.y() - Offsets.ay;                         // and removes the initial offsets
    Data_State.az = accel.z() - Offsets.az;
    Data_State.wx = gyro.x() - Offsets.wx;
    Data_State.wy = gyro.y() - Offsets.wy;
    Data_State.wz = gyro.z() - Offsets.wz;

    Data_State.height = analogRead(0);                              // Reads the analog infrared output

    SI_convert();                                                   // Convert to SI units from raw data type

    if(fabs(Data_State.ax) < Settings.g_threshold) {Data_State.ax = 0;}   // Check if the data is less than the threshold
    if(fabs(Data_State.ay) < Settings.g_threshold) {Data_State.ay = 0;}
    if(fabs(Data_State.az) < Settings.g_threshold) {Data_State.az = 0;}
    if(fabs(Data_State.wx) < Settings.d_threshold) {Data_State.wx = 0;}
    if(fabs(Data_State.wy) < Settings.d_threshold) {Data_State.wy = 0;}
    if(fabs(Data_State.wz) < Settings.d_threshold) {Data_State.wz = 0;}

    Data_State.t_previous = Data_State.t_current;                   // Update the timestamps
    Data_State.t_current = micros();
    double time = (Data_State.t_current - Data_State.t_previous) / (1000000); // Converts time from microseconds to seconds
    Data_State.heading = Data_State.heading + Data_State.wz * (time);// Integrates wz to find the angular displacement

    Data_State.alpha = atan2(Data_State.ax, Data_State.az) *180/Pi ; // Arctan of the two values returns the angle,
    Data_State.beta = atan2(Data_State.ay, Data_State.az) *180/Pi;   // in rads, and convert to degrees.
};
