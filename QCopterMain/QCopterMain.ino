 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013

  Dependancies:
    #include <I2C.h>
    #include <MMA8453_n0m1.h>
    #include <OseppGyro.h>
    #include <DataIntegrator.h>
    #include <LiquidCrystal.h>




    -----------------------------------------------------------------------*/


#include <I2C.h>
#include <MMA8453_n0m1.h>
#include <OseppGyro.h>
#include <DataIntegrator.h>
#include <LiquidCrystal.h>
#include <Wire.h>

 /*=========================================================================
    Class instances
    -----------------------------------------------------------------------*/
MMA8453_n0m1 accel;
OseppGyro gyro;

 /*=========================================================================
    Device settings
    -----------------------------------------------------------------------*/
const int dScaleRange = 2000; // Degrees/sec maximum on gyro sensor
const int DLPF = 6; // DLPF settings on gyro
float g_threshold = 1.00; //Upper threshold for data deletion from accel
float d_threshold = 2.00; //Upper threshold for data deletion from gyro
const int gScaleRange = 2; // +- acceleration maximum on accelerometer
const bool HighDef = true; //Accelerometer readings are high definition

 /*=========================================================================
    Data arrays (global for convenience)
    -----------------------------------------------------------------------*/
float init_offset[6]= {0,0,0,0,0,0}; // Stores the base offsets due to board mounting on both devices
float acceldata[3]; // Stores the procesed accel data (after noise reduction)
float gyrodata[3]; // Stores the processed gyro data (after noise reduction)
float raw_acceldata[3]; // Stores the raw acceleration data as read from the accelerometer
float raw_gyrodata[3]; // Stores the raw rotation data as read from the gyro
float accel_total[3]; //Used in the moving average of accel xyz data
float gyro_total[3]; //Used in the moving average of gyro wx wy wz data
float sensor_buffer[96]; //Used to store previous data from both sensors
float noise_threshold[2];

 /*=========================================================================
    Time keeping variables
    -----------------------------------------------------------------------*/
unsigned long time; //Measures time since the program started (micros). important for integrator
unsigned long current_time; //Time last run _100HzTask
unsigned long elapsed_time; //Time since last run _100HzTask
int sensor_buffer_counter = 0; // Position in sensor_buffer
int _100HzCounter = 0;

 /*=========================================================================
    void get_baseline()
    Finds the baseline offsets for all sensors
    -----------------------------------------------------------------------*/
void get_baseline() {

  int offset_counter = 100;
  float counter = 1;

  Serial.println("Getting baseline offsets...");

  while(counter <= offset_counter)
  {
    accel.update();  // Updates the accelerometer registers
    raw_acceldata[0] = accel.x();
    raw_acceldata[1] = accel.y();
    raw_acceldata[2] = accel.z();
    gyro.update();   // Updates the gyro output registers
    raw_gyrodata[0] = gyro.x();
    raw_gyrodata[1] = gyro.y();
    raw_gyrodata[2] = gyro.z();
    init_offset[0] = (init_offset[0] + raw_acceldata[0] ); // Sum
    init_offset[1] = (init_offset[1] + raw_acceldata[1] );
    init_offset[2] = (init_offset[2] + raw_acceldata[2] );
    init_offset[3] = (init_offset[3] + raw_gyrodata[0] );
    init_offset[4] = (init_offset[4] + raw_gyrodata[1] );
    init_offset[5] = (init_offset[5] + raw_gyrodata[2] );
    counter = counter + 1 ;
    delayMicroseconds(10);
  }

  Serial.println(" ");
  init_offset[0] = (init_offset[0])/offset_counter;
  Serial.print("accelerometer x-offset: ");
  Serial.println(init_offset[0]);
  init_offset[1] = (init_offset[1])/offset_counter;
  Serial.print("accelerometer y-offset: ");
  Serial.println(init_offset[1]);
  init_offset[2] = ((init_offset[2])/offset_counter)+256;
  Serial.print("accelerometer z-offset: ");
  Serial.println(init_offset[2]);
  init_offset[3] = (init_offset[3])/offset_counter;
  Serial.print("gyro x-offset: ");
  Serial.println(init_offset[3]);
  init_offset[4] = (init_offset[4])/offset_counter;
  Serial.print("gyro y-offset: ");
  Serial.println(init_offset[4]);
  init_offset[5] = (init_offset[5])/offset_counter;
  Serial.print("gyro z-offset: ");
  Serial.println(init_offset[5]);
  Serial.println(" ");
}

 /*=========================================================================
    void 100HzTask()
    Performs main tasks involved in control
    -----------------------------------------------------------------------*/
void _100HzTask(int count) {
  // Set the elapsed time to zero
  // Divides the "total" arrays to achieve an average
  divide_100HzCount(count);
  // Removes the 0g offset, removes the 0d/s offset
  remove_offset(acceldata, gyrodata, init_offset);
  //Removes oscillatory noise from signal
  remove_noise(acceldata, gyrodata, noise_threshold);

  //Updates the sensor_buffer[192] which stores previous data in circular format
  update_sensor_buffer();

  //Detects motion from
  // Sets acceldata,gyrodata,accel_total,gyro_total to 0
  zero_my_arrays();

  time = micros();
  current_time = time;
}

/*=========================================================================
    void update_sensors()
    Gets raw data from sensors
    -----------------------------------------------------------------------*/
void update_sensors() {

  accel.update(); //Serial.println("1"); // Debug
  raw_acceldata[0] = accel.x(); //Update the raw_accel array
  raw_acceldata[1] = accel.y();
  raw_acceldata[2] = accel.z();
  gyro.update();
  raw_gyrodata[0] = gyro.x(); // Update the raw_gyro array
  raw_gyrodata[1] = gyro.y();
  raw_gyrodata[2] = gyro.z();

}
 /*=========================================================================
    void update_sensor_buffer()
    Adds data to the circular sensor buffer (used in calculations)
    -----------------------------------------------------------------------*/
void update_sensor_buffer(){

//  Regular addition to the buffer, no rollover
  int i = sensor_buffer_counter;
  //Each data set is 6 pieces of info
  sensor_buffer[i] = acceldata[0]; // 0  //6
  sensor_buffer[i+1] = acceldata[1]; // 1 //7
  sensor_buffer[i+2] = acceldata[2]; // 2 //8
  sensor_buffer[i+3] = gyrodata[0]; // 3 //9
  sensor_buffer[i+4] = gyrodata[1]; //4 //10
  sensor_buffer[i+5] = gyrodata[2]; //5 //11 .... //191
  sensor_buffer_counter = sensor_buffer_counter+6;
  Serial.println(sensor_buffer[i]);
//  Rollover
  if( i == 96 ){
    sensor_buffer_counter =0;
    Serial.println("Buffer update");
  }
}

 /*=========================================================================
    void divide_100HzCount(int _100HzCounter)
    -----------------------------------------------------------------------*/
void divide_100HzCount(int _100HzCounter){
  acceldata[0] = accel_total[0]/_100HzCounter; // Take the 10ms average of the data
  acceldata[1] = accel_total[1]/_100HzCounter;
  acceldata[2] = accel_total[2]/_100HzCounter;
  gyrodata[0] = gyro_total[0]/_100HzCounter;
  gyrodata[1] = gyro_total[1]/_100HzCounter;
  gyrodata[2] = gyro_total[2]/_100HzCounter;
}


 /*=========================================================================
    void array3_zero(float array[])
    -----------------------------------------------------------------------*/
void array3_zero(float array[]) {
  for(int i = 0;i <= 2;i++){
    array[i] = 0;
  }
}

 /*=========================================================================
    void zero_my_arrays() {
    -----------------------------------------------------------------------*/
void zero_my_arrays() {
  array3_zero(accel_total);
  array3_zero(gyro_total);
  array3_zero(acceldata);
  array3_zero(gyrodata);
}


/***************************************MAIN INITIALIZATION ********************************/
void setup() {
  Serial.begin(9600);

  /************************ Gyro initialization ************************/
  Serial.println(" ");

  Serial.println("Initializing gyro:  ");
  Serial.println(" ");

  Serial.print("  - Setting gyro I2C addr... ");
  gyro.setI2CAddr(0x69);
  Serial.println("   Gyro I2C addr set!");
  Serial.println(" ");

  Serial.print("  - Setting gyro data mode... ");
  gyro.dataMode(dScaleRange, DLPF);
  Serial.println("   Gyro data mode set!");
  Serial.println(" ");

  Serial.print("  - Setting gyro 3rd party accel enable... ");
  byte x = 0x0;
  while(x != B100000) {
    gyro.regRead(0x3D,&x);
    gyro.regWrite(0x3D,B00100000); // Sets USER_CTRL to 00100000 to allow 3rd party accel on I2C
  }
  Serial.println("   Gryo 3rd party accel enabled!");
  Serial.println(" ");

  Serial.println("Gyro initialization complete!");
  Serial.println(" ");

  /******************** End Gyro initialization ************************/
  /*********************** Accel initialization ************************/

  Serial.println("Initializing accel:  ");
  Serial.println(" ");
  Serial.print("  - Setting accel I2C addr... ");
  accel.setI2CAddr(0x1D); // I2C address of the acceleromter
  Serial.println("   Accel I2C addr set!");
  Serial.println(" ");
  Serial.print("  - Setting accel data mode... ");
  accel.dataMode(HighDef,gScaleRange); //
  Serial.println("   Accel data mode set!");
  Serial.println(" ");
  Serial.println("Accel initialization complete!");
  Serial.println(" ");

  /******************* End Accel initialization ************************/


  get_baseline(); // Sets the offset registers in both sensors to accomodate the starting point
  // MAKE SURE IT STARTS LEVEL =O
  time = micros();
  current_time = time;
  noise_threshold[0] = g_threshold; // Threshold for noise on accel
  noise_threshold[1] = d_threshold; // Threshold for noise on gyro

}
/*********************************END MAIN INITIALIZATION *********************************/

void loop() {

  time = micros(); /* Updates the time, in microseconds */
  elapsed_time = time - current_time;

  update_sensors();
  if(elapsed_time >= 10000){
     _100HzTask(_100HzCounter); // After noise reduction and processing of sensors, do functions
    elapsed_time = 0;
  }
  else {
    _100HzCounter = get_total(accel_total, gyro_total, raw_acceldata, raw_gyrodata, elapsed_time,_100HzCounter);
    // Carries a running total
  }
}

