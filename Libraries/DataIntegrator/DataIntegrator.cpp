/*********************************************************************************************
DataIntegrator.cpp

Author : Brandon Riches
Date   : May, 2013
Version: 0.1

Notes: Contains some necessary functions used in the QCopterMain.ino script.
Dependancies


Changelog:
    V0.1
         - Added
            int get_total
            void remove_offset
            void remove_noise
         - Updated header comments
**********************************************************************************************/
#include <DataIntegrator.h>
#include <math.h>


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
void zero_my_arrays(float accel_total[],float gyro_total[],float acceldata[],float gyrodata[]) {
  array3_zero(accel_total);
  array3_zero(gyro_total);
  array3_zero(acceldata);
  array3_zero(gyrodata);
}

/*=========================================================================
    void SI_convert()
    -----------------------------------------------------------------------*/
void SI_convert(){
    //Convert accelerometer readouts to CM/s^2
    switch(g_ScaleRange) {
        case FULL_SCALE_RANGE_2g:
        Serial.println("Case 1");
            for (int i = 0; i <= 2, i++;) {
                acceldata[i] = acceldata[i] * (SI_CONVERT_2g)*100; // readouts in CM/s
            }
            break;
        case FULL_SCALE_RANGE_4g:
        Serial.println("Case 2");
            for (int i = 0; i <= 2, i++;){
                acceldata[i] = acceldata[i]*SI_CONVERT_4g*100; // readouts in CM/s
            }
            break;
        case FULL_SCALE_RANGE_8g:
        Serial.println("Case 3");
            for (int i = 0; i <= 2, i++;){
                acceldata[i] = acceldata[i]*SI_CONVERT_4g*100; // readouts in CM/s
            }
            break;
    }
    // Convert gyro readouts to degrees/s
    switch(d_ScaleRange) {

        case FULL_SCALE_RANGE_250:
            for (int i = 0; i <= 2, i++;){
                gyrodata[i] = gyrodata[i]*SI_CONVERT_250; // readouts in deg/s
            }
            break;
        case FULL_SCALE_RANGE_500:
            for (int i = 0; i <= 2, i++;){
                gyrodata[i] = gyrodata[i]*SI_CONVERT_500; // readouts in deg/s
            }
            break;
        case FULL_SCALE_RANGE_1000:
            for (int i = 0; i <= 2, i++;){
                gyrodata[i] = gyrodata[i]*SI_CONVERT_1000; // readouts in deg/s
            }
            break;
        case FULL_SCALE_RANGE_2000:
            for (int i = 0; i <= 2, i++;){
                gyrodata[i] = gyrodata[i]*SI_CONVERT_2000; // readouts in deg/s
            }
            break;
    }
}


 /*=========================================================================
    void divide_100HzCount(int mycount)
    -----------------------------------------------------------------------*/
void divide_100HzCount(float acceldata[],float gyrodata[],float accel_total[],float gyro_total[],int count){
  acceldata[0] = accel_total[0]/mycount; // Take the 10ms average of the data
  acceldata[1] = accel_total[1]/mycount;
  acceldata[2] = accel_total[2]/mycount;
  gyrodata[0] = gyro_total[0]/mycount;
  gyrodata[1] = gyro_total[1]/mycount;
  gyrodata[2] = gyro_total[2]/mycount;
}

 /*=========================================================================
    void update_sensor_buffer()
    Adds data to the circular sensor buffer (used in calculations)
    -----------------------------------------------------------------------*/
int update_sensor_buffer(float acceldata[], float gyrodata[],float sensor_buffer[],int sensor_buffer_counter){

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
//  Rollover
  if( i == 96 ){
    sensor_buffer_counter =0;
  }
  return sensor_buffer_counter;
}


int get_total(float accel_total[], float gyro_total[], float raw_acceldata[],float raw_gyrodata[],int sum){

    accel_total[0] = accel_total[0] + raw_acceldata[0];
    accel_total[1] = accel_total[1] + raw_acceldata[1];
    accel_total[2] = accel_total[2] + raw_acceldata[2];
    gyro_total[0] = gyro_total[0] + raw_gyrodata[0];
    gyro_total[1] = gyro_total[1] + raw_gyrodata[1];
    gyro_total[2] = gyro_total[2] + raw_gyrodata[2];

    sum ++;

    return sum;
}

// Removes the zero-state offset from the sensor data
void remove_offset(float acceldata[],float gyrodata[],float init_offset[]) {
    // Cases for accel x
    acceldata[0] = acceldata[0] - init_offset[0]; //X offset

    // Cases for accel y
    acceldata[1] = acceldata[1] - init_offset[1]; //y offset

    // Cases for accel z
    acceldata[2] = acceldata[2] - init_offset[2]; //z offset

    // Cases for gyro x
    gyrodata[0] = gyrodata[0] - init_offset[3]; //X offset

    //Cases for gyro y
    gyrodata[1] = gyrodata[1] - init_offset[4]; //y offset

    //Cases for gyro z
    gyrodata[2] = gyrodata[2] - init_offset[5]; //X offset


}


void remove_noise(float acceldata[], float gyrodata[], float noise_threshold[]) {
    if(fabs(acceldata[0])<= noise_threshold[0]) {acceldata[0] = 0;}
    if(fabs(acceldata[1])<= noise_threshold[0]) {acceldata[1] = 0;}
    if(fabs(acceldata[2])<= noise_threshold[0]) {acceldata[2] = 0;}
    if(fabs(gyrodata[0]) <= noise_threshold[1]) {gyrodata[0] = 0;}
    if(fabs(gyrodata[1]) <= noise_threshold[1]) {gyrodata[1] = 0;}
    if(fabs(gyrodata[2]) <= noise_threshold[1]) {gyrodata[2] = 0;}
}



