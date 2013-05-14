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



