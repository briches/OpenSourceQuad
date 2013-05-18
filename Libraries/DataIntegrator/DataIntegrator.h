#ifndef DATAINTEGRATOR_H_INCLUDED
#define DATAINTEGRATOR_H_INCLUDED
//Header

/*********************************************************************************************
DataIntegrator.h

Author : Brandon Riches
Date   : May, 2013
Version: 0.1

Notes: Contains some necessary function declarations used in the QCopterMain.ino script.


Dependancies: None


Changelog:
    V0.1
        Added
            int get_total
            void remove_offset
            void remove_noise
**********************************************************************************************/
void array3_zero(float array[]);

void zero_my_arrays(float accel_total[],float gyro_total[],float acceldata[],float gyrodata[]);

void remove_offset(float acceldata[],float gyrodata[],float init_offset[]);

void SI_convert();

void remove_noise(float acceldata[], float gyrodata[], float noise_threshold[]);

int get_total(float accel_total[], float gyro_total[], float raw_acceldata[],float raw_gyrodata[], int sum);

#endif // DATAINTEGRATOR_H_INCLUDED
