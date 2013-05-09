Name	: DataIntegrator Library
Authors : Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date	: May, 2013
Version : 0.1
Notes	: Arduino Library containing some functions found in the QCopter Main script, especially those related to noise
         removal and data integration


Dependancies: None
 


List of Functions:
-------------------------------------------------------------------------------
Function: remove_offset
Description: Removes zero state offset from the readouts of the sensors
Parameters: float acceldata[],float gyrodata[],float init_offset[]
-------------------------------------------------------------------------------
Function: remove_noise
Description: Sets data to zero if it is below a user-set threshold
Parameters: float acceldata[], float gyrodata[], float noise_threshold[])
-------------------------------------------------------------------------------
Function: get_total
Description: Does basic summation of values in various arrays
Parameters: float accel_total[], float gyro_total[], float raw_acceldata[],float raw_gyrodata[], int sum
-------------------------------------------------------------------------------
