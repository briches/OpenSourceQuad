Quadcopter
==========

Authors: Brandon Riches, Andrew Coulthard, Branden Yue  <br />
Date   : May 2013 <br />
Version: 0.7 <br />

Description
-----------

Its quadcopter flight software!  
<br /><br />

MCU: Osepp Mega 2560 R3   
(A port the chipKIT uC32 with the PIC32 is underway)


<br />
Current sensors lineup:
 - LSM303 Accelerometer / Magnetometer 
 - MPU3050 Gyro
 - Adafruit ultimate GPS module
 - RTC Module (DS1307)
 - Maxbotix USRF (LV-EZ1)
 - BMP085 Barometeric Pressure/Temperature/Altitude
 

Under development currently:
----------------------------

 <br />
  - Digital filtering of accel and gyro data <br />

  - GPS integration and datalogging <br />
      - See [QCopterMain.ino](https://github.com/briches/Quadcopter/blob/master/QCopterMain/QCopterMain.ino) for info. <br />


Features and long term goals:
----------------------------
   <br />
  - Fully autonomous takeoff and landing. Has the option to be controlled via radio comms. <br />

  - 9DOF sensor module (IMU) with GPS and localization  <br />
  
  - Open source =]
   <br />
   <br />


