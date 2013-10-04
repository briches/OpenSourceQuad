OpenSourceQuad
==========

Authors: Brandon Riches, Andrew Coulthard, Branden Yue  <br />
Date   : May 2013 <br />
Version: 0.91 <br />

Programming the flight controller:  
avrdude -P comport -b 19200 -c avrisp -p m2560 -v -e -U flash:w:hexfilename.hex

ESC Settings: 
- Battery Type: LiXX  
- Cut-off Voltage: high  
- Cut-off Type: hard   
- Start Up: medium   
- Timing: auto  
- Brake: off  
   <br />

Description
-----------

Its quadcopter flight software! 

Our end goal is to create future proof, easy-to-use 
and customizable firmware for any flight configuration ( which is quite the lofty goal, but we're working on it).
<br /><br />



<br />
Current sensors lineup:
 - LSM303 Accelerometer / Magnetometer 
 - MPU3050 Gyro
 - Adafruit ultimate GPS module
 - Maxbotix USRF (LV-EZ1)
 - BMP085 Barometeric Pressure/Temperature/Altitude
 

Under development currently:
----------------------------

- Testing and debugging


Features and long term goals:
----------------------------
   <br />
  - Fully autonomous takeoff and landing. Has the option to be controlled via radio comms. <br />

  - 9DOF sensor module (IMU) with GPS and localization  <br />
  
  - Open source =]
   <br />
   <br />


