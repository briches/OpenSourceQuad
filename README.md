<b>OpenSourceQuad</b>
==========

<b>Authors</b>: Brandon Riches, Andrew Coulthard, Branden Yue  <br />
<b>Date</b>   : May 2013 <br />
<b>Version</b>: 0.91 <br />

<b>Programming the flight controller:</b>  
Using arduino avrisp:  
<code>avrdude -P comport -b 19200 -c avrisp -p m2560 -v -e -U flash:w:hexfilename.hex</code>  
Using USBasp  
<code>avrdude -c usbasp -p m328p -u -U flash:w:hexfilename.hex</code>  


<b>Reading fuse bits</b>  
<code>avrdude -c usbasp -p m328p -U hfuse:r:con:h -U lfuse:r:con:h -U efuse:r:con:h -U lock:r:con:h</code>    

<b>Writing fuse bits</b>  
<code>avrdude -c usbasp -p m328p -U hfuse:w:0xDA:m -U lfuse:w:0xE2:m -U efuse:w:0x05:m   


<b>ESC Settings</b>  
- Batt Type: LiXX
- Cut off: high
- Cut off Type: hard
- Brake: off
- Timing: auto
- Start up: medium

Description
-----------

Quadcopter flight firmware! 

Our end goal is to create future proof, easy-to-use 
and customizable firmware for any flight configuration ( which is quite the lofty goal, but we're working on it).
<br /><br />



<br />
Sensors:
 - LSM303 Accelerometer / Magnetometer 
 - MPU3050 Gyro
 - Adafruit ultimate GPS module
 - Maxbotix USRF (LV-EZ1)
 - BMP085 Barometeric Pressure/Temperature/Altitude
 

Features and long term goals:
----------------------------
   <br />
  - Fully autonomous takeoff and landing. Has the option to be controlled via radio comms. <br />

  - 9DOF sensor module (IMU) with GPS and localization  <br />
  
  - Open source =]
   <br />
   <br />


