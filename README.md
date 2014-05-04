<b>OpenSourceQuad</b>
==========

<b>Author</b>: Brandon Riches<br />

Started in May of 2013, and always a work in progress! <br />

Feel free to <b>[fork](https://github.com/briches/OpenSourceQuad/fork)</b> the repo and submit a pull request.  

Released under the GNU Public License  

<b>Programming the flight controller:</b>  
----

(Finally) <b>Safe Fuse Bytes</b>  
<code>lfuse = 0xD6, hfuse = 0xB1, efuse = 0xfd</code>  

<b>Using arduino avrisp:</b>  
<code>avrdude -P comport -b 19200 -c avrisp -p m2560 -v -e -U flash:w:hexfilename.hex</code>  
<b>Using USBasp</b>  
<code>avrdude -c usbasp -p m2560 -v -e -U flash:w:tmp.hex</code>  
<b>Writing fuse bits</b>  
<code>avrdude -c usbasp -p m2560 -U hfuse:w:0xB1:m -U lfuse:w:0xD6:m -U efuse:w:0xFD:m</code>


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

The end goal is to create future proof, easy-to-use 
and customizable firmware for any flight configuration ( which is quite the lofty goal, but I'm working on it).
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


