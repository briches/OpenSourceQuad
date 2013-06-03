Quadcopter
==========

Authors: Brandon Riches, Andrew Coulthard, Patrick Fairbanks  <br />
Date   : May 2013 <br />
Version: 0.5 <br />

Description
-----------

The code contained within this repository is intended to act as the complete control system for a X-configuration
quadcopter with 6-axis sensor capability. Specifically, the code is written for compatability with the LSM303DLHC 
Accel+Compass and the Osepp MPU3000/MPU3050 gyro.

Under development currently:
----------------------------

 <br />
  - Sensor noise reduction. Motor operation induces high magnitude high frequency noise.<br />
      - See [QCopterDebug.pde](https://github.com/briches/Quadcopter/blob/master/QCopterMain/QCopterDebug/QCopterDebug.pde) for info. <br />

  - PID algorithm for sustained stability <br />
      - See [QCopterMain.ino](https://github.com/briches/Quadcopter/blob/master/QCopterMain/QCopterMain.ino) for info. <br />


Features and long term goals:
----------------------------
   <br />
  - Fully autonomous flight and landing. Has the option to be controlled via radio comms. <br />

  - 6 DOF Sensor capability, upgrade to 9 DOF including magnetometer module.  <br />
  
  - Obstacle avoidance and tracking, mapping cabalilities  <br />
  
  - Open source =]
   <br />
   <br />


