<b>QCopterMain.ino </b>
===============

Authors     : Brandon Riches, Andrew Coulthard, Patrick Fairbanks <br />
Date        : May 2013 <br />
Version     : V0.1 <br />

Notes       : Main source code for quad-copter project. <br />

Dependancies: 
            
[I2C.h](https://github.com/briches/Quadcopter/tree/master/I2C) <br />
[MMA8453_n0m1.h](https://github.com/briches/Quadcopter/tree/master/MMA8453_n0m1) <br />
[OseppGyro.h](https://github.com/briches/Quadcopter/tree/master/OseppGyro) <br />
[Control.h](https://github.com/briches/Quadcopter/tree/master/Libraries/Control) <br />
math.h <br />
            

What the code does so far:
-------------------
 
- Initializes sensors
    + Writes to various registers of each sensor to set data modes, etc
    + 
- Fetches raw data outputs from each of the sensors
- Takes a small running average to smooth the raw data, and sets data to zero if below thresholds
- 
