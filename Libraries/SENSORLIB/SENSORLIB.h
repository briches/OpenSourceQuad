#ifndef SENSORLIB_H_INCLUDED
#define SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//#include "WProgram.h"
#endif

#include "I2C.h"

class SENSORLIB
{
public:
    SENSORLIB();
    void Easy_Start();
    void Read_Accel(float[]);
};


#endif
