#ifndef SENSORLIB_H_INCLUDED
#define SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <I2c.h>

class SENSORLIB
{
public:
    SENSORLIB();
    void Easy_Start();
    void update();
    void Cleanup();

	double ax();
	double ay();
	double az();

	double ax_data;
	double ay_data;
	double az_data;

	double mx();
	double my();
	double mz();

	double mx_data;
	double my_data;
	double mz_data;
};
#endif
