
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Kinematics.h>
#include <Quadcopter.h>
#include <QuadGlobalDefined.h>
#include <SENSORLIB.h>
#include <OseppGyro.h>



static float _GYRO_CONVERT_	= SI_CONVERT_250;

void getGyroConvert();
void normalize(double x, double y, double z);
void nan_quad_Check(double num1, double num2, double num3);
double complementary(double mynum, int select, double coeff,  struct kinematicData data);

// Kinematic events include yaw, pitch and roll calculations
// as well as altitudes
void kinematicEvent(int eventType,
					struct kinematicData data,
					SENSORLIB_accel accel,
					SENSORLIB_mag mag,
					OseppGyro gyro,
					fourthOrderData fourthOrderXAXIS,
					fourthOrderData fourthOrderYAXIS,
					fourthOrderData fourthOrderZAXIS)
{

	sensors_event_t	accel_event,
					mag_event;

	double 		elapsed_time = 0,
					t_convert = 1000000;

	double 		pitch_accel,
					roll_accel,
					pitchRollCoeff = 0.9,
					yawCoeff = 0.9;

	getGyroConvert();

	if(eventType == 1)
	{
		mag.getEvent(&mag_event);
		data.yaw_mag = atan2(mag_event.magnetic.y, mag_event.magnetic.x)* 180/Pi;
	}

	if(eventType == 0)
	{
		accel.getEvent(&accel_event);
		gyro.update();

		/// Store Raw values from the sensor registers, and remove initial offsets
		double ax = accel_event.acceleration.x - data.io_ax;
		double ay = accel_event.acceleration.y - data.io_ay;
		double az = accel_event.acceleration.z - data.io_az + SENSORS_GRAVITY_STANDARD;

		double wx =  -(gyro.x() - data.io_wx)	* _GYRO_CONVERT_;
		double wy =  -(gyro.y() - data.io_wy) 	* _GYRO_CONVERT_;
		double wz = 	gyro.z() - data.io_wz 	* _GYRO_CONVERT_;

		// Compute a Chebyshev 4th order filter
		ax = computeFourthOrder(ax, &fourthOrderXAXIS);
		ay = computeFourthOrder(ay, &fourthOrderYAXIS);
		az = computeFourthOrder(az, &fourthOrderZAXIS);

		normalize(ax, ay, az);

		elapsed_time = (micros() - data.timestamp) / t_convert;
		data.timestamp = micros();

		data.pitch_gyro += wy * elapsed_time;
		data.roll_gyro 	+= wx * elapsed_time;
		data.yaw_gyro	+= wz * elapsed_time;

		pitch_accel = acos(az / (cos(asin(ay))))* 180/Pi;
		roll_accel 	= asin(ay)*180/Pi;

		// Remove pesky NaNs that seem to occur around 0.
		// Check the quadrant of vector
		nan_quad_Check(pitch_accel, roll_accel, data.yaw_mag);

		data.pitch 	= complementary(pitch_accel, 0, pitchRollCoeff, data);
		data.roll  	= complementary(roll_accel, 1, pitchRollCoeff, data);
		data.yaw	= complementary(data.yaw_gyro, 2, yawCoeff, data);

	}

};

double complementary(double mynum, int select, double coeff, struct kinematicData data)
{
	if (select == 0)
	{
		return coeff * data.pitch_gyro 	+ (1 - coeff) * mynum;
	}
	if (select == 1)
	{
		return coeff * data.roll_gyro 	+ (1 - coeff) * mynum;
	}
	if (select == 2)
	{
		return coeff * data.yaw_gyro		+ (1 - coeff) * mynum;
	}
};

float computeFourthOrder(float currentInput,
						struct fourthOrderData *filterParameters)
{
	// cheby2(4,60,12.5/50)
	#define _b0  0.001893594048567
	#define _b1 -0.002220262954039
	#define _b2  0.003389066536478
	#define _b3 -0.002220262954039
	#define _b4  0.001893594048567

	#define _a1 -3.362256889209355
	#define _a2  4.282608240117919
	#define _a3 -2.444765517272841
	#define _a4  0.527149895089809

	float output;
	// No flip, has phase shift
	output = _b0 * currentInput                +
		   _b1 * filterParameters->inputTm1  +
		   _b2 * filterParameters->inputTm2  +
		   _b3 * filterParameters->inputTm3  +
		   _b4 * filterParameters->inputTm4  -
		   _a1 * filterParameters->outputTm1 -
		   _a2 * filterParameters->outputTm2 -
		   _a3 * filterParameters->outputTm3 -
		   _a4 * filterParameters->outputTm4;

	filterParameters->inputTm4 = filterParameters->inputTm3;
	filterParameters->inputTm3 = filterParameters->inputTm2;
	filterParameters->inputTm2 = filterParameters->inputTm1;
	filterParameters->inputTm1 = currentInput;

	filterParameters->outputTm4 = filterParameters->outputTm3;
	filterParameters->outputTm3 = filterParameters->outputTm2;
	filterParameters->outputTm2 = filterParameters->outputTm1;
	filterParameters->outputTm1 = output;

	return output;
};

void setupFourthOrder(	fourthOrderData fourthOrderXAXIS,
						fourthOrderData fourthOrderYAXIS,
						fourthOrderData fourthOrderZAXIS)
{
  fourthOrderXAXIS.inputTm1 = 0.0;
  fourthOrderXAXIS.inputTm2 = 0.0;
  fourthOrderXAXIS.inputTm3 = 0.0;
  fourthOrderXAXIS.inputTm4 = 0.0;

  fourthOrderXAXIS.outputTm1 = 0.0;
  fourthOrderXAXIS.outputTm2 = 0.0;
  fourthOrderXAXIS.outputTm3 = 0.0;
  fourthOrderXAXIS.outputTm4 = 0.0;

  //////////
  fourthOrderYAXIS.inputTm1 = 0.0;
  fourthOrderYAXIS.inputTm2 = 0.0;
  fourthOrderYAXIS.inputTm3 = 0.0;
  fourthOrderYAXIS.inputTm4 = 0.0;

  fourthOrderYAXIS.outputTm1 = 0.0;
  fourthOrderYAXIS.outputTm2 = 0.0;
  fourthOrderYAXIS.outputTm3 = 0.0;
  fourthOrderYAXIS.outputTm4 = 0.0;

  //////////
  fourthOrderZAXIS.inputTm1 = -9.8065;
  fourthOrderZAXIS.inputTm2 = -9.8065;
  fourthOrderZAXIS.inputTm3 = -9.8065;
  fourthOrderZAXIS.inputTm4 = -9.8065;

  fourthOrderZAXIS.outputTm1 = -9.8065;
  fourthOrderZAXIS.outputTm2 = -9.8065;
  fourthOrderZAXIS.outputTm3 = -9.8065;
  fourthOrderZAXIS.outputTm4 = -9.8065;
};

void normalize(double x, double y, double z)
{
	double *px = &x;
	double *py = &y;
	double *pz = &z;

	double norm = sqrt( *px * *px + *py * *py + *pz * *pz );
	// normalize the vector
	*px /= norm;
	*py /= norm;
	*pz /= norm;
};

void getGyroConvert()
{
	switch (d_ScaleRange)
	{
		case FULL_SCALE_RANGE_250:
			_GYRO_CONVERT_ = SI_CONVERT_250;
			break;

		case FULL_SCALE_RANGE_500:
			_GYRO_CONVERT_ = SI_CONVERT_500;
			break;

		case FULL_SCALE_RANGE_1000:
			_GYRO_CONVERT_ = SI_CONVERT_1000;
			break;

		case FULL_SCALE_RANGE_2000:
			_GYRO_CONVERT_ = SI_CONVERT_2000;
			break;
	}
};

void nan_quad_Check(double num1, double num2, double num3)
{
	double *pnum1 = &num1;
	double *pnum2 = &num2;
	double *pnum3 = &num3;

	if(isnan(num1))
	{
		*pnum1 = 0;
	}
	if(isnan(num2))
	{
		*pnum2 = 0;
	}
	// Check quadrants
	if (num2 < 0)
	{
		*pnum1 = - num1;
	}
	if (num2 < 0)
	{
		*pnum2 	= - num2;
	}
	if (num3 < 0)
	{
		*pnum3 += 360;
	}
};
