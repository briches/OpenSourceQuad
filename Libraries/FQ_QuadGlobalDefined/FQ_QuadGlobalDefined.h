////========================================================================
	/*//*//*   FreeQuad   *//*//*
	QuadGlobalDefined.h
	August 2013

	A comprehensive functional list of all (most) variables used in any of
	the libraries written by our team.
	Currently, these include:
		Quadcopter.h
		SENSORLIB.h
		OseppGyro.h
    -----------------------------------------------------------------------*/


#ifndef QUADGLOBALDEFINED_H_INCLUDED
#define QUADGLOBALDEFINED_H_INCLUDED

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/* For PIC32 */
#if defined(__PIC32MX__)
    #include <p32xxxx.h>    /* this gives all the CPU/hardware definitions */
    #include <plib.h>       /* this gives the i/o definitions */
#endif


/***************************************************************************
/////////////////////////////////////////////////////////////////////////////
// General Definitions
/////////////////////////////////////////////////////////////////////////////
****************************************************************************/

/*=========================================================================
    Math related definitions
    -----------------------------------------------------------------------*/
#define Pi  			(3.14159265359F)			// Its pi.

/*=========================================================================
    Chebyshev 4th order LPF
    -----------------------------------------------------------------------*/

#define ORDER 4

// Comment out one of these defines to select the coefficent set to use.
// Remember that Wst is a fraction of the Nyquist frequency, Nq = Ws/2
// "BrandonCoeffs" uses n = 4, r = 10, Wc = 0.1
// AeroQuad filter uses n = 4, r = 60, Wc = 12.5/50

#define AeroQuad
// #define BrandonCoeffs

#ifdef AeroQuad
	#define ORDER 4
	#define _b0  0.001893594048567
	#define _b1 -0.002220262954039
	#define _b2  0.003389066536478
	#define _b3 -0.002220262954039
	#define _b4  0.001893594048567

	#define _a0  1
	#define _a1 -3.362256889209355
	#define _a2  4.282608240117919
	#define _a3 -2.444765517272841
	#define _a4  0.527149895089809
#endif

#ifdef BrandonCoeffs
	#define ORDER 4
	#define _b0  0.267411759560506
	#define _b1 -1.018535973364803
	#define _b2  1.503499251793105
	#define _b3 -1.018535973364804
	#define _b4  0.267411759560506

	#define _a0  1.000000000000000
	#define _a1 -3.561271663800053
	#define _a2  4.788976593687705
	#define _a3 -2.881867760094174
	#define _a4  0.655413654391032
#endif

#define XAXIS   (0)
#define YAXIS	(1)
#define ZAXIS	(2)

struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
};

/*=========================================================================
    Battery Monitor
    -----------------------------------------------------------------------*/
#define NOMINAL_V	11.1

/*=========================================================================
    STATUS LED pins
    -----------------------------------------------------------------------*/
#define GREEN_LED 	27						// Digital I/O pin
#define RED_LED 	25						// Digital I/O pin
#define YELLOW_LED 	23						// Digital I/O pin

/*=========================================================================
    Motors definitions
    -----------------------------------------------------------------------*/
#define MIN_PULSE_WIDTH       544     		// the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     		// the longest pulse sent to a servo

#define MOTOR1PIN			2
#define MOTOR2PIN			3
#define MOTOR3PIN			4
#define MOTOR4PIN			5

#define _PLUSconfig			1

//#define _Xconfig			1


/*=========================================================================
    SD logging definitions
    -----------------------------------------------------------------------*/
// Hardware SS pin on the ATmega2560
#define chipSelect  (53)


/***************************************************************************
/////////////////////////////////////////////////////////////////////////////
// Osepp Gyro
/////////////////////////////////////////////////////////////////////////////
****************************************************************************/

/*=========================================================================
    Osepp Gyro Registers
    -----------------------------------------------------------------------*/
#define WHO_AM_I   		(0x0)
#define PRODUCT_ID  	(0x1)
#define X_OFFS_USRH  	(0xC)// (R/W) User offset of H byte of X gyro (2's complement)
#define X_OFFS_USRL  	(0xD) // (R/W) User offset of L byte of X gyro (2's complement)
#define Y_OFFS_USRH  	(0xE) // (R/W) User offset of H byte of Y gyro (2's complement)
#define Y_OFFS_USRL  	(0xF) // (R/W) User offset of L byte of Y gyro (2's complement)
#define Z_OFFS_USRH  	(0x10) // (R/W) User offset of H byte of Z gyro (2's complement)
#define Z_OFFS_USRL  	(0x11) // (R/W) User offset of L byte of Z gyro (2's complement)
#define FIFO_EN  		(0x12) // (R/W)
#define AUX_VDDIO  		(0x13) //(R/W)
#define AUX_SLV_ADDR  	(0x14) // (R/W)
#define SMPLRT_DIV  	(0x15) // (R/W) Sample rate divider, divides analog sample rate
#define DLPF_FS_SYNC  	(0x16)
#define INT_CFG  		(0x17) // (R/W) Configures interrupt operation
#define AUX_ADDR 		(0x18) // (R/W)
#define INT_STATUS 		(0x1A) // (R) Interrupt status
#define TEMP_OUT_H 		(0x1B) // (R)
#define TEMP_OUT_L 		(0x1C) // (R)
#define GYRO_XOUT_H 	(0x1D) // (R) 16 bit x gyro data (2's complement)
#define GYRO_XOUT_L 	(0x1E)
#define GYRO_YOUT_H 	(0x1F) // (R) 16 bit y gyro data (2's complement)
#define GYRO_YOUT_L 	(0x20)
#define GYRO_ZOUT_H 	(0x21) // (R) 16 bit z gyro data (2's complement)
#define GYRO_ZOUT_L 	(0x22)
#define AUX_XOUT_H 		(0x23)
#define AUX_XOUT_L 		(0x24)
#define AUX_YOUT_H		(0x25)
#define AUX_YOUT_L 		(0x26)
#define AUX_ZOUT_H 		(0x27)
#define AUX_ZOUT_L 		(0x28)
#define FIFO_COUNTH 	(0x3A)
#define FIFO_COUNTL 	(0x3B)
#define FIFO_R 			(0x3C)
#define USER_CTRL 		(0x3D)
#define PWR_MGM 		(0x3E)

/*=========================================================================
    Osepp Gyro Range settings and conversions
    -----------------------------------------------------------------------*/
#define FULL_SCALE_RANGE_250  	(0x0)
#define FULL_SCALE_RANGE_500  	(0x1)
#define FULL_SCALE_RANGE_1000  	(0x2)
#define FULL_SCALE_RANGE_2000  	(0x3)

#define SI_CONVERT_250  		(0.0076511)
#define SI_CONVERT_500  		(0.01524) // WIP
#define SI_CONVERT_1000  		(0.015259)
#define SI_CONVERT_2000  		(0.030518)

// These are the drift rates in d/(s*mus)
#define GYRO_DRIFT_RATE_X   	(-9E-10)
#define GYRO_DRIFT_RATE_Y    	(3E-10)
#define GYRO_DRIFT_RATE_Z   	(-3E-11)

/***************************************************************************
/////////////////////////////////////////////////////////////////////////////
// LSM303
/////////////////////////////////////////////////////////////////////////////
****************************************************************************/

/*=========================================================================
	Conversion factors
    -----------------------------------------------------------------------*/

#define AX_SCALE  		(0.15696)
#define AY_SCALE  		(0.15449)
#define AZ_SCALE  		(0.9036550534)

#define MXY_SCALE  		(1055)
#define MZ_SCALE    	(950)

#define SENSORS_GRAVITY_STANDARD 			(9.81369388)
#define SENSORS_GAUSS_TO_MICROTESLA       	(100)

/*=========================================================================
    Accelerometer Register Addresses
    -----------------------------------------------------------------------*/
												// DEFAULT    TYPE
#define ACCEL_CTRL_REG1_A           (0x20)  	// 00000111   rw
#define ACCEL_CTRL_REG2_A          	(0x21)  	// 00000000   rw
#define ACCEL_CTRL_REG3_A          	(0x22)   	// 00000000   rw
#define ACCEL_CTRL_REG4_A          	(0x23)   	// 00000000   rw
#define ACCEL_CTRL_REG5_A          	(0x24)   	// 00000000   rw
#define ACCEL_CTRL_REG6_A          	(0x25)   	// 00000000   rw
#define ACCEL_REFERENCE_A          	(0x26)   	// 00000000   r
#define ACCEL_STATUS_REG_A         	(0x27)   	// 00000000   r
#define ACCEL_OUT_X_L_A            	(0x28)
#define ACCEL_OUT_X_H_A            	(0x29)
#define ACCEL_OUT_Y_L_A            	(0x2A)
#define ACCEL_OUT_Y_H_A            	(0x2B)
#define ACCEL_OUT_Z_L_A            	(0x2C)
#define ACCEL_OUT_Z_H_A            	(0x2D)
#define ACCEL_FIFO_CTRL_REG_A     	(0x2E)
#define ACCEL_FIFO_SRC_REG_A        (0x2F)
#define ACCEL_INT1_CFG_A           	(0x30)
#define ACCEL_INT1_SOURCE_A         (0x31)
#define ACCEL_INT1_THS_A           	(0x32)
#define ACCEL_INT1_DURATION_A    	(0x33)
#define ACCEL_INT2_CFG_A           	(0x34)
#define ACCEL_INT2_SOURCE_A        	(0x35)
#define ACCEL_INT2_THS_A           	(0x36)
#define ACCEL_INT2_DURATION_A    	(0x37)
#define ACCEL_CLICK_CFG_A          	(0x38)
#define ACCEL_CLICK_SRC_A          	(0x39)
#define ACCEL_CLICK_THS_A          	(0x3A)
#define ACCEL_TIME_LIMIT_A         	(0x3B)
#define ACCEL_TIME_LATENCY_A       	(0x3C)
#define ACCEL_TIME_WINDOW_A    		(0x3D)

/*=========================================================================
    Accelerometer Register Settings
    -----------------------------------------------------------------------*/
// Sets the ODR configuration and the low-pass cut-off frequencies
#define ACCEL_ODR_50_37				(0x00)
#define ACCEL_ODR_100_74			(0x01)
#define ACCEL_ODR_400_292			(0x02)
#define ACCEL_ODR_1000_780			(0x03)

// Sets the maximum sensing range and sensitivity
#define ACCEL_FULL_SCALE_2g			(0x00)
#define ACCEL_FULL_SCALE_4g			(0x01)
#define ACCEL_FULL_SCALE_8g			(0x02)

/*=========================================================================
    Magnetometer Register Addresses
    -----------------------------------------------------------------------*/
#define MAG_CRA_REG_M              	(0x00)
#define MAG_CRB_REG_M              	(0x01)
#define MAG_MR_REG_M               	(0x02)
#define MAG_OUT_X_H_M              	(0x03)
#define MAG_OUT_X_L_M              	(0x04)
#define MAG_OUT_Z_H_M              	(0x05)
#define MAG_OUT_Z_L_M              	(0x06)
#define MAG_OUT_Y_H_M              	(0x07)
#define MAG_OUT_Y_L_M              	(0x08)
#define MAG_SR_REG_Mg              	(0x09)
#define MAG_IRA_REG_M              	(0x0A)
#define MAG_IRB_REG_M              	(0x0B)
#define MAG_IRC_REG_M              	(0x0C)
#define MAG_TEMP_OUT_H_M           	(0x31)
#define MAG_TEMP_OUT_L_M           	(0x32)

/*=========================================================================
    Magnetometer Register Settings
    -----------------------------------------------------------------------*/
// Sets the rate at which data is written to the three output registers
#define MAG_ODR_0_75				(0x00)
#define MAG_ODR_1_50				(0x01)
#define MAG_ODR_3_00				(0x02)
#define MAG_ODR_7_50				(0x03)
#define MAG_ODR_15_0				(0x04)
#define MAG_ODR_30_0				(0x05)
#define MAG_ODR_75_0				(0x06)

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
typedef enum
{
  LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
  LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
  LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
  LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
  LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
  LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
  LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} lsm303MagGain;
/*=========================================================================
    INTERNAL VECTOR DATA TYPE
    -----------------------------------------------------------------------*/
/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        /* Orientation sensors */
        struct {
            float azimuth;    /**< Angle between the magnetic north direction and the Y axis, around the Z axis (0<=azimuth<360).  0=North, 90=East, 180=South, 270=West */
            float pitch;      /**< Rotation around X axis (-180<=pitch<=180), with positive values when the z-axis moves toward the y-axis. */
            float roll;       /**< Rotation around Y axis (-90<=roll<=90), with positive values when the x-axis moves towards the z-axis. */
        };
    };
    int8_t status;
    uint8_t reserved[3];
} sensors_vec_t;

/*=========================================================================
    INTERNAL SENSOR EVENT DATA TYPE
    -----------------------------------------------------------------------*/
/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
    int32_t version;                          /**< must be sizeof(struct sensors_event_t) */
    int32_t sensor_id;                        /**< unique sensor identifier */
    int32_t type;                             /**< sensor type */
    int32_t reserved0;                        /**< reserved */
    int32_t timestamp;                        /**< time is in milliseconds */
    union
    {
        float          data[4];
        sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
        sensors_vec_t   orientation;          /**< orientation values are in degrees */
        sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
        float          temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float          distance;             /**< distance in centimeters */
        float          light;                /**< light in SI lux units */
        float          pressure;             /**< pressure in hectopascal (hPa) */
        float          relative_humidity;    /**< relative humidity in percent */
        float          current;              /**< current in milliamps (mA) */
        float          voltage;              /**< voltage in volts (V) */
    };
} sensors_event_t;

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct lsm303MagData_s
{
	float x;
	float y;
	float z;
  float orientation;
} lsm303MagData;

/*=========================================================================
    INTERNAL ACCELERATION DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct lsm303AccelData_s
{
  float x;
  float y;
  float z;
} lsm303AccelData;

/*=========================================================================
    Kinematics Data Type
    -----------------------------------------------------------------------*/
struct kinematicData
{
	double pitch,
			roll,
			yaw,

			io_ax,
			io_ay,
			io_az,
			io_wx,
			io_wy,
			io_wz,

			pitch_gyro,
			roll_gyro,
			yaw_gyro,

			yaw_mag;

	unsigned long timestamp;

};

/*=========================================================================
    CHIP ID
    -----------------------------------------------------------------------*/
#define LSM303_ID                     (0b11010100)

/***************************************************************************
/////////////////////////////////////////////////////////////////////////////
// All Sensors
/////////////////////////////////////////////////////////////////////////////
****************************************************************************/

/*=========================================================================
    Polling Rates.
    -----------------------------------------------------------------------*/

#define _100HzPoll 		(10000)				// us Period of 100 Hz poll
#define	_75HzPoll		(13333)				// us Period of 75 Hz poll
#define _20HzPoll 		(50000)				// us Period of 50 Hz poll
#define _10HzPoll		(100000)			// us Period of 10 Hz poll

/*=========================================================================
    Sensor analog pins
    -----------------------------------------------------------------------*/

#define VBATT_PIN 	(0x0)
#define USRF_pin 	(0x0)				// TODO: Connect the USRF AN pin to this pin


/*=========================================================================
    I2C or TWI addresses
    -----------------------------------------------------------------------*/

#define Gyro_Address    (0x69)				// I2C Address of the gyro
#define ACCEL_ADDR  	(0x32 >> 1)
#define MAG_ADDR  		(0x3C >> 1)

/*===============================================
	Device settings
	-----------------------------------------------------------------------*/
const int d_ScaleRange = FULL_SCALE_RANGE_250; // x250,x500,x1000,x2000
const int DLPF = 6;                 // 0,1,2,3,4,5,6,7 // See data sheet
const bool HighDef = true;          // Is accel output 2byte or 1byte


#endif // QUADGLOBALDEFINED_H_INCLUDED
