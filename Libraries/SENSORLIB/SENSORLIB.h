#ifndef SENSORLIB_H_INCLUDED
#define SENSORLIB_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <I2c.h>

#define ACCEL_ADDR  (0x19)
#define MAG_ADDR_W   (0x3C >> 1)  				// Use for WRITE operations
#define MAG_ADDR_R    (0x3C >> 1)  		// Use for READ operations

#define AX_SCALE  (0.15696)
#define AY_SCALE  (0.15449)
#define AZ_SCALE  (0.14115)

#define MXY_SCALE  (1055)
#define MZ_SCALE    (950)
#define GRAV_STANDARD (9.80665)

																			// DEFAULT    TYPE
#define ACCEL_CTRL_REG1_A            0x20  // 00000111   rw
#define ACCEL_CTRL_REG2_A          0x21    // 00000000   rw
#define ACCEL_CTRL_REG3_A          0x22    // 00000000   rw
#define ACCEL_CTRL_REG4_A          0x23    // 00000000   rw
#define ACCEL_CTRL_REG5_A          0x24    // 00000000   rw
#define ACCEL_CTRL_REG6_A          0x25    // 00000000   rw
#define ACCEL_REFERENCE_A          0x26    // 00000000   r
#define ACCEL_STATUS_REG_A         0x27    // 00000000   r
#define ACCEL_OUT_X_L_A            0x28
#define ACCEL_OUT_X_H_A            0x29
#define ACCEL_OUT_Y_L_A            0x2A
#define ACCEL_OUT_Y_H_A            0x2B,
#define ACCEL_OUT_Z_L_A            0x2C
#define ACCEL_OUT_Z_H_A            0x2D
#define ACCEL_FIFO_CTRL_REG_A      0x2E
#define ACCEL_FIFO_SRC_REG_A       0x2F
#define ACCEL_INT1_CFG_A           0x30
#define ACCEL_INT1_SOURCE_A        0x31
#define ACCEL_INT1_THS_A           0x32
#define ACCEL_INT1_DURATION_A      0x33
#define ACCEL_INT2_CFG_A           0x34
#define ACCEL_INT2_SOURCE_A        0x35
#define ACCEL_INT2_THS_A           0x36
#define ACCEL_INT2_DURATION_A      0x37
#define ACCEL_CLICK_CFG_A          0x38
#define ACCEL_CLICK_SRC_A          0x39
#define ACCEL_CLICK_THS_A          0x3A
#define ACCEL_TIME_LIMIT_A         0x3B
#define ACCEL_TIME_LATENCY_A       0x3C
#define ACCEL_TIME_WINDOW_A        0x3D

#define  MAG_CRA_REG_M              0x00
#define  MAG_CRB_REG_M              0x01
#define MAG_MR_REG_M               0x02
#define MAG_OUT_X_H_M              0x03
#define MAG_OUT_X_L_M              0x04
#define MAG_OUT_Z_H_M              0x05
#define MAG_OUT_Z_L_M              0x06
#define MAG_OUT_Y_H_M              0x07
#define MAG_OUT_Y_L_M              0x08
#define MAG_SR_REG_Mg              0x09
#define MAG_IRA_REG_M              0x0A
#define MAG_IRB_REG_M              0x0B
#define MAG_IRC_REG_M              0x0C
#define MAG_TEMP_OUT_H_M           0x31
#define MAG_TEMP_OUT_L_M           0x32


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
