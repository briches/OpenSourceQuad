// First attempts at deciphering the 9150
// Pending sanity.

#ifndef MPU_9150_H_INCLUDED
#define MPU_9150_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <Print.h>
#else
#include "WProgram.h"
#endif


#include <Wire.h>
#include <Limits.h> // MAX_INT, etc.


/*=========================================================================
 I2C 7bit address
 -----------------------------------------------------------------------*/
byte MPU_9150_Addr = 0x68;

/*=========================================================================
 MPU Registers
 -----------------------------------------------------------------------*/

/** Accel and gyro **/
#define MPU_9150_SMPLRT_DIV        	(0x19)
#define MPU_9150_CONFIG            	(0x1A)
#define MPU_9150_GYRO_CONFIG       	(0x1B)
#define MPU_9150_ACCEL_CONFIG      	(0x1C)

#define MPU_9150_FIFO_EN		(0x23)

#define MPU_9150_I2C_MST_CTRL	   	(0x24)

#define MPU_9150_I2C_SLV0_ADDR	   	(0x25)
#define MPU_9150_I2C_SLV0_REG		(0x26)
#define MPU_9150_I2C_SLV0_CTRL		(0x27)

#define MPU_9150_I2C_SLV1_ADDR		(0x28)
#define MPU_9150_I2C_SLV1_REG		(0x29)
#define MPU_9150_I2C_SLV1_CTRL		(0x2A)

#define MPU_9150_I2C_SLV2_ADDR		(0x2B)
#define MPU_9150_I2C_SLV2_REG		(0x2C)
#define MPU_9150_I2C_SLV2_CTRL		(0x2D)

#define MPU_9150_I2C_SLV3_ADDR		(0x2E)
#define MPU_9150_I2C_SLV3_REG		(0x2F)
#define MPU_9150_I2C_SLV3_CTRL		(0x30)

#define MPU_9150_I2C_SLV4_ADDR		(0x31)
#define MPU_9150_I2C_SLV4_REG		(0x32)
#define MPU_9150_I2C_SLV4_DO		(0x33)
#define MPU_9150_I2C_SLV4_CTRL		(0x34)
#define MPU_9150_I2C_SLV4_DI		(0x35)

#define MPU_9150_I2C_MST_STATUS		(0x36)
#define MPU_9150_INT_PIN_CFG       	(0x37)
#define MPU_9150_INT_ENABLE		(0x38)
#define MPU_9150_INT_STATUS		(0x3A)

#define MPU_9150_ACCEL_XOUT_H		(0x3B)
#define MPU_9150_ACCEL_XOUT_L		(0x3C)
#define MPU_9150_ACCEL_YOUT_H		(0x3D)
#define MPU_9150_ACCEL_YOUT_L		(0x3E)
#define MPU_9150_ACCEL_ZOUT_H		(0x3F)
#define MPU_9150_ACCEL_ZOUT_L		(0x40)

#define MPU_9150_TEMP_OUT_H		(0x41)
#define MPU_9150_TEMP_OUT_L		(0x42)

#define MPU_9150_GYRO_XOUT_H		(0x43)
#define MPU_9150_GYRO_XOUT_L		(0x44)
#define MPU_9150_GYRO_YOUT_H		(0x45)
#define MPU_9150_GYRO_YOUT_L		(0x46)
#define MPU_9150_GYRO_ZOUT_H		(0x47)
#define MPU_9150_GYRO_ZOUT_L		(0x48)

#define MPU_9150_EXT_SENS_DATA_00	(0x49)
#define MPU_9150_EXT_SENS_DATA_01	(0x4A)
#define MPU_9150_EXT_SENS_DATA_02	(0x4B)
#define MPU_9150_EXT_SENS_DATA_03	(0x4C)
#define MPU_9150_EXT_SENS_DATA_04	(0x4D)
#define MPU_9150_EXT_SENS_DATA_05	(0x4E)
#define MPU_9150_EXT_SENS_DATA_06	(0x4F)
#define MPU_9150_EXT_SENS_DATA_07	(0x50)
#define MPU_9150_EXT_SENS_DATA_08	(0x51)
#define MPU_9150_EXT_SENS_DATA_09	(0x52)
#define MPU_9150_EXT_SENS_DATA_10	(0x53)
#define MPU_9150_EXT_SENS_DATA_11	(0x54)
#define MPU_9150_EXT_SENS_DATA_12	(0x55)
#define MPU_9150_EXT_SENS_DATA_13	(0x56)
#define MPU_9150_EXT_SENS_DATA_14	(0x57)
#define MPU_9150_EXT_SENS_DATA_15	(0x58)
#define MPU_9150_EXT_SENS_DATA_16	(0x59)
#define MPU_9150_EXT_SENS_DATA_17	(0x5A)
#define MPU_9150_EXT_SENS_DATA_18	(0x5B)
#define MPU_9150_EXT_SENS_DATA_19	(0x5C)
#define MPU_9150_EXT_SENS_DATA_20	(0x5D)
#define MPU_9150_EXT_SENS_DATA_21	(0x5E)
#define MPU_9150_EXT_SENS_DATA_22	(0x5F)
#define MPU_9150_EXT_SENS_DATA_23	(0x60)

#define MPU_9150_I2C_SLV0_DO		(0x63)
#define MPU_9150_I2C_SLV0_D1		(0x64)
#define MPU_9150_I2C_SLV0_D2		(0x65)
#define MPU_9150_I2C_SLV0_D3		(0x66)

#define MPU_9150_I2C_MST_DELAY_CTRL	(0x67)
#define MPU_9150_SIGNAL_PATH_RESET	(0x68)

#define MPU_9150_USER_CTRL         	(0x6A)
#define MPU_9150_PWR_MGMT_1        	(0x6B)
#define MPU_9150_PWR_MGMT_2		(0x6C)

#define MPU_9150_FIFO_COUNTH		(0x72)
#define MPU_9150_FIFO_COUNTL		(0x73)
#define MPU_9150_FIFO_R_W		(0x74)

#define MPU_9150_WHO_AM_I          	(0x75)

/** Magnetometer **/
#define MPU_9150_MAG_WIA		(0x00) // Device ID
#define MPU_9150_MAG_INFO		(0x01)
#define MPU_9150_MAG_ST1		(0x02)

#define MPU_9150_MAG_HXL		(0x03)
#define MPU_9150_MAG_HXH		(0x04)
#define MPU_9150_MAG_HYL		(0x05)
#define MPU_9150_MAG_HYH		(0x06)
#define MPU_9150_MAG_HZL		(0x07)
#define MPU_9150_MAG_HZH		(0x08)

#define MPU_9150_MAG_ST2		(0x09)
#define MPU_9150_MAG_CNTL		(0x0A)
#define MPU_9150_MAG_I2CDIS		(0x0F)

#define MPU_9150_MAG_ASAX		(0x10) // Read in fuse mode only
#define MPU_9150_MAG_ASAY		(0x11) // " "
#define MPU_9150_MAG_ASAZ		(0x12) // " "

/*=========================================================================
 Internal Acceleration Type
 -----------------------------------------------------------------------*/
typedef struct MPU_9150_AccelData_s
{
        float x, y, z;
}MPU_9150_AccelData;

/*=========================================================================
 Internal Gyroscope Type
 -----------------------------------------------------------------------*/
typedef struct MPU_9150_GyroData_s
{
        float x, y, z;
}MPU_9150_GyroData;

/*=========================================================================
 Internal Magnetometer Type
 -----------------------------------------------------------------------*/
typedef struct MPU_9150_MagData_s
{
        float x, y, z;
}MPU_9150_MagData;

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
        sensors_vec_t   gyro;                 /**< gyroscope values are in deg/s */
        float          temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float          distance;             /**< distance in meters */
        float          pressure;             /**< pressure in hectopascal (hPa) */
    };
} sensors_event_t;


/*========================================================================*/
/**=========================================================================
 Class and functions
 -----------------------------------------------------------------------*/
 /*========================================================================*/
class MPU_9150_c
{
        public:
        
                //Constructor
                MPU_9150_c(void);
        
                // Scan the I2C bus for devices
                byte scanBus(void);
        
                // Initialize the MPU
                byte initMPU(void);
        
                void newEvent(sensors_event_t*);
        
        private:
                
                // Data
                MPU_9150_AccelData accelData;
                MPU_9150_GyroData gyroData;
                MPU_9150_MagData magData;
                double temperature;
                
                // Methods
                void newAccelEvent();
                void newGyroEvent();
                byte write8(byte I2Caddress, byte reg, byte value);
                byte read8(byte I2Caddress, byte reg);
};
/* End Class */


/*=========================================================================
 Class Constructor
 -----------------------------------------------------------------------*/
MPU_9150_c :: MPU_9150_c()
{};


/*=========================================================================
 begin()        - Sets up the device to return normal sensor data
 -----------------------------------------------------------------------*/
byte MPU_9150_c::initMPU()
{
        byte error;

        // Move out of sleep mode and set the clock source to Gyro x PLL
        error = write8(MPU_9150_Addr, MPU_9150_PWR_MGMT_1, 0x01);

        // Disable I2C Passthrough
        error = write8(MPU_9150_Addr, MPU_9150_INT_PIN_CFG, 0x00);

        // Disable I2C Master (for now?)
        error = write8(MPU_9150_Addr, MPU_9150_USER_CTRL, 0x00);

        // Gyro Config
        error = write8(MPU_9150_Addr, MPU_9150_GYRO_CONFIG, 0x00);

        // Accel Config
        error = write8(MPU_9150_Addr, MPU_9150_ACCEL_CONFIG, 0x00);

        // Sample rate divider
        error = write8(MPU_9150_Addr, MPU_9150_SMPLRT_DIV, 0x00);
        
        // Low Pass Filter
        error = write8(MPU_9150_Addr, MPU_9150_CONFIG, 0x06);


        return read8(MPU_9150_Addr, MPU_9150_WHO_AM_I);
};

/*=========================================================================
 newEvent(sensors_event_t *newEvent)
 - Reads the new sensor data from the accelerometer and gyro
 - Call to get the new data. The newAccelEvent and newGyroEvent are private
 -----------------------------------------------------------------------*/
void MPU_9150_c :: newEvent(sensors_event_t *event)
{
        newAccelEvent(); // Read new accelerometer data.
        newGyroEvent(); // Read new gyro data
        
        event->type = 1;
        event->timestamp = micros();
        
        event->acceleration.x = (float)accelData.x / 16384.0;
        event->acceleration.y = (float)accelData.y / 16384.0;
        event->acceleration.z = (float)accelData.z / 16384.0;
        
        Serial.print(event->acceleration.x); Serial.print("   "); Serial.println(accelData.x);
        
        event->gyro.x = (float)gyroData.x;
        event->gyro.y = (float)gyroData.y;
        event->gyro.z = (float)gyroData.z;
};

/*=========================================================================
newAccelEvent() 
- Reads the latest data stored in the device, and saves it as a private
var in the class
 -----------------------------------------------------------------------*/
void MPU_9150_c :: newAccelEvent()
{
        // Read the accelerometer
        Wire.beginTransmission((byte)MPU_9150_Addr);
        #if ARDUINO >= 100
                Wire.write(MPU_9150_ACCEL_XOUT_H);
        #else
                Wire.send(MPU_9150_ACCEL_XOUT_H);
        #endif
        Wire.endTransmission();
        Wire.requestFrom((byte)MPU_9150_Addr, (byte)6);
        
        // Wait for enough data
        while (Wire.available() < 6);
        
        #if ARDUINO >= 100
                uint8_t xhi = Wire.read();
                uint8_t xlo = Wire.read();
                uint8_t yhi = Wire.read();
                uint8_t ylo = Wire.read();
                uint8_t zhi = Wire.read();
                uint8_t zlo = Wire.read();
        #else
                uint8_t xhi = Wire.receive();
                uint8_t xlo = Wire.receive();
                uint8_t yhi = Wire.receive();
                uint8_t ylo = Wire.receive();
                uint8_t zhi = Wire.receive();
                uint8_t zlo = Wire.receive();
        #endif
        
        // Two's complement and combine hi-lo bytes
        accelData.x = ((xlo | (xhi << 8)) >> 1);
        accelData.y = ((ylo | (yhi << 8)) << 1) >> 1;
        accelData.z = ((zlo | (zhi << 8)) << 1) >> 1;        
};

/*=========================================================================
newGyroEvent() 
- Reads the latest data stored in the device, and saves it as a private
var in the class
 -----------------------------------------------------------------------*/
void MPU_9150_c :: newGyroEvent()
{
        // Read the accelerometer
        Wire.beginTransmission((byte)MPU_9150_Addr);
        #if ARDUINO >= 100
                Wire.write(MPU_9150_GYRO_XOUT_H);
        #else
                Wire.send(MPU_9150_GYRO_XOUT_H);
        #endif
        Wire.endTransmission();
        Wire.requestFrom((byte)MPU_9150_Addr, (byte)6);
        
        // Wait for enough data
        while (Wire.available() < 6);
        
        #if ARDUINO >= 100
                uint8_t xhi = Wire.read();
                uint8_t xlo = Wire.read();
                uint8_t yhi = Wire.read();
                uint8_t ylo = Wire.read();
                uint8_t zhi = Wire.read();
                uint8_t zlo = Wire.read();
        #else
                uint8_t xhi = Wire.receive();
                uint8_t xlo = Wire.receive();
                uint8_t yhi = Wire.receive();
                uint8_t ylo = Wire.receive();
                uint8_t zhi = Wire.receive();
                uint8_t zlo = Wire.receive();
        #endif
        
        // Two's complement and combine hi-lo bytes
        gyroData.x = (xlo | (xhi << 8))>>1;
        gyroData.y = (ylo | (yhi << 8))>>1;
        gyroData.z = (zlo | (zhi << 8))>>1;
        
};


/*=========================================================================
 write8(uint8_t I2Caddress, uint8_t reg, uint8_t value)
 - Write 8 bits to a specified register. Internal Function
 -----------------------------------------------------------------------*/
byte MPU_9150_c :: write8(byte I2Caddress, byte reg, byte value)
{
        byte error = 0x01;
        Wire.beginTransmission(I2Caddress);
	#if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
	#else
        Wire.send(reg);
        Wire.send(value);
	#endif
        error = Wire.endTransmission();
        return error;
};

/*=========================================================================
 read8(uint8_t I2Caddress, uint8_t reg)
 - Read 8 bits from a specified register. Internal Function
 -----------------------------------------------------------------------*/
byte MPU_9150_c :: read8(byte I2Caddress, byte reg)
{
        byte value;

        Wire.beginTransmission(I2Caddress);
	#if ARDUINO >= 100
        Wire.write(reg);
	#else
        Wire.send(reg);
	#endif
        Wire.endTransmission();

        Wire.requestFrom(I2Caddress, (byte)1);
	#if ARDUINO >= 100
        value = Wire.read();
	#else
        value = Wire.receive();
	#endif
        Wire.endTransmission();

        return value;
};

/*=========================================================================
 scanBus - Mostly a debug thing.
 -----------------------------------------------------------------------*/
byte MPU_9150_c::scanBus()
{
        byte error, address;
        int nDevices;

        Serial.println("Scanning...");

        nDevices = 0;
        for(address = 1; address < 127; address++ )
        {
                Wire.beginTransmission(address);
                error = Wire.endTransmission();

                if (error == 0)
                {
                        Serial.print("I2C device found at address decimal ");
                        if (address<16) Serial.print("0");
                        Serial.print(address);
                        Serial.print(", hex 0x");
                        Serial.print(address, HEX);
                        Serial.println("  !");

                        nDevices++;
                }
                else if (error==4)
                {
                        Serial.print("Unknow error at address 0x");
                        if (address<16)
                                Serial.print("0");
                        Serial.println(address);
                }
        }
        if (nDevices == 0)
                Serial.println("No I2C devices found\n");
        else
                Serial.println("done");
        return nDevices;
};

#endif // MPU_9150_H_INCLUDED

