#include "OSQ_SENSORLIB.h"
#include "Wire.h"


void setup()
{
    Serial.begin(19200);
    Serial.println("OSQ Flight Control Board V3.0 Sensors Test");
    accel.begin();
    gyro.begin();
    
}

void loop()
{
    sensors_event_t gyro_event;
    sensors_event_t accel_event;
    
    accel.getEvent(&accel_event);
    gyro.getEvent(&gyro_event);
    
    Serial.println(accel_event.acceleration.z);
    delay(100);
}
