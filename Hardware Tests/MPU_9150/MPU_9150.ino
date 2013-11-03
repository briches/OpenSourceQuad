#include "MPU_9150.h"
#include <Wire.h>

// Class initialization
MPU_9150_c        MPU_9150;



void setup()
{
        Serial.begin(19200);
        Wire.begin();
        Serial.println("\n\nMPU-9150 Test and Learning\n");
        
        delay(1000);
        
        //Scan the bus
        uint8_t I2CStatus = MPU_9150.scanBus();
        while(I2CStatus != 0x01)
        {
                I2CStatus = MPU_9150.scanBus();
                Serial.print(I2CStatus);
                Serial.println(" device(s) found. \n");
                delay(1000);
        }
        
        
        // Setup the MPU
        uint8_t deviceID = MPU_9150.initMPU();
        Serial.println(deviceID);
}

void loop()
{
}
