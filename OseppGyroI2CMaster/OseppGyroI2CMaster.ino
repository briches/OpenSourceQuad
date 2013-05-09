/* 
Incorporates the MPU-3000 (OseppGyro) unit's ability to act as a master device
to a secondary I2C accelerometer. 

Should allow the accelerometer to function

*/ 


#include <OseppGyro.h>
#include <I2C.h>
#include <MMA8453_n0m1.h>

OseppGyro gyro;
MMA8453_n0m1 accel;

int ScaleRange = 2000;
int DLPF = 6;

int raw_accel[3];
int raw_gyro[3];



void setup()
{
  //Start the serial port for output
  Serial.begin(9600);
  
  /************************ Gyro initialization ************************/
  Serial.println(" ");
  
  Serial.println("Initializing gyro:  "); Serial.println(" ");
  
  Serial.print("  - Setting gyro I2C addr... ");
  gyro.setI2CAddr(0x69);
  Serial.println("   Gyro I2C addr set!"); Serial.println(" ");
  
  Serial.print("  - Setting gyro data mode... ");
  gyro.dataMode(ScaleRange, DLPF);
  Serial.println("   Gyro data mode set!"); Serial.println(" ");

  Serial.print("  - Setting gyro 3rd party accel enable... ");  
  byte x = 0x1;
  while(x != B100000) {
    gyro.regRead(0x3D,&x);
    gyro.regWrite(0x3D,B00100000); // Sets USER_CTRL to 00100000 to allow 3rd party accel on I2C
  }
  Serial.println("   Gryo 3rd party accel enabled!"); Serial.println(" ");
  
  Serial.println("Gyro initialization complete!"); Serial.println(" ");
  delay(1000);
  
  /******************** End Gyro initialization ************************/
  
  /*********************** Accel initialization ************************/

  Serial.println("Initializing accel:  "); Serial.println(" ");
  Serial.print("  - Setting accel I2C addr... ");  
  accel.setI2CAddr(0x1D); // I2C address of the acceleromter
  Serial.println("   Accel I2C addr set!"); Serial.println(" ");
  Serial.print("  - Setting accel data mode... ");
  accel.dataMode(true,2); //
  Serial.println("   Accel data mode set!"); Serial.println(" ");
  Serial.println("Accel initialization complete!"); Serial.println(" ");
  delay(1000);
  /******************* End Accel initialization ************************/
}

void loop()
{
  accel.update();
  raw_accel[0] = accel.x();
  raw_accel[1] = accel.y();
  raw_accel[2] = accel.z();
  Serial.print(" Accel:"); Serial.print(" X: "); Serial.print(raw_accel[0]);
                           Serial.print(" Y: "); Serial.print(raw_accel[1]);
                           Serial.print(" Z: "); Serial.print(raw_accel[2]); Serial.print("  ");
 
 
  gyro.update(); 
  raw_gyro[0] = gyro.x();
  raw_gyro[1] = gyro.y();
  raw_gyro[2] = gyro.z();
  Serial.print(" Gyro: "); Serial.print(" X: "); Serial.print(raw_gyro[0]);
                           Serial.print(" Y: "); Serial.print(raw_gyro[1]);
                           Serial.print(" Z: "); Serial.println(raw_gyro[2]);
}
  
  
  
  
  
  
  
  
  
  
  
