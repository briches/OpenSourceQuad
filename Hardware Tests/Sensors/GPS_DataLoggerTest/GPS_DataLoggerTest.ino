/********************
This sketch tests the data logging capability of the sd card, only.
Use with, or without, motors.



 ****** Important *********
 Do ctrl + f, search this:
 
 "75wxT1.txt"
 
 and REPLACE ALL with your new file name.


 
********************/
#include <SdFat.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <MMA8453_n0m1.h>
#include <Servo.h>
#include <cstdio.h>
 
 
int x =0;
// Adafruit sd shields and modules: pin 10
const int chipSelect = 10;  
// Create the file
SdFat sd;
SdFile myFile;

String BUFFER;
char tstring[15];
const int myBufLen = 32;
int count;

OseppGyro gyro;
MMA8453_n0m1 accel;

int ScaleRange = 250;
int DLPF = 6;

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

void setup() 
{
 // Open serial communications and wait for port to open:
  Serial.begin(19200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // Initialize sdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) sd.initErrorHalt();


  // open the file for write at end like the Native sd library
  if (!myFile.open("75wxT1.txt", O_RDWR | O_CREAT)) {
    sd.errorHalt("opening ""75wxT1.txt"" for write failed");
  }
  // if the file opened okay, write to it:
  Serial.print("Writing to ""75wxT1.txt""...");
  myFile.println("75wxT1.txt");

  // close the file:
  myFile.close();
  Serial.println("done.");


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
  delay(10);
  
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
  delay(10);
  /******************* End Accel initialization ************************/
  
  if (!myFile.open("75wxT1.txt", O_RDWR | O_AT_END)) 
  {
    sd.errorHalt("opening test.txt for write failed");
  }
  else {
    myFile.println("Begin Data: ");
  }
  
  Motor1.attach(5);
  Motor2.attach(10);
  Motor3.attach(9);
  Motor4.attach(6);
  
  
}

void loop() 
{
  if (millis() <= 30000)
  {
    if (x == 0)
    {
      for(x = 0; x <= 75; x += 1)
      {
        Motor1.write(x);
        Motor2.write(x);
        Motor3.write(x);
        Motor4.write(x);
        delay(50);
      }
    }
    
    accel.update();
    int ax = accel.x();  // Divide by 100 to get to m/s^2
    
    int ay = accel.y();

    int az = accel.z();

    gyro.update(); 
    int wx = gyro.x();

    int wy = gyro.y();
    
    int wz = gyro.z();
    
//    int myvar = 100*ax;
    
    myFile.print((String)micros() + ", ");
    sprintf(tstring, "%d", wx);
    myFile.println(tstring);
    
  }
  
  else {
    myFile.close();
    
  }
}

