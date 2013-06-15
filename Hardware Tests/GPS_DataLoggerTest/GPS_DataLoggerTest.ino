/********************
This sketch tests the data logging capability of the SD card, only.
Use with, or without, motors.



 ****** Important *********
 Do ctrl + f on this
 "LogTest1.txt"
 
 and REPLACE ALL with your new file name.


 
********************/
#include <SD.h>
#include <OseppGyro.h>
#include <I2C.h>
#include <MMA8453_n0m1.h>
#include <Servo.h>
#include <cstdio.h>
 
// Adafruit SD shields and modules: pin 10
const int chipSelect = 10;  
// Create the file
File myFile;
String BUFFER;
char tstring[15];
const int myBufLen = 10;
int count;

OseppGyro gyro;
MMA8453_n0m1 accel;

int ScaleRange = 250;
int DLPF = 6;

void setup() 
{
 // Open serial communications and wait for port to open:
  Serial.begin(19200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  if (SD.exists("LogTest1.txt"))
  {
    SD.remove("LogTest1.txt");
  }
  myFile = SD.open("LogTest1.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Write test.txt...");
    myFile.println("LogTest1.txt");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
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
  
  
}
void loop() 
{
  accel.update();
  long ax = accel.x();
  ax *= 38.32;      // Divide by 100 to get to m/s^2
  
  int ay = accel.y();
  ay *= 0.03832;
  
  int az = accel.z();
  az *= 0.03832;
  
  gyro.update(); 
  int wx = gyro.x();
  wx *= 0.007608;
  
  int wy = gyro.y();
  wy *= 0.007608;
  
  int wz = gyro.z();
  wz *= 0.007608;
  
  sprintf(tstring, "%d", ax);
  BUFFER += ( tstring + "\n");
  count ++;
  
  if(count >= myBufLen)
  {
    myFile = SD.open("LogTest1.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(String(millis()) + ", ");
      myFile.print(BUFFER);
      myFile.close();
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
    }
    BUFFER = "";
    count = 0;
  }
}

