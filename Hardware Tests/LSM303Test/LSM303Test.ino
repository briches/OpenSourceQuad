/*
LSM303Test
Accelerometer Test. 
Magnetometer Test

Using the SENSORLIB library
*/
#include <I2C.h>
#include <SENSORLIB.h>

SENSORLIB accelmag;

void setup()
{
  Serial.begin(19200);
  accelmag.Easy_Start();
}

void loop()
{
  // Run at 10 Hz
  delay(100);
  
  // Update the values
  accelmag.update();
  
  // Fetch the values from the class
  double ax = accelmag.ax();
  double ay = accelmag.ay();
  double az = accelmag.az();
  
  double mx = accelmag.mx();
  double my = accelmag.my();
  double mz = accelmag.mz();
  
  Serial.print("ax: ");
  Serial.print(ax);
  Serial.print(" ay: ");
  Serial.print(ay);
  Serial.print(" az: ");
  Serial.print(az);
  
  Serial.print(" mx: ");
  Serial.print(mx);
  Serial.print(" my: ");
  Serial.print(my);
  Serial.print(" mz: ");
  Serial.println(mz);
}

