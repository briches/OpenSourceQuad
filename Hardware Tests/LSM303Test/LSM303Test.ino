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
  Serial.begin(115200);
  accelmag.Easy_Start(ACCEL_ODR_100_74, ACCEL_FULL_SCALE_2g, MAG_ODR_75_0, MAG_GAIN_1_3);
}

void loop()
{
  // Update the values
  accelmag.update_accel();
  accelmag.update_mag();
  
  // Fetch the values from the class
  double ax = accelmag.ax();
  double ay = accelmag.ay();
  double az = accelmag.az();
  
  double mx = accelmag.mx();
  double my = accelmag.my();
  double mz = accelmag.mz();
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(my,mx) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  
//  Serial.print("Compass Heading: ");
//  Serial.println(heading);
  delay(250);
  
  Serial.print("  ax: ");
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

