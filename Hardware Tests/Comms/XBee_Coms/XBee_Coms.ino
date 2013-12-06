#include <SoftwareSerial.h>

void setup()  {
  Serial.begin(19200);
  // set the data rate for the SoftwareSerial port
  Serial3.begin(19200);
}



void loop()                     // run over and over again
{

  if (Serial3.available() > 0) {
      Serial.print((char)Serial3.read());
  }
  if (Serial.available() > 0) {
      Serial3.print((char)Serial.read());
  }
}
