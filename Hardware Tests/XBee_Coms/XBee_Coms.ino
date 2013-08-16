#include <SoftwareSerial.h>

SoftwareSerial mySerial =  SoftwareSerial(43, 41);


void setup()  {
  Serial.begin(19200);
  Serial.println("Goodnight moon!");
  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);
  mySerial.println("Hello, world?");
}



void loop()                     // run over and over again
{

  if (mySerial.available() > 0) {
      Serial.print((char)mySerial.read());
  }
  if (Serial.available() > 0) {
      mySerial.print((char)Serial.read());
  }
}
