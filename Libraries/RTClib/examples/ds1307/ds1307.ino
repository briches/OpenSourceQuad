// Date and time functions using a DS1307 RTC connected via I2C and Wire lib

#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 rtc;

char logFilename[12];

void getFilename(DateTime now)
{
  int DigitOne = now.day()/10;
  int DigitTwo = now.day() - 10*DigitOne;
  
  logFilename[0] = (char)(((int)'0')+DigitOne);
  logFilename[1] = (char)(((int)'0')+DigitTwo);
  logFilename[2] = '_';
  
  DigitOne = now.hour()/10;
  DigitTwo = now.hour() - 10*DigitOne;
  
  logFilename[3] = (char)(((int)'0')+DigitOne);
  logFilename[4] = (char)(((int)'0')+DigitTwo);
  logFilename[5] = '_';  
  
  DigitOne = now.minute()/10;
  DigitTwo = now.minute() - 10*DigitOne;
  
  logFilename[6] = (char)(((int)'0')+DigitOne);
  logFilename[7] = (char)(((int)'0')+DigitTwo);
 
  logFilename[8] = '.';
  logFilename[9] = 't';
  logFilename[10] = 'x';
  logFilename[11] = 't';

}

void setup () {
  Serial.begin(19200);
#ifdef AVR
  Wire.begin();
#else
  Wire.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
}

void loop () {
    DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    
    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");
    
    // calculate a date which is 7 days and 30 seconds into the future
    DateTime future (now.unixtime() + 7 * 86400L + 30);
    
    Serial.print(" now + 7d + 30s: ");
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();
    
    Serial.println("Filename: ");
    
    getFilename(now);
    for(int i =0; i< 12; i++)
    {
      Serial.print(logFilename[i]);
    }
    Serial.println("");
    delay(3000);
}
