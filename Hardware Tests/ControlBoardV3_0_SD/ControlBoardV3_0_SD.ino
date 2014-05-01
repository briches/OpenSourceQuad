#include <SD.h>

File testFile;
char filename[12] = "test.txt";

void setup()
{
    pinMode(SS, OUTPUT);
    Serial.begin(19200);
    Serial.println("OSQ Control Board V3.0 SD Card Test");
    if (!SD.begin(SS)) 
    {
        Serial.println("--------Initialization failed!");
    }
    
    testFile = SD.open(filename, FILE_WRITE);
    
    if(testFile)
    {
        Serial.println("Success!");
        testFile.println("It worked!");
    }
    else
    {
        Serial.println("Error opening file for write!");
    }
    
    testFile.close();
}

void loop()
{
    
}
