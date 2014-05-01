

void setup()
{
    Serial.begin(19200);
    Serial1.begin(19200);
    
    Serial.println("OSQ Control Board V3.0 XB Test");
    Serial1.println("Hello World?");
}

void loop()
{
    Serial1.println("loop");
    delay(1000);
}
