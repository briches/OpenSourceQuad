// OSQ Flight Controller REV 0 Test

void setup()
{
        Serial.begin(19200);
}

void loop()
{
        if(Serial.available())
        {
                Serial.print((char)Serial.read());
        }
}
