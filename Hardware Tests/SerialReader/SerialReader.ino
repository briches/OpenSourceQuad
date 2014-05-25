
void setup()
{
    Serial3.begin(38400);
    Serial.begin(38400);
}

void loop()
{
    if(Serial3.available())
    {
         Serial.print((char)Serial3.read());
    }   
}
