
void setup()
{
    Serial3.begin(115200);
    Serial.begin(115200);
}

void loop()
{
    if(Serial3.available())
    {
         Serial.print((char)Serial3.read());
    }   
}
