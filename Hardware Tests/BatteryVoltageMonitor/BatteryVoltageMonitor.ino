// Voltage monitor test

#define battPin  3
#define conversion  0.2342F

double Vmeasure;


void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Vmeasure = analogRead(battPin)* (5.0/1023.0) / conversion;
  Serial.print("Measured battery voltage: ");
  Serial.println(Vmeasure);
  Serial.println("");
  delay(1000);
}

