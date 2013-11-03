
#define USRF_PIN 3
#define SCALE   0.01266762
void setup()
{
  Serial.begin(19200);
}

void loop()
{
  double value = analogRead(USRF_PIN);
  double range = value * SCALE;
  Serial.println(range);
  delay(20);
}
