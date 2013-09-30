

void setup()
{
  Serial.begin(19200);
}

void loop()
{
  float range = 0;
  
  for (int i = 0; i<= 7; i++)
  {
    range += analogRead(0);
  }
  range /= 8; // Divide to finish the average calculation
  range /= 2; // Divide by two to convert to inches.
  range *= 0.0254; // Convert inches to m.
  Serial.println(range);
}

  
