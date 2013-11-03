

int dist = 0;
const int readpin = 0;

void convert_dist();
void setup()
{
  Serial.begin(9600);                // Begins serial comm
}

void loop()
{
  dist = analogRead(readpin);        // Reads the analog Voltage
  convert_dist();                    // Changes to cm
  Serial.println(dist);              // Prints to serial monitor
}

void convert_dist()
{
  if (dist >= 400) {dist = 10;}
  else if( (dist <= 400) && (dist > 290)) {dist = 15;}
  else if( (dist <= 290) && (dist > 230)) {dist = 20;}
  else if( (dist <= 230) && (dist > 190)) {dist = 25;}
  else if( (dist <= 190) && (dist > 170)) {dist = 30;}
  else if( (dist <= 170) && (dist > 150)) {dist = 35;}
  else if( (dist <= 150) && (dist > 130)) {dist = 40;}
  else if( (dist <= 130) && (dist >= 118)) {dist = 45;}
  else if( (dist <= 118) && (dist >= 107)) {dist = 50;}
  else if( (dist <= 107) && (dist >= 97)) {dist = 55;}
  else if( (dist <= 97) && (dist >= 91)) {dist = 60;}
  else if( (dist <= 91) && (dist >= 81)) {dist = 65;}
  else if( (dist <= 81) && (dist >= 75)) {dist = 70;}
  else if( (dist <= 75) && (dist >= 71)) {dist = 75;}
  else if( (dist <= 71) && (dist >= 69)) {dist = 80;}
  else if( (dist <= 69) && (dist >= 67)) {dist = 85;}
  else if( (dist <= 67) && (dist >= 65)) {dist = 90;}
  else if( (dist <= 65) && (dist >= 64)) {dist = 95;}
  else if( (dist <= 64) && (dist >= 63)) {dist = 100;}
  else if( (dist <= 63) && (dist >= 62)) {dist = 105;}
  else {dist = 999;}
}
