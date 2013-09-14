// Continuous laser tachometer

double estimatedRPM;
volatile bool checkLowISR = false;
bool propellorDetected = false;

unsigned long t_Detect = 0;

void setup()
{
        Serial.begin(115200);
        attachInterrupt(0, diodeLowISR, CHANGE);        // Attach to pin 2, falling edge interrupt
}

void loop()
{
        double rpm = 0.0;
        double scale = 30000000;
        if(checkLowISR)
        {
                rpm = scale / (double)(micros() - t_Detect); // 60 * (1/T) / 2 
                t_Detect = micros();
                Serial.println(rpm,1);
                checkLowISR = false;
        }
}

void diodeLowISR()
{
        checkLowISR = true;
}

