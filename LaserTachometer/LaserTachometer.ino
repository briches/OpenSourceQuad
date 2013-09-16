// Continuous laser tachometer

#define laser 13
#include "OSQ_Motors.h"

double estimatedRPM;
bool propellorDetected = false;

unsigned long t_Detect = 0;
double rpmFilter[10] = {
        0,0,0,0,0,0,0,0,0,0};

OSQ_MotorControl motor;

void setup()
{
        Serial.begin(115200);
        motor.calibrateESC();
        motor.startMotors();
        delay(500);
        motorSpeeds[0] = 1350;
        writeMotors();

        pinMode(laser, OUTPUT);
}

double sumArray()
{
        double sum = 0;
        for(int i = 0; i <10; i++)
        {
                sum += rpmFilter[i];
        }
        return sum;
}

void loop()
{        
        static int rpmPointer = 0;
        double scale = 30000000;

        static double laserFrequency = 10;
        double laserPeriod;
        static double laserPeriodOn = 2000;
        static double laserTimerOn;
        static bool laserOn = false;

        laserPeriod = 1000000 / laserFrequency;

        double nextPulseTime;
        static double phi;

        static int phiCount;
        static int pulseCount;


        if ( micros() > nextPulseTime ) 
        {
                digitalWrite(laser, HIGH);
                laserOn = true;
                nextPulseTime = micros() + laserPeriod - phi;
                laserTimerOn = micros();
        }

        if ( micros() - laserTimerOn > laserPeriodOn && laserOn)
        {
                digitalWrite(laser, LOW);
                laserOn = false;
        }

        unsigned long t_halfPeriod = 0;

        int diodeV = analogRead(8);

        double diodeFrequency;

        if(diodeV < 55)
        {
                if(!propellorDetected)
                {
                        t_halfPeriod = micros() - t_Detect;
                        rpmFilter[rpmPointer] = 500000/(t_halfPeriod);
                        diodeFrequency = sumArray() / 10;


                        t_Detect = micros();
                        propellorDetected = true;
                }
        }

        if(diodeV > 70)
        {
                if(propellorDetected)
                {
                        propellorDetected = false;
                }
        }

        Serial.print("This is NOT the answer:");
        Serial.println(laserFrequency);

        static int PulseCount;

        if ( abs(diodeFrequency - laserFrequency) < 0.5 ) 
        {
                while(1) {
                        Serial.print("HOLY SHIT, it's the answer! The frequency is: ");
                        Serial.println(laserFrequency);
                }
        }
        else {  
                PulseCount++; 
        }

        if (PulseCount == 10)
        {
                pulseCount = 0;
                phi += laserPeriod / 100;
        }
        if (phiCount++ == 100) {
                phiCount = 0;
                laserFrequency++;                
        }




        rpmPointer++;
        if(rpmPointer > 9) rpmPointer = 0;
}


