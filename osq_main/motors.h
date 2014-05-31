/*=====================================================================
     Motors library
     OpenSourceQuad
     -------------------------------------------------------------------*/
/*================================================================================
 
     Author		: Brandon Riches
     Date		: August 2013
     License		: GNU Public License
     
     Implements an atomicPWM control of different motor configurations for the
     atmega2560 uC
     
     Copyright (C) 2013  Brandon Riches
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     
     -----------------------------------------------------------------------------*/

#ifndef MOTORS_H_INCLUDED
#define MOTORS_H_INCLUDED

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#define USE_300HZ_PWM 
#define USE_4MOTORS

#ifdef USE_150HZ_PWM
    #define PWM_FREQUENCY   150
#endif

#ifdef USE_200HZ_PWM
    #define PWM_FREQUENCY   200
#endif

#ifdef USE_250HZ_PWM
    #define PWM_FREQUENCY   250
#endif

#ifdef USE_300HZ_PWM
    #define PWM_FREQUENCY   300
#endif

#ifdef USE_400HZ_PWM
    #define PWM_FREQUENCY   400
#endif

#define PRESCALER		8
#define PWM_COUNTER_PERIOD	(F_CPU/PRESCALER/PWM_FREQUENCY)


#define MIN_COMMAND      	1075     // the shortest pulse
#define MAX_COMMAND     	1600     // the longest pulse

#define _PLUSconfig

//#define _Xconfig			1

enum {motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8};

int motorSpeeds[8] = {
    0, 0, 0, 0, 0, 0, 0, 0};

void initializePWM()
{
    // Using counters 3 and 4
    // These are 16bit counter registers
    DDRE = DDRE | B00111000;			// Set PE3-5, OC3A, OC3B, OC3C to outputs
    DDRH = DDRH | B00001000;			// Set PH3, OC4A to outputs

    // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
    ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.

    // Init PWM Timer 4
    TCCR4A = (1<<WGM41)|(1<<COM4A1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
};

void writeMotors()
{
    OCR3A = motorSpeeds[motor1] * 2;
    OCR3B = motorSpeeds[motor2] * 2;
    OCR3C = motorSpeeds[motor3] * 2;
    OCR4A = motorSpeeds[motor4] * 2;
};

void commandAllMotors(int speed)
{
    motorSpeeds[motor1] = speed;
    motorSpeeds[motor2] = speed;
    motorSpeeds[motor3] = speed;
    motorSpeeds[motor4] = speed;
    motorSpeeds[motor5] = speed;
    motorSpeeds[motor6] = speed;
    motorSpeeds[motor7] = speed;
    motorSpeeds[motor8] = speed;

    writeMotors();
};

/** Main Class **/
class MotorControl
{
    public:
        MotorControl(int num = 4);
    
        // Function ideas //
        void calibrateESC(int numESC = 4);
        void motorDISARM();
        void startMotors();
        void updateMotors(double pitchPID, double rollPID, double yawPID, double elevPID);
    
        double changeOperatingPoint(double opChange);
    
        double operatingPoint;
    
    private:
        bool 	MOTORS_ARMED;
        bool	ESC_READY;
        int 	NUM_MOTORS;
    
    
        int 	passiveMIN;
        int	passiveMAX;

};

MotorControl :: MotorControl(int num)
{
    this->NUM_MOTORS 	= num;
    this->MOTORS_ARMED 	= false;
    this->ESC_READY		= false;
    this->passiveMIN	= MIN_COMMAND+25;
    this->passiveMAX	= MAX_COMMAND-100;
    this->operatingPoint    = 1200;
    commandAllMotors(this->operatingPoint);
};

void MotorControl :: calibrateESC(int numESC)
{
    initializePWM();

    this->MOTORS_ARMED = true;

    // Calibration procedure
    // TODO: calibrate ESC

    this->ESC_READY = true;
};

void MotorControl :: startMotors()
{
    if(this->ESC_READY)
    {
        #ifdef USE_4MOTORS
            for(int DC = 0; DC < MIN_COMMAND; DC += 10)
            {
                motorSpeeds[motor1] = DC;
                motorSpeeds[motor2] = DC;
                motorSpeeds[motor3] = DC;
                motorSpeeds[motor4] = DC;
                writeMotors();
                delay(20);
            }
    
            delay(100);
    
            motorSpeeds[motor1] = MIN_COMMAND+100;
            motorSpeeds[motor2] = MIN_COMMAND+100;
            motorSpeeds[motor3] = MIN_COMMAND+100;
            motorSpeeds[motor4] = MIN_COMMAND+100;
            writeMotors();
        #endif
    }
};

void MotorControl :: updateMotors(double p_pitchPID, double p_rollPID, double p_yawPID, double p_elevPID)
{
    double PID_scalar = 1;

    double rollPID = (p_rollPID) / PID_scalar;
    double pitchPID = (p_pitchPID) / PID_scalar;
    double yawPID = (p_yawPID) / PID_scalar;
    double elevPID = (p_elevPID) / PID_scalar;

    if (MOTORS_ARMED)
    {
        #ifdef _PLUSconfig
            #ifdef USE_4MOTORS
                    /* //Control pitch/roll */
                    motorSpeeds[motor1] = operatingPoint + pitchPID;
                    motorSpeeds[motor2] = operatingPoint - rollPID;
                    motorSpeeds[motor3] = operatingPoint + rollPID;
                    motorSpeeds[motor4] = operatingPoint - pitchPID;
            
                    /* //Control elevation */
                    operatingPoint += elevPID;
            #endif

            #ifdef USE_4MOTORS
                    /* //Control yaw */
                    motorSpeeds[motor1] += yawPID;        // CW
                    motorSpeeds[motor2] -= yawPID;
                    motorSpeeds[motor3] -= yawPID;        // CCW
                    motorSpeeds[motor4] += yawPID;
            #endif

            // Restrict duty cycle to max/min
            motorSpeeds[motor1] = constrain(motorSpeeds[motor1], passiveMIN, passiveMAX);
            motorSpeeds[motor2] = constrain(motorSpeeds[motor2], passiveMIN, passiveMAX);
            motorSpeeds[motor3] = constrain(motorSpeeds[motor3], passiveMIN, passiveMAX);
            motorSpeeds[motor4] = constrain(motorSpeeds[motor4], passiveMIN, passiveMAX);
            motorSpeeds[motor5] = constrain(motorSpeeds[motor5], passiveMIN, passiveMAX);
            motorSpeeds[motor6] = constrain(motorSpeeds[motor6], passiveMIN, passiveMAX);
            motorSpeeds[motor7] = constrain(motorSpeeds[motor7], passiveMIN, passiveMAX);
            motorSpeeds[motor8] = constrain(motorSpeeds[motor8], passiveMIN, passiveMAX);
    
            writeMotors();
        #endif

        #ifdef _Xconfig
        //TODO:
        // Write motor logic for X config
        #endif
    }
};

// Used to manually control altitude.
double MotorControl :: changeOperatingPoint(double opChange)
{
    operatingPoint += opChange;
    constrain(operatingPoint, 1200, 1450);
    return operatingPoint;
};

void MotorControl :: motorDISARM()
{
    #ifdef USE_4MOTORS
        motorSpeeds[motor1] = 875;
        motorSpeeds[motor2] = 875;
        motorSpeeds[motor3] = 875;
        motorSpeeds[motor4] = 875;
    #endif

    writeMotors();

    MOTORS_ARMED = false;
};

MotorControl motorControl;
#endif // FQ_MOTORS_H_INCLUDED


