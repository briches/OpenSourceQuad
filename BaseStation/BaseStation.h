/*======================================================================
 	BaseStation library
 	OpenSourceQuad
 	-------------------------------------------------------------------*/
/*================================================================================

 	Author		: Brandon Riches
 	Date		: August 2013
 	License		: GNU Public License

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

#ifndef BASESTATION_H_INCLUDED
#define BASESTATION_H_INCLUDED

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

int scanButtonInput()
{
        // * = 11, # = 12

        /*

        disarm = 0x00,
        autoland = 0x01,
        start = 0x02,
        broadcastData = 0x03,
        setAngleP = 0x0C,
        setAngleI = 0x0D,
        setAngleD = 0x0E

        */

        int d9state, d8state, d7state, d6state, d5state, d4state, d3state, d2state, d10state, d1state,d11state,d12state;

        d1state = digitalRead(30);
        d2state = digitalRead(31);
        d3state = digitalRead(32);
        d4state = digitalRead(33);
        d5state = digitalRead(34);
        d6state = digitalRead(35);
        d7state = digitalRead(36);
        d8state = digitalRead(37);
        d9state = digitalRead(38);
        d10state = digitalRead(39);
        d11state = digitalRead(40);
        d12state = digitalRead(41);
        

        if(d1state) Serial.println("You selected 1");
        if(d2state) Serial.println("You selected 2");
        if(d3state) Serial.println("You selected 3");
        if(d4state) Serial.println("You selected 4");
        if(d5state) Serial.println("You selected 5");
        if(d6state) Serial.println("You selected 6");
        if(d7state) Serial.println("You selected 7");
        if(d8state) Serial.println("You selected 8");
        if(d9state) Serial.println("You selected 9");
        if(d10state) Serial.println("You selected *");        
        if(d11state) Serial.println("You selected 0");
        if(d12state) Serial.println("You selected #");
        
        if(d1state) return 1;
        if(d2state) return 2;
        if(d3state) return 3;
        if(d4state) return 4;
        if(d5state) return 5;
        if(d6state) return 6;
        if(d7state) return 7;
        if(d8state) return 8;
        if(d9state) return 9;
        if(d10state) return 10;
        if(d11state) return 11;
        if(d12state) return 12;
        
        
        if(analogRead(0) > 512)
        {
                return 100;
                Serial.println("Saving PID coefficients: ");
        }
        
        return 0;
};

void printData(int P, int I, int D)
{
        Serial.print("P: ");
        Serial.print(P);
        Serial.print("  I: ");
        Serial.print(I);
        Serial.print("  D: ");
        Serial.println(D);
        Serial.println();
        
};

#endif // BASESTATION_H_INCLUDED
