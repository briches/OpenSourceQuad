/*
    filt.cpp

    Preliminary Chebyshev filter code
    If you have your array of A and B coefficients from the MATLAB script, this code applies the IIR Digital filter formula
    to filter the signal.



    Author: Brandon Riches with (a lot of) help from http://www.dspguide.com/ch20/4.htm .

*/


#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <iomanip>

double A[22];
double B[22];

int main()
{
    cout << setprecision(8) << endl;


    getChebyCoeff(A,B, 0.1, 0.0, 4);

    for(int i = 0; i<= 22; i++)
    {
        cout << "A[" << i << "] = " << A[i];
        cout << "    B[" << i << "] = " << B[i] << endl;
    }
    return 0;

}
