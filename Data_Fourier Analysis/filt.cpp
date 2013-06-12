/*
    filt.cpp

    Preliminary Chebyshev filter code
    Gets the coefficients for the chebyshev digital filter, stores in A and B arrays



    Author: Brandon Riches with (a lot of) help from http://www.dspguide.com/ch20/4.htm .
    I took out all the functionality for cascaded filters.

*/


#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <iomanip>


using namespace std;


/*=========================================================================
    functions!
    -----------------------------------------------------------------------*/
void getChebyCoeff(double A[], double B[], double fcut, double PR, double NP);
void GOSUB(double A[], double B[], double fcut, double PR, double NP, int p);

double A[22];
double B[22];

int main()
{
    cout << setprecision(8) << endl;


    getChebyCoeff(A,B, 0.5, 0.5, 6);

    for(int i = 0; i<= 22; i++)
    {
        cout << "A[" << i << "] = " << A[i];
        cout << "    B[" << i << "] = " << B[i] << endl;
    }
    return 0;

}

void getChebyCoeff(double A[], double B[], double fcut, double PR, double NP)
{
    double TA[22];       // transient calculation arrays
    double TB[22];
    double SA = 0;
    double SB = 0;
    double GAIN =0;
    A[2] = 1;
    B[2] = 1;
    /* Debug
    cout << "FC = " << fcut << endl;
    cout << "LH = " << "0" << endl;
    cout << "PR = " << PR << endl;
    cout << "NP = " << NP << endl;
    */

    for (int p = 1; p <= NP/2; p++)
    {

        GOSUB(A, B, fcut, PR, NP, p);        // The magic, part one

        for (int i =0; i <= 22; i++)
        {
            TA[i] = A[i];
            TB[i] = B[i];
        }

        for (int i =2; i<=22; i++)
        {
            A[i] = A[0]*TA[i] + A[1]*TA[i-1] + A[2]*TA[i-2];      // The magic part two
            B[i] = TB[i] - B[1]*TB[i-1] - B[2]*TB[i-2];
        }

    }
    B[2] = 0;

    for (int i =0; i<= 20; i++)
    {
        A[i] = A[i+2];
        B[i] = -B[i+2];
    }

    for(int i = 0; i<=20; i++)
    {

        SA += A[i];
        SB += B[i];
    }

    GAIN = SA/(1-SB);

    for(int i = 0; i<=20; i++)
    {
        A[i] /= GAIN;
    }
}

void GOSUB(double A[], double B[], double fcut, double PR, double NP, int p)
{
    double RP;
    double IP;
    double ES;
    double VX, KX, T, W, M, D, K, X0, X1, X2, Y1, Y2;                 // some vars
    double Pi = 3.141592;

    RP = -cos(Pi/(NP*2) + (p-1)*Pi/NP);                                   // Pole location on unit circle
    IP = sin(Pi/(NP*2)+ (p-1)*Pi/NP);

    /* Debug
    cout << endl;
    cout << "RP = " << RP << endl;
    cout << "IP = " << IP << endl;
    */

    if(PR != 0)
    {
        ES = sqrt(pow(100/(100-PR),2) -1);
        VX = (1/NP) * log( (1/ES) +sqrt( (1/pow(ES,2)) + 1 ) );
        KX = (1/NP) * log( (1/ES) +sqrt( (1/pow(ES,2)) - 1 ) );
        KX = (exp(KX) + exp(-KX))/2;
        RP = RP*( (exp(VX) - exp(-VX))/2) / KX;
        IP = IP*( (exp(VX) + exp(-VX))/2) / KX;
    }

    /* Debug
    cout << "ES = " << ES << endl;
    cout << "VX = " << VX << endl;
    cout << "KX = " << KX << endl;
    */

    T = 2*tan(0.5);
    W = 2*Pi*fcut;
    M = pow(RP,2) + pow(IP,2);
    D = 4 -4*RP*T + M*pow(T,2);
    X0 = pow(T,2)/ D;
    X1 = 2*pow(T,2)/D;
    X2 = pow(T,2)/D;
    Y1 = (8- 2*M*pow(T,2))/D;
    Y2 = (-4-4*RP*T-M*pow(T,2))/D;

    K = sin(0.5 - W/2) / sin(0.5 + W/2);
    D = 1 + Y1*K - Y2*pow(K,2);
    A[0] = ((X0 - X1*K + X2*pow(K,2))/D);
    A[1] = (-2*X0*K + X1 + X1*pow(K,2) - 2*X2*K)/D;
    A[2] = (X0*pow(K,2) - X1*K + X2)/D;
    B[1] = (2*K + Y1 + Y1*pow(K,2) - 2*Y2*K)/D;
    B[2] = (-pow(K,2) - Y1*K + Y2)/D;

    /* Debug
    cout << "T = " << T << endl;
    cout << "W = " << W << endl;
    cout << "M = " << M << endl;
    cout << "D = " << D << endl;
    cout << "X0 = " << X0 << endl;
    cout << "X1 = " << X1 << endl;
    cout << "X2 = " << X2 << endl;
    cout << "Y1 = " << Y1 << endl;
    cout << "Y2 = " << Y2 << endl;

    cout << "A0 = " << A[0] << endl;
    cout << "A1 = " << A[1] << endl;
    cout << "A2 = " << A[2] << endl;
    cout << "B1 = " << B[1] << endl;
    cout << "B2 = " << B[2] << endl;
    */
}
