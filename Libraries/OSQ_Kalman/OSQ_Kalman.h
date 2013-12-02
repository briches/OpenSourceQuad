/*=====================================================================
	OSQ_Kalman library
	OpenSourceQuad
	-------------------------------------------------------------------*/
/*================================================================================

	Author		: Andrew Coulthard
	Date		: August 2013
	License		: GNU Public License

	Implements a two-dimensional Kalman Filter

	Copyright (C) 2013  Andrew Coulthard

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
// Nice job andrew

#ifndef OSQ_KALMAN_H_INCLUDED
#define OSQ_KALMAN_H_INCLUDED

#define initialUncertainty 1000

typedef struct Kalman_t
{
        double  P0_,
                P1_,
                P2_,
                P3_,
                x1_,
                x2_;
        
        unsigned long timestamp;
        
        double measurementNoise, processNoise;
        
        double kalmanUpdate(double z); // Measurement and update
        double kalmanSpeed(); // Returns x'
        double kalmanCovariance(int selection); // Returns either P0_ or P3_
        
        Kalman_t(double mNoise, double pNoise);
};

// Initialize the instances of Kalman Filter we will be using
Kalman_t rollKalman(10., 0.5);
Kalman_t pitchKalman(10., 0.5);

// Constructor, also initializes vars
Kalman_t :: Kalman_t(double mNoise, double pNoise) 
{
        measurementNoise = mNoise;
        processNoise = pNoise; // Gaussian rand added to final measurement
        P0_ = initialUncertainty;
        P1_ = 0;
        P2_ = 0;
        P3_ = initialUncertainty;
        x1_ = 0;
        x2_ = 0;
};

// Main update
double Kalman_t :: kalmanUpdate(double z) 
{
        double y, S, K[2], Ky[2];
        double dt = (micros() - timestamp)/1000000.;
        
        /** Measurement Update **/
	// y = z - Hx
	y = z - x1_;


	// S = H*P*HT + measurement_noise
	S = P0_ + measurementNoise * random(-1,1);

	// K = P*HT*S^-1
	K[0] = P0_ * (1 / S);
	K[1] = P2_ * (1 / S);

	// X^ = X + K*y
	Ky[0] = K[0]*y;
	Ky[1] = K[1]*y;
	x1_ += Ky[0];
	x2_ += Ky[1];
        
        
        /** Prediction Step **/
        double alpha, beta, gamma, epsilon;
        
        alpha =   P0_ - P0_ * P0_ / (P0_ + measurementNoise);
        beta =    P1_ - P0_ * P1_ / (P0_ + measurementNoise);
        gamma =   P2_ - P0_ * P2_ / (P0_ + measurementNoise);
        epsilon = P3_ - P1_ * P2_ / (P0_ + measurementNoise);
        
        P0_ = alpha + dt * (gamma + beta) + dt * dt * epsilon;
        P1_ = beta + dt * epsilon;
        P2_ = gamma + dt * epsilon;
        P3_ = epsilon;

	// x
	x1_ += x2_ * dt + processNoise * random(-1,1);

	return x1_; // position
};

double Kalman_t :: kalmanSpeed() 
{
        return x2_; // speed
};

double Kalman_t :: kalmanCovariance(int selection)
{
        if (selection == 0) return P0_; // position covar
        if (selection == 1) return P3_; // speed covar
        else return 0;
};

#endif // OSQ_KALMAN_H_INCLUDED

