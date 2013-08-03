/**
IIR Filter
**/
/**
Testing using a 512 point long data string.
The string is composed of three sine waves, with random noise.
**/

#include <iostream>
#include <math.h>
#include <fstream>

using namespace std;

#define ORDER 4

#define _b0  0.001893594048567
#define _b1 -0.002220262954039
#define _b2  0.003389066536478
#define _b3 -0.002220262954039
#define _b4  0.001893594048567

#define _a0  1
#define _a1 -3.362256889209355
#define _a2  4.282608240117919
#define _a3 -2.444765517272841
#define _a4  0.527149895089809

#define SIN_1_MAG  5
#define SIN_2_MAG  4
#define SIN_3_MAG  3

#define SIN_1_FREQ  1
#define SIN_2_FREQ  5
#define SIN_3_FREQ  1

#define pi  3.141593


double FILTERED_DATA[512];
double data[512];
double sample_freq = 100;

int overwrite = 4;

double IIRF(double data[], double FILTERED_DATA[], int *UPDATE)
{


	double result = (1 / _a0) *( (_b0 * data[overwrite]) + (_b1 * data[overwrite - 1]) + (_b2 * data[overwrite - 2]) + (_b3 * data[overwrite - 4]) + (_b4 * data[overwrite - 4]) - (_a1 * FILTERED_DATA[overwrite - 1]) - (_a2 * FILTERED_DATA[overwrite - 2]) - (_a3 * FILTERED_DATA[overwrite - 3]) - (_a4 * FILTERED_DATA[overwrite - 4]) );

	UPDATE++;

	return result;

}

void generate_data(int num_data, int noise_level, int num_sines)
{
	// t is 512 elements long
	double t[num_data];
	double dt = 1/sample_freq;

	// Fill the time array. These are the times that each data point was "sampled"
	t[0] = 0;
	for (int i =0; i< 512; i++)
	{
		t[i+1] = t[i] + dt;
	}

	// Construct the data from 1-3 sine waves and random noise.
	if (num_sines > 0)
	{
		for (int i = 0; i <= 512; i++)
		{
			data[i] = SIN_1_MAG * sin(2*pi*t[i]*SIN_1_FREQ);
		}
	}
	if (num_sines > 1)
	{
		for (int i = 0; i <= 512; i++)
		{
			data[i] = data[i] + SIN_2_MAG * sin(2*pi*t[i]*SIN_2_FREQ);
		}
	}
	if (num_sines > 2)
	{
		for (int i = 0; i <= 512; i++)
		{
			data[i] = data[i] + SIN_3_MAG * sin(2*pi*t[i]*SIN_3_FREQ);
		}
	}

	// Add random noise

}

int main()
{
	ofstream myfile;
	myfile.open("Data.csv");
	myfile << "Time, Unfiltered, Filtered \n";
	generate_data(512,0,3);


	for (int i = 0; i < 507; i ++ )
	{
		FILTERED_DATA[i+ORDER+1] = IIRF(data, FILTERED_DATA, &overwrite);
	}

	for (int i = 0; i <= 512; i++)\
	{
		myfile << i/sample_freq << "," << data[i] << "," <<  FILTERED_DATA[i] <<"\n";
	}

	myfile.close();

	return 0;
}
