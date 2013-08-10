
#include <filtmath.h>

double A[5] = {1.000000000000000, -3.362256889209355,   4.282608240117919, -2.444765517272841, 0.527149895089809};

double B[5] = {0.001893594048567, -0.002220262954039,  0.003389066536478, -0.002220262954039, 0.001893594048567};

double x[14] = {1,
                2,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1};


// Length of y should be: length of x + two times nfact
double y[14] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
          // i:{0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13};

void setup()
{
  Serial.begin(19200);
}

void loop()
{
  int SIZE_A = sizeof(A)/sizeof(A[0]);
  int SIZE_B = sizeof(B)/sizeof(B[0]);
  int SIZE_X = sizeof(x)/sizeof(x[0]);
  filtcheby2(A, B, x, y, SIZE_A, SIZE_B, SIZE_X);
  
  Serial.println("********************");
  while(1);
  
  
}
