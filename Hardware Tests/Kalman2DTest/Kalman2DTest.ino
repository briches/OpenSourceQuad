/* Kalman Filter test */
#include <KalmanFilter.h>

Kalman2D myKalman;

double measurements[6] = {1, 4, 7, 10, 13, 16};

void show(int matlength, int mat_cols, double matrix[])
{
  Serial.println("*********");
  Serial.println(" ");
  int col= 0;
  for (int i = 0; i< matlength; i++)
  {
    col ++;
    Serial.print(matrix[i]); Serial.print(" ");
   
    if ((col % mat_cols) == 0)
    {
      Serial.println();
      col = 0;
    }
    
  }
  Serial.println("");
  Serial.println("*********");
}

void setup()
{
  Serial.begin(115200);
  
  myKalman.KalmanInit_2D(1000, 1000, 1, 0.01);
}

void loop()
{
  for (int i = 0; i < 6; i++)
  {
    Serial.println(" Measure: ");
    myKalman.Kalman2DMeasure(measurements[i]);
    show(2, 1, myKalman.x);
    show(4, 2, myKalman.P);
    delay(10);
    
    Serial.println(" Predict: ");
    myKalman.Kalman2DPredict();
    show(2, 1, myKalman.x);
    show(4, 2, myKalman.P);
    delay(10);
 
  }
  while(1);
}
