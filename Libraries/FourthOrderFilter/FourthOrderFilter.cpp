#include <FourthOrderFilter.h>
#include <GlobalDefined.h>

float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters)
{
  // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567

  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809

  float output;

  output = _b0 * currentInput                +
           _b1 * filterParameters->inputTm1  +
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;

  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;

  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;

  return output;
}

void setupFourthOrder(void)
{
  fourthOrder[XAXIS].inputTm1 = 0.0;
  fourthOrder[XAXIS].inputTm2 = 0.0;
  fourthOrder[XAXIS].inputTm3 = 0.0;
  fourthOrder[XAXIS].inputTm4 = 0.0;

  fourthOrder[XAXIS].outputTm1 = 0.0;
  fourthOrder[XAXIS].outputTm2 = 0.0;
  fourthOrder[XAXIS].outputTm3 = 0.0;
  fourthOrder[XAXIS].outputTm4 = 0.0;

  //////////
  fourthOrder[YAXIS].inputTm1 = 0.0;
  fourthOrder[YAXIS].inputTm2 = 0.0;
  fourthOrder[YAXIS].inputTm3 = 0.0;
  fourthOrder[YAXIS].inputTm4 = 0.0;

  fourthOrder[YAXIS].outputTm1 = 0.0;
  fourthOrder[YAXIS].outputTm2 = 0.0;
  fourthOrder[YAXIS].outputTm3 = 0.0;
  fourthOrder[YAXIS].outputTm4 = 0.0;

  //////////
  fourthOrder[ZAXIS].inputTm1 = -9.8065;
  fourthOrder[ZAXIS].inputTm2 = -9.8065;
  fourthOrder[ZAXIS].inputTm3 = -9.8065;
  fourthOrder[ZAXIS].inputTm4 = -9.8065;

  fourthOrder[ZAXIS].outputTm1 = -9.8065;
  fourthOrder[ZAXIS].outputTm2 = -9.8065;
  fourthOrder[ZAXIS].outputTm3 = -9.8065;
  fourthOrder[ZAXIS].outputTm4 = -9.8065;
}
