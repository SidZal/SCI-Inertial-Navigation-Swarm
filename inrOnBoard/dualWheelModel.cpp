#include "dualWheelModel.h"
#include <math.h>

void dualWheelModel(double t, int n, double X[], double Xprime[], double omegaLeft, double omegaRight)
{
  if (n < 3) {
    Xprime = nullptr;
    return;
  }
  
  const double r = 0.020; // Radius of the wheel
  const double d = 0.081; // Distance between wheels

  Xprime[0] = r / 2. * (omegaLeft + omegaRight) * cos(X[2]);
  Xprime[1] = r / 2. * (omegaLeft + omegaRight) * sin(X[2]);
  Xprime[2] = r * ((omegaRight - omegaLeft)) / d;
}