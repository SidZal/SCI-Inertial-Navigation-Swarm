#include "dualWheelModel.h"
#include <math.h>

double* dualWheelModel(double t, int n, double X[], double omegaLeft, double omegaRight)
{
    if (n < 3) {
        return nullptr;
    }

    double* Xprime;

    const double r = 0.036; // Radius of the wheel
    const double d = 0.149; // Distance between wheels

    Xprime = new double[n];

    Xprime[0] = r / 2. * (omegaLeft + omegaRight) * cos(X[2]);
    Xprime[1] = r / 2. * (omegaLeft + omegaRight) * sin(X[2]);
    Xprime[2] = r * ((omegaRight - omegaLeft)) / d;

    return Xprime;
}