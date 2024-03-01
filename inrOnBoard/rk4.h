#ifndef RK4_H
#define RK4_H

#include <iostream>
using namespace std;

double* rk4(double t0, int m, double u0[], double dt, double omegaLeft, double omegaRight, double* f(double t, int m, double u[], double omegaLeft, double omegaRight));

#endif
