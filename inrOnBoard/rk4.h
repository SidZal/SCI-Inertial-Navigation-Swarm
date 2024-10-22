#ifndef RK4_H
#define RK4_H

#include <iostream>
using namespace std;

void rk4(double t0, int m, double u0[], double up[], double dt, double omegaLeft, double omegaRight, void f(double t, int m, double u[], double up[], double omegaLeft, double omegaRight));

#endif
