#ifndef RK4_H
#define RK4_H

#include <iostream>
using namespace std;

double* rk4vec(double t0, int m, double u0[], double dt, double wL, double wR,
	double* f(double t, int m, double u[], double wL, double wR));
double* rk4vec_test_f(double t, int n, double u[], double wL, double wR); 

#endif