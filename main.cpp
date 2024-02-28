#include <iostream>
#include <math.h>
#include "rk4.h"

int main()
{
    double dt = 0.1;
    int i;
    int n = 3;
    double t0;
    double t1;
    double tmax = 12.0;
    double* u0;
    double* u1;

    t0 = 0.0;

    u0 = new double[n];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;

    double wL = 0;
    double wR = 1;

    while (t0 <= tmax)
    {
        t1 = t0 + dt;
        u1 = rk4vec(t0, n, u0, dt, wL, wR, rk4vec_test_f);

        t0 = t1;
        for (i = 0; i < n; i++)
        {
            u0[i] = u1[i];
        }
        cout << u0[0] << " " << u0[1] << " " << u0[2] << endl;
    }


    return (0);
}

