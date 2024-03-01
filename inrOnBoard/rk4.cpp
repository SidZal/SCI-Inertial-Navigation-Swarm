#include "rk4.h"

double* rk4(double t0, int m, double u0[], double dt, double omegaLeft, double omegaRight, double* f(double t, int m, double u[], double omegaLeft, double omegaRight))
{
    double* f0;
    double* f1;
    double* f2;
    double* f3;
    int i;
    double t1;
    double t2;
    double t3;
    double* u;
    double* u1;
    double* u2;
    double* u3;

    f0 = f(t0, m, u0, omegaLeft, omegaRight);

    t1 = t0 + dt / 2.0;
    u1 = new double[m];
    for (i = 0; i < m; i++)
    {
        u1[i] = u0[i] + dt * f0[i] / 2.0;
    }
    f1 = f(t1, m, u1, omegaLeft, omegaRight);

    t2 = t0 + dt / 2.0;
    u2 = new double[m];
    for (i = 0; i < m; i++)
    {
        u2[i] = u0[i] + dt * f1[i] / 2.0;
    }
    f2 = f(t2, m, u2, omegaLeft, omegaRight);

    t3 = t0 + dt;
    u3 = new double[m];
    for (i = 0; i < m; i++)
    {
        u3[i] = u0[i] + dt * f2[i];
    }
    f3 = f(t3, m, u3, omegaLeft, omegaRight);

    u = new double[m];
    for (i = 0; i < m; i++)
    {
        u[i] = u0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) / 6.0;
    }

    return u;
}
