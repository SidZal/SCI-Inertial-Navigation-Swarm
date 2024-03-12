#include "rk4.h"

void rk4(double t0, int m, double u0[], double up[], double dt, double omegaLeft, double omegaRight, void f(double t, int m, double u[], double up[], double omegaLeft, double omegaRight))
{
  if(m<3) {
    up = nullptr;
    return;
  }
  
  double f0[m], f1[m], f2[m], f3[m];
  int i;
  double t1;
  double t2;
  double t3;
  double u1[m], u2[m], u3[m];

  f(t0, m, u0, f0, omegaLeft, omegaRight);

  t1 = t0 + dt / 2.0;
  for (i = 0; i < m; i++)
  {
    u1[i] = u0[i] + dt * f0[i] / 2.0;
  }
  f(t1, m, u1, f1, omegaLeft, omegaRight);

  t2 = t0 + dt / 2.0;
  for (i = 0; i < m; i++)
  {
    u2[i] = u0[i] + dt * f1[i] / 2.0;
  }
  f(t2, m, u2, f2, omegaLeft, omegaRight);

  t3 = t0 + dt;
  for (i = 0; i < m; i++)
  {
    u3[i] = u0[i] + dt * f2[i];
  }
  f(t3, m, u3, f3, omegaLeft, omegaRight);

  for (i = 0; i < m; i++)
  {
    up[i] = u0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) / 6.0;
  }
}
