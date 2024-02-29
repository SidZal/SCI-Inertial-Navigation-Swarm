#include <iostream>
#include <math.h>
#include "rk4.h"
#include "dualWheelModel.h"
#include "utility.h"

int main()
{
    cout.setf(ios::fixed);
    cout.setf(ios::showpoint);
    cout.precision(4);

    // Initialization 
    int i;
    int n = 3; // Number of the state 
    double dt = 0.1, t0 = 0, t1, tmax = 20.; // Time variables
    double* X0;
    double* X1;
    double wL = 0, wR = 0;
    const double pi = 3.141592653;
    double theta_desired = pi/2;
    double kp = 5;
    double wMax = 10, wComm;
    double error;

    X0 = new double[n];
    X0[0] = 0.0;
    X0[1] = 0.0;
    X0[2] = 0.0;

    while (t0 <= tmax)
    {
        // Orientation controller
        error = theta_desired - X0[2];
        wComm = speedSaturator(kp * error, wMax);
        wL = -wComm;
        wR = wComm;

        // March the dynamics system with a single time step
        t1 = t0 + dt;
        X1 = rk4(t0, n, X0, dt, wL, wR, dualWheelModel);
        t0 = t1;
        for (i = 0; i < n; i++)
        {
            X0[i] = X1[i];
        }
        cout << t0 << " " << X1[0] << " " << X1[1] << " " << X1[2] << " " << wL << " " << wR << endl;

    }

    return (0);
}


