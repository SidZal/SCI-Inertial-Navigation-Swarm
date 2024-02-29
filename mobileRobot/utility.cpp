#include "utility.h"

double speedSaturator(double omega, double omegaMax)
{
    if (omega > omegaMax)
        return omegaMax;
    else if (omega < -omegaMax)
        return -omegaMax;
    else
        return omega;
}