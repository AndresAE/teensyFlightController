#ifndef COMMON_H
#define COMMON_H
#include <cmath>

class Common
{
public:
    Common();

    double _PI = 3.141592653589793;
    double GRAVITY_fps2 = 32.174;    // ft/s²
    double WGS84_r_ft = 20925646.33; // ft
    double WGS84_e = 0.08181919078;
    double WGS84_f = 0.00335281066;
    double DEG2RAD = 0.01745329251994;
    double RAD2DEG = 57.2957795130823;
    double FT2M = 0.3048;
    double M2FT = 3.2808;
    double MIN2SEC = 60.0;

    double norm(double x, double y, double z);
    double calcAlpha(double cutoff_Hz, double dt_s);
    double trapIntegral(double x, double x_prev, double y, double dt_s);
    double lowPassFilter(double x, double x_prev, double alpha);

private:
    double x, y, z;
    double x_prev, alpha, dt_s;
};
#endif // COMMON_H
