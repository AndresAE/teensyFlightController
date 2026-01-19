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

    // bus definitions
    struct busEcef
    {
        double x_ft;
        double y_ft;
        double z_ft;
    };

    struct busLLA
    {
        double latG_d;
        double lon_d;
        double hae_ft;
    };

    struct matrix
    {
        int rows;
        int cols;
        float m[10][10]; // max size, but you only use rows×cols
    };

    // common methods
    double norm(double x, double y, double z);
    double calcAlpha(double cutoff_Hz, double dt_s);
    double trapIntegral(double x, double x_prev, double y, double dt_s);
    double lowPassFilter(double x, double x_prev, double alpha);
    double primeVerticalRadiusOfCurvature(double latG_d);

    matrix matrixMultiply(matrix A, matrix B);
    matrix matrixAdd(matrix A, matrix B);
    matrix matrixAdd(matrix A, matrix B, bool negate_b = false);
    matrix matrixTranspose(matrix A);
    matrix matrixInverse(matrix A);
    matrix crossProductMatrix(matrix A, matrix B);
    busEcef lla2ecef(busLLA lla_ddft);
    busLLA ecef2lla(busEcef ecef_ft);

private:
    double x, y, z;
    double x_prev, alpha, dt_s;
    double n_ft;
    busLLA m_LLA = {0.0, 0.0, 0.0};
    busEcef m_ECEF = {0.0, 0.0, 0.0};
};
#endif // COMMON_H
