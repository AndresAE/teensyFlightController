#include "Common.h"

Common::Common()
{
  // Constructor logic if needed
}

double Common::norm(double x, double y, double z)
{
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

double Common::trapIntegral(double x, double x_prev, double y, double dt_s)
{
  return (x + x_prev) * dt_s / 2 + y;
}

double Common::lowPassFilter(double x, double x_prev, double alpha)
{
  return alpha * x + (1.0 - alpha) * x_prev;
}

double Common::calcAlpha(double cutoff_Hz, double dt_s)
{
  return dt_s / (1.0 / (2.0 * _PI * cutoff_Hz) + dt_s);
}

double Common::primeVerticalRadiusOfCurvature(double latG_d) {
    return WGS84_r_ft / (pow(1 - pow(WGS84_e, 2) * (pow(sin(DEG2RAD * latG_d), 2)), 0.5));
}

Common::busEcef Common::lla2ecef(busLLA lla_ddft) {
    n_ft = primeVerticalRadiusOfCurvature(lla_ddft.latG_d);
    m_ECEF.x_ft = (n_ft + lla_ddft.hae_ft) * cos(DEG2RAD * lla_ddft.latG_d) * cos(DEG2RAD * lla_ddft.lon_d);
    m_ECEF.y_ft = (n_ft + lla_ddft.hae_ft) * cos(DEG2RAD * lla_ddft.latG_d) * sin(DEG2RAD * lla_ddft.lon_d);
    m_ECEF.z_ft = ((1 - pow(WGS84_e, 2))*n_ft + lla_ddft.hae_ft) * sin(DEG2RAD * lla_ddft.latG_d);
    return m_ECEF;
}
