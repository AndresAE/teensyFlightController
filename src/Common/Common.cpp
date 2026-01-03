#include "Common.h"

Common::Common()
{
  // Constructor logic if needed
}

double Common::norm(double x, double y, double z)
{
  return sqrt(x * x + y * y + z * z);
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

double Common::primeVerticalRadiusOfCurvature(double latG_d)
{
  double sinLat = sin(DEG2RAD * latG_d);
  double e2 = WGS84_e * WGS84_e;
  return WGS84_r_ft / sqrt(1.0 - e2 * (sinLat * sinLat));
}

Common::busEcef Common::lla2ecef(busLLA lla_ddft)
{
  n_ft = primeVerticalRadiusOfCurvature(lla_ddft.latG_d);
  double cosLat = cos(DEG2RAD * lla_ddft.latG_d);
  double cosLon = cos(DEG2RAD * lla_ddft.lon_d);
  double sinLat = sin(DEG2RAD * lla_ddft.latG_d);
  double e2 = WGS84_e * WGS84_e;
  m_ECEF.x_ft = (n_ft + lla_ddft.hae_ft) * cosLat * cosLon;
  m_ECEF.y_ft = (n_ft + lla_ddft.hae_ft) * cosLat * sin(DEG2RAD * lla_ddft.lon_d);
  m_ECEF.z_ft = ((1.0 - e2) * n_ft + lla_ddft.hae_ft) * sinLat;
  return m_ECEF;
}
