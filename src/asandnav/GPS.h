#include <cmath>
#include <Arduino.h>
#include "Common.h"

class GPS {
public:
  GPS();

  struct busLLA{
    double latG_d;
    double lon_d;
    double hae_ft;
};

struct busEcef{
    double x_ft;
    double y_ft;
    double z_ft;
};

  struct busGPS{
    busLLA lla_ddft;
    busEcef ecef_ft;
    double time_s;
    int fixQuality;
    int nSatellites;
    double hdop;
};
  busLLA m_LLA = {0.0, 0.0, 0.0};
  busEcef m_ECEF = {0.0, 0.0, 0.0};
  busGPS m_GPS = {m_LLA, m_ECEF, 0.0, 0, 0, 0.0};

  int gps_baud = 9600;
  int boot_time_ms = 2000;

  void begin();
  void step();

private:
  Common C;
  double getX();
  double getY();
  double getVX();
  double getVY();
  void processInputs();
  double convertToDecimalDegrees(String raw, String dir);
  void splitNMEA(String s, String* fields, int maxFields);
  void processGNGGA(String s);
  double primeVerticalRadiusOfCurvature(double latG_d);
  busEcef lla2ecef(busLLA lla_ddft);

  double x, y, vx, vy, n_ft;
};
