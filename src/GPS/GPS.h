#ifndef GPS_H
#define GPS_H

#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../config/config.h"

class GPS
{
public:
  GPS();

  // public methods
  void begin();
  void step();

  // bus definitions
  struct busGPS
  {
    Common::busLLA lla_ddft;
    Common::busEcef ecef_ft;
    double timeStamp_s;
    int fixQuality;
    int nSatellites;
    double hdop;
  };

  Common::busLLA m_LLA = {0.0, 0.0, 0.0};
  Common::busEcef m_ECEF = {0.0, 0.0, 0.0};
  busGPS m_GPS = {m_LLA, m_ECEF, 0, 0, 0, 0.0};

private:
  Common C;
  Config cfg;

  const Config::gpsParams &gpsParams;

  void processInputs();
  double convertToDecimalDegrees(String raw, String dir);
  void splitNMEA(String s, String *fields, int maxFields);
  void processGNGGA(String s);
};

#endif // GPS_H
