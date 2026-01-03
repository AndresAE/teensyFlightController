#ifndef GDC_H
#define GDC_H

#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../config/config.h"

class GDC
{
public:
  GDC(int sampleRate_Hz);

  int sampleRate_Hz;
  double dt_s;

  // public methods
  void begin();
  void step();
  void getControlInputs();
  void processControlInputs();
  void assignChannels();

  enum gdcRollMode
  {
    GRM_ROLL = 0,
    GRM_ROLLRATE = 1,
    GRM_BANK = 2,
  };

  enum gdcPitchMode
  {
    GPM_PITCH = 0,
    GPM_FPA = 1,
    GPM_ACCEL = 2,
    GPM_ALTITUDE = 3,
    GPM_PITCHRATE = 4,
  };

  enum gdcYawMode
  {
    GYM_HEADING = 0,
    GYM_AZIMUTH = 1,
    GYM_ACCEL = 2,
    GYM_YAWRATE = 3
  };

  struct busGDC
  {
    gdcRollMode rollMode_e;
    gdcPitchMode pitchMode_e;
    gdcYawMode yawMode_e;
    double rollCmd_d;
    double bankCmd_d;
    double rollRateCmd_dps;
    double pitchCmd_d;
    double fpaCmd_d;
    double nzCmd_B_fps2;
    double altitudeCmd_ft;
    double pitchRateCmd_dps;
    double headingCmd_d;
    double azimuthCmd_d;
    double nyCmd_B_fps2;
    double yawRateCmd_dps;
    double throttleCmd_pct;
  };

  busGDC m_GDC;

private:
  Common C;
  Config cfg;

  Config::gdcParams gdcParams = cfg.getGdcParams();

  int _nChannels = 0;
};

#endif // GDC_H
