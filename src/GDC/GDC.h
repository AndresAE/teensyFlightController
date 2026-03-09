#ifndef GDC_H
#define GDC_H

#include <cmath>
#include <Arduino.h>
#include "../CMN/CMN.h"
#include "../TST/TST.h"
#include "../MSN/MSN.h"
#include "../CFG/CFG.h"

class GDC
{
public:
  GDC(int sampleRate_Hz, TST &tst, MSN &msn);

  int sampleRate_Hz;
  double dt_s;

  // public methods
  void begin();
  void step();
  void getTestInputs();
  void processGuidanceCommands();
  void applyCommandLimits(double value, double minLimit, double maxLimit);

  enum gdcRollMode
  {
    GRM_ROLL = 0,
    GRM_ROLLRATE = 1,
    GRM_BANK = 2,
    GRM_UNDEFINED = 256,
  };

  enum gdcPitchMode
  {
    GPM_PITCH = 0,
    GPM_FPA = 1,
    GPM_ACCEL = 2,
    GPM_ALTITUDE = 3,
    GPM_PITCHRATE = 4,
    GPM_UNDEFINED = 256,
  };

  enum gdcYawMode
  {
    GYM_HEADING = 0,
    GYM_AZIMUTH = 1,
    GYM_ACCEL = 2,
    GYM_YAWRATE = 3,
    GYM_UNDEFINED = 256,
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

  // references to other modules
  TST &m_TST_obj;
  MSN &m_MSN_obj;

private:
  CMN C;
  CFG cfg;

  CFG::gdcParams gdcParams = cfg.getGdcParams();
};

#endif // GDC_H
