#ifndef TST_H
#define TST_H

#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../config/config.h"

class TST
{
public:
  TST(int sampleRate_Hz);

  int sampleRate_Hz;
  double dt_s;

  // public methods
  void begin();
  void step();
  void getControlInputs();
  void processControlInputs();
  void assignChannels();

  struct busTST
  {
    int controlIn_us[5];
    double controlInRaw[5];
    double controlIn[5];
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
    bool directMode_b = false;
  };

  busTST m_TST;

private:
  Common C;
  Config cfg;

  unsigned long _pw;
  unsigned long _now;
  unsigned long _timeout_us;
  unsigned long _lastUpdate;

  Config::tstParams tstParams = cfg.getTstParams();

  int _nChannels = 0;
};

#endif // TST_H
