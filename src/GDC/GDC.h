#ifndef GDC_H
#define GDC_H

#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"

class GDC
{
public:
  GDC(int sampleRate_Hz = 50);

  int sampleRate_Hz;
  double dt_s;

  // public methods
  void begin();
  void step();
  void getControlInputs();
  void processControlInputs();

  int controlInPins[4] = {0, 1, 2, 3}; // roll, pitch, yaw, throttle
  double controlIn_us[4] = {0.0, 0.0, 0.0, 0.0};
  int controlInTimeOut_ms = 25000; // 25 ms timeout
  double pwmOffset_us[4] = {1500.0, 1833.0, 1500.0, 1000};
  double k_pwm[4] = {-0.12, 0.1, -0.016, 0.002}; // scale factor from pwm to control input
  double controlInHiLimit[4] = {60, 16.1, 8.0, 1.0};
  double controlInLoLimit[4] = {-60.0, -80.5, -8.0, 0.0};
  double controlInRaw[4] = {0.0, 0.0, 0.0, 0.0};
  double controlIn[4] = {0.0, 0.0, 0.0, 0.0};
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
};

#endif // GDC_H
