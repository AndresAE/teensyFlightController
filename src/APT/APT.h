#ifndef APT_H
#define APT_H

#include <cmath>
#include <Arduino.h>
#include <Servo.h>
#include "../Common/Common.h"
#include "../GDC/GDC.h"
#include "../IMU/IMU.h"
#include "../config/config.h"

class APT
{
public:
  APT(int sampleRate_Hz, GDC &gdc, IMU &imu);

  int sampleRate_Hz;
  double dt_s;
  double integralError = 0.0;
  double wBI_B_dps2[3];
  double aBI_B_fps2[3];

  // public methods
  void begin();
  void step();
  void rollMiddleLoop();
  void rollInnerLoop();
  void pitchMiddleLoop();
  void pitchInnerLoop();
  void yawMiddleLoop();
  void yawInnerLoop();
  void speedMiddleLoop();
  void speedInnerLoop();
  void controlAllocation();
  void setupEffectors();
  void setEffectors();
  double proportionalControl(double k_p, double cvc, double cv);
  double proportionalIntegralControl(double k_p, double k_i, double cvc, double cv, bool resetIntegral_b);
  double proportionalIntegralFeedforwardControl(double k_p, double k_i, double k_f, double cvc, double cv, bool resetIntegral_b);

  struct busAPT
  {
    double pbCmd_dps;
    double qbCmd_dps;
    double rbCmd_dps;
    double pbDotCmd_dps2;
    double qbDotCmd_dps2;
    double rbDotCmd_dps2;
    double ubCmd_fps;
    double vbCmd_fps;
    double wbCmd_fps;
    double ubDotCmd_fps2;
    double vbDotCmd_fps2;
    double wbDotCmd_fps2;
    double effectorCmd[5];
  };

  busAPT m_APT;
  GDC &m_GDC_obj;
  IMU &m_IMU_obj;

private:
  Common C;
  Config cfg;

  const Config::aptParams &aptParams;

  bool _busAttached_b = false;
  int _nxTakeoffDebounceCount = 0;
  int _prevRollState_e;
  int _prevPitchState_e;
  int _prevYawState_e;
  bool _resetRollIntegral_b = false;
  bool _resetPitchIntegral_b = false;
  bool _resetYawIntegral_b = false;
  int _nEffectors;
  int _nRotationalAxes = 3;
  int _nLinearAxes = 3;
  int _frame_us = 20000;
  int _loopTime_us;
  Servo _servos[5];
};

#endif // APT_H
