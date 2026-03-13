#ifndef APT_H
#define APT_H

#include <cmath>
#include <Arduino.h>
#include <Servo.h>
#include "../CMN/CMN.h"
#include "../TST/TST.h"
#include "../MSN/MSN.h"
#include "../GDC/GDC.h"
#include "../IMU/IMU.h"
#include "../CFG/CFG.h"

class APT
{
public:
  APT(int sampleRate_Hz, TST &tst, MSN &msn, GDC &gdc, IMU &imu);

  int sampleRate_Hz;
  double dt_s;
  double integralError = 0.0;
  double rotationalPseudoEffector[3];
  double translationalPseudoEffector[3];

  // public methods
  void begin();
  void step();
  void evalRoll();
  void evalPitch();
  void evalYaw();
  void evalSpeed();
  void evalDirect();
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
  void applyLimits();
  void setEffectors();
  double proportionalControl(double k_p, double cvc, double cv);
  void mapServos();

  struct busAPT
  {
    double pbCmd_dps;
    double qbCmd_dps;
    double rbCmd_dps;
    double rollEffectorCmd;
    double pitchEffectorCmd;
    double yawEffectorCmd;
    double ubCmd_fps;
    double vbCmd_fps;
    double wbCmd_fps;
    double axialEffectorCmd;
    double lateralEffectorCmd;
    double normalEffectorCmd;
    double rollIntegralError;
    double pitchIntegralError;
    double yawIntegralError;
    bool effectorOnLimit_b[CFG::aptParams::nEffector] = {false};
    double effectorCmd[CFG::aptParams::nEffector];
  };

  busAPT m_APT;

  TST &m_TST_obj;
  MSN &m_MSN_obj;
  GDC &m_GDC_obj;
  IMU &m_IMU_obj;

private:
  CMN C;
  CFG cfg;

  const CFG::aptParams &aptParams;

  int _prevRollState_e;
  int _prevPitchState_e;
  int _prevYawState_e;
  bool _resetRollIntegral_b = false;
  bool _resetPitchIntegral_b = false;
  bool _resetYawIntegral_b = false;
  double _rollIntegralError = 0.0;
  double _pitchIntegralError = 0.0;
  double _yawIntegralError = 0.0;
  double _rollError = 0.0;
  double _pitchError = 0.0;
  double _yawError = 0.0;
  double _rollErrorPrev = 0.0;
  double _pitchErrorPrev = 0.0;
  double _yawErrorPrev = 0.0;
  double _xiRoll;
  double _xiPitch;
  double _xiYaw;
  double _rollEffectorCmdPrev = 0;
  double _pitchEffectorCmdPrev = 0;
  double _yawEffectorCmdPrev = 0;
  double _rollEffectorCmdAlt = 0;
  double _pitchEffectorCmdAlt = 0;
  double _yawEffectorCmdAlt = 0;
  double _servoCmd[CFG::aptParams::nEffector];
  int _nEffectors;
  int _nRotationalAxes = 3;
  int _nTranslationalAxes = 3;
  int _frame_us = 20000;
  int _loopTime_us;
  int _pulse_us;
  int _init_us;
  double _meanPWM;
  Servo _servos[CFG::aptParams::nEffector];
};

#endif // APT_H
