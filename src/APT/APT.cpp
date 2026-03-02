#include "APT.h"
#include "Wire.h"

APT::APT(int sampleRate_Hz, TST &tst, MSN &msn, GDC &gdc, IMU &imu)
    : sampleRate_Hz(sampleRate_Hz), m_TST_obj(tst), m_MSN_obj(msn), m_GDC_obj(gdc), m_IMU_obj(imu), aptParams(cfg.getAptParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void APT::begin()
{
  setupEffectors();
  m_APT.effectorCmd[0] = aptParams.effectorOffset[0];
  m_APT.effectorCmd[1] = aptParams.effectorOffset[1];
  m_APT.effectorCmd[2] = aptParams.effectorOffset[2];
  m_APT.effectorCmd[3] = aptParams.effectorOffset[3];
}

void APT::step()
{
  if (m_MSN_obj.missionPhase_e == MSN::fcsMissionPhase::MSN_DIRECT)
  {
    evalDirect();
    applyLimits();
    setEffectors();
  }
  else
  {
    evalRoll();
    evalPitch();
    evalYaw();
    evalSpeed();

    controlAllocation();
    applyLimits();
    setEffectors();
  }
}

void APT::evalRoll()
{
  rollMiddleLoop();
  rollInnerLoop();
}

void APT::evalPitch()
{
  pitchMiddleLoop();
  pitchInnerLoop();
}

void APT::evalYaw()
{
  yawMiddleLoop();
  yawInnerLoop();
}

void APT::evalSpeed()
{
  speedMiddleLoop();
  speedInnerLoop();
}

void APT::evalDirect()
{
  // In direct mode, APT does not modify commands; they are passed through from TST to effectors
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    _meanPWM = (aptParams.muMin_us[ii] + aptParams.muMax_us[ii]) / 2;
    m_APT.effectorCmd[ii] = (((aptParams.directModeGain[ii] * (static_cast<double>(m_TST_obj.m_TST.controlIn_us[ii]) - _meanPWM) + _meanPWM) - aptParams.muMin_us[ii]) / (aptParams.muMax_us[ii] - aptParams.muMin_us[ii])) * aptParams.servoRange_d[ii] - aptParams.effectorOffset[ii];
  }

  _prevRollState_e = GDC::gdcRollMode::GRM_UNDEFINED;
  _prevPitchState_e = GDC::gdcPitchMode::GPM_UNDEFINED;
  _prevYawState_e = GDC::gdcYawMode::GYM_UNDEFINED;
}

void APT::rollMiddleLoop()
{
  if (m_GDC_obj.m_GDC.rollMode_e != _prevRollState_e)
  {
    // reset integrators or states if needed
    _resetRollIntegral_b = true;
  }
  else
  {
    _resetRollIntegral_b = false;
  }
  switch (m_GDC_obj.m_GDC.rollMode_e)
  {
  case GDC::gdcRollMode::GRM_ROLL:
    m_APT.pbCmd_dps = proportionalControl(aptParams.gains.roll.K_p, m_GDC_obj.m_GDC.rollCmd_d, m_IMU_obj.m_IMU.euler.roll_d);
    break;
  case GDC::gdcRollMode::GRM_BANK:
    // implement bank angle to roll angle conversion if needed
    m_APT.pbCmd_dps = 0.0;
    break;
  case GDC::gdcRollMode::GRM_ROLLRATE:
    m_APT.pbCmd_dps = m_GDC_obj.m_GDC.rollRateCmd_dps;
    break;
  default:
    m_APT.pbCmd_dps = 0.0;
  }
  _prevRollState_e = m_GDC_obj.m_GDC.rollMode_e;
}

void APT::rollInnerLoop()
{
  if (_resetRollIntegral_b)
  {
    _rollIntegralError = 0.0;
  }
  if (aptParams.integratorOff_b)
  {
    _xiRoll = m_APT.pbCmd_dps;
  }
  else
  {
    _rollIntegralError = C.trapIntegral(m_APT.pbCmd_dps, _pbCmdPrev_dps, _rollIntegralError, dt_s);
    _pbCmdPrev_dps = m_APT.pbCmd_dps;

    _xiRoll = _rollIntegralError * aptParams.gains.rollRate.K_i;
  }
  m_APT.rollEffectorCmd = proportionalControl(aptParams.gains.rollRate.K_d, _xiRoll, m_IMU_obj.m_IMU.states.angularRateX_dps);
}

void APT::pitchMiddleLoop()
{
  if (m_GDC_obj.m_GDC.pitchMode_e != _prevPitchState_e)
  {
    // reset integrators or states if needed
    _resetPitchIntegral_b = true;
  }
  else
  {
    _resetPitchIntegral_b = false;
  }

  switch (m_GDC_obj.m_GDC.pitchMode_e)
  {
  case GDC::gdcPitchMode::GPM_PITCH:
    m_APT.qbCmd_dps = proportionalControl(aptParams.gains.pitch.K_p, m_GDC_obj.m_GDC.pitchCmd_d, m_IMU_obj.m_IMU.euler.pitch_d);
    break;
  case GDC::gdcPitchMode::GPM_FPA:
    // implement flight path angle to pitch angle conversion if needed
    m_APT.qbCmd_dps = 0.0;
    break;
  case GDC::gdcPitchMode::GPM_ACCEL:
    m_APT.qbCmd_dps = proportionalControl(aptParams.gains.nz.K_p, m_GDC_obj.m_GDC.nzCmd_B_fps2, m_IMU_obj.m_IMU.states.specificForceZ_fps2);
    break;
  case GDC::gdcPitchMode::GPM_ALTITUDE:
    // implement altitude to flight path angle conversion if needed
    m_APT.qbCmd_dps = 0.0;
    break;
  case GDC::gdcPitchMode::GPM_PITCHRATE:
    m_APT.qbCmd_dps = m_GDC_obj.m_GDC.pitchRateCmd_dps;
    break;
  default:
    m_APT.qbCmd_dps = 0.0;
  }
  _prevPitchState_e = m_GDC_obj.m_GDC.pitchMode_e;
}

void APT::pitchInnerLoop()
{
  if (_resetPitchIntegral_b)
  {
    _pitchIntegralError = 0.0;
  }
  if (aptParams.integratorOff_b)
  {
    _xiPitch = m_APT.qbCmd_dps;
  }
  else
  {
    _pitchIntegralError = C.trapIntegral(m_APT.qbCmd_dps, _qbCmdPrev_dps, _pitchIntegralError, dt_s);
    _qbCmdPrev_dps = m_APT.qbCmd_dps;

    _xiPitch = _pitchIntegralError * aptParams.gains.pitchRate.K_i;
  }
  m_APT.pitchEffectorCmd = proportionalControl(aptParams.gains.pitchRate.K_d, _xiPitch, m_IMU_obj.m_IMU.states.angularRateY_dps);
}

void APT::yawMiddleLoop()
{
  if (m_GDC_obj.m_GDC.yawMode_e != _prevYawState_e)
  {
    // reset integrators or states if needed
    _resetYawIntegral_b = true;
  }
  else
  {
    _resetYawIntegral_b = false;
  }

  switch (m_GDC_obj.m_GDC.yawMode_e)
  {
  case GDC::gdcYawMode::GYM_HEADING:
    m_APT.rbCmd_dps = proportionalControl(aptParams.gains.heading.K_p, m_GDC_obj.m_GDC.headingCmd_d, m_IMU_obj.m_IMU.euler.yaw_d);
    break;
  case GDC::gdcYawMode::GYM_AZIMUTH:
    // implement azimuth to heading conversion if needed
    m_APT.rbCmd_dps = 0.0;
    break;
  case GDC::gdcYawMode::GYM_ACCEL:
    m_APT.rbCmd_dps = proportionalControl(aptParams.gains.ny.K_p, m_GDC_obj.m_GDC.nyCmd_B_fps2, m_IMU_obj.m_IMU.states.specificForceY_fps2);
    break;
  case GDC::gdcYawMode::GYM_YAWRATE:
    m_APT.rbCmd_dps = m_GDC_obj.m_GDC.yawRateCmd_dps;
    break;
  default:
    m_APT.rbCmd_dps = 0.0;
  }
  _prevYawState_e = m_GDC_obj.m_GDC.yawMode_e;
}

void APT::yawInnerLoop()
{
  if (_resetYawIntegral_b)
  {
    _yawIntegralError = 0.0;
  }
  if (aptParams.integratorOff_b)
  {
    _xiYaw = m_APT.rbCmd_dps;
  }
  else
  {
    _yawIntegralError = C.trapIntegral(m_APT.rbCmd_dps, _rbCmdPrev_dps, _yawIntegralError, dt_s);
    _rbCmdPrev_dps = m_APT.rbCmd_dps;

    _xiYaw = _yawIntegralError * aptParams.gains.yawRate.K_i;
  }
  m_APT.yawEffectorCmd = proportionalControl(aptParams.gains.yawRate.K_d, _xiYaw, m_IMU_obj.m_IMU.states.angularRateZ_dps);
}

void APT::speedMiddleLoop()
{
}

void APT::speedInnerLoop()
{
  m_APT.axialEffectorCmd = m_GDC_obj.m_GDC.throttleCmd_pct;
  m_APT.lateralEffectorCmd = 0.0;
  m_APT.normalEffectorCmd = 0.0;
}

void APT::controlAllocation()
{
  rotationalPseudoEffector[0] = m_APT.rollEffectorCmd;
  rotationalPseudoEffector[1] = m_APT.pitchEffectorCmd;
  rotationalPseudoEffector[2] = m_APT.yawEffectorCmd;

  linearPseudoEffector[0] = m_APT.axialEffectorCmd;
  linearPseudoEffector[1] = m_APT.lateralEffectorCmd;
  linearPseudoEffector[2] = m_APT.normalEffectorCmd;

  // reset effector commands
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    m_APT.effectorCmd[ii] = 0.0;
  }

  for (int ii = 0; ii < _nEffectors; ii++)
  {
    for (int jj = 0; jj < _nRotationalAxes; jj++)
    {
      m_APT.effectorCmd[ii] += aptParams.mixingRatioRotational[ii][jj] * rotationalPseudoEffector[jj];
    }
  }

  for (int ii = 0; ii < _nEffectors; ii++)
  {
    for (int jj = 0; jj < _nLinearAxes; jj++)
    {
      m_APT.effectorCmd[ii] += aptParams.mixingRatioLinear[ii][jj] * linearPseudoEffector[jj];
    }
  }
}

void APT::applyLimits()
{
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    // apply limits and offsets
    if (m_APT.effectorCmd[ii] > aptParams.effectorLimitHi[ii])
    {
      m_APT.effectorCmd[ii] = aptParams.effectorLimitHi[ii];
      _effectorOnLimit_b[ii] = true;
    }
    else if (m_APT.effectorCmd[ii] < aptParams.effectorLimitLo[ii])
    {
      m_APT.effectorCmd[ii] = aptParams.effectorLimitLo[ii];
      _effectorOnLimit_b[ii] = true;
    }
    else
    {
      _effectorOnLimit_b[ii] = false;
    }
    m_APT.effectorCmd[ii] += aptParams.effectorOffset[ii];
  }
}

void APT::setupEffectors()
{
  _nEffectors = static_cast<int>(sizeof(m_APT.effectorCmd) / sizeof(m_APT.effectorCmd[0]));
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    // attach servo to pin and initialize to offset position
    _servos[ii].attach(aptParams.effectorPins[ii]);
    _init_us = static_cast<int>((aptParams.effectorOffset[ii] / aptParams.servoRange_d[ii]) *
                                    (aptParams.muMax_us[ii] - aptParams.muMin_us[ii]) +
                                aptParams.muMin_us[ii]);
    _servos[ii].writeMicroseconds(_init_us);
  }
}

void APT::setEffectors()
{
  // Compute pulse widths (microseconds) and write to servos (non-blocking)
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    _pulse_us = static_cast<int>((m_APT.effectorCmd[ii] / aptParams.servoRange_d[ii]) *
                                     (aptParams.muMax_us[ii] - aptParams.muMin_us[ii]) +
                                 aptParams.muMin_us[ii]);
    _servos[ii].writeMicroseconds(_pulse_us);
  }
}

double APT::proportionalControl(double k_p, double cvc, double cv)
{
  return k_p * (cvc - cv);
}
