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
    mapServos();
    setEffectors();
  }
  else
  {
    if (m_MSN_obj.missionPhase_e == MSN::fcsMissionPhase::MSN_IDLE)
    {
      _resetRollIntegral_b = true;
      _resetPitchIntegral_b = true;
      _resetYawIntegral_b = true;
    }
    else
    {
      _resetRollIntegral_b = false;
      _resetPitchIntegral_b = false;
      _resetYawIntegral_b = false;
    }
    evalRoll();
    evalPitch();
    evalYaw();
    evalSpeed();

    controlAllocation();
    applyLimits();
    mapServos();
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
    m_APT.effectorCmd[ii] = aptParams.directModeGain[ii] * ((static_cast<double>(m_TST_obj.m_TST.controlIn_us[ii]) - aptParams.muMin_us[ii]) / (aptParams.muMax_us[ii] - aptParams.muMin_us[ii]) * (aptParams.effectorLimitHi[ii] - aptParams.effectorLimitLo[ii]) + aptParams.effectorLimitLo[ii]);
  }

  _prevRollState_e = GDC::gdcRollMode::GRM_UNDEFINED;
  _prevPitchState_e = GDC::gdcPitchMode::GPM_UNDEFINED;
  _prevYawState_e = GDC::gdcYawMode::GYM_UNDEFINED;
}

void APT::rollMiddleLoop()
{
  if ((m_GDC_obj.m_GDC.rollMode_e != _prevRollState_e) || _resetRollIntegral_b)
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
  _rollError = m_APT.pbCmd_dps - m_IMU_obj.m_IMU.states.angularRateX_dps;
  if (aptParams.integratorOff_b)
  {
    _rollIntegralError = m_APT.pbCmd_dps;
  }
  else if (_resetRollIntegral_b)
  {
    _rollIntegralError = 0.0;
  }
  else if (m_APT.effectorOnLimit_b[0] && (C.sign(_rollEffectorCmdPrev) == C.sign(_rollError)))
  {
    _rollIntegralError = (_rollEffectorCmdPrev / aptParams.gains.rollRate.K_d + m_IMU_obj.m_IMU.states.angularRateX_dps) / aptParams.gains.rollRate.K_i;
  }
  else
  {
    _rollIntegralError = C.tustinIntegral(_rollError, _rollErrorPrev, _rollIntegralError, dt_s);
  }
  _rollErrorPrev = _rollError;
  _xiRoll = _rollIntegralError * aptParams.gains.rollRate.K_i;
  m_APT.rollIntegralError = _rollIntegralError;

  m_APT.rollEffectorCmd = proportionalControl(aptParams.gains.rollRate.K_d, _xiRoll, m_IMU_obj.m_IMU.states.angularRateX_dps);
  _rollEffectorCmdPrev = m_APT.rollEffectorCmd;
}

void APT::pitchMiddleLoop()
{
  if ((m_GDC_obj.m_GDC.pitchMode_e != _prevPitchState_e) || _resetPitchIntegral_b)
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
  _pitchError = m_APT.qbCmd_dps - m_IMU_obj.m_IMU.states.angularRateY_dps;
  if (aptParams.integratorOff_b)
  {
    _pitchIntegralError = m_APT.qbCmd_dps;
  }
  else if (_resetPitchIntegral_b)
  {
    _pitchIntegralError = 0.0;
  }
  else if (m_APT.effectorOnLimit_b[1] && (C.sign(_pitchEffectorCmdPrev) == C.sign(_pitchError)))
  {
    _pitchIntegralError = (_pitchEffectorCmdPrev / aptParams.gains.pitchRate.K_d + m_IMU_obj.m_IMU.states.angularRateY_dps) / aptParams.gains.pitchRate.K_i;
  }
  else
  {
    _pitchIntegralError = C.tustinIntegral(_pitchError, _pitchErrorPrev, _pitchIntegralError, dt_s);
  }
  _pitchErrorPrev = _pitchError;
  _xiPitch = _pitchIntegralError * aptParams.gains.pitchRate.K_i;
  m_APT.pitchIntegralError = _pitchIntegralError;

  m_APT.pitchEffectorCmd = proportionalControl(aptParams.gains.pitchRate.K_d, _xiPitch, m_IMU_obj.m_IMU.states.angularRateY_dps);
  _pitchEffectorCmdPrev = m_APT.pitchEffectorCmd;
}

void APT::yawMiddleLoop()
{
  if ((m_GDC_obj.m_GDC.yawMode_e != _prevYawState_e) || _resetYawIntegral_b)
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
  _yawError = m_APT.rbCmd_dps - m_IMU_obj.m_IMU.states.angularRateZ_dps;
  if (aptParams.integratorOff_b)
  {
    _yawIntegralError = _yawError;
  }
  else if (_resetYawIntegral_b)
  {
    _yawIntegralError = 0.0;
  }
  else if (m_APT.effectorOnLimit_b[2] && (C.sign(_yawEffectorCmdPrev) == C.sign(_yawError)))
  {
    _yawIntegralError = (_yawEffectorCmdPrev / aptParams.gains.yawRate.K_d + m_IMU_obj.m_IMU.states.angularRateZ_dps) / aptParams.gains.yawRate.K_i;
  }
  else
  {
    _yawIntegralError = C.tustinIntegral(_yawError, _yawErrorPrev, _yawIntegralError, dt_s);
  }
  _yawErrorPrev = _yawError;
  _xiYaw = _yawIntegralError * aptParams.gains.yawRate.K_i;
  m_APT.yawIntegralError = _yawIntegralError;

  m_APT.yawEffectorCmd = proportionalControl(aptParams.gains.yawRate.K_d, _xiYaw, m_IMU_obj.m_IMU.states.angularRateZ_dps);
  _yawEffectorCmdPrev = m_APT.yawEffectorCmd;
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

  translationalPseudoEffector[0] = m_APT.axialEffectorCmd;
  translationalPseudoEffector[1] = m_APT.lateralEffectorCmd;
  translationalPseudoEffector[2] = m_APT.normalEffectorCmd;

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
    for (int jj = 0; jj < _nTranslationalAxes; jj++)
    {
      m_APT.effectorCmd[ii] += aptParams.mixingRatioTranslational[ii][jj] * translationalPseudoEffector[jj];
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
      m_APT.effectorOnLimit_b[ii] = true;
    }
    else if (m_APT.effectorCmd[ii] < aptParams.effectorLimitLo[ii])
    {
      m_APT.effectorCmd[ii] = aptParams.effectorLimitLo[ii];
      m_APT.effectorOnLimit_b[ii] = true;
    }
    else
    {
      m_APT.effectorOnLimit_b[ii] = false;
    }
    m_APT.effectorCmd[ii] += aptParams.effectorOffset[ii];
  }
}

void APT::setupEffectors()
{
  _nEffectors = static_cast<int>(sizeof(m_APT.effectorCmd) / sizeof(m_APT.effectorCmd[0]));
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    m_APT.effectorCmd[ii] = aptParams.effectorOffset[ii];
  }

  mapServos();

  for (int ii = 0; ii < _nEffectors; ii++)
  {
    // attach servo to pin and initialize to offset position
    _servos[ii].attach(aptParams.effectorPins[ii]);
    _init_us = static_cast<int>((_servoCmd[ii] / aptParams.servoRange_d[ii]) *
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
    _pulse_us = static_cast<int>((_servoCmd[ii] / aptParams.servoRange_d[ii]) *
                                     (aptParams.muMax_us[ii] - aptParams.muMin_us[ii]) +
                                 aptParams.muMin_us[ii]);
    _servos[ii].writeMicroseconds(_pulse_us);
  }
}

double APT::proportionalControl(double k_p, double cvc, double cv)
{
  return k_p * (cvc - cv);
}

void APT::mapServos()
{
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    _servoCmd[ii] = C.interpolate(aptParams.servoMappingBreakpoints, aptParams.servoMappingTable, m_APT.effectorCmd[ii], ii);
  }
}
