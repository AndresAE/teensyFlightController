#include "APT.h"
#include "Wire.h"

APT::APT(int sampleRate_Hz, GDC &gdc, IMU &imu)
    : sampleRate_Hz(sampleRate_Hz), m_GDC_obj(gdc), m_IMU_obj(imu), aptParams(cfg.getAptParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
  _busAttached_b = true;
}

void APT::begin()
{
  setupEffectors();
  m_APT.effectorCmd[0] = aptParams.effectorOffset[0];
  m_APT.effectorCmd[1] = aptParams.effectorOffset[1];
  m_APT.effectorCmd[2] = aptParams.effectorOffset[2];
  m_APT.effectorCmd[3] = aptParams.effectorOffset[3];
  m_APT.effectorCmd[4] = aptParams.effectorOffset[4];
}

void APT::step()
{
  if (!_busAttached_b)
  {
    return;
  }
  rollMiddleLoop();
  pitchMiddleLoop();
  yawMiddleLoop();
  speedMiddleLoop();

  rollInnerLoop();
  pitchInnerLoop();
  yawInnerLoop();
  speedInnerLoop();

  controlAllocation();
  setEffectors();
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
  m_APT.pbDotCmd_dps2 = proportionalControl(aptParams.gains.rollRate.K_p, m_APT.pbCmd_dps, m_IMU_obj.m_IMU.states.angularRateX_dps);
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
  m_APT.qbDotCmd_dps2 = proportionalControl(aptParams.gains.pitchRate.K_p, m_APT.qbCmd_dps, m_IMU_obj.m_IMU.states.angularRateY_dps);
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
  m_APT.rbDotCmd_dps2 = proportionalControl(aptParams.gains.yawRate.K_p, m_APT.rbCmd_dps, m_IMU_obj.m_IMU.states.angularRateZ_dps);
}

void APT::speedMiddleLoop()
{
}

void APT::speedInnerLoop()
{
  m_APT.ubDotCmd_fps2 = m_GDC_obj.m_GDC.throttleCmd_pct;
  m_APT.vbDotCmd_fps2 = 0.0;
  m_APT.wbDotCmd_fps2 = 0.0;
}

void APT::controlAllocation()
{
  wBI_B_dps2[0] = m_APT.pbDotCmd_dps2;
  wBI_B_dps2[1] = m_APT.qbDotCmd_dps2;
  wBI_B_dps2[2] = m_APT.rbDotCmd_dps2;

  aBI_B_fps2[0] = m_APT.ubDotCmd_fps2;
  aBI_B_fps2[1] = m_APT.vbDotCmd_fps2;
  aBI_B_fps2[2] = m_APT.wbDotCmd_fps2;

  // reset effector commands
  m_APT.effectorCmd[0] = 0.0;
  m_APT.effectorCmd[1] = 0.0;
  m_APT.effectorCmd[2] = 0.0;
  m_APT.effectorCmd[3] = 0.0;
  m_APT.effectorCmd[4] = 0.0;

  for (int ii = 0; ii < _nEffectors; ii++)
  {
    for (int jj = 0; jj < _nRotationalAxes; jj++)
    {
      m_APT.effectorCmd[ii] += aptParams.mixingRatioRotational[ii][jj] * wBI_B_dps2[jj];
    }
  }

  for (int ii = 0; ii < _nEffectors; ii++)
  {
    for (int jj = 0; jj < _nLinearAxes; jj++)
    {
      m_APT.effectorCmd[ii] += aptParams.mixingRatioLinear[ii][jj] * aBI_B_fps2[jj];
    }
  }

  for (int ii = 0; ii < _nEffectors; ii++)
  {
    m_APT.effectorCmd[ii] += aptParams.effectorOffset[ii];
    // apply limits and offsets
    if (m_APT.effectorCmd[ii] > aptParams.effectorLimitHi[ii])
    {
      m_APT.effectorCmd[ii] = aptParams.effectorLimitHi[ii] + aptParams.effectorOffset[ii];
    }
    else if (m_APT.effectorCmd[ii] < aptParams.effectorLimitLo[ii])
    {
      m_APT.effectorCmd[ii] = aptParams.effectorLimitLo[ii] + aptParams.effectorOffset[ii];
    }
    else
    {
    }
  }
}

void APT::setupEffectors()
{
  _nEffectors = static_cast<int>(sizeof(m_APT.effectorCmd) / sizeof(m_APT.effectorCmd[0]));
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    analogWriteFrequency(aptParams.effectorPins[ii], sampleRate_Hz);
  }
}

void APT::setEffectors()
{
  for (int ii = 0; ii < _nEffectors; ii++)
  {
    double degRange = (aptParams.degMax[ii] - aptParams.degMin[ii]);
    double pwmRange = static_cast<double>(aptParams.pwmMax[ii] - aptParams.pwmMin[ii]);
    double normalized = (degRange == 0.0) ? 0.0 : (m_APT.effectorCmd[ii] - aptParams.degMin[ii]) / degRange;
    pwm_val = aptParams.pwmMin[ii] + static_cast<int>(normalized * pwmRange);
    analogWrite(aptParams.effectorPins[ii], pwm_val);
  }
}

double APT::proportionalControl(double k_p, double cvc, double cv)
{
  return k_p * (cvc - cv);
}

double APT::proportionalIntegralControl(double k_p, double k_i, double cvc, double cv, bool resetIntegral_b)
{
  if (resetIntegral_b)
  {
    integralError = 0.0;
  }
  integralError += (cvc - cv) * dt_s;
  return k_p * (cvc - cv) + k_i * integralError;
}

double APT::proportionalIntegralFeedforwardControl(double k_p, double k_i, double k_f, double cvc, double cv, bool resetIntegral_b)
{
  if (resetIntegral_b)
  {
    integralError = 0.0;
  }
  integralError += (cvc - cv) * dt_s;
  return k_p * (cvc - cv) + k_i * integralError + k_f * cvc;
}
