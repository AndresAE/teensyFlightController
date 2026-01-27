#include "GDC.h"
#include "Wire.h"

GDC::GDC(int sampleRate_Hz, TST &tst, MSN &msn)
    : sampleRate_Hz(sampleRate_Hz), m_TST_obj(tst), m_MSN_obj(msn), gdcParams(cfg.getGdcParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void GDC::begin()
{
  m_GDC.rollMode_e = gdcRollMode::GRM_ROLLRATE;
  m_GDC.pitchMode_e = gdcPitchMode::GPM_ACCEL;
  m_GDC.yawMode_e = gdcYawMode::GYM_ACCEL;
}

void GDC::step()
{
  if (m_TST_obj.m_TST.directMode_b)
  {
    getTestInputs();
  }
  else
  {
    // future: get inputs from MSN/NAV when not in direct mode
    getTestInputs();
  }
  processGuidanceCommands();
}

void GDC::getTestInputs()
{
  m_GDC.rollRateCmd_dps = m_TST_obj.m_TST.rollRateCmd_dps;
  m_GDC.nzCmd_B_fps2 = m_TST_obj.m_TST.nzCmd_B_fps2;
  m_GDC.nyCmd_B_fps2 = m_TST_obj.m_TST.nyCmd_B_fps2;
  m_GDC.throttleCmd_pct = m_TST_obj.m_TST.throttleCmd_pct;
}

void GDC::processGuidanceCommands()
{
  switch (m_GDC.rollMode_e)
  {
  case gdcRollMode::GRM_ROLL:
    applyCommandLimits(m_GDC.rollCmd_d, gdcParams.rollCmdLimits_d[0], gdcParams.rollCmdLimits_d[1]);
    break;
  case gdcRollMode::GRM_ROLLRATE:
    applyCommandLimits(m_GDC.rollRateCmd_dps, gdcParams.rollRateCmdLimits_dps[0], gdcParams.rollRateCmdLimits_dps[1]);
    break;
  case gdcRollMode::GRM_BANK:
    applyCommandLimits(m_GDC.bankCmd_d, gdcParams.bankCmdLimits_d[0], gdcParams.bankCmdLimits_d[1]);
    break;
  case gdcRollMode::GRM_UNDEFINED:
    break;
  }

  switch (m_GDC.pitchMode_e)
  {
  case gdcPitchMode::GPM_PITCH:
    applyCommandLimits(m_GDC.pitchCmd_d, gdcParams.pitchCmdLimits_d[0], gdcParams.pitchCmdLimits_d[1]);
    break;
  case gdcPitchMode::GPM_FPA:
    applyCommandLimits(m_GDC.fpaCmd_d, gdcParams.fpaCmdLimits_d[0], gdcParams.fpaCmdLimits_d[1]);
    break;
  case gdcPitchMode::GPM_ACCEL:
    applyCommandLimits(m_GDC.nzCmd_B_fps2, gdcParams.nzCmdLimits_B_fps2[0], gdcParams.nzCmdLimits_B_fps2[1]);
    break;
  case gdcPitchMode::GPM_ALTITUDE:
    applyCommandLimits(m_GDC.altitudeCmd_ft, gdcParams.altitudeCmdLimits_ft[0], gdcParams.altitudeCmdLimits_ft[1]);
    break;
  case gdcPitchMode::GPM_PITCHRATE:
    applyCommandLimits(m_GDC.pitchRateCmd_dps, gdcParams.pitchRateCmdLimits_dps[0], gdcParams.pitchRateCmdLimits_dps[1]);
    break;
  case gdcPitchMode::GPM_UNDEFINED:
    break;
  }

  switch (m_GDC.yawMode_e)
  {
  case gdcYawMode::GYM_HEADING:
    applyCommandLimits(m_GDC.headingCmd_d, gdcParams.headingCmdLimits_d[0], gdcParams.headingCmdLimits_d[1]);
    break;
  case gdcYawMode::GYM_AZIMUTH:
    applyCommandLimits(m_GDC.azimuthCmd_d, gdcParams.azimuthCmdLimits_d[0], gdcParams.azimuthCmdLimits_d[1]);
    break;
  case gdcYawMode::GYM_ACCEL:
    applyCommandLimits(m_GDC.nyCmd_B_fps2, gdcParams.nyCmdLimits_B_fps2[0], gdcParams.nyCmdLimits_B_fps2[1]);
    break;
  case gdcYawMode::GYM_YAWRATE:
    applyCommandLimits(m_GDC.yawRateCmd_dps, gdcParams.yawRateCmdLimits_dps[0], gdcParams.yawRateCmdLimits_dps[1]);
    break;
  case gdcYawMode::GYM_UNDEFINED:
    break;
  }
}

void GDC::applyCommandLimits(double value, double minLimit, double maxLimit)
{
  if (value < minLimit)
  {
    value = minLimit;
  }
  else if (value > maxLimit)
  {
    value = maxLimit;
  }
}
