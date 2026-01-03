#include "GDC.h"
#include "Wire.h"

GDC::GDC(int sampleRate_Hz)
    : sampleRate_Hz(sampleRate_Hz), gdcParams(cfg.getGdcParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void GDC::begin()
{
  _nChannels = static_cast<int>(sizeof(gdcParams.controlInPins) / sizeof(gdcParams.controlInPins[0]));
  for (int ii = 0; ii < _nChannels; ii++)
  {
    pinMode(gdcParams.controlInPins[ii], INPUT);
  }
  m_GDC.rollMode_e = gdcRollMode::GRM_ROLLRATE;
  m_GDC.pitchMode_e = gdcPitchMode::GPM_ACCEL;
  m_GDC.yawMode_e = gdcYawMode::GYM_ACCEL;
}

void GDC::step()
{
  getControlInputs();
  processControlInputs();
  assignChannels();
}

void GDC::getControlInputs()
{
  for (int ii = 0; ii < _nChannels; ii++)
  {
    gdcParams.controlIn_us[ii] = pulseIn(gdcParams.controlInPins[ii], HIGH, gdcParams.controlInTimeOut_ms);
  }
}

void GDC::processControlInputs()
{
  for (int ii = 0; ii < _nChannels; ii++)
  {
    // scale pwm to control input raw value
    gdcParams.controlInRaw[ii] = gdcParams.k_pwm[ii] * (static_cast<double>(gdcParams.controlIn_us[ii]) - gdcParams.pwmOffset_us[ii]);

    // apply limits
    if (gdcParams.controlInRaw[ii] > gdcParams.controlInHiLimit[ii])
    {
      gdcParams.controlIn[ii] = gdcParams.controlInHiLimit[ii];
    }
    else if (gdcParams.controlInRaw[ii] < gdcParams.controlInLoLimit[ii])
    {
      gdcParams.controlIn[ii] = gdcParams.controlInLoLimit[ii];
    }
    else
    {
      gdcParams.controlIn[ii] = gdcParams.controlInRaw[ii];
    }
  }
}

void GDC::assignChannels()
{
  m_GDC.rollRateCmd_dps = gdcParams.controlIn[0];
  m_GDC.nzCmd_B_fps2 = gdcParams.controlIn[1];
  m_GDC.nyCmd_B_fps2 = gdcParams.controlIn[2];
  m_GDC.throttleCmd_pct = gdcParams.controlIn[3];
}