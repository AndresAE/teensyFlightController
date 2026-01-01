#include "GDC.h"
#include "Wire.h"

GDC::GDC(int sampleRate_Hz)
    : sampleRate_Hz(sampleRate_Hz)
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void GDC::begin()
{
  for (int ii = 0; ii < static_cast<int>(sizeof(controlInPins) / sizeof(controlInPins[0])); ii++)
  {
    pinMode(controlInPins[ii], INPUT);
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
  for (int ii = 0; ii < static_cast<int>(sizeof(controlInPins) / sizeof(controlInPins[0])); ii++)
  {
    controlIn_us[ii] = pulseIn(controlInPins[ii], HIGH, controlInTimeOut_ms);
  }
}

void GDC::processControlInputs()
{
  for (int ii = 0; ii < static_cast<int>(sizeof(controlInPins) / sizeof(controlInPins[0])); ii++)
  {
    // scale pwm to control input raw value
    controlInRaw[ii] = k_pwm[ii] * (static_cast<double>(controlIn_us[ii]) - pwmOffset_us[ii]);

    // apply limits
    if (controlInRaw[ii] > controlInHiLimit[ii])
    {
      controlIn[ii] = controlInHiLimit[ii];
    }
    else if (controlInRaw[ii] < controlInLoLimit[ii])
    {
      controlIn[ii] = controlInLoLimit[ii];
    }
    else
    {
      controlIn[ii] = controlInRaw[ii];
    }
  }
}

void GDC::assignChannels()
{
  m_GDC.rollRateCmd_dps = controlIn[0];
  m_GDC.nzCmd_B_fps2 = controlIn[1];
  m_GDC.nyCmd_B_fps2 = controlIn[2];
  m_GDC.throttleCmd_pct = controlIn[3];
}