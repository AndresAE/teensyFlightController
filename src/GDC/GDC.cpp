#include "GDC.h"
#include "Wire.h"

// --- interrupt-driven PWM capture (non-blocking) ---
static volatile unsigned long gdc_lastRiseMicros[4] = {0, 0, 0, 0};
static volatile unsigned long gdc_pulseWidthMicros[4] = {0, 0, 0, 0};
static volatile unsigned long gdc_lastUpdateMicros[4] = {0, 0, 0, 0};
static int gdc_controlPins[4] = {0, 0, 0, 0};
static int gdc_nChannels = 0;

static inline void gdc_handleEdge(int ch)
{
  unsigned long now = micros();
  int pin = gdc_controlPins[ch];
  if (digitalRead(pin) == HIGH)
  {
    gdc_lastRiseMicros[ch] = now;
  }
  else
  {
    unsigned long last = gdc_lastRiseMicros[ch];
    if (last != 0)
    {
      gdc_pulseWidthMicros[ch] = now - last;
      gdc_lastUpdateMicros[ch] = now;
      gdc_lastRiseMicros[ch] = 0;
    }
  }
}

// Dedicated ISR wrappers for each channel (attachInterrupt requires a void(*)())
extern "C" void gdc_isr_ch0() { gdc_handleEdge(0); }
extern "C" void gdc_isr_ch1() { gdc_handleEdge(1); }
extern "C" void gdc_isr_ch2() { gdc_handleEdge(2); }
extern "C" void gdc_isr_ch3() { gdc_handleEdge(3); }

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
    int pin = gdcParams.controlInPins[ii];
    pinMode(pin, INPUT);
    // store mapping for ISR handler
    if (ii < _nChannels)
    {
      gdc_controlPins[ii] = pin;
    }
    // attach ISR wrappers per channel
    switch (ii)
    {
    case 0:
      attachInterrupt(digitalPinToInterrupt(pin), gdc_isr_ch0, CHANGE);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(pin), gdc_isr_ch1, CHANGE);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(pin), gdc_isr_ch2, CHANGE);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(pin), gdc_isr_ch3, CHANGE);
      break;
    default:
      break;
    }
  }
  gdc_nChannels = _nChannels;
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
  unsigned long now = micros();
  unsigned long timeout_us = static_cast<unsigned long>(gdcParams.controlInTimeOut_us);
  for (int ii = 0; ii < _nChannels; ii++)
  {
    unsigned long pw = 0;
    if (ii < _nChannels)
    {
      unsigned long lastUpdate = gdc_lastUpdateMicros[ii];
      if (lastUpdate != 0 && (now - lastUpdate) <= timeout_us)
      {
        pw = gdc_pulseWidthMicros[ii];
      }
      else
      {
        pw = 0;
      }
    }
    gdcParams.controlIn_us[ii] = static_cast<double>(pw);
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