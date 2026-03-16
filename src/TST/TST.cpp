#include "TST.h"
#include "Wire.h"

// --- interrupt-driven PWM capture (non-blocking) ---
static volatile unsigned long tst_lastRiseMicros[5] = {0, 0, 0, 0, 0};
static volatile unsigned long tst_pulseWidthMicros[5] = {0, 0, 0, 0, 0};
static volatile unsigned long tst_lastUpdateMicros[5] = {0, 0, 0, 0, 0};
static int tst_controlPins[5] = {0, 0, 0, 0, 0};
static int tst_nChannels = 0;
static inline void tst_handleEdge(int ch)
{
  unsigned long now = micros();
  int pin = tst_controlPins[ch];
  if (digitalRead(pin) == HIGH)
  {
    tst_lastRiseMicros[ch] = now;
  }
  else
  {
    unsigned long last = tst_lastRiseMicros[ch];
    if (last != 0)
    {
      tst_pulseWidthMicros[ch] = now - last;
      tst_lastUpdateMicros[ch] = now;
      tst_lastRiseMicros[ch] = 0;
    }
  }
}

// Dedicated ISR wrappers for each channel (attachInterrupt requires a void(*)())
extern "C" void tst_isr_ch0() { tst_handleEdge(0); }
extern "C" void tst_isr_ch1() { tst_handleEdge(1); }
extern "C" void tst_isr_ch2() { tst_handleEdge(2); }
extern "C" void tst_isr_ch3() { tst_handleEdge(3); }
extern "C" void tst_isr_ch4() { tst_handleEdge(4); }

TST::TST(int sampleRate_Hz)
    : sampleRate_Hz(sampleRate_Hz), tstParams(cfg.getTstParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void TST::begin()
{
  _nChannels = static_cast<int>(sizeof(tstParams.controlInPins) / sizeof(tstParams.controlInPins[0]));
  for (int ii = 0; ii < _nChannels; ii++)
  {
    int pin = tstParams.controlInPins[ii];
    pinMode(pin, INPUT);
    // store mapping for ISR handler
    if (ii < _nChannels)
    {
      tst_controlPins[ii] = pin;
    }
    // attach ISR wrappers per channel
    switch (ii)
    {
    case 0:
      attachInterrupt(digitalPinToInterrupt(pin), tst_isr_ch0, CHANGE);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(pin), tst_isr_ch1, CHANGE);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(pin), tst_isr_ch2, CHANGE);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(pin), tst_isr_ch3, CHANGE);
      break;
    case 4:
      attachInterrupt(digitalPinToInterrupt(pin), tst_isr_ch4, CHANGE);
    default:
      break;
    }
  }
  tst_nChannels = _nChannels;
}

void TST::step()
{
  getControlInputs();
  processControlInputs();
  assignChannels();
}

void TST::getControlInputs()
{
  _now = micros();
  _timeout_us = static_cast<unsigned long>(tstParams.controlInTimeOut_us);
  for (int ii = 0; ii < _nChannels; ii++)
  {
    _pw = 0;
    if (ii < _nChannels)
    {
      _lastUpdate = tst_lastUpdateMicros[ii];
      if (_lastUpdate != 0 && (_now - _lastUpdate) <= _timeout_us)
      {
        _pw = tst_pulseWidthMicros[ii];
      }
      else
      {
        _pw = 0;
      }
    }
    m_TST.controlIn_us[ii] = _pw;
  }
}

void TST::processControlInputs()
{
  for (int ii = 0; ii < _nChannels; ii++)
  {
    // scale pwm to control input raw value
    m_TST.controlInRaw[ii] = (2 * (static_cast<double>(m_TST.controlIn_us[ii]) - tstParams.muMin_us[ii]) / (tstParams.muMax_us[ii] - tstParams.muMin_us[ii]) - 1) * tstParams.kCmd[ii] + tstParams.cmdOffset[ii];

    // apply limits
    if (m_TST.controlInRaw[ii] > tstParams.controlInHiLimit[ii])
    {
      m_TST.controlIn[ii] = tstParams.controlInHiLimit[ii];
    }
    else if (m_TST.controlInRaw[ii] < tstParams.controlInLoLimit[ii])
    {
      m_TST.controlIn[ii] = tstParams.controlInLoLimit[ii];
    }
    else
    {
      m_TST.controlIn[ii] = m_TST.controlInRaw[ii];
    }
  }
}

void TST::assignChannels()
{
  m_TST.rollRateCmd_dps = m_TST.controlIn[0];
  m_TST.nzCmd_B_fps2 = m_TST.controlIn[1];
  m_TST.nyCmd_B_fps2 = m_TST.controlIn[2];
  m_TST.throttleCmd_pct = m_TST.controlIn[3];

  if (m_TST.controlIn_us[4] > tstParams.directModeThreshold_us)
  {
    m_TST.directMode_b = true;
  }
  else
  {
    m_TST.directMode_b = false;
  }
}
