#include "MSN.h"
#include "Wire.h"

MSN::MSN(int sampleRate_Hz, IMU &imu)
    : sampleRate_Hz(sampleRate_Hz), m_IMU_obj(imu), msnParams(cfg.getMsnParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
  _busAttached_b = true;
}

void MSN::begin()
{
}

void MSN::step()
{
  if (!_busAttached_b)
  {
    return;
  }
  missionManager();
}

// attachBus removed; MSN uses IMU reference passed in constructor

void MSN::missionManager()
{
  switch (missionPhase_e)
  {
  case fcsModeMissionPhase::MSN_TAKEOFF:
    // takeoff state
    evaluateTakeoff();
    break;
  case fcsModeMissionPhase::MSN_CRUISE:
    // cruise state
    evaluateCruise();
    break;
  case fcsModeMissionPhase::MSN_LAND:
    // landing state
    evaluateLanding();
    break;
  default:
    // idle unknown state
    evaluateIdle();
    break;
  }
}

void MSN::evaluateTakeoff()
{
}

void MSN::evaluateCruise()
{
}

void MSN::evaluateLanding()
{
}

void MSN::evaluateIdle()
{
  if (m_IMU_obj.m_IMU.states.specificForceX_fps2 >= msnParams.takeoffThreshold_fps2)
  {
    _nxTakeoffDebounceCount++;
    if (_nxTakeoffDebounceCount >= msnParams.takeoffDebounceLimit)
    {
      missionPhase_e = fcsModeMissionPhase::MSN_TAKEOFF;
    }
  }
}