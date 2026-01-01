#include "MSN.h"
#include "Wire.h"

MSN::MSN(int sampleRate_Hz)
    : sampleRate_Hz(sampleRate_Hz)
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void MSN::begin()
{
}

void MSN::step()
{
  missionManager();
}

void MSN::attachBus(IMU::busIMU *imuBus)
{
  m_IMU = imuBus;
}

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
  if (m_IMU->states.specificForceX_fps2 >= msnParams.takeoffThreshold_fps2)
  {
    _nxTakeoffDebounceCount++;
    if (_nxTakeoffDebounceCount >= msnParams.takeoffDebounceLimit)
    {
      missionPhase_e = fcsModeMissionPhase::MSN_TAKEOFF;
    }
  }
}