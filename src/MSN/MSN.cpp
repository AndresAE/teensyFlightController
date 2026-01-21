#include "MSN.h"
#include "Wire.h"

MSN::MSN(int sampleRate_Hz, TST &tst, IMU &imu)
    : sampleRate_Hz(sampleRate_Hz), m_TST_obj(tst), m_IMU_obj(imu), msnParams(cfg.getMsnParams())
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

// attachBus removed; MSN uses IMU reference passed in constructor

void MSN::missionManager()
{
  if (m_TST_obj.m_TST.directMode_b)
  {
    missionPhase_e = fcsMissionPhase::MSN_DIRECT;
    return;
  }

  switch (missionPhase_e)
  {
  case fcsMissionPhase::MSN_TAKEOFF:
    // takeoff state
    evaluateTakeoff();
    break;
  case fcsMissionPhase::MSN_CRUISE:
    // cruise state
    evaluateCruise();
    break;
  case fcsMissionPhase::MSN_LAND:
    // landing state
    evaluateLanding();
    break;
  case fcsMissionPhase::MSN_DIRECT:
    // direct state
    evaluateDirect();
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
      missionPhase_e = fcsMissionPhase::MSN_TAKEOFF;
    }
  }
}

void MSN::evaluateDirect()
{
  // remain in direct mode until TST directMode_b is cleared
}