#ifndef MSN_H
#define MSN_H

#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../TST/TST.h"
#include "../IMU/IMU.h"
#include "../config/config.h"

class MSN
{
public:
    MSN(int sampleRate_Hz, TST &tst, IMU &imu);

    enum fcsMissionPhase
    {
        MSN_IDLE = 0,
        MSN_TAKEOFF = 1,
        MSN_CRUISE = 2,
        MSN_LAND = 3,
        MSN_DIRECT = 4
    };

    fcsMissionPhase missionPhase_e = MSN_IDLE;

    int sampleRate_Hz;
    double dt_s;

    // public methods
    void begin();
    void step();
    void missionManager();
    void evaluateTakeoff();
    void evaluateCruise();
    void evaluateLanding();
    void evaluateIdle();
    void evaluateDirect();

    // attachBus removed: MSN holds reference to IMU object
    TST &m_TST_obj;
    IMU &m_IMU_obj;

private:
    Config cfg;
    const Config::msnParams &msnParams = cfg.getMsnParams();
    int _nxTakeoffDebounceCount = 0;
    bool _busAttached_b = false;
};

#endif // MSN_H
