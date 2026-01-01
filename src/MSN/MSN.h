#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../IMU/IMU.h"
#include "../config/config.h"

class MSN
{
public:
    MSN(int sampleRate_Hz);

    enum fcsModeMissionPhase
    {
        MSN_IDLE = 0,
        MSN_TAKEOFF = 1,
        MSN_CRUISE = 2,
        MSN_LAND = 3
    };

    fcsModeMissionPhase missionPhase_e = MSN_IDLE;
    Config::msnParams msnParams = cfg.getMsnParams();

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
    void attachBus(IMU::busIMU *imuBus);

    // pointers to external bus objects (set from main sketch)
    IMU::busIMU *m_IMU = nullptr;

private:
    Config cfg;
    int _nxTakeoffDebounceCount = 0;
};
