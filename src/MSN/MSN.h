#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../IMU/IMU.h"

class MSN
{
public:
    MSN(int sampleRate_Hz = 50);

    enum fcsModeMissionPhase
    {
        MSN_IDLE = 0,
        MSN_TAKEOFF = 1,
        MSN_CRUISE = 2,
        MSN_LAND = 3
    };

    fcsModeMissionPhase missionPhase_e = MSN_IDLE;

    int takeoffDebounceLimit = 5;
    double takeoffThreshold_fps2 = 10.0;

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
    int _nxTakeoffDebounceCount = 0;
};
