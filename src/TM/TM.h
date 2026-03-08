#ifndef TM_H
#define TM_H

#include <Arduino.h>
#include "../TST/TST.h"
#include "../MSN/MSN.h"
#include "../IMU/IMU.h"
#include "../GPS/GPS.h"
#include "../NAV/NAV.h"
#include "../GDC/GDC.h"
#include "../APT/APT.h"

void printStatus(Stream &serialPort, int now_ms, int endTime_ms, TST &tst, MSN &msn, IMU &imu, GPS &gps, NAV &nav, GDC &gdc, APT &apt);
void printStatusBin(Stream &serialPort, int now_ms, int endTime_ms, TST &tst, MSN &msn, IMU &imu, GPS &gps, NAV &nav, GDC &gdc, APT &apt);
uint16_t calculateCRC(const uint8_t *data, size_t length);

struct __attribute__((packed)) busTimeTM
{
    double met_s;
    int8_t stp_ms;
};

struct __attribute__((packed)) busMsnTM
{
    int8_t phs;
};

struct __attribute__((packed)) busImuTM
{
    double ax_fps2;
    double ay_fps2;
    double az_fps2;
    double gx_dps;
    double gy_dps;
    double gz_dps;
    double roll_d;
    double pitch_d;
    double yaw_d;
};

struct __attribute__((packed)) busGpsTM
{
    double timeStamp_s;
    double latG_d;
    double lon_d;
    double HAE_ft;
    double vECEF_X_fps;
    double vECEF_Y_fps;
    double vECEF_Z_fps;
    double hdop;
    int8_t fixQual;
    int8_t nSatellites;
};

struct __attribute__((packed)) busGdcTM
{
    double pbC_dps;
    double nyC_B_fps2;
    double nzC_B_fps2;
    double pwrCmd_pct;
};

struct __attribute__((packed)) busAptTM
{
    double rollIntegralError;
    double pitchIntegralError;
    double yawIntegralError;
    double effectorCmd[CFG::aptParams::nEffector];
};

struct __attribute__((packed)) busTM
{
    uint16_t header;
    struct busTimeTM timeTM;
    struct busMsnTM msnTM;
    struct busImuTM imuTM;
    struct busGpsTM gpsTM;
    struct busGdcTM gdcTM;
    struct busAptTM aptTM;
    uint16_t crc;
};

#endif // TM_H
