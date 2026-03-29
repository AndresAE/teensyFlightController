#ifndef CFG_H
#define CFG_H
#include "../CMN/CMN.h"

class CFG
{
public:
    CFG() {}
    struct fcsParams
    {
        bool useTM_b = true;
        int serialBaud = 57600;
        int sampleRate_Hz = 100;
        int sampleRateLo_Hz = 100;
        int tmDelay_ms = 5000;
    };
    struct busGains
    {
        double K_p;
        double K_i;
        double K_d;
        double K_f;
        double k_antiWindup;
    };
    struct busAptGains
    {
        busGains roll;
        busGains rollRate;
        busGains pitch;
        busGains pitchRate;
        busGains nz;
        busGains heading;
        busGains ny;
        busGains yawRate;
    };
    struct tstParams
    {
        int controlInPins[5] = {2, 3, 4, 5, 6};        // roll, pitch, yaw, throttle
        int controlInTimeOut_us = 25000;               // 25 ms timeout
        int muMax_us[5] = {1900, 1900, 2000, 1900, 2}; // microseconds
        int muMin_us[5] = {1100, 1100, 1000, 1100, 0}; // microseconds
        double cmdOffset[5] = {0.0, -32.174, 0.0, 0.5, 1.0};
        double kCmd[5] = {-60.0, 32.174, -8.0, 0.5, 1.0}; // scale factor from pwm to control input
        double controlInHiLimit[5] = {60, 0.0, 8.0, 1.0, 2000.0};
        double controlInLoLimit[5] = {-60.0, -65, -8.0, 0.0, 1000.0};
        int directModeThreshold_us = 1700;
    };
    struct aptParams
    {
        busAptGains gains = {
            {0.0, 0.0, 0.0, 0.0, 0.0},   // roll, k_p, k_i, k_f, k_antiWindup
            {0.0, 5, 0.5, 0.0, 0.0},     // rollRate, k_p, k_i, k_f, k_antiWindup
            {0.0, 0.0, 0.0, 0.0, 0.0},   // pitch, k_p, k_i, k_f, k_antiWindup
            {0.0, 5, 0.5, 0.0, 0.0},     // pitchRate, k_p, k_i, k_f, k_antiWindup
            {-10.0, 0.0, 0.0, 0.0, 0.0}, // nz, k_p, k_i, k_f
            {0.0, 0.0, 0.0, 0.0, 0.0},   // heading,
            {10, 0.0, 0.0, 0.0, 0.0},    // ny, k_p, k_i, k_f, k_antiWindup
            {0.0, 5, 0.5, 0.0, 0.0}};    // yawRate, k_p, k_i, k_f, k_antiWindup
        bool integratorOff_b = false;
        static const int nEffector = 4;
        double effectorLimitHi[nEffector] = {20.0, 25.0, 25.0, 1.0};
        double effectorLimitLo[nEffector] = {-20.0, -25.0, -25.0, 0.0};
        double effectorOffset[nEffector] = {0.0, -5.0, 0.0, 0.0};
        int effectorPins[nEffector] = {7, 8, 9, 10};
        int muMax_us[nEffector] = {2000, 2000, 2000, 1900}; // microseconds
        int muMin_us[nEffector] = {1000, 1000, 1000, 1100}; // microseconds
        double servoRange_d[nEffector] = {180.0, 180.0, 180.0, 1.0};
        double directModeGain[nEffector] = {-1.45, 1.0, 1.25, 1.0};
        double mixingRatioRotational[nEffector][3] = {
            {1.0, 0.0, 0.0},  // aileron (+LTED)
            {0.0, -1.0, 0.0}, // elevator (+TED)
            {0.0, 0.0, -1.0}, // rudder (+TEL)
            {0.0, 0.0, 0.0}}; // throttle
        double mixingRatioTranslational[nEffector][3] = {
            {0.0, 0.0, 0.0},  // aileron (+LTED)
            {0.0, 0.0, 0.0},  // elevator (+TED)
            {0.0, 0.0, 0.0},  // rudder (+TEL)
            {1.0, 0.0, 0.0}}; // throttle
        static const int nMappingBP = 7;
        CMN::matrix servoMappingBreakpoints = {
            nEffector,
            nMappingBP,
            {{-25.0, -17.0, -10.0, 0, 10.0, 17.0, 25.0},
             {-31.0, -20.0, -9.0, 0, 7.0, 17.0, 23.0},
             {-30.0, -22.0, -16.0, 0, 17.0, 32.0, 39.0},
             {0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0}}};
        CMN::matrix servoMappingTable = {
            nEffector,
            nMappingBP,
            {{0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0},
             {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0},
             {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0},
             {0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0}}};
    };
    struct imuParams
    {
        int imu_baud = 115200;
        int boot_time_ms = 3000;
        int nInit = 100;
        double accel_LSB = 0.00981;
        double gyro_LSB = 0.0625;
        double xRot_d = 180.0;
        double yRot_d = 0.0;
        double zRot_d = -90.0;
        double cutoffAx_Hz = 5.0;
        double cutoffAy_Hz = 5.0;
        double cutoffAz_Hz = 5.0;
        double cutoffGx_Hz = 5.0;
        double cutoffGy_Hz = 5.0;
        double cutoffGz_Hz = 5.0;
    };
    struct gpsParams
    {
        int gps_baud = 9600;
        int boot_time_ms = 2000;
        double dt_gps_s = 0.25;
        double cutoff_Hz = 0.5;
        int min_satellites = 6;
        double max_hdop = 1.5;
    };
    struct gdcParams
    {
        double rollCmdLimits_d[2] = {-45.0, 45.0};
        double bankCmdLimits_d[2] = {-60.0, 60.0};
        double rollRateCmdLimits_dps[2] = {-180.0, 180.0};
        double pitchCmdLimits_d[2] = {-80.0, 80.0};
        double fpaCmdLimits_d[2] = {-80.0, 80.0};
        double nzCmdLimits_B_fps2[2] = {-80.5, 32.2};
        double altitudeCmdLimits_ft[2] = {0.0, 1000.0};
        double pitchRateCmdLimits_dps[2] = {-100.0, 100.0};
        double headingCmdLimits_d[2] = {0.0, 360.0};
        double azimuthCmdLimits_d[2] = {0.0, 360.0};
        double nyCmdLimits_B_fps2[2] = {-10.0, 10.0};
        double yawRateCmdLimits_dps[2] = {-100.0, 100.0};
        double throttleCmdLimits_pct[2] = {0.0, 100.0};
    };
    struct msnParams
    {
        int takeoffDebounceLimit = 3;
        float takeoffThreshold_fps2 = 10.0;
    };

    fcsParams fcsParams_;
    tstParams tstParams_;
    gdcParams gdcParams_;
    aptParams aptParams_;
    imuParams imuParams_;
    gpsParams gpsParams_;
    msnParams msnParams_;

    const fcsParams &getFcsParams()
    {
        return fcsParams_;
    }
    const tstParams &getTstParams()
    {
        return tstParams_;
    }
    const aptParams &getAptParams()
    {
        return aptParams_;
    }
    const imuParams &getImuParams()
    {
        return imuParams_;
    }
    const gpsParams &getGpsParams()
    {
        return gpsParams_;
    }
    const gdcParams &getGdcParams()
    {
        return gdcParams_;
    }
    const msnParams &getMsnParams()
    {
        return msnParams_;
    }
};

#endif // CFG_H