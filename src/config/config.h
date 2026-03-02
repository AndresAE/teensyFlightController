#ifndef CONFIG_H
#define CONFIG_H

class Config
{
public:
    Config() {}
    struct fcsParams
    {
        bool useTM_b = false;
        int serialBaud = useTM_b ? 57600 : 9600;
        int sampleRate_Hz = 50;
        int sampleRateLo_Hz = 5;
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
        int controlInPins[5] = {2, 3, 4, 5, 6}; // roll, pitch, yaw, throttle
        int controlInTimeOut_us = 25000;        // 25 ms timeout
        double pwmOffset_us[5] = {1500.0, 1770, 1500.0, 1100, 0};
        double k_pwm[5] = {-0.15, 0.12, -0.02, 0.00125, 1}; // scale factor from pwm to control input
        double controlInHiLimit[5] = {60, 16.0, 8.0, 1.0, 2000.0};
        double controlInLoLimit[5] = {-60.0, -80.0, -8.0, 0.0, 1000.0};
        int directModeThreshold_us = 1700;
    };
    struct aptParams
    {
        busAptGains gains = {
            {0.0, 0.0, 0.0, 0.0, 0.0},          // roll, k_p, k_i, k_f, k_antiWindup
            {0.0, 0.5, 0.5 * 2.5, 0.0, 0.0},    // rollRate, k_p, k_i, k_f, k_antiWindup
            {0.0, 0.0, 0.0, 0.0, 0.0},          // pitch, k_p, k_i, k_f, k_antiWindup
            {0.0, 0.5, 0.5 * 1.6, 0.0, 0.0},    // pitchRate, k_p, k_i, k_f, k_antiWindup
            {-1.0, 0.0, 0.0, 0.0, 0.0},         // nz, k_p, k_i, k_f
            {0.0, 0.0, 0.0, 0.0, 0.0},          // heading,
            {1.0, 0.0, 0.0, 0.0, 0.0},          // ny, k_p, k_i, k_f, k_antiWindup
            {0.0, 0.5, -0.5 * 1.75, 0.0, 0.0}}; // yawRate, k_p, k_i, k_f, k_antiWindup
        bool integratorOff_b = true;
        double effectorLimitHi[4] = {60.0, 60.0, 60.0, 1.0};
        double effectorLimitLo[4] = {-60.0, -60.0, -60.0, 0.0};
        double effectorOffset[4] = {90.0, 80.0, 90.0, 0.0};
        int effectorPins[4] = {7, 8, 9, 10};
        int muMax_us[4] = {2500, 2500, 2500, 2000};        // microseconds
        int muMin_us[4] = {500, 500, 500, 1000};           // microseconds
        double directModeGain[4] = {-0.5, 0.5, 0.5, 1.25}; // scale factor from control input to effector command in direct mode
        double servoRange_d[4] = {180.0, 180.0, 180.0, 1.0};
        double mixingRatioRotational[4][3] = {
            {1.0, 0.0, 0.2},   // aileron
            {0.0, -1.0, 0.0},  // elevator
            {-0.2, 0.0, -1.0}, // rudder (+TER)
            {0.0, 0.0, 0.0}};  // throttle
        double mixingRatioLinear[4][3] = {
            {0.0, 0.0, 0.0},  // aileron (+LTED)
            {0.0, 0.0, 0.0},  // elevator (+TEU)
            {0.0, 0.0, 0.0},  // rudder (+TER)
            {1.0, 0.0, 0.0}}; // throttle
    };
    struct imuParams
    {
        int imu_baud = 115200;
        int boot_time_ms = 2000;
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
        int takeoffDebounceLimit = 2;
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

#endif // CONFIG_H