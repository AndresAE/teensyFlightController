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
        int sampleRate_Hz = 100;
        int tmDelay_ms = 5000;
    };
    struct busGains
    {
        double K_p;
        double K_i;
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
    struct aptParams
    {
        busAptGains gains = {
            {1.0, 0.0, 0.0, 0.0},  // roll, k_p, k_i, k_f, k_antiWindup
            {0.5, 0.0, 0.0, 0.0},  // rollRate, k_p, k_i, k_f, k_antiWindup
            {1.0, 0.0, 0.0, 0.0},  // pitch, k_p, k_i, k_f, k_antiWindup
            {1.0, 0.0, 0.0, 0.0},  // pitchRate, k_p, k_i, k_f, k_antiWindup
            {-0.5, 0.0, 0.0, 0.0}, // nz, k_p, k_i, k_f, k_antiWindup
            {1.0, 0.0, 0.0, 0.0},  // heading, k_p, k_i, k_f, k_antiWindup
            {1.0, 0.0, 0.0, 0.0},  // ny, k_p, k_i, k_f, k_antiWindup
            {1.0, 0.0, 0.0, 0.0}}; // yawRate, k_p, k_i, k_f, k_antiWindup
        double effectorLimitHi[5] = {20.0, 20.0, 20.0, 20.0, 1.0};
        double effectorLimitLo[5] = {-20.0, -20.0, -20.0, -20.0, 0.0};
        double effectorOffset[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        double effectorPins[5] = {5, 6, 7, 8, 9};
        int pwmMax[5] = {2000, 2000, 2000, 2000, 2000}; // microseconds
        int pwmMin[5] = {1000, 1000, 1000, 1000, 1000}; // microseconds
        double degMax[5] = {30.0, 30.0, 30.0, 30.0, 1.0};
        double degMin[5] = {-30.0, -30.0, -30.0, -30.0, 0.0};
        double mixingRatioRotational[5][3] = {
            {1.0, 0.0, 0.1},   // left aileron (+LTED)
            {-1.0, 0.0, -0.1}, // right aileron (+RTEU)
            {0.0, 1.0, 0.0},   // elevator (+TEU)
            {0.1, 0.0, 1.0},   // rudder (+TER)
            {0.0, 0.0, 0.0}    // throttle
        };
        double mixingRatioLinear[5][3] = {
            {0.0, 0.0, 0.0}, // left aileron (+LTED)
            {0.0, 0.0, 0.0}, // right aileron (+RTEU)
            {0.0, 0.0, 0.0}, // elevator (+TEU)
            {0.0, 0.0, 0.0}, // rudder (+TER)
            {1.0, 0.0, 0.0}  // throttle
        };
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
        double zRot_d = 0.0;
        double cutoffAx_Hz = 10.0;
        double cutoffAy_Hz = 10.0;
        double cutoffAz_Hz = 10.0;
        double cutoffGx_Hz = 10.0;
        double cutoffGy_Hz = 10.0;
        double cutoffGz_Hz = 10.0;
        double gyroX_offset_dps = 0.0;
        double gyroY_offset_dps = 0.0;
        double gyroZ_offset_dps = 0.0;
        double accelMagnitudeScaling = 1.0;
    };
    struct gpsParams
    {
        int gps_baud = 9600;
        int boot_time_ms = 2000;
    };
    struct gdcParams
    {
        int controlInPins[4] = {0, 1, 2, 3}; // roll, pitch, yaw, throttle
        int controlInTimeOut_ms = 25000;     // 25 ms timeout
        double pwmOffset_us[4] = {1500.0, 1833.0, 1500.0, 1000};
        double k_pwm[4] = {-0.12, 0.1, -0.016, 0.002}; // scale factor from pwm to control input
        double controlInHiLimit[4] = {60, 16.1, 8.0, 1.0};
        double controlInLoLimit[4] = {-60.0, -80.5, -8.0, 0.0};
        double controlIn_us[4] = {0.0, 0.0, 0.0, 0.0};
        double controlInRaw[4] = {0.0, 0.0, 0.0, 0.0};
        double controlIn[4] = {0.0, 0.0, 0.0, 0.0};
    };
    struct msnParams
    {
        int takeoffDebounceLimit = 5;
        float takeoffThreshold_fps2 = 10.0;
    };

    fcsParams fcsParams_;
    gdcParams gdcParams_;
    aptParams aptParams_;
    imuParams imuParams_;
    gpsParams gpsParams_;
    msnParams msnParams_;

    fcsParams getFcsParams()
    {
        return fcsParams_;
    }
    aptParams getAptParams()
    {
        return aptParams_;
    }
    imuParams getImuParams()
    {
        return imuParams_;
    }
    gpsParams getGpsParams()
    {
        return gpsParams_;
    }
    gdcParams getGdcParams()
    {
        return gdcParams_;
    }
    msnParams getMsnParams()
    {
        return msnParams_;
    }
};

#endif // CONFIG_H