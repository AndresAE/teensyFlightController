#include "../Common/Common.h"
#include "../IMU/IMU.h"
#include "../GPS/GPS.h"

class NAV
{
public:
    // Constructor
    NAV(double sampleRate_Hz, IMU &imu, GPS &gps);

    double sampleRate_Hz;
    double dt_s;
    double initialCovariance = 1000.0;
    double processNoise = 1.0;
    double measurementNoise = 5.0;

    // Public methods
    void begin();
    void step();
    void predictState();
    void updateState();
    void getNed2body();
    void getNed2ecef();
    void getBody2ecef();
    void ecef2lla();

    struct busNAV
    {
        double xECEF_ft;
        double yECEF_ft;
        double zECEF_ft;
        double latG_d;
        double lon_d;
        double hae_ft;
        double vxECEF_fps;
        double vyECEF_fps;
        double vzECEF_fps;
        double vN_fps;
        double vE_fps;
        double vD_fps;
        double u_fps2;
        double v_fps2;
        double w_fps2;
    };

    busNAV m_NAV;

    IMU &m_IMU_obj;
    GPS &m_GPS_obj;

private:
    Common C;

    Common::matrix _ned2ecefMatrix = {3, 3, {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    Common::matrix _ned2bodyMatrix = {3, 3, {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    Common::matrix _body2ecefMatrix = {3, 3, {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};

    Common::matrix _gravityNED_fps2 = {3, 1, {{0.0f}, {0.0f}, {static_cast<float>(C.GRAVITY_fps2)}}};

    Common::matrix _vb_fps;
    Common::matrix _sf_fps2;
    Common::matrix _P = {6, 6, {{1e3f, 1e3f, 1e3f, 1e3f, 1e3f, 1e3f}, {1e3f, 1e3f, 1e3f, 1e3f, 1e3f, 1e3f}, {1e3f, 1e3f, 1e3f, 1e3f, 1e3f, 1e3f}, {1e3f, 1e3f, 1e3f, 1e3f, 1e3f, 1e3f}, {1e3f, 1e3f, 1e3f, 1e3f, 1e3f, 1e3f}, {1e3f, 1e3f, 1e3f, 1e3f, 1e3f, 1e3f}}}; // state covariance
    Common::matrix _Q = {6, 6, {{0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}, {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}, {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}, {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}, {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}, {0.1f, 0.1f, 0.1f, 0.1f, 0.1f}}};       // process noise
    Common::matrix _R = {3, 3, {{5.0f, 5.0f, 5.0f}, {5.0f, 5.0f, 5.0f}, {5.0f, 5.0f, 5.0f}}};                                                                                                                                                                         // measurement noise
    Common::matrix _B = {6, 3, {{0.5f * dt_s * dt_s, 0.0f, 0.0f}, {0.0f, 0.5f * dt_s *dt_s, 0.0f}, {0.0f, 0.0f, 0.5f * dt_s *dt_s}, {dt_s, 0.0f, 0.0f}, {0.0f, dt_s, 0.0f}, {0.0f, 0.0f, dt_s}}};                                                                     // control input
    Common::matrix _F = {6, 6, {{1.0f, 0.0f, 0.0f, dt_s, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f, dt_s, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt_s}, {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}}}; // control input
    Common::matrix _H = {3, 6, {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}}};                                                                                                                   // measurement matrix
    Common::matrix _I = {6, 6, {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}}}; // identity

    Common::matrix _K = {6, 3, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}}; // Kalman gain
    Common::matrix _X = {6, 1, {{0.0f}, {0.0f}, {0.0f}, {0.0f}, {0.0f}, {0.0f}}};                                                                         // state vector
    Common::matrix _U = {3, 1, {{0.0f}, {0.0f}, {0.0f}}};                                                                                                 // control input vector
    Common::matrix _Z = {3, 1, {{0.0f}, {0.0f}, {0.0f}}};
    // measurement vector
    long _lastGpsTime_ms = 0;
};