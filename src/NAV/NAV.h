#ifndef NAV_H
#define NAV_H

#include "../CMN/CMN.h"
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
    void posEcef2lla();
    void velEcef2ned();
    void velNed2body();

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
    CMN C;

    CMN::matrix _ned2ecefMatrix;
    CMN::matrix _ned2bodyMatrix;
    CMN::matrix _body2ecefMatrix;

    CMN::matrix _gravityNED_fps2;

    CMN::matrix _vb_fps;
    CMN::matrix _sf_fps2;
    CMN::matrix _Pmat;
    CMN::matrix _Q;
    CMN::matrix _R;
    CMN::matrix _Bmat;
    CMN::matrix _F;
    CMN::matrix _H;
    CMN::matrix _I;
    CMN::matrix _K;
    CMN::matrix _Xmat;
    CMN::matrix _Umat;
    CMN::matrix _Z;

    long _lastGpsTime_s = 0;
};

#endif // NAV_H
