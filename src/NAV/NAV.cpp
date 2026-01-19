#include "NAV.h"

NAV::NAV(double sampleRate_Hz, IMU &imu, GPS &gps) : sampleRate_Hz(sampleRate_Hz), m_IMU_obj(imu), m_GPS_obj(gps)
{
    dt_s = 1.0 / sampleRate_Hz;
}

void NAV::begin()
{
    // initialize state from GPS if available
    _X.m[0][0] = static_cast<float>(m_GPS_obj.m_ECEF.x_ft);
    _X.m[1][0] = static_cast<float>(m_GPS_obj.m_ECEF.y_ft);
    _X.m[2][0] = static_cast<float>(m_GPS_obj.m_ECEF.z_ft);
    _X.m[3][0] = 0.0f;
    _X.m[4][0] = 0.0f;
    _X.m[5][0] = 0.0f;

    getNed2body();
    getBody2ecef();
}

void NAV::step()
{
    // Main navigation step logic
    predictState();
    if (m_GPS_obj.m_GPS.timeStamp_ms != _lastGpsTime_ms)
    {
        updateState();
        _lastGpsTime_ms = m_GPS_obj.m_GPS.timeStamp_ms;
    }
}

void NAV::predictState()
{
    // transform gravity to ecef
    getNed2body();
    getBody2ecef();
    Common::matrix _gravityB_fps2 = C.matrixMultiply(_ned2bodyMatrix, _gravityNED_fps2);
    Common::matrix _w_rps = {3, 1, {{static_cast<float>(m_IMU_obj.m_IMU.states.angularRateX_dps * C.DEG2RAD)}, {static_cast<float>(m_IMU_obj.m_IMU.states.angularRateY_dps * C.DEG2RAD)}, {static_cast<float>(m_IMU_obj.m_IMU.states.angularRateZ_dps * C.DEG2RAD)}}};

    _vb_fps = {3, 1, {{static_cast<float>(m_NAV.u_fps2)}, {static_cast<float>(m_NAV.v_fps2)}, {static_cast<float>(m_NAV.w_fps2)}}};

    Common::matrix _wxv = C.crossProductMatrix(_w_rps, _vb_fps);

    Common::matrix _tmp = C.matrixAdd(_sf_fps2, _gravityB_fps2);
    _U = C.matrixMultiply(_body2ecefMatrix, C.matrixAdd(_tmp, _wxv));

    _X = C.matrixAdd(C.matrixMultiply(_F, _X), C.matrixMultiply(_B, _U));
    _P = C.matrixAdd(C.matrixMultiply(C.matrixMultiply(_F, _P), C.matrixTranspose(_F)), _Q);
}

void NAV::updateState()
{
    // Update state logic
    _Z.m[0][0] = static_cast<float>(m_GPS_obj.m_ECEF.x_ft);
    _Z.m[1][0] = static_cast<float>(m_GPS_obj.m_ECEF.y_ft);
    _Z.m[2][0] = static_cast<float>(m_GPS_obj.m_ECEF.z_ft);

    Common::matrix temp = C.matrixAdd(C.matrixMultiply(_H, C.matrixMultiply(_P, C.matrixTranspose(_H))), _R);
    Common::matrix tempInv = C.matrixInverse(temp);
    _K = C.matrixMultiply(C.matrixMultiply(_P, C.matrixTranspose(_H)), tempInv);

    _X = C.matrixAdd(_X, C.matrixMultiply(_K, C.matrixAdd(_Z, C.matrixMultiply(_H, _X), true)));
    _P = C.matrixMultiply(C.matrixAdd(_I, C.matrixMultiply(_K, _H), true), _P);

    m_NAV.xECEF_ft = _X.m[0][0];
    m_NAV.yECEF_ft = _X.m[1][0];
    m_NAV.zECEF_ft = _X.m[2][0];
    m_NAV.vxECEF_fps = _X.m[3][0];
    m_NAV.vyECEF_fps = _X.m[4][0];
    m_NAV.vzECEF_fps = _X.m[5][0];

    ecef2lla();
    // note: ecef2ned and ned2body helpers not implemented here; they can be added later
}

void NAV::getNed2body()
{
    // NED to body frame transformation logic using IMU euler angles
    double pitch = m_IMU_obj.m_IMU.euler.pitch_d;
    double yaw = m_IMU_obj.m_IMU.euler.yaw_d;
    double roll = m_IMU_obj.m_IMU.euler.roll_d;

    _ned2bodyMatrix.m[0][0] = static_cast<float>(cos(pitch) * cos(yaw));
    _ned2bodyMatrix.m[0][1] = static_cast<float>(cos(pitch) * sin(yaw));
    _ned2bodyMatrix.m[0][2] = static_cast<float>(-sin(pitch));
    _ned2bodyMatrix.m[1][0] = static_cast<float>(sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw));
    _ned2bodyMatrix.m[1][1] = static_cast<float>(sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw));
    _ned2bodyMatrix.m[1][2] = static_cast<float>(sin(roll) * cos(pitch));
    _ned2bodyMatrix.m[2][0] = static_cast<float>(cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw));
    _ned2bodyMatrix.m[2][1] = static_cast<float>(cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw));
    _ned2bodyMatrix.m[2][2] = static_cast<float>(cos(roll) * cos(pitch));
}

void NAV::getNed2ecef()
{
    double lat = m_GPS_obj.m_GPS.lla_ddft.latG_d * C.DEG2RAD;
    double lon = m_GPS_obj.m_GPS.lla_ddft.lon_d * C.DEG2RAD;
    _ned2ecefMatrix.m[0][0] = static_cast<float>(-sin(lat) * cos(lon));
    _ned2ecefMatrix.m[0][1] = static_cast<float>(-sin(lon));
    _ned2ecefMatrix.m[0][2] = static_cast<float>(-cos(lat) * cos(lon));
    _ned2ecefMatrix.m[1][0] = static_cast<float>(-sin(lat) * sin(lon));
    _ned2ecefMatrix.m[1][1] = static_cast<float>(cos(lon));
    _ned2ecefMatrix.m[1][2] = static_cast<float>(-cos(lat) * sin(lon));
    _ned2ecefMatrix.m[2][0] = static_cast<float>(cos(lat));
    _ned2ecefMatrix.m[2][1] = 0.0f;
    _ned2ecefMatrix.m[2][2] = static_cast<float>(-sin(lat));
}

void NAV::getBody2ecef()
{
    getNed2ecef();
    _body2ecefMatrix = C.matrixMultiply(_ned2ecefMatrix, C.matrixTranspose(_ned2bodyMatrix));
}

void NAV::ecef2lla()
{
    // ECEF to LLA conversion logic
    Common::busLLA lla = C.ecef2lla({m_NAV.xECEF_ft, m_NAV.yECEF_ft, m_NAV.zECEF_ft});
    m_NAV.latG_d = lla.latG_d;
    m_NAV.lon_d = lla.lon_d;
    m_NAV.hae_ft = lla.hae_ft;
}
