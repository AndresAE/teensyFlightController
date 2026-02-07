#include "NAV.h"

NAV::NAV(double sampleRate_Hz, IMU &imu, GPS &gps) : sampleRate_Hz(sampleRate_Hz), m_IMU_obj(imu), m_GPS_obj(gps)
{
    dt_s = 1.0 / sampleRate_Hz;
}

void NAV::begin()
{
    // initialize state from GPS if available
    _Xmat.m[0][0] = static_cast<float>(m_GPS_obj.m_ECEF.x_ft);
    _Xmat.m[1][0] = static_cast<float>(m_GPS_obj.m_ECEF.y_ft);
    _Xmat.m[2][0] = static_cast<float>(m_GPS_obj.m_ECEF.z_ft);
    _Xmat.m[3][0] = 0.0f;
    _Xmat.m[4][0] = 0.0f;
    _Xmat.m[5][0] = 0.0f;

    _ned2bodyMatrix = C.getNed2body(_ned2bodyMatrix, m_IMU_obj.m_Euler.roll_d, m_IMU_obj.m_Euler.pitch_d, m_IMU_obj.m_Euler.yaw_d);
    _body2ecefMatrix = C.getBody2ecef(_body2ecefMatrix, m_NAV.latG_d, m_NAV.lon_d, m_IMU_obj.m_Euler.roll_d, m_IMU_obj.m_Euler.pitch_d, m_IMU_obj.m_Euler.yaw_d);
}

void NAV::step()
{
    // Main navigation step logic
    predictState();
    if (m_GPS_obj.m_GPS.timeStamp_s != _lastGpsTime_s)
    {
        updateState();
        _lastGpsTime_s = m_GPS_obj.m_GPS.timeStamp_s;
    }
}

void NAV::predictState()
{
    // transform gravity to ecef
    _ned2bodyMatrix = C.getNed2body(_ned2bodyMatrix, m_IMU_obj.m_Euler.roll_d, m_IMU_obj.m_Euler.pitch_d, m_IMU_obj.m_Euler.yaw_d);
    _body2ecefMatrix = C.getBody2ecef(_body2ecefMatrix, m_NAV.latG_d, m_NAV.lon_d, m_IMU_obj.m_Euler.roll_d, m_IMU_obj.m_Euler.pitch_d, m_IMU_obj.m_Euler.yaw_d);
    Common::matrix _gravityB_fps2 = C.matrixMultiply(_ned2bodyMatrix, _gravityNED_fps2);
    Common::matrix _w_rps = {3, 1, {{static_cast<float>(m_IMU_obj.m_IMU.states.angularRateX_dps * C.DEG2RAD)}, {static_cast<float>(m_IMU_obj.m_IMU.states.angularRateY_dps * C.DEG2RAD)}, {static_cast<float>(m_IMU_obj.m_IMU.states.angularRateZ_dps * C.DEG2RAD)}}};

    _vb_fps = {3, 1, {{static_cast<float>(m_NAV.u_fps2)}, {static_cast<float>(m_NAV.v_fps2)}, {static_cast<float>(m_NAV.w_fps2)}}};

    Common::matrix _wxv = C.crossProductMatrix(_w_rps, _vb_fps);

    Common::matrix _tmp = C.matrixAdd(_sf_fps2, _gravityB_fps2);
    _Umat = C.matrixMultiply(_body2ecefMatrix, C.matrixAdd(_tmp, _wxv));

    _Xmat = C.matrixAdd(C.matrixMultiply(_F, _Xmat), C.matrixMultiply(_Bmat, _Umat));
    _Pmat = C.matrixAdd(C.matrixMultiply(C.matrixMultiply(_F, _Pmat), C.matrixTranspose(_F)), _Q);
}

void NAV::updateState()
{
    // Update state logic
    _Z.m[0][0] = static_cast<float>(m_GPS_obj.m_ECEF.x_ft);
    _Z.m[1][0] = static_cast<float>(m_GPS_obj.m_ECEF.y_ft);
    _Z.m[2][0] = static_cast<float>(m_GPS_obj.m_ECEF.z_ft);

    Common::matrix temp = C.matrixAdd(C.matrixMultiply(_H, C.matrixMultiply(_Pmat, C.matrixTranspose(_H))), _R);
    Common::matrix tempInv = C.matrixInverse(temp);
    _K = C.matrixMultiply(C.matrixMultiply(_Pmat, C.matrixTranspose(_H)), tempInv);

    _Xmat = C.matrixAdd(_Xmat, C.matrixMultiply(_K, C.matrixAdd(_Z, C.matrixMultiply(_H, _Xmat), true)));
    _Pmat = C.matrixMultiply(C.matrixAdd(_I, C.matrixMultiply(_K, _H), true), _Pmat);

    m_NAV.xECEF_ft = _Xmat.m[0][0];
    m_NAV.yECEF_ft = _Xmat.m[1][0];
    m_NAV.zECEF_ft = _Xmat.m[2][0];
    m_NAV.vxECEF_fps = _Xmat.m[3][0];
    m_NAV.vyECEF_fps = _Xmat.m[4][0];
    m_NAV.vzECEF_fps = _Xmat.m[5][0];

    posEcef2lla();
    velEcef2ned();
    velNed2body();
}

void NAV::posEcef2lla()
{
    // ECEF to LLA conversion logic
    Common::busLLA lla = C.ecef2lla({m_NAV.xECEF_ft, m_NAV.yECEF_ft, m_NAV.zECEF_ft});
    m_NAV.latG_d = lla.latG_d;
    m_NAV.lon_d = lla.lon_d;
    m_NAV.hae_ft = lla.hae_ft;
}

void NAV::velEcef2ned()
{
    _ned2ecefMatrix = C.getNed2ecef(_ned2ecefMatrix, m_NAV.latG_d, m_NAV.lon_d);
    Common::matrix velECEF = {3, 1, {{static_cast<float>(m_NAV.vxECEF_fps)}, {static_cast<float>(m_NAV.vyECEF_fps)}, {static_cast<float>(m_NAV.vzECEF_fps)}}};
    Common::matrix velNED = C.matrixMultiply(_ned2ecefMatrix, velECEF);
    m_NAV.vN_fps = velNED.m[0][0];
    m_NAV.vE_fps = velNED.m[1][0];
    m_NAV.vD_fps = velNED.m[2][0];
}

void NAV::velNed2body()
{
    _ned2bodyMatrix = C.getNed2body(_ned2bodyMatrix, m_IMU_obj.m_Euler.roll_d, m_IMU_obj.m_Euler.pitch_d, m_IMU_obj.m_Euler.yaw_d);
    Common::matrix velNED = {3, 1, {{static_cast<float>(m_NAV.vN_fps)}, {static_cast<float>(m_NAV.vE_fps)}, {static_cast<float>(m_NAV.vD_fps)}}};
    Common::matrix velBody = C.matrixMultiply(C.matrixTranspose(_ned2bodyMatrix), velNED);
    m_NAV.u_fps2 = velBody.m[0][0];
    m_NAV.v_fps2 = velBody.m[1][0];
    m_NAV.w_fps2 = velBody.m[2][0];
}
