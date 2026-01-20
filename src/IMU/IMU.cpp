#include "IMU.h"
#include "Wire.h"

IMU::IMU(int sampleRate_Hz, double yaw0_d)
    : sampleRate_Hz(sampleRate_Hz), yaw0_d(yaw0_d), imuParams(cfg.getImuParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void IMU::begin()
{
  Wire1.begin();
  writeRegister(BNO055_OPR_MODE, 0x00); // Config mode
  writeRegister(BNO055_PWR_MODE, 0x00); // Normal power
  writeRegister(BNO055_PAGE_ID, 0x00);  // Page 0
  writeRegister(BNO055_OPR_MODE, 0x08); // IMUPLUS mode
  _alphaAx = C.calcAlpha(imuParams.cutoffAx_Hz, dt_s);
  _alphaAy = C.calcAlpha(imuParams.cutoffAy_Hz, dt_s);
  _alphaAz = C.calcAlpha(imuParams.cutoffAz_Hz, dt_s);
  _alphaGx = C.calcAlpha(imuParams.cutoffGx_Hz, dt_s);
  _alphaGy = C.calcAlpha(imuParams.cutoffGy_Hz, dt_s);
  _alphaGz = C.calcAlpha(imuParams.cutoffGz_Hz, dt_s);

  initializeRotMatrix();

  delay(imuParams.boot_time_ms);
  // initialize
  for (int ii = 0; ii < imuParams.nInit; ii++)
  {
    processStatesRaw();

    _axPrev_fps2 += m_States.specificForceX_fps2;
    _ayPrev_fps2 += m_States.specificForceY_fps2;
    _azPrev_fps2 += m_States.specificForceZ_fps2;

    // roll and angle
    _tmpPitch_d = asin(m_States.specificForceX_fps2 / C.GRAVITY_fps2) * C.RAD2DEG;
    m_Euler.pitch_d += _tmpPitch_d;
    m_Euler.roll_d += asin(-m_States.specificForceY_fps2 / (C.GRAVITY_fps2 * cos(_tmpPitch_d * C.DEG2RAD))) * C.RAD2DEG;
  };

  m_Euler.pitch_d = m_Euler.pitch_d / static_cast<double>(imuParams.nInit);
  m_Euler.roll_d = m_Euler.roll_d / static_cast<double>(imuParams.nInit);
  _axPrev_fps2 = _axPrev_fps2 / static_cast<double>(imuParams.nInit);
  _ayPrev_fps2 = _ayPrev_fps2 / static_cast<double>(imuParams.nInit);
  _azPrev_fps2 = _azPrev_fps2 / static_cast<double>(imuParams.nInit);
}
void IMU::step()
{
  processStatesRaw();

  // filtering
  m_States.angularRateX_dps = C.lowPassFilter(m_States.angularRateX_dps, _gxPrev_dps, _alphaGx);
  m_States.angularRateY_dps = C.lowPassFilter(m_States.angularRateY_dps, _gyPrev_dps, _alphaGy);
  m_States.angularRateZ_dps = C.lowPassFilter(m_States.angularRateZ_dps, _gzPrev_dps, _alphaGz);
  m_States.specificForceX_fps2 = C.lowPassFilter(m_States.specificForceX_fps2, _axPrev_fps2, _alphaAx);
  m_States.specificForceY_fps2 = C.lowPassFilter(m_States.specificForceY_fps2, _ayPrev_fps2, _alphaAy);
  m_States.specificForceZ_fps2 = C.lowPassFilter(m_States.specificForceZ_fps2, _azPrev_fps2, _alphaAz);

  // euler angle integration
  // precompute trig to avoid repeated calls
  double roll_r = C.DEG2RAD * m_Euler.roll_d;
  double pitch_r = C.DEG2RAD * m_Euler.pitch_d;
  double sinR = sin(roll_r);
  double cosR = cos(roll_r);
  double cosP = cos(pitch_r);
  double tanP = tan(pitch_r);
  double secP = (cosP == 0.0) ? 0.0 : 1.0 / cosP;

  m_eulerRate.phiDot_dps = m_States.angularRateX_dps + sinR * tanP * m_States.angularRateY_dps + cosR * tanP * m_States.angularRateZ_dps;
  m_eulerRate.thetaDot_dps = cosR * m_States.angularRateY_dps - sinR * m_States.angularRateZ_dps;
  m_eulerRate.psiDot_dps = (sinR * secP) * m_States.angularRateY_dps + (cosR * secP) * m_States.angularRateZ_dps;

  m_Euler.roll_d = C.trapIntegral(m_eulerRate.phiDot_dps, m_eulerRatePrev.phiDot_dps, m_Euler.roll_d, dt_s);
  m_Euler.pitch_d = C.trapIntegral(m_eulerRate.thetaDot_dps, m_eulerRatePrev.thetaDot_dps, m_Euler.pitch_d, dt_s);
  m_Euler.yaw_d = C.trapIntegral(m_eulerRate.psiDot_dps, m_eulerRatePrev.psiDot_dps, m_Euler.yaw_d, dt_s);
  m_eulerRatePrev = m_eulerRate;

  m_IMU = {m_States, m_Euler};

  _axPrev_fps2 = m_States.specificForceX_fps2;
  _ayPrev_fps2 = m_States.specificForceY_fps2;
  _azPrev_fps2 = m_States.specificForceZ_fps2;
  _gxPrev_dps = m_States.angularRateX_dps;
  _gyPrev_dps = m_States.angularRateY_dps;
  _gzPrev_dps = m_States.angularRateZ_dps;
}

void IMU::processStatesRaw()
{
  readVector(BNO055_ACCEL_DATA_X_LSB, _axRaw, _ayRaw, _azRaw);
  readVector(BNO055_GYRO_DATA_X_LSB, _gxRaw, _gyRaw, _gzRaw);
  rotateRaw();

  double k_Accel = imuParams.accel_LSB * C.M2FT * imuParams.accelMagnitudeScaling;
  m_States = {
      _gxRaw * imuParams.gyro_LSB + imuParams.gyroX_offset_dps,
      _gyRaw * imuParams.gyro_LSB + imuParams.gyroY_offset_dps,
      _gzRaw * imuParams.gyro_LSB + imuParams.gyroZ_offset_dps,
      _axRaw * k_Accel,
      _ayRaw * k_Accel,
      _azRaw * k_Accel};
}

void IMU::rotateRaw()
{
  // use temporaries to avoid in-place overwrites
  int16_t ax0 = _axRaw;
  int16_t ay0 = _ayRaw;
  int16_t az0 = _azRaw;

  _axRaw = (_cZ * _cY) * ax0 + (_sZ * _cY) * ay0 - (_sY)*az0;
  _ayRaw = (_cZ * _sY * _sX - _sZ * _cX) * ax0 + (_sZ * _sY * _sX + _cZ * _cX) * ay0 + (_cY * _sX) * az0;
  _azRaw = (_cZ * _sY * _cX - _sZ * _sX) * ax0 + (_sZ * _sY * _cX - _cZ * _sX) * ay0 + (_cY * _cX) * az0;

  int16_t gx0 = _gxRaw;
  int16_t gy0 = _gyRaw;
  int16_t gz0 = _gzRaw;

  _gxRaw = (_cZ * _cY) * gx0 + (_sZ * _cY) * gy0 - (_sY)*gz0;
  _gyRaw = (_cZ * _sY * _sX - _sZ * _cX) * gx0 + (_sZ * _sY * _sX + _cZ * _cX) * gy0 + (_cY * _sX) * gz0;
  _gzRaw = (_cZ * _sY * _cX - _sZ * _sX) * gx0 + (_sZ * _sY * _cX - _cZ * _sX) * gy0 + (_cY * _cX) * gz0;
}

void IMU::initializeRotMatrix()
{
  _cX = cos(imuParams.xRot_d * C.DEG2RAD);
  _sX = sin(imuParams.xRot_d * C.DEG2RAD);
  _cY = cos(imuParams.yRot_d * C.DEG2RAD);
  _sY = sin(imuParams.yRot_d * C.DEG2RAD);
  _cZ = cos(imuParams.zRot_d * C.DEG2RAD);
  _sZ = sin(imuParams.zRot_d * C.DEG2RAD);
}

void IMU::writeRegister(uint8_t reg, uint8_t value)
{
  Wire1.beginTransmission(BNO055_ADDRESS);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

void IMU::readVector(uint8_t startReg, int16_t &xVector, int16_t &yVector, int16_t &zVector)
{
  Wire1.beginTransmission(BNO055_ADDRESS);
  Wire1.write(startReg);
  Wire1.endTransmission();
  Wire1.requestFrom(BNO055_ADDRESS, 6);
  xVector = Wire1.read() | (Wire1.read() << 8);
  yVector = Wire1.read() | (Wire1.read() << 8);
  zVector = Wire1.read() | (Wire1.read() << 8);
}
