#include "IMU.h"
#include "Wire.h"

IMU::IMU(int sampleRate_Hz, double yaw0_d) 
: sampleRate_Hz(sampleRate_Hz), yaw0_d(yaw0_d) {
  _dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void IMU::begin() {
  Wire1.begin();
  writeRegister(BNO055_OPR_MODE, 0x00); // Config mode
  writeRegister(BNO055_PWR_MODE, 0x00); // Normal power
  writeRegister(BNO055_PAGE_ID, 0x00);  // Page 0
  writeRegister(BNO055_OPR_MODE, 0x08); // IMUPLUS mode
  alphaAx = C.calcAlpha(cutoffAx_Hz, _dt_s);
  alphaAy = C.calcAlpha(cutoffAy_Hz, _dt_s);
  alphaAz = C.calcAlpha(cutoffAz_Hz, _dt_s);
  alphaGx = C.calcAlpha(cutoffGx_Hz, _dt_s);
  alphaGy = C.calcAlpha(cutoffGy_Hz, _dt_s);
  alphaGz = C.calcAlpha(cutoffGz_Hz, _dt_s);

  initializeRotMatrix();

  delay(boot_time_ms);

  // initialize
  for (int ii=0; ii<nInit; ii++) {
    processStatesRaw();

    gxPrev_dps += m_States.angularRateX_dps;
    gyPrev_dps += m_States.angularRateY_dps;
    gzPrev_dps += m_States.angularRateZ_dps;
    axPrev_fps2 += m_States.specificForceX_fps2;
    ayPrev_fps2 += m_States.specificForceY_fps2;
    azPrev_fps2 += m_States.specificForceZ_fps2;

    // roll and angle
    m_Euler.pitch_d += asin(m_States.specificForceX_fps2 / C.GRAVITY_fps2) * C.RAD2DEG;
    m_Euler.roll_d += asin(m_States.specificForceY_fps2 / (C.GRAVITY_fps2 * cos(m_Euler.pitch_d * C.DEG2RAD))) * C.RAD2DEG;
    
    // accel scale factor error
    _accelMagnitudeScaling +=  C.GRAVITY_fps2 / C.norm(m_States.specificForceX_fps2, m_States.specificForceY_fps2, m_States.specificForceZ_fps2);

    // gyro bias error 
    _gyroX_offset_dps += m_States.angularRateX_dps;
    _gyroY_offset_dps += m_States.angularRateY_dps;
    _gyroZ_offset_dps += m_States.angularRateZ_dps;
  };

  m_Euler.pitch_d = m_Euler.pitch_d / static_cast<double>(nInit);
  m_Euler.roll_d = m_Euler.roll_d / static_cast<double>(nInit);
  accelMagnitudeScaling = _accelMagnitudeScaling / static_cast<double>(nInit);
  gyroX_offset_dps = -_gyroX_offset_dps / static_cast<double>(nInit);
  gyroY_offset_dps = -_gyroY_offset_dps / static_cast<double>(nInit);
  gyroZ_offset_dps = -_gyroZ_offset_dps / static_cast<double>(nInit);
  gxPrev_dps = gxPrev_dps / static_cast<double>(nInit);
  gyPrev_dps = gyPrev_dps / static_cast<double>(nInit);
  gzPrev_dps = gzPrev_dps / static_cast<double>(nInit);
  axPrev_fps2 = axPrev_fps2 / static_cast<double>(nInit);
  ayPrev_fps2 = ayPrev_fps2 / static_cast<double>(nInit);
  azPrev_fps2 = azPrev_fps2 / static_cast<double>(nInit);
}

void IMU::step() {
  processStatesRaw();

  // filtering
  // m_States.angularRateX_dps = C.lowPassFilter(m_States.angularRateX_dps, gxPrev_dps, alphaGx);
  // m_States.angularRateY_dps = C.lowPassFilter(m_States.angularRateY_dps, gyPrev_dps, alphaGy);
  // m_States.angularRateZ_dps = C.lowPassFilter(m_States.angularRateZ_dps, gzPrev_dps, alphaGz);
  // m_States.specificForceX_fps2 = C.lowPassFilter(m_States.specificForceX_fps2, axPrev_fps2, alphaAx);
  // m_States.specificForceY_fps2 = C.lowPassFilter(m_States.specificForceY_fps2, ayPrev_fps2, alphaAy);
  // m_States.specificForceZ_fps2 = C.lowPassFilter(m_States.specificForceZ_fps2, azPrev_fps2, alphaAz);

  // euler angle integration
  m_eulerRate.phiDot_dps = m_States.angularRateX_dps + sin(C.DEG2RAD * m_Euler.roll_d) * tan(C.DEG2RAD * m_Euler.pitch_d) * m_States.angularRateY_dps + cos(C.DEG2RAD * m_Euler.roll_d) * tan(C.DEG2RAD * m_Euler.pitch_d) * m_States.angularRateZ_dps;
  m_eulerRate.thetaDot_dps = cos(C.DEG2RAD * m_Euler.roll_d) * m_States.angularRateY_dps - sin(C.DEG2RAD * m_Euler.roll_d) * m_States.angularRateZ_dps;
  m_eulerRate.psiDot_dps = sin(C.DEG2RAD * m_Euler.roll_d) / cos(C.DEG2RAD * m_Euler.pitch_d) * m_States.angularRateY_dps + cos(C.DEG2RAD * m_Euler.roll_d) / cos(C.DEG2RAD * m_Euler.pitch_d) * m_States.angularRateZ_dps;
 
  m_Euler.roll_d = C.trapIntegral(m_eulerRate.phiDot_dps, m_eulerRatePrev.phiDot_dps, m_Euler.roll_d, _dt_s);
  m_Euler.pitch_d = C.trapIntegral(m_eulerRate.thetaDot_dps, m_eulerRatePrev.thetaDot_dps, m_Euler.pitch_d, _dt_s);
  m_Euler.yaw_d = C.trapIntegral(m_eulerRate.psiDot_dps, m_eulerRatePrev.psiDot_dps, m_Euler.yaw_d, _dt_s);
  m_eulerRatePrev = m_eulerRate;

  m_IMU = {m_States, m_Euler};

  axPrev_fps2 = m_States.specificForceX_fps2;
  ayPrev_fps2 = m_States.specificForceY_fps2;
  azPrev_fps2 = m_States.specificForceZ_fps2;
  gxPrev_dps = m_States.angularRateX_dps;
  gyPrev_dps = m_States.angularRateY_dps;
  gzPrev_dps = m_States.angularRateZ_dps;
}

void IMU::processStatesRaw() {
  readVector(BNO055_ACCEL_DATA_X_LSB, ax_raw, ay_raw, az_raw);
  readVector(BNO055_GYRO_DATA_X_LSB, gx_raw, gy_raw, gz_raw);
  rotateRaw();
  
double k_Accel = accel_LSB * C.M2FT * accelMagnitudeScaling;
  m_States = {
    gx_raw * gyro_LSB + gyroX_offset_dps,
    gy_raw * gyro_LSB + gyroY_offset_dps,
    gz_raw * gyro_LSB + gyroZ_offset_dps,
    ax_raw * k_Accel,
    ay_raw * k_Accel,
    az_raw * k_Accel
  };
}

void IMU::rotateRaw() {
  ax_raw = (cZ*cY)*ax_raw + (sZ*cY)*ay_raw - (sY)*az_raw;
  ay_raw = (cZ*sY*sX - sZ*cX)*ax_raw + (sZ*sY*sX + cZ*cX)*ay_raw + (cY*sX)*az_raw;
  az_raw = (cZ*sY*cX - sZ*sX)*ax_raw + (sZ*sY*cX - cZ*sX)*ay_raw + (cY*cX)*az_raw;

  gx_raw = (cZ*cY)*gx_raw + (sZ*cY)*gy_raw - (sY)*gz_raw;
  gy_raw = (cZ*sY*sX - sZ*cX)*gx_raw + (sZ*sY*sX + cZ*cX)*gy_raw + (cY*sX)*gz_raw;
  gz_raw = (cZ*sY*cX - sZ*sX)*gx_raw + (sZ*sY*cX - cZ*sX)*gy_raw + (cY*cX)*gz_raw;
}

void IMU::initializeRotMatrix() {
  cX = cos(xRot_d * C.DEG2RAD);
  sX = sin(xRot_d * C.DEG2RAD);
  cY = cos(yRot_d * C.DEG2RAD);
  sY = sin(yRot_d * C.DEG2RAD);
  cZ = cos(zRot_d * C.DEG2RAD);
  sZ = sin(zRot_d * C.DEG2RAD);
}

void IMU::writeRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(BNO055_ADDRESS);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

void IMU::readVector(uint8_t startReg, int16_t &xVector, int16_t &yVector, int16_t &zVector) {
  Wire1.beginTransmission(BNO055_ADDRESS);
  Wire1.write(startReg);
  Wire1.endTransmission();
  Wire1.requestFrom(BNO055_ADDRESS, 6);
  xVector = Wire1.read() | (Wire1.read() << 8);
  yVector = Wire1.read() | (Wire1.read() << 8);
  zVector = Wire1.read() | (Wire1.read() << 8);
}
