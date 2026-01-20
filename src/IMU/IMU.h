#ifndef IMU_H
#define IMU_H

#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../config/config.h"

class IMU
{
public:
  IMU(int sampleRate_Hz, double yaw0_d = 0.0);

  int sampleRate_Hz;
  double yaw0_d, dt_s;

  // public methods
  void begin();
  void step();
  void processStatesRaw();
  void writeRegister(uint8_t reg, uint8_t value);
  void readVector(uint8_t startReg, int16_t &xVector, int16_t &yVector, int16_t &zVector);
  void initializeRotMatrix();
  void rotateRaw();

  // enum definitions
  enum imuType
  {
    IMU_ADAFRUIT = 0,
    IMU_TEYLETEN = 1,
    IMU_MPU6050 = 2,
    IMU_MPU9250 = 3
  };

  // bus definitions
  struct busStates
  {
    double angularRateX_dps;
    double angularRateY_dps;
    double angularRateZ_dps;
    double specificForceX_fps2;
    double specificForceY_fps2;
    double specificForceZ_fps2;
  };

  struct busEuler
  {
    double roll_d;
    double pitch_d;
    double yaw_d;
  };

  struct busEulerRates
  {
    double phiDot_dps;
    double thetaDot_dps;
    double psiDot_dps;
  };

  struct busIMU
  {
    busStates states;
    busEuler euler;
  };

  busEulerRates m_eulerRate, m_eulerRatePrev;
  busStates m_States = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  busEuler m_Euler = {0, 0, yaw0_d};
  busIMU m_IMU = {m_States, m_Euler};

private:
  Common C;
  Config cfg;

  const Config::imuParams &imuParams = cfg.getImuParams();

  double _gyroX_offset_dps = 0.0, _gyroY_offset_dps = 0.0, _gyroZ_offset_dps = 0.0;
  double _accelMagnitudeScaling = 0.0;
  double _tmpPitch_d = 0.0;

  int16_t _axRaw, _ayRaw, _azRaw;
  int16_t _gxRaw, _gyRaw, _gzRaw;

  double _axPrev_fps2 = 0.0, _ayPrev_fps2 = 0.0, _azPrev_fps2 = 0.0;
  double _gxPrev_dps = 0.0, _gyPrev_dps = 0.0, _gzPrev_dps = 0.0;

  double _alphaAx, _alphaAy, _alphaAz;
  double _alphaGx, _alphaGy, _alphaGz;

  double _cX, _sX, _cY, _sY, _cZ, _sZ;

  int8_t BNO055_ADDRESS = 0x29;
  int8_t BNO055_OPR_MODE = 0x3D;
  int8_t BNO055_PWR_MODE = 0x3E;
  int8_t BNO055_PAGE_ID = 0x07;
  int8_t BNO055_ACCEL_DATA_X_LSB = 0x08;
  int8_t BNO055_GYRO_DATA_X_LSB = 0x14;
};

#endif // IMU_H
