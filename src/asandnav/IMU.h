#include <cmath>
#include <Arduino.h>
#include "Common.h"

class IMU {
public:
  IMU(int sampleRate_Hz=50, double yaw0_d=0.0);

  int imu_baud = 115200;
  int boot_time_ms = 2000;
  int nInit = 100; 
  double accel_LSB = 0.00981;
  double gyro_LSB = 0.0625;

  #define BNO055_ADDRESS          0x28
  #define BNO055_OPR_MODE         0x3D
  #define BNO055_PWR_MODE         0x3E
  #define BNO055_PAGE_ID          0x07
  #define BNO055_ACCEL_DATA_X_LSB 0x08
  #define BNO055_GYRO_DATA_X_LSB  0x14 

  double gyroX_offset_dps = 0.0, gyroY_offset_dps = 0.0, gyroZ_offset_dps = 0.0;
  double accelMagnitudeScaling = 1.0;

  double cutoffAx_Hz = 5, cutoffAy_Hz = 5, cutoffAz_Hz = 5;
  double cutoffGx_Hz = 5, cutoffGy_Hz = 5, cutoffGz_Hz = 5;

  double xRot_d = 180.0;
  double yRot_d = 0.0;
  double zRot_d = 0.0;

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
  
  // bus definitions
  struct busStates {
    double angularRateX_dps;
    double angularRateY_dps;
    double angularRateZ_dps;
    double specificForceX_fps2;
    double specificForceY_fps2;
    double specificForceZ_fps2;
  };

  struct busEuler {
    double roll_d;
    double pitch_d;
    double yaw_d;
  };

  struct busEulerRates {
    double phiDot_dps;
    double thetaDot_dps;
    double psiDot_dps;
  };

  struct busIMU {
    busStates states;
    busEuler euler;
  };

    busEulerRates m_eulerRate, m_eulerRatePrev;
    busStates m_States = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    busEuler m_Euler = {0, 0, yaw0_d};
    busIMU m_IMU = {m_States, m_Euler};

private:
  Common C;

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
};
