#pragma once

struct ConfigEKF {
  float accelNoise;
  float gpsPosNoise;
  float gpsVelNoise;
  float initialCov;
};

struct ConfigIMU {
  uint8_t i2cAddress;
  float scaleFactor;
};

struct ConfigGPS {
  int baudRate;
  int updateRateHz;
};

class Config {
public:
  bool load(const char* filename);
  ConfigEKF ekf;
  ConfigIMU imu;
  ConfigGPS gps;
};