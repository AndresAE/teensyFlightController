#include "Config.h"
#include <SD.h> // or SPIFFS for ESP boards

bool Config::load(const char* filename) {
  File file = SD.open(filename);
  if (!file) return false;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.startsWith("#") || line.length() == 0) continue;

    if (line.startsWith("accel_noise")) ekf.accelNoise = line.substring(line.indexOf('=') + 1).toFloat();
    else if (line.startsWith("gps_pos_noise")) ekf.gpsPosNoise = line.substring(line.indexOf('=') + 1).toFloat();
    else if (line.startsWith("gps_vel_noise")) ekf.gpsVelNoise = line.substring(line.indexOf('=') + 1).toFloat();
    else if (line.startsWith("initial_covariance")) ekf.initialCov = line.substring(line.indexOf('=') + 1).toFloat();
    else if (line.startsWith("i2c_address")) imu.i2cAddress = strtol(line.substring(line.indexOf('=') + 1).c_str(), nullptr, 16);
    else if (line.startsWith("scale_factor")) imu.scaleFactor = line.substring(line.indexOf('=') + 1).toFloat();
    else if (line.startsWith("baud_rate")) gps.baudRate = line.substring(line.indexOf('=') + 1).toInt();
    else if (line.startsWith("update_rate_hz")) gps.updateRateHz = line.substring(line.indexOf('=') + 1).toInt();
  }

  file.close();
  return true;
}