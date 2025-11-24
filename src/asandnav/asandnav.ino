// #include <SPI.h>
// #include <SD.h>
// #include "Config.h"
#include "IMU.h"
#include "GPS.h"
// #include "EKF.h"

// Config config;
int sampleRate_Hz = 50;
double dt_ms = 1000 / (static_cast<double>(sampleRate_Hz));
double tStart_ms, tEnd_ms;

IMU imu(sampleRate_Hz, 0.0);
GPS gps;
// EKF ekf;

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial on some boards

  // // Initialize SD card
  // if (!SD.begin(BUILTIN_SDCARD)) {
  //   Serial.println("SD card init failed!");
  //   while (1);
  // }

  // // Load configuration
  // if (!config.load("/config.toml")) {
  //   Serial.println("Failed to load config.toml");
  //   while (1);
  // }

  // Initialize sensors with config
  Serial.println("starting IMU...");
  imu.begin();

  Serial.println("starting GPS...");
  gps.begin();

  // Optionally initialize EKF with config values
  // ekf.setNoiseParameters(config.ekf.accelNoise, config.ekf.gpsPosNoise, config.ekf.gpsVelNoise);
  // ekf.setInitialCovariance(config.ekf.initialCov);

  // lastUpdate = millis();
}

void loop() {
  tStart_ms = millis();

  imu.step();
  Serial.println("----- IMU -----");
  Serial.print("ax: "); Serial.println(imu.m_IMU.states.specificForceX_fps2);
  Serial.print("ay: "); Serial.println(imu.m_IMU.states.specificForceY_fps2);
  Serial.print("az: "); Serial.println(imu.m_IMU.states.specificForceZ_fps2);
  Serial.print("gx: "); Serial.println(imu.m_IMU.states.angularRateX_dps);
  Serial.print("gy: "); Serial.println(imu.m_IMU.states.angularRateY_dps);
  Serial.print("gz: "); Serial.println(imu.m_IMU.states.angularRateZ_dps);
  Serial.print("roll: "); Serial.println(imu.m_IMU.euler.roll_d);
  Serial.print("pitch: "); Serial.println(imu.m_IMU.euler.pitch_d);
  Serial.print("yaw: "); Serial.println(imu.m_IMU.euler.yaw_d);

  gps.step();
  Serial.println("----- GNGGA -----");
  Serial.print("Latitude: "); Serial.println(gps.m_GPS.lla_ddft.latG_d);
  Serial.print("Longitude: "); Serial.println(gps.m_GPS.lla_ddft.lon_d);
  Serial.print("HAE: "); Serial.println(gps.m_GPS.lla_ddft.hae_ft);
  Serial.print("Fix Quality: "); Serial.println(gps.m_GPS.fixQuality);
  Serial.print("Satellites: "); Serial.println(gps.m_GPS.nSatellites);
  Serial.print("HDOP: "); Serial.println(gps.m_GPS.hdop);
  Serial.print("ECEF X: "); Serial.println(gps.m_ECEF.x_ft);
  Serial.print("ECEF Y: "); Serial.println(gps.m_ECEF.y_ft);
  Serial.print("ECEF Z: "); Serial.println(gps.m_ECEF.z_ft);

  tEnd_ms = millis();
  delay(dt_ms - (tEnd_ms - tStart_ms));
  // EKF prediction
  // ekf.predict(imu.getAX(), imu.getAY(), dt);

  // EKF update if GPS data is fresh
  // if (gps.isUpdated()) {
  //   ekf.update(gps.getX(), gps.getY(), gps.getVX(), gps.getVY());
  // }

  // Output fused state
  // EKFState state = ekf.getState();
  // Serial.print("X: "); Serial.print(state.x[0], 6);
  // Serial.print(" Y: "); Serial.print(state.x[1], 6);
  // Serial.print(" VX: "); Serial.print(state.x[2], 3);
  // Serial.print(" VY: "); Serial.println(state.x[3], 3);
}
