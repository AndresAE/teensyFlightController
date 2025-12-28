// #include <SPI.h>
// #include <SD.h>
// #include "Config.h"
#include "IMU.h"
#include "GPS.h"
// #include "EKF.h"

// Config config;
int next100HzTask = 0;
int next10HzTask = 0;
int now_ms;
bool iseTM_b = true;  // set to true for TM
int tmDelay_ms = 5000;  // hold time for TM startup
int serialBaud = iseTM_b ? 57600 : 9600;
Stream* serialPort;

IMU imu(50, 0.0);
GPS gps;
// EKF ekf;

void setup() {
  if (iseTM_b) {
    Serial5.begin(serialBaud);
    serialPort = &Serial5;

    serialPort->println("----- Holding for TM -----");
    delay(tmDelay_ms);
  } else {
    Serial.begin(serialBaud);
    serialPort = &Serial;
  }
  serialPort->println("----- Starting IMU -----");
  imu.begin();
  serialPort->println("----- Starting GPS -----");
  gps.begin();
}

void loop() {
  now_ms = millis();
  if (now_ms>=next100HzTask) {
    fastTask();
    next100HzTask += 10;
  };
  if (now_ms>=next10HzTask) {
    slowTask();
    next10HzTask += 100;
  };
}

void fastTask() {
  imu.step();
};

void slowTask() {
  gps.step();
    serialPort->println("----- Time -----");
    serialPort->print("met_s: "); serialPort->println(now_ms / 1000.0);
    serialPort->println("----- IMU -----");
    serialPort->print("ax: "); serialPort->println(imu.m_IMU.states.specificForceX_fps2);
    serialPort->print("ay: "); serialPort->println(imu.m_IMU.states.specificForceY_fps2);
    serialPort->print("az: "); serialPort->println(imu.m_IMU.states.specificForceZ_fps2);
    serialPort->print("gx: "); serialPort->println(imu.m_IMU.states.angularRateX_dps);
    serialPort->print("gy: "); serialPort->println(imu.m_IMU.states.angularRateY_dps);
    serialPort->print("gz: "); serialPort->println(imu.m_IMU.states.angularRateZ_dps);
    serialPort->print("roll: "); serialPort->println(imu.m_IMU.euler.roll_d);
    serialPort->print("pitch: "); serialPort->println(imu.m_IMU.euler.pitch_d);
    serialPort->print("yaw: "); serialPort->println(imu.m_IMU.euler.yaw_d);
    serialPort->println("----- GNGGA -----");
    serialPort->print("Latitude: "); serialPort->println(gps.m_GPS.lla_ddft.latG_d);
    serialPort->print("Longitude: "); serialPort->println(gps.m_GPS.lla_ddft.lon_d);
    serialPort->print("HAE: "); serialPort->println(gps.m_GPS.lla_ddft.hae_ft);
    serialPort->print("ECEF X: "); serialPort->println(gps.m_ECEF.x_ft);
    serialPort->print("ECEF Y: "); serialPort->println(gps.m_ECEF.y_ft);
    serialPort->print("ECEF Z: "); serialPort->println(gps.m_ECEF.z_ft);
    serialPort->print("Fix Quality: "); serialPort->println(gps.m_GPS.fixQuality);
    serialPort->print("Satellites: "); serialPort->println(gps.m_GPS.nSatellites);
    serialPort->print("HDOP: "); serialPort->println(gps.m_GPS.hdop);
};
