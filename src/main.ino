
#include "MSN/MSN.h"
#include "IMU/IMU.h"
#include "GPS/GPS.h"
#include "GDC/GDC.h"
#include "APT/APT.h"

int next100HzTask = 0;
int next10HzTask = 0;
int sampleRate_Hz = 100;
int now_ms;
bool useTM_b = false;  // set to true for TM
int tmDelay_ms = 5000; // hold time for TM startup
int serialBaud = useTM_b ? 57600 : 9600;
Stream *serialPort;

int endTime = 0;

MSN msn(sampleRate_Hz);
IMU imu(sampleRate_Hz);
GPS gps;
GDC gdc(sampleRate_Hz);
APT apt(sampleRate_Hz);

void setup()
{
  if (useTM_b)
  {
    Serial5.begin(serialBaud);
    serialPort = &Serial5;

    serialPort->println("----- Holding for TM -----");
    delay(tmDelay_ms);
  }
  else
  {
    Serial.begin(serialBaud);
    serialPort = &Serial;
  }

  serialPort->println("----- Starting MSN -----");
  msn.attachBus(&imu.m_IMU);
  msn.begin();
  serialPort->println("----- Starting NAV -----");
  imu.begin();
  gps.begin();
  serialPort->println("----- Starting GDC -----");
  gdc.begin();
  serialPort->println("----- Starting APT -----");
  apt.attachBus(&gdc.m_GDC, &imu.m_IMU);
  apt.begin();
}

void loop()
{
  now_ms = millis();
  if (now_ms >= next100HzTask)
  {
    fastTask();
    next100HzTask += 10;
  };
  if (now_ms >= next10HzTask)
  {
    slowTask();
    next10HzTask += 100;
  }
  endTime = millis();
  serialPort->print("stepTime_ms=");
  serialPort->println(endTime - now_ms);
};

void fastTask()
{
  msn.step();
  imu.step();
  gdc.step();
  apt.step();
  printStatus();
};

void slowTask()
{
  gps.step();
}

void printStatus()
{
  // print status to serial port
  serialPort->print("time: ");
  serialPort->print("met_s=");
  serialPort->println(now_ms / 1000.0);

  serialPort->print("msn: ");
  serialPort->print("phase=");
  serialPort->println(msn.missionPhase_e);

  serialPort->print("imu: ");
  serialPort->print("ax_fps2=");
  serialPort->print(imu.m_IMU.states.specificForceX_fps2);
  serialPort->print(", ");
  serialPort->print("ay_fps2=");
  serialPort->print(imu.m_IMU.states.specificForceY_fps2);
  serialPort->print(", ");
  serialPort->print("az_fps2=");
  serialPort->print(imu.m_IMU.states.specificForceZ_fps2);
  serialPort->print(", ");
  serialPort->print("gx_dps=");
  serialPort->print(imu.m_IMU.states.angularRateX_dps);
  serialPort->print(", ");
  serialPort->print("gy_dps=");
  serialPort->print(imu.m_IMU.states.angularRateY_dps);
  serialPort->print(", ");
  serialPort->print("gz_dps=");
  serialPort->print(imu.m_IMU.states.angularRateZ_dps);
  serialPort->print(", ");
  serialPort->print("roll_d=");
  serialPort->print(imu.m_IMU.euler.roll_d);
  serialPort->print(", ");
  serialPort->print("pitch_d=");
  serialPort->print(imu.m_IMU.euler.pitch_d);
  serialPort->print(", ");
  serialPort->print("yaw_d=");
  serialPort->println(imu.m_IMU.euler.yaw_d);

  serialPort->print("gps: ");
  serialPort->print("latG_d=");
  serialPort->print(gps.m_GPS.lla_ddft.latG_d);
  serialPort->print(", ");
  serialPort->print("lon_d=");
  serialPort->print(gps.m_GPS.lla_ddft.lon_d);
  serialPort->print(", ");
  serialPort->print("HAE_ft=");
  serialPort->print(gps.m_GPS.lla_ddft.hae_ft);
  serialPort->print(", ");
  serialPort->print("ECEF_X=");
  serialPort->print(gps.m_ECEF.x_ft);
  serialPort->print(", ");
  serialPort->print("ECEF_Y=");
  serialPort->print(gps.m_ECEF.y_ft);
  serialPort->print(", ");
  serialPort->print("ECEF_Z=");
  serialPort->print(gps.m_ECEF.z_ft);
  serialPort->print(", ");
  serialPort->print("fixQual=");
  serialPort->print(gps.m_GPS.fixQuality);
  serialPort->print(", ");
  serialPort->print("nSat=");
  serialPort->print(gps.m_GPS.nSatellites);
  serialPort->print(", ");
  serialPort->print("HDOP=");
  serialPort->println(gps.m_GPS.hdop);

  serialPort->print("gdc: ");
  serialPort->print("pbCmd_dps=");
  serialPort->print(gdc.m_GDC.rollRateCmd_dps);
  serialPort->print(", ");
  serialPort->print("nyCmd_B_fps2=");
  serialPort->print(gdc.m_GDC.nyCmd_B_fps2);
  serialPort->print(", ");
  serialPort->print("nzCmd_B_fps2=");
  serialPort->print(gdc.m_GDC.nzCmd_B_fps2);
  serialPort->print(", ");
  serialPort->print("throttleCmd_pct=");
  serialPort->println(gdc.m_GDC.throttleCmd_pct);

  serialPort->print("apt: ");
  for (int ii = 0; ii < static_cast<int>(sizeof(apt.m_APT.effectorCmd) / sizeof(apt.m_APT.effectorCmd[0])); ii++)
  {
    serialPort->print("effector");
    serialPort->print(ii);
    serialPort->print("_cmd: ");
    if (ii < static_cast<int>(sizeof(apt.m_APT.effectorCmd) / sizeof(apt.m_APT.effectorCmd[0])) - 1)
    {
      serialPort->print(apt.m_APT.effectorCmd[ii]);
      serialPort->print(", ");
    }
    else
    {
      serialPort->println(apt.m_APT.effectorCmd[ii]);
    }
  }
}
