#include "MSN/MSN.h"
#include "IMU/IMU.h"
#include "GPS/GPS.h"
#include "GDC/GDC.h"
#include "APT/APT.h"
#include "Config/config.h"
#include "Status/Status.h"

Config cfg;
Config::fcsParams fcsParams = cfg.getFcsParams();
int next100HzTask = 0;
int next10HzTask = 0;
int now_ms;
int dt_ms = 0;
int endTime_ms = 0;

Stream &chooseSerial()
{
  if (fcsParams.useTM_b)
  {
    return Serial5;
  }
  else
  {
    return Serial;
  }
}

Stream &serialPort = chooseSerial();
IMU imu(fcsParams.sampleRate_Hz);
GPS gps;
GDC gdc(fcsParams.sampleRate_Hz);
MSN msn(fcsParams.sampleRate_Hz, imu);
APT apt(fcsParams.sampleRate_Hz, gdc, imu);

void setup()
{
  if (fcsParams.useTM_b)
  {
    Serial5.begin(fcsParams.serialBaud);

    serialPort.println("----- Holding for TM -----");
    delay(fcsParams.tmDelay_ms);
  }
  else
  {
    Serial.begin(fcsParams.serialBaud);
  }

  serialPort.println("----- Starting MSN -----");
  msn.begin();
  serialPort.println("----- Starting NAV -----");
  imu.begin();
  gps.begin();
  serialPort.println("----- Starting GDC -----");
  gdc.begin();
  serialPort.println("----- Starting APT -----");
  apt.begin();

  dt_ms = static_cast<int>(1000 / fcsParams.sampleRate_Hz);
  next100HzTask = millis() + dt_ms;
}

void loop()
{
  now_ms = millis();
  if (now_ms >= next100HzTask)
  {
    msn.step();
    imu.step();
    gps.step();
    gdc.step();
    apt.step();
    endTime_ms = millis();
    printStatus(serialPort, now_ms, endTime_ms, msn, imu, gps, gdc, apt);
    next100HzTask += dt_ms;
  };
};
