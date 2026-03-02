#include "TST/TST.h"
#include "MSN/MSN.h"
#include "IMU/IMU.h"
#include "GPS/GPS.h"
#include "NAV/NAV.h"
#include "GDC/GDC.h"
#include "APT/APT.h"
#include "Config/config.h"
#include "Status/Status.h"

Config cfg;
Config::fcsParams fcsParams = cfg.getFcsParams();
int nextTask_ms = 0;
int next5Hz_ms = 0;
int now_ms;
int dt_ms = 0;
int dt5Hz_ms = 0;
int endTime_ms = 0;

Stream &chooseSerial()
{
  if (fcsParams.useTM_b)
  {
    return Serial1;
  }
  else
  {
    return Serial;
  }
}

Stream &serialPort = chooseSerial();
IMU imu(fcsParams.sampleRate_Hz);
GPS gps(fcsParams.sampleRateLo_Hz);
NAV nav(fcsParams.sampleRate_Hz, imu, gps);
TST tst(fcsParams.sampleRate_Hz);
MSN msn(fcsParams.sampleRate_Hz, tst, imu);
GDC gdc(fcsParams.sampleRate_Hz, tst, msn);
APT apt(fcsParams.sampleRate_Hz, tst, msn, gdc, imu);

void setup()
{
  if (fcsParams.useTM_b)
  {
    Serial1.begin(fcsParams.serialBaud);

    serialPort.println("----- Holding for TM -----");
    delay(fcsParams.tmDelay_ms);
  }
  else
  {
    Serial.begin(fcsParams.serialBaud);
  }
  serialPort.println("----- Starting TST -----");
  tst.begin();
  serialPort.println("----- Starting MSN -----");
  msn.begin();
  serialPort.println("----- Starting IMU -----");
  imu.begin();
  serialPort.println("----- Starting GPS -----");
  gps.begin();
  serialPort.println("----- Starting NAV -----");
  nav.begin();
  serialPort.println("----- Starting GDC -----");
  gdc.begin();
  serialPort.println("----- Starting APT -----");
  apt.begin();

  dt_ms = static_cast<int>(1000 / fcsParams.sampleRate_Hz);
  dt5Hz_ms = static_cast<int>(1000 / fcsParams.sampleRateLo_Hz);
  nextTask_ms = millis() + dt_ms;
  next5Hz_ms = millis() + dt5Hz_ms;
  serialPort.println("----- Starting Flight Loop -----");
}

void loop()
{
  now_ms = millis();
  if (now_ms >= nextTask_ms)
  {
    tst.step();
    msn.step();
    imu.step();
    gdc.step();
    apt.step();
    gps.step();
    // nav.step();
    endTime_ms = millis();
    if (endTime_ms >= next5Hz_ms)
    {
      if (fcsParams.useTM_b)
      {
        printStatusTM(serialPort, now_ms, endTime_ms, tst, msn, imu, gps, nav, gdc, apt);
      }
      next5Hz_ms += dt5Hz_ms;
    }
    if (!fcsParams.useTM_b)
    {
      printStatus(serialPort, now_ms, endTime_ms, tst, msn, imu, gps, nav, gdc, apt);
    }
    nextTask_ms += dt_ms;
  };
};
