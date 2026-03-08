#include "TST/TST.h"
#include "MSN/MSN.h"
#include "IMU/IMU.h"
#include "GPS/GPS.h"
#include "NAV/NAV.h"
#include "GDC/GDC.h"
#include "APT/APT.h"
#include "CFG/CFG.h"
#include "TM/TM.h"

CFG cfg;
CFG::fcsParams fcsParams = cfg.getFcsParams();
int nextTask_ms = 0;
int nextSlowTask_ms = 0;
int now_ms;
int dt_ms = 0;
int dtSlow_ms = 0;
int endTime_ms = 0;
int sampleRateGPS_Hz = 5;

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
GPS gps(sampleRateGPS_Hz);
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
  dtSlow_ms = static_cast<int>(1000 / fcsParams.sampleRateLo_Hz);
  nextTask_ms = millis() + dt_ms;
  nextSlowTask_ms = millis() + dtSlow_ms;
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
    if (now_ms >= nextSlowTask_ms)
    {
      printStatus(serialPort, now_ms, endTime_ms, tst, msn, imu, gps, nav, gdc, apt);
      // printStatusBin(serialPort, now_ms, endTime_ms, tst, msn, imu, gps, nav, gdc, apt);
      nextSlowTask_ms += dtSlow_ms;
    }
    nextTask_ms += dt_ms;
  };
};
