#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>
#include "../MSN/MSN.h"
#include "../IMU/IMU.h"
#include "../GPS/GPS.h"
#include "../NAV/NAV.h"
#include "../GDC/GDC.h"
#include "../APT/APT.h"

void printStatus(Stream &serialPort, int now_ms, int endTime_ms, MSN &msn, IMU &imu, GPS &gps, NAV &nav, GDC &gdc, APT &apt);

#endif // STATUS_H
