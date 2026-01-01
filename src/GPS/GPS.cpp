#include "GPS.h"

String sentence = "";

GPS::GPS() {
    // Constructor logic if needed
}

void GPS::begin() {
  Serial3.begin(gps_baud);
  while (!Serial3); // Wait for serial
  delay(boot_time_ms);
}

void GPS::step() {
  processInputs();
}

void GPS::processInputs() {
  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\n' || c == '\r') {
      if (sentence.startsWith("$GNGGA")) {
        processGNGGA(sentence);
      }
      sentence = "";
    } else {
      sentence += c;
    }
  }
}

void GPS::processGNGGA(String s) {
  String fields[15];
  splitNMEA(s, fields, 15);

  m_LLA = {
    convertToDecimalDegrees(fields[2], fields[3]),
    convertToDecimalDegrees(fields[4], fields[5]),
    (fields[9].toFloat() + fields[10].toFloat()) * C.M2FT
  };

  m_ECEF = lla2ecef(m_LLA);

  m_GPS = {
    m_LLA,
    m_ECEF,
    0.0,
    fields[6].toInt(),
    fields[7].toInt(),
    fields[8].toFloat()
  };
}

void GPS::splitNMEA(String s, String* fields, int maxFields) {
  int index = 0;
  for (int i = 0; i < maxFields; i++) {
    int commaIndex = s.indexOf(',', index);
    if (commaIndex == -1) {
      fields[i] = s.substring(index);
      break;
    }
    fields[i] = s.substring(index, commaIndex);
    index = commaIndex + 1;
  }
}

double GPS::convertToDecimalDegrees(String raw, String dir) {
  if (raw.length() < 6) return 0.0;
  int dotIndex = raw.indexOf('.');
  int degLength = dotIndex - 2;
  double deg = raw.substring(0, degLength).toFloat();
  double min = raw.substring(degLength).toFloat();
  double decimal = deg + (min / C.MIN2SEC);
  if (dir == "S" || dir == "W") decimal *= -1;
  return decimal;
}

double GPS::primeVerticalRadiusOfCurvature(double latG_d) {
    return C.WGS84_r_ft / (pow(1 - pow(C.WGS84_e, 2) * (pow(sin(C.DEG2RAD * latG_d), 2)), 0.5));
}

GPS::busEcef GPS::lla2ecef(busLLA lla_ddft) {
    n_ft = primeVerticalRadiusOfCurvature(lla_ddft.latG_d);
    m_ECEF.x_ft = (n_ft + lla_ddft.hae_ft) * cos(C.DEG2RAD * lla_ddft.latG_d) * cos(C.DEG2RAD * lla_ddft.lon_d);
    m_ECEF.y_ft = (n_ft + lla_ddft.hae_ft) * cos(C.DEG2RAD * lla_ddft.latG_d) * sin(C.DEG2RAD * lla_ddft.lon_d);
    m_ECEF.z_ft = ((1 - pow(C.WGS84_e, 2))*n_ft + lla_ddft.hae_ft) * sin(C.DEG2RAD * lla_ddft.latG_d);
    return m_ECEF;
}
