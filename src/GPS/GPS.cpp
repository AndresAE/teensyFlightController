#include "GPS.h"

String sentence = "";

GPS::GPS()
{
  // Constructor logic if needed
}

void GPS::begin()
{
  Serial3.begin(gpsParams.gps_baud);
  while (!Serial3)
    ; // Wait for serial
  delay(gpsParams.boot_time_ms);
}

void GPS::step()
{
  processInputs();
}

void GPS::processInputs()
{
  while (Serial3.available())
  {
    char c = Serial3.read();
    if (c == '\n' || c == '\r')
    {
      if (sentence.startsWith("$GNGGA"))
      {
        processGNGGA(sentence);
      }
      sentence = "";
    }
    else
    {
      sentence += c;
    }
  }
}

void GPS::processGNGGA(String s)
{
  String fields[15];
  splitNMEA(s, fields, 15);

  m_LLA = {
      convertToDecimalDegrees(fields[2], fields[3]),
      convertToDecimalDegrees(fields[4], fields[5]),
      (fields[9].toFloat() + fields[10].toFloat()) * C.M2FT};

  m_ECEF = C.lla2ecef(m_LLA);

  m_GPS = {
      m_LLA,
      m_ECEF,
      0.0,
      fields[6].toInt(),
      fields[7].toInt(),
      fields[8].toFloat()};
}

void GPS::splitNMEA(String s, String *fields, int maxFields)
{
  int index = 0;
  for (int i = 0; i < maxFields; i++)
  {
    int commaIndex = s.indexOf(',', index);
    if (commaIndex == -1)
    {
      fields[i] = s.substring(index);
      break;
    }
    fields[i] = s.substring(index, commaIndex);
    index = commaIndex + 1;
  }
}

double GPS::convertToDecimalDegrees(String raw, String dir)
{
  if (raw.length() < 6)
    return 0.0;
  int dotIndex = raw.indexOf('.');
  int degLength = dotIndex - 2;
  double deg = raw.substring(0, degLength).toFloat();
  double min = raw.substring(degLength).toFloat();
  double decimal = deg + (min / C.MIN2SEC);
  if (dir == "S" || dir == "W")
    decimal *= -1;
  return decimal;
}
