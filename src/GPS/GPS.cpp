#include "GPS.h"

String sentence = "";

constexpr uint8_t UBX_DISABLE_GLL[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x2B};

constexpr uint8_t UBX_DISABLE_GSA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2C};

constexpr uint8_t UBX_DISABLE_GSV[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x2D};

constexpr uint8_t UBX_DISABLE_RMC[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x2E};

constexpr uint8_t UBX_DISABLE_VTG[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x2F};

constexpr uint8_t UBX_ENABLE_GGA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x2A};

constexpr uint8_t UBX_RATE_5HZ[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

constexpr uint8_t UBX_SAVE[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31};

GPS::GPS()
    : gpsParams(cfg.getGpsParams())
{
  // Constructor logic if needed
}

void GPS::begin()
{
  Serial3.begin(gpsParams.gps_baud);
  while (!Serial3)
  {
  }; // Wait for serial

  sendUBX(UBX_DISABLE_GLL, sizeof(UBX_DISABLE_GLL));
  sendUBX(UBX_DISABLE_GSA, sizeof(UBX_DISABLE_GSA));
  sendUBX(UBX_DISABLE_GSV, sizeof(UBX_DISABLE_GSV));
  sendUBX(UBX_DISABLE_RMC, sizeof(UBX_DISABLE_RMC));
  sendUBX(UBX_DISABLE_VTG, sizeof(UBX_DISABLE_VTG));

  sendUBX(UBX_ENABLE_GGA, sizeof(UBX_ENABLE_GGA));
  sendUBX(UBX_RATE_5HZ, sizeof(UBX_RATE_5HZ));
  sendUBX(UBX_SAVE, sizeof(UBX_SAVE));

  Serial.println("GT-U7 configured for 5 Hz, GNGGA only");

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
      fields[1].toFloat(),
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

void GPS::sendUBX(const uint8_t *msg, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    Serial3.write(msg[i]);
  }
  Serial3.flush();
  delay(50);
}
