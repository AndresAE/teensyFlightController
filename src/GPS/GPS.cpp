#include "GPS.h"

String sentence = "";

GPS::GPS(int sampleRate_Hz)
    : sampleRate_Hz(sampleRate_Hz), gpsParams(cfg.getGpsParams())
{
  dt_s = 1 / static_cast<double>(sampleRate_Hz);
}

void GPS::begin()
{
  Serial3.begin(gpsParams.gps_baud);
  while (!Serial3)
  {
  }; // Wait for serial

  C.calcAlpha(gpsParams.cutoff_Hz, dt_s);

  sendUBX(UBX_DISABLE_GLL, sizeof(UBX_DISABLE_GLL));
  sendUBX(UBX_DISABLE_GSA, sizeof(UBX_DISABLE_GSA));
  sendUBX(UBX_DISABLE_GSV, sizeof(UBX_DISABLE_GSV));
  sendUBX(UBX_DISABLE_RMC, sizeof(UBX_DISABLE_RMC));
  sendUBX(UBX_DISABLE_VTG, sizeof(UBX_DISABLE_VTG));

  sendUBX(UBX_ENABLE_GGA, sizeof(UBX_ENABLE_GGA));
  sendUBX(UBX_RATE_5HZ, sizeof(UBX_RATE_5HZ));
  sendUBX(UBX_SAVE, sizeof(UBX_SAVE));

  delay(gpsParams.boot_time_ms);
}

void GPS::step()
{
  _velocityValid_b = false;
  processInputs();

  if (m_GPS.timeStamp_s - _timeStampPrev_s <= gpsParams.dt_gps_s)
  {
    calcVelocity(m_GPS.ecef_ft.x_ft, _ecef_prev[0], m_vECEF.x_fps);
    calcVelocity(m_GPS.ecef_ft.y_ft, _ecef_prev[1], m_vECEF.y_fps);
    calcVelocity(m_GPS.ecef_ft.z_ft, _ecef_prev[2], m_vECEF.z_fps);
    _velocityValid_b = true;
  }
  _ecef_prev[0] = m_GPS.ecef_ft.x_ft;
  _ecef_prev[1] = m_GPS.ecef_ft.y_ft;
  _ecef_prev[2] = m_GPS.ecef_ft.z_ft;
  _timeStampPrev_s = m_GPS.timeStamp_s;
}

void GPS::processInputs()
{
  while (Serial3.available())
  {
    _c = Serial3.read();
    if (_c == '\n' || _c == '\r')
    {
      if (sentence.startsWith("$GNGGA"))
      {
        processGNGGA(sentence);
      }
      sentence = "";
    }
    else
    {
      sentence += _c;
    }
  }
}

void GPS::processGNGGA(String s)
{
  splitNMEA(s, _fields, 15);

  m_LLA = {
      convertToDecimalDegrees(_fields[2], _fields[3]),
      convertToDecimalDegrees(_fields[4], _fields[5]),
      (_fields[9].toFloat() + _fields[10].toFloat()) * C.M2FT};

  m_ECEF = C.lla2ecef(m_LLA);

  m_GPS = {
      m_LLA,
      m_ECEF,
      m_vECEF,
      _fields[1].toFloat(),
      _fields[6].toInt(),
      _fields[7].toInt(),
      _fields[8].toFloat(),
      _velocityValid_b};
}

void GPS::splitNMEA(String s, String *fields, int maxFields)
{
  _index = 0;
  for (int i = 0; i < maxFields; i++)
  {
    int commaIndex = s.indexOf(',', _index);
    if (commaIndex == -1)
    {
      fields[i] = s.substring(_index);
      break;
    }
    fields[i] = s.substring(_index, commaIndex);
    _index = commaIndex + 1;
  }
}

double GPS::convertToDecimalDegrees(String raw, String dir)
{
  if (raw.length() < 6)
    return 0.0;
  _dotIndex = raw.indexOf('.');
  _degLength = _dotIndex - 2;
  _deg = raw.substring(0, _degLength).toFloat();
  _min = raw.substring(_degLength).toFloat();
  _decimal = _deg + (_min / C.MIN2SEC);
  if (dir == "S" || dir == "W")
    _decimal *= -1;
  return _decimal;
}

void GPS::sendUBX(const uint8_t *msg, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    Serial3.write(msg[i]);
  }
  Serial3.flush();
  delay(_sendUBXDelay_ms);
}

void GPS::calcVelocity(double posCurrent_ft, double posPrevious_ft, double vel_ftps)
{
  vel_ftps = C.Common::filteredDerivative(posCurrent_ft, posPrevious_ft, vel_ftps, _alpha, dt_s);
}
