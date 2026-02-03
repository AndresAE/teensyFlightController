#ifndef GPS_H
#define GPS_H

#include <cmath>
#include <Arduino.h>
#include <cstdint>
#include "../Common/Common.h"
#include "../config/config.h"

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

class GPS
{
public:
    GPS(int sampleRate_Hz);

    int sampleRate_Hz;
    double dt_s;

    // public methods
    void begin();
    void step();

    // bus definitions
    struct busGPS
    {
        Common::busLLA lla_ddft;
        Common::busEcef ecef_ft;
        Common::busvEcef vBE_E_fps;
        double timeStamp_s;
        int fixQuality;
        int nSatellites;
        double hdop;
        bool velocityValid_b;
    };

    Common::busLLA m_LLA = {0.0, 0.0, 0.0};
    Common::busEcef m_ECEF = {0.0, 0.0, 0.0};
    Common::busvEcef m_vECEF = {0.0, 0.0, 0.0};
    busGPS m_GPS = {m_LLA, m_ECEF, m_vECEF, 0.0, 0, 0, 0.0, false};

private:
    Common C;
    Config cfg;

    const int _sendUBXDelay_ms = 50;
    char _c;
    String _fields[15];
    int _index;
    int _dotIndex;
    int _degLength;
    double _deg;
    double _min;
    double _decimal;
    double _alpha;
    double _timeStampPrev_s = 0.0;
    double _ecef_prev[3] = {0.0, 0.0, 0.0};
    bool _velocityValid_b = false;

    const Config::gpsParams &gpsParams;

    void processInputs();
    double convertToDecimalDegrees(String raw, String dir);
    void splitNMEA(String s, String *fields, int maxFields);
    void processGNGGA(String s);
    void sendUBX(const uint8_t *msg, uint8_t len);
    void calcVelocity(double posCurrent_ft, double posPrevious_ft, double vel_ftps);
};

#endif // GPS_H
