#include <cmath>
// #include <Arduino.h>
// #include <cstdint>
#include "../src/CMN/CMN.h"
#include "iostream"
#include <iomanip>

CMN C;

const int _sendUBXDelay_ms = 50;
char _c;
std::string _fields[15];
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

// bus definitions
struct busGPS
{
    CMN::busLLA lla_ddft;
    CMN::busEcef ecef_ft;
    CMN::busvEcef vBE_E_fps;
    double timeStamp_s;
    int fixQuality;
    int nSatellites;
    double hdop;
    bool velocityValid_b;
};

CMN::busLLA m_LLA = {0.0, 0.0, 0.0};
CMN::busEcef m_ECEF = {0.0, 0.0, 0.0};
CMN::busvEcef m_vECEF = {0.0, 0.0, 0.0};
busGPS m_GPS = {m_LLA, m_ECEF, m_vECEF, 0.0, 0, 0, 0.0, false};

void splitNMEA(std::string s, std::string *fields, int maxFields)
{
    _index = 0;
    for (int i = 0; i < maxFields; i++)
    {
        int commaIndex = s.find(',', _index);
        if (commaIndex == -1)
        {
            fields[i] = s.substr(_index);
            break;
        }
        fields[i] = s.substr(_index, commaIndex - _index);
        _index = commaIndex + 1;
    }
}

double convertToDecimalDegrees(std::string raw, std::string dir)
{
    if (raw.length() < 6)
        return 0.0;
    _dotIndex = raw.find('.');
    _degLength = _dotIndex - 2;
    _deg = static_cast<double>(std::stoi(raw.substr(0, _degLength)));
    _min = static_cast<double>(std::stof(raw.substr(_degLength)));
    _decimal = _deg + (_min / C.MIN2SEC);
    if (dir == "S" || dir == "W")
        _decimal *= -1;
    return _decimal;
}

int main()
{

    std::string sentence = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    splitNMEA(sentence, _fields, 15);

    std::cout << "Parsed fields:\n";
    for (int i = 0; i < 15; i++)
    {
        std::cout << "Field " << i << ": " << _fields[i] << "\n";
    }

    m_LLA = {
        convertToDecimalDegrees(_fields[2], _fields[3]),
        convertToDecimalDegrees(_fields[4], _fields[5]),
        (static_cast<double>(std::stof(_fields[9])) + static_cast<double>(std::stof(_fields[11]))) * C.M2FT};

    m_ECEF = C.lla2ecef(m_LLA);

    m_GPS = {
        m_LLA,
        m_ECEF,
        m_vECEF,
        static_cast<double>(std::stof(_fields[1])),
        static_cast<int>(std::stoi(_fields[6])),
        static_cast<int>(std::stoi(_fields[7])),
        static_cast<double>(std::stof(_fields[8])),
        _velocityValid_b};

    std::cout << std::setprecision(17) << "LLA: " << m_GPS.lla_ddft.latG_d << ", " << m_GPS.lla_ddft.lon_d << ", " << m_GPS.lla_ddft.hae_ft << "\n";
    std::cout << "ECEF: " << m_GPS.ecef_ft.x_ft << ", " << m_GPS.ecef_ft.y_ft << ", " << m_GPS.ecef_ft.z_ft << "\n";
}

// g++ -std=c++11 navTest.cpp src/CMN/CMN.cpp -I src/CMN -o test_app
// ./test_app
// Expected output:
// Result of matrix multiplication:
// 38
// 92
// 146