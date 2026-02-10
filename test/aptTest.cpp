#include "../src/APT/APT.h"
#include "../src/TST/TST.h"
#include "../src/MSN/MSN.h"
#include "../src/GDC/GDC.h"
#include "../src/IMU/IMU.h"
#include "iostream"

int sampleRate_Hz = 50;

TST tst(sampleRate_Hz);
IMU imu(sampleRate_Hz);
MSN msn(sampleRate_Hz, tst, imu);
GDC gdc(sampleRate_Hz, tst, msn);
APT apt(sampleRate_Hz, tst, msn, gdc, imu);

int main()
{

    apt.begin();
    gdc.begin();
    msn.begin();
    tst.begin();
    imu.begin();

    msn.missionPhase_e = MSN::fcsMissionPhase::MSN_CRUISE;
    gdc.m_GDC.rollMode_e = GDC::gdcRollMode::GRM_ROLLRATE;
    gdc.m_GDC.rollRateCmd_dps = 10.0;
    imu.m_IMU.states.angularRateX_dps = 2.0;
    apt.step();
    std::cout << "APT Roll Effector Command: " << apt.m_APT.rollEffectorCmd << "\n";

    return 0;
}

// g++ -std=c++11 src/aptTest.cpp -I -o test_app
// ./test_app
// Expected output:
// Result of matrix multiplication:
// 38
// 92
// 146