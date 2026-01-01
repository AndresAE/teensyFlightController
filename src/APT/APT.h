#include <cmath>
#include <Arduino.h>
#include "../Common/Common.h"
#include "../GDC/GDC.h"
#include "../IMU/IMU.h"

class APT
{
public:
  APT(int sampleRate_Hz = 50);

  // control gains
  struct busGains
  {
    double K_p;
    double K_i;
    double K_f;
    double k_antiWindup;
  };

  struct busAptGains
  {
    busGains roll;
    busGains rollRate;
    busGains pitch;
    busGains pitchRate;
    busGains nz;
    busGains heading;
    busGains ny;
    busGains yawRate;
  };

  busAptGains m_aptGains;

  // commands to effectors
  double mixingRatioRotational[5][3] = {
      {1.0, 0.0, 0.1},   // left aileron (+LTED)
      {-1.0, 0.0, -0.1}, // right aileron (+RTEU)
      {0.0, 1.0, 0.0},   // elevator (+TEU)
      {0.1, 0.0, 1.0},   // rudder (+TER)
      {0.0, 0.0, 0.0}    // throttle
  };

  double mixingRatioLinear[5][3] = {
      {0.0, 0.0, 0.0}, // left aileron (+LTED)
      {0.0, 0.0, 0.0}, // right aileron (+RTEU)
      {0.0, 0.0, 0.0}, // elevator (+TEU)
      {0.0, 0.0, 0.0}, // rudder (+TER)
      {1.0, 0.0, 0.0}  // throttle
  };
  double effectorLimitHi[5] = {20.0, 20.0, 20.0, 20.0, 1.0};
  double effectorLimitLo[5] = {-20.0, -20.0, -20.0, -20.0, 0.0};
  double effectorOffset[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double effectorPins[5] = {5, 6, 7, 8, 9};
  int pwmMax[5] = {2000, 2000, 2000, 2000, 2000}; // microseconds
  int pwmMin[5] = {1000, 1000, 1000, 1000, 1000}; // microseconds
  double degMax[5] = {30.0, 30.0, 30.0, 30.0, 1.0};
  double degMin[5] = {-30.0, -30.0, -30.0, -30.0, 0.0};
  int pwm_val;

  int sampleRate_Hz;
  double dt_s;
  double wBI_B_dps2[3];
  double aBI_B_fps2[3];

  double integralError = 0.0;

  // public methods
  void begin();
  void step();
  void setGains();
  void rollMiddleLoop();
  void rollInnerLoop();
  void pitchMiddleLoop();
  void pitchInnerLoop();
  void yawMiddleLoop();
  void yawInnerLoop();
  void speedMiddleLoop();
  void speedInnerLoop();
  void controlAllocation();
  void setupEffectors();
  void setEffectors();
  double proportionalControl(double k_p, double cvc, double cv);
  double proportionalIntegralControl(double k_p, double k_i, double cvc, double cv, bool resetIntegral_b);
  double proportionalIntegralFeedforwardControl(double k_p, double k_i, double k_f, double cvc, double cv, bool resetIntegral_b);
  void attachBus(GDC::busGDC *gdcBus, IMU::busIMU *imuBus);

  struct busAPT
  {
    double pbCmd_dps;
    double qbCmd_dps;
    double rbCmd_dps;
    double pbDotCmd_dps2;
    double qbDotCmd_dps2;
    double rbDotCmd_dps2;
    double ubCmd_fps;
    double vbCmd_fps;
    double wbCmd_fps;
    double ubDotCmd_fps2;
    double vbDotCmd_fps2;
    double wbDotCmd_fps2;
    double effectorCmd[5];
  };

  busAPT m_APT;

  // pointers to external bus objects (set from main sketch)
  GDC::busGDC *m_GDC = nullptr;
  IMU::busIMU *m_IMU = nullptr;

private:
  Common C;
  int _nxTakeoffDebounceCount = 0;

  int _prevRollState_e;
  int _prevPitchState_e;
  int _prevYawState_e;
  bool _resetRollIntegral_b = false;
  bool _resetPitchIntegral_b = false;
  bool _resetYawIntegral_b = false;

  int _nEffectors;
  int _nRotationalAxes = 3;
  int _nLinearAxes = 3;
};
