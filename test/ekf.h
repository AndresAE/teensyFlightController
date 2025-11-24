#pragma once

struct EKFState {
  float x[4];     // [x, y, vx, vy]
  float P[4][4];  // Covariance
};

class EKF {
public:
  EKF();
  void predict(float ax, float ay, float dt);
  void update(float gps_x, float gps_y, float gps_vx, float gps_vy);
  EKFState getState();

private:
  EKFState state;
};
