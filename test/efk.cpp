#include "EKF.h"

EKF::EKF() {
  for (int i = 0; i < 4; i++) {
    state.x[i] = 0;
    for (int j = 0; j < 4; j++) {
      state.P[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
}

void EKF::predict(float ax, float ay, float dt) {
  state.x[0] += state.x[2] * dt + 0.5 * ax * dt * dt;
  state.x[1] += state.x[3] * dt + 0.5 * ay * dt * dt;
  state.x[2] += ax * dt;
  state.x[3] += ay * dt;

  for (int i = 0; i < 4; i++) {
    state.P[i][i] += 0.1 * dt;
  }
}

void EKF::update(float gps_x, float gps_y, float gps_vx, float gps_vy) {
  float z[4] = {gps_x, gps_y, gps_vx, gps_vy};
  float R[4] = {3.0, 3.0, 0.5, 0.5};

  for (int i = 0; i < 4; i++) {
    float y = z[i] - state.x[i];
    float K = state.P[i][i] / (state.P[i][i] + R[i]);
    state.x[i] += K * y;
    state.P[i][i] *= (1 - K);
  }
}

EKFState EKF::getState() {
  return state;
}