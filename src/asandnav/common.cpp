#include "Common.h"

Common::Common() {
    // Constructor logic if needed
}

double Common::norm(double x, double y, double z) {
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

double Common::trapIntegral(double x, double x_prev, double y, double dt_s) {
  return (x + x_prev) * dt_s / 2 + y;
}

double Common::lowPassFilter(double x, double x_prev, double alpha) {
  return alpha * x + (1.0f - alpha) * x_prev;
}

double Common::calcAlpha(double cutoff_Hz, double dt_s) {
  return dt_s / (1.0 / (2.0 * _PI * cutoff_Hz) + dt_s);
}
