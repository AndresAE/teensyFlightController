#include "Common.h"

Common::Common()
{
  // Constructor logic if needed
}

double Common::norm(double x, double y, double z)
{
  return sqrt(x * x + y * y + z * z);
}

double Common::trapIntegral(double x, double x_prev, double y, double dt_s)
{
  return (x + x_prev) * dt_s / 2 + y;
}

double Common::lowPassFilter(double x, double x_prev, double alpha)
{
  return alpha * x + (1.0 - alpha) * x_prev;
}

double Common::filteredDerivative(double x, double x_prev, double derivPrev, double alpha, double dt_s)
{
  return lowPassFilter((x - x_prev) / dt_s, derivPrev, alpha);
}

double Common::calcAlpha(double cutoff_Hz, double dt_s)
{
  return dt_s / (1.0 / (2.0 * _PI * cutoff_Hz) + dt_s);
}

double Common::primeVerticalRadiusOfCurvature(double latG_d)
{
  double sinLat = sin(DEG2RAD * latG_d);
  return WGS84_r_ft / sqrt(1.0 - WGS84_e2 * (sinLat * sinLat));
}

Common::busEcef Common::lla2ecef(busLLA lla_ddft)
{
  n_ft = primeVerticalRadiusOfCurvature(lla_ddft.latG_d);
  double cosLat = cos(DEG2RAD * lla_ddft.latG_d);
  double cosLon = cos(DEG2RAD * lla_ddft.lon_d);
  double sinLat = sin(DEG2RAD * lla_ddft.latG_d);
  m_ECEF.x_ft = (n_ft + lla_ddft.hae_ft) * cosLat * cosLon;
  m_ECEF.y_ft = (n_ft + lla_ddft.hae_ft) * cosLat * sin(DEG2RAD * lla_ddft.lon_d);
  m_ECEF.z_ft = ((1.0 - WGS84_e2) * n_ft + lla_ddft.hae_ft) * sinLat;
  return m_ECEF;
}

Common::busLLA Common::ecef2lla(busEcef ecef_ft)
{
  m_LLA.lon_d = RAD2DEG * atan2(ecef_ft.y_ft, ecef_ft.x_ft);
  _nIter = 0;
  m_LLA.hae_ft = 0.0;
  n_ft = WGS84_r_ft;

  while (_nIter < _nIterLat)
  {
    m_LLA.latG_d = RAD2DEG * atan((ecef_ft.z_ft) / (sqrt(ecef_ft.x_ft * ecef_ft.x_ft + ecef_ft.y_ft * ecef_ft.y_ft) * (1 - (WGS84_e2 * n_ft) / (n_ft + m_LLA.hae_ft))));
    n_ft = primeVerticalRadiusOfCurvature(m_LLA.latG_d);
    m_LLA.hae_ft = sqrt(ecef_ft.x_ft * ecef_ft.x_ft + ecef_ft.y_ft * ecef_ft.y_ft) / cos(DEG2RAD * m_LLA.latG_d) - n_ft;
    _nIter++;
  }
  return m_LLA;
}

Common::matrix Common::matrixMultiply(Common::matrix A, Common::matrix B)
{
  Common::matrix mat;
  mat.rows = A.rows;
  mat.cols = B.cols;
  for (int i = 0; i < A.rows; i++)
  {
    for (int j = 0; j < B.cols; j++)
    {
      mat.m[i][j] = 0.0f;
      for (int k = 0; k < A.cols; k++)
      {
        mat.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return mat;
}

Common::matrix Common::matrixAdd(Common::matrix A, Common::matrix B, bool negate_b)
{
  Common::matrix R;
  R.rows = A.rows;
  R.cols = A.cols;
  for (int i = 0; i < A.rows; i++)
  {
    for (int j = 0; j < A.cols; j++)
    {
      if (negate_b)
      {
        R.m[i][j] = A.m[i][j] - B.m[i][j];
      }
      else
      {
        R.m[i][j] = A.m[i][j] + B.m[i][j];
      }
    }
  }
  return R;
}

Common::matrix Common::matrixTranspose(Common::matrix A)
{
  Common::matrix T;
  T.rows = A.cols;
  T.cols = A.rows;
  for (int i = 0; i < A.rows; i++)
  {
    for (int j = 0; j < A.cols; j++)
    {
      T.m[j][i] = A.m[i][j];
    }
  }
  return T;
}

Common::matrix Common::matrixInverse(Common::matrix A)
{
  int n = A.rows;
  Common::matrix aug;
  aug.rows = n;
  aug.cols = 2 * n;

  // build augmented matrix [A | I]
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      aug.m[i][j] = A.m[i][j];
    }
    for (int j = 0; j < n; j++)
    {
      aug.m[i][n + j] = (i == j) ? 1.0f : 0.0f;
    }
  }

  // Gauss-Jordan elimination
  for (int i = 0; i < n; i++)
  {
    // find pivot
    int pivot = i;
    float maxVal = fabs(aug.m[i][i]);
    for (int r = i + 1; r < n; r++)
    {
      if (fabs(aug.m[r][i]) > maxVal)
      {
        maxVal = fabs(aug.m[r][i]);
        pivot = r;
      }
    }
    if (pivot != i)
    {
      for (int c = 0; c < 2 * n; c++)
      {
        float tmp = aug.m[i][c];
        aug.m[i][c] = aug.m[pivot][c];
        aug.m[pivot][c] = tmp;
      }
    }

    float diag = aug.m[i][i];
    if (fabs(diag) < 1e-9f)
    {
      // singular; return zero-sized matrix
      Common::matrix zero;
      zero.rows = 0;
      zero.cols = 0;
      return zero;
    }

    // normalize row
    for (int c = 0; c < 2 * n; c++)
      aug.m[i][c] /= diag;

    // eliminate other rows
    for (int r = 0; r < n; r++)
    {
      if (r == i)
        continue;
      float factor = aug.m[r][i];
      for (int c = 0; c < 2 * n; c++)
      {
        aug.m[r][c] -= factor * aug.m[i][c];
      }
    }
  }

  Common::matrix inv;
  inv.rows = n;
  inv.cols = n;
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      inv.m[i][j] = aug.m[i][n + j];
    }
  }
  return inv;
}

Common::matrix Common::crossProductMatrix(Common::matrix A, Common::matrix B)
{
  Common::matrix C;
  C.rows = 3;
  C.cols = 1;
  C.m[0][0] = A.m[1][0] * B.m[2][0] - A.m[2][0] * B.m[1][0];
  C.m[1][0] = A.m[2][0] * B.m[0][0] - A.m[0][0] * B.m[2][0];
  C.m[2][0] = A.m[0][0] * B.m[1][0] - A.m[1][0] * B.m[0][0];
  return C;
}
