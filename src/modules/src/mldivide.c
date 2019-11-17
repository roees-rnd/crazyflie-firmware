/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/mldivide.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Declarations */
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double b_y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    b_y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    b_y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    b_y = b;
  } else {
    b_y = a * 1.4142135623730951;
  }

  return b_y;
}

/*
 * Arguments    : const double A[2]
 *                const double B[2]
 * Return Type  : double
 */
double mldivide(const double A[2], const double B[2])
{
  double Y;
  int i;
  double atmp;
  double b_A[2];
  double tau;
  double xnorm;
  int rankR;
  double b_B[2];
  for (i = 0; i < 2; i++) {
    b_A[i] = A[i];
  }

  atmp = A[0];
  tau = 0.0;
  xnorm = fabs(A[1]);
  if (xnorm != 0.0) {
    xnorm = rt_hypotd_snf(A[0], xnorm);
    if (A[0] >= 0.0) {
      xnorm = -xnorm;
    }

    if (fabs(xnorm) < 1.0020841800044864E-292) {
      i = 0;
      do {
        i++;
        b_A[1] *= 9.9792015476736E+291;
        xnorm *= 9.9792015476736E+291;
        atmp *= 9.9792015476736E+291;
      } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

      xnorm = rt_hypotd_snf(atmp, fabs(b_A[1]));
      if (atmp >= 0.0) {
        xnorm = -xnorm;
      }

      tau = (xnorm - atmp) / xnorm;
      b_A[1] *= 1.0 / (atmp - xnorm);
      for (rankR = 1; rankR <= i; rankR++) {
        xnorm *= 1.0020841800044864E-292;
      }

      atmp = xnorm;
    } else {
      tau = (xnorm - A[0]) / xnorm;
      b_A[1] = 1.0 / (A[0] - xnorm) * A[1];
      atmp = xnorm;
    }
  }

  rankR = 0;
  xnorm = 2.0 * fabs(atmp) * 2.2204460492503131E-16;
  while ((rankR < 1) && (!(fabs(atmp) <= xnorm))) {
    rankR = 1;
  }

  for (i = 0; i < 2; i++) {
    b_B[i] = B[i];
  }

  Y = 0.0;
  if (tau != 0.0) {
    xnorm = tau * (B[0] + b_A[1] * B[1]);
    if (xnorm != 0.0) {
      b_B[0] = B[0] - xnorm;
    }
  }

  i = 1;
  while (i <= rankR) {
    Y = b_B[0];
    i = 2;
  }

  while (rankR > 0) {
    Y /= atmp;
    rankR = 0;
  }

  return Y;
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
