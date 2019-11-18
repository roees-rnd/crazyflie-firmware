/*
 * File: norm.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "CyclicBuffer_addPos.h"
#include "CyclicBuffer_addProx.h"
#include "initCyclicBuffer.h"
#include "initSlam.h"
#include "slamOnVehicle.h"
#include "stepBetweenSync.h"
#include "upSampAndFilt_AttPos.h"
#include "norm.h"

/* Function Definitions */

/*
 * Arguments    : const double b_x[2]
 * Return Type  : double
 */
double b_norm(const double b_x[2])
{
  double b_y;
  double scale;
  int k;
  double absxk;
  double b_t;
  b_y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 2; k++) {
    absxk = fabs(b_x[k]);
    if (absxk > scale) {
      b_t = scale / absxk;
      b_y = 1.0 + b_y * b_t * b_t;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      b_y += b_t * b_t;
    }
  }

  return scale * sqrt(b_y);
}

/*
 * Arguments    : const double b_x[2]
 * Return Type  : double
 */
double norm(const double b_x[2])
{
  double b_y;
  double scale;
  int k;
  double absxk;
  double b_t;
  b_y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 2; k++) {
    absxk = fabs(b_x[k]);
    if (absxk > scale) {
      b_t = scale / absxk;
      b_y = 1.0 + b_y * b_t * b_t;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      b_y += b_t * b_t;
    }
  }

  return scale * sqrt(b_y);
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
