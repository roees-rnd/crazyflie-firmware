/*
 * File: norm.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/norm.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

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
