/*
 * File: filtfilt.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
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
#include "filtfilt.h"

#include "mtlb_filter.h"

/* Function Definitions */

/*
 * Arguments    : const double x_in[64]
 *                double y_out[64]
 * Return Type  : void
 */
void filtfilt(const double x_in[64], double y_out[64])
{
  double xtmp;
  double d3;
  int i;
  double b_y[124];
  double dv13[11];
  double c_y[124];
  double dv14[10];
  xtmp = 2.0 * x_in[0];
  d3 = 2.0 * x_in[63];
  for (i = 0; i < 30; i++) {
    b_y[i] = xtmp - x_in[30 - i];
  }

  memcpy(&b_y[30], &x_in[0], sizeof(double) << 6);
  for (i = 0; i < 30; i++) {
    b_y[i + 94] = d3 - x_in[62 - i];
  }

  for (i = 0; i < 11; i++) {
    dv13[i] = 1.0;
  }

  for (i = 0; i < 10; i++) {
    dv14[i] = (10.0 - (double)i) * b_y[0];
  }

  memcpy(&c_y[0], &b_y[0], 124U * sizeof(double));
  mtlb_filter(dv13, c_y, dv14, b_y);
  for (i = 0; i < 62; i++) {
    xtmp = b_y[i];
    b_y[i] = b_y[123 - i];
    b_y[123 - i] = xtmp;
  }

  for (i = 0; i < 11; i++) {
    dv13[i] = 1.0;
  }

  for (i = 0; i < 10; i++) {
    dv14[i] = (10.0 - (double)i) * b_y[0];
  }

  memcpy(&c_y[0], &b_y[0], 124U * sizeof(double));
  mtlb_filter(dv13, c_y, dv14, b_y);
  for (i = 0; i < 62; i++) {
    xtmp = b_y[i];
    b_y[i] = b_y[123 - i];
    b_y[123 - i] = xtmp;
  }

  memcpy(&y_out[0], &b_y[30], sizeof(double) << 6);
}

/*
 * File trailer for filtfilt.c
 *
 * [EOF]
 */
