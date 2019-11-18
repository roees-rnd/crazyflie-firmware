/*
 * File: mtlb_filter.c
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
#include "mtlb_filter.h"

/* Function Definitions */

/*
 * Arguments    : double b[11]
 *                const double b_x[124]
 *                const double zi[10]
 *                double b_y[124]
 * Return Type  : void
 */
void mtlb_filter(double b[11], const double b_x[124], const double zi[10], double
            b_y[124])
{
  int k;
  int j;
  for (k = 0; k < 11; k++) {
    b[k] /= 11.0;
  }

  memcpy(&b_y[0], &zi[0], 10U * sizeof(double));
  memset(&b_y[10], 0, 114U * sizeof(double));
  for (k = 0; k < 11; k++) {
    for (j = k; j + 1 < 125; j++) {
      b_y[j] += b[k] * b_x[j - k];
    }
  }
}

/*
 * File trailer for mtlb_filter.c
 *
 * [EOF]
 */
