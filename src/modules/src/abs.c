/*
 * File: abs.c
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
#include "abs.h"

/* Function Definitions */

/*
 * Arguments    : const double b_x[3]
 *                double b_y[3]
 * Return Type  : void
 */
void b_abs(const double b_x[3], double b_y[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    b_y[k] = fabs(b_x[k]);
  }
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
