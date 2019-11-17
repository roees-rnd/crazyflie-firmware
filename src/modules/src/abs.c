/*
 * File: abs.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/abs.h"

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
