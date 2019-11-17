/*
 * File: mean.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/mean.h"

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
 * Arguments    : const double b_x[4]
 *                double b_y[2]
 * Return Type  : void
 */
void mean(const double b_x[4], double b_y[2])
{
  int i;
  int xpageoffset;
  for (i = 0; i < 2; i++) {
    xpageoffset = i << 1;
    b_y[i] = (b_x[xpageoffset] + b_x[xpageoffset + 1]) / 2.0;
  }
}

/*
 * File trailer for mean.c
 *
 * [EOF]
 */
