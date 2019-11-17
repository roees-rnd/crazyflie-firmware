/*
 * File: sqrt.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/sqrt.h"

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
 * Arguments    : double *b_x
 * Return Type  : void
 */
void b_sqrt(double *b_x)
{
  *b_x = sqrt(*b_x);
}

/*
 * Arguments    : double b_x[199]
 * Return Type  : void
 */
void c_sqrt(double b_x[199])
{
  int k;
  for (k = 0; k < 199; k++) {
    b_x[k] = sqrt(b_x[k]);
  }
}

/*
 * File trailer for sqrt.c
 *
 * [EOF]
 */
