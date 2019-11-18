/*
 * File: sqrt.c
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
#include "sqrt.h"

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
