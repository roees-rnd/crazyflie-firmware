/*
 * File: sign.c
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
#include "sign.h"

/* Function Definitions */

/*
 * Arguments    : double *b_x
 * Return Type  : void
 */
void b_sign(double *b_x)
{
  if (*b_x < 0.0) {
    *b_x = -1.0;
  } else if (*b_x > 0.0) {
    *b_x = 1.0;
  } else {
    if (*b_x == 0.0) {
      *b_x = 0.0;
    }
  }
}

/*
 * File trailer for sign.c
 *
 * [EOF]
 */
