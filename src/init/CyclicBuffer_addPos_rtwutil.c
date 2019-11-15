/*
 * File: CyclicBuffer_addPos_rtwutil.c
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
#include "CyclicBuffer_addPos_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : double b_u
 * Return Type  : double
 */
double rt_roundd_snf(double b_u)
{
  double b_y;
  if (fabs(b_u) < 4.503599627370496E+15) {
    if (b_u >= 0.5) {
      b_y = floor(b_u + 0.5);
    } else if (b_u > -0.5) {
      b_y = b_u * 0.0;
    } else {
      b_y = ceil(b_u - 0.5);
    }
  } else {
    b_y = b_u;
  }

  return b_y;
}

/*
 * File trailer for CyclicBuffer_addPos_rtwutil.c
 *
 * [EOF]
 */
