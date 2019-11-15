/*
 * File: sum.c
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
#include "sum.h"

/* Function Definitions */

/*
 * Arguments    : const double b_x[398]
 *                double b_y[199]
 * Return Type  : void
 */
void b_sum(const double b_x[398], double b_y[199])
{
  int i;
  int xpageoffset;
  for (i = 0; i < 199; i++) {
    xpageoffset = i << 1;
    b_y[i] = b_x[xpageoffset] + b_x[xpageoffset + 1];
  }
}

/*
 * Arguments    : const boolean_T b_x[200]
 * Return Type  : double
 */
double c_sum(const boolean_T b_x[200])
{
  double b_y;
  int k;
  b_y = b_x[0];
  for (k = 0; k < 199; k++) {
    b_y += (double)b_x[k + 1];
  }

  return b_y;
}

/*
 * Arguments    : const double b_x[2]
 * Return Type  : double
 */
double d_sum(const double b_x[2])
{
  return b_x[0] + b_x[1];
}

/*
 * Arguments    : const double b_x[3]
 * Return Type  : double
 */
double e_sum(const double b_x[3])
{
  double b_y;
  int k;
  b_y = b_x[0];
  for (k = 0; k < 2; k++) {
    b_y += b_x[k + 1];
  }

  return b_y;
}

/*
 * Arguments    : const double b_x[2]
 * Return Type  : double
 */
double sum(const double b_x[2])
{
  return b_x[0] + b_x[1];
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
