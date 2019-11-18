/*
 * File: mtlb_power.c
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
#include "mtlb_power.h"

/* Function Definitions */

/*
 * Arguments    : const double a[2]
 *                double b_y[2]
 * Return Type  : void
 */
void b_mtlb_power(const double a[2], double b_y[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    b_y[k] = a[k] * a[k];
  }
}

/*
 * Arguments    : const double a[2]
 *                double b_y[2]
 * Return Type  : void
 */
void c_mtlb_power(const double a[2], double b_y[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    b_y[k] = a[k] * a[k];
  }
}

/*
 * Arguments    : const double a[398]
 *                double b_y[398]
 * Return Type  : void
 */
void mtlb_power(const double a[398], double b_y[398])
{
  int k;
  for (k = 0; k < 398; k++) {
    b_y[k] = a[k] * a[k];
  }
}

/*
 * File trailer for mtlb_power.c
 *
 * [EOF]
 */
