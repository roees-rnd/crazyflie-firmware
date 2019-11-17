/*
 * File: power.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "../modules/src/mtlb_power.h"

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
 * Arguments    : const double a[2]
 *                double b_y[2]
 * Return Type  : void
 */
void b_power(const double a[2], double b_y[2])
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
void c_power(const double a[2], double b_y[2])
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
 * File trailer for power.c
 *
 * [EOF]
 */
