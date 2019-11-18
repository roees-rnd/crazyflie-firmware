/*
 * File: mod.c
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
#include "mod.h"
#include "mod1.h"

/* Function Definitions */

/*
 * Arguments    : const double b_x[200]
 *                double b_r[200]
 * Return Type  : void
 */
void b_mod(const double b_x[200], double b_r[200])
{
  int k;
  for (k = 0; k < 200; k++) {
    b_r[k] = floatmod(b_x[k]);
  }
}

/*
 * File trailer for mod.c
 *
 * [EOF]
 */
