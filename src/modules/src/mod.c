/*
 * File: mod.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/mod.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/mod1.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

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
