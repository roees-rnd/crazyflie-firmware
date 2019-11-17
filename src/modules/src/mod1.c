/*
 * File: mod1.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/mod1.h"

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
 * Arguments    : double b_x
 * Return Type  : double
 */
double floatmod(double b_x)
{
  double b_r;
  boolean_T rEQ0;
  double q;
  if ((!rtIsInf(b_x)) && (!rtIsNaN(b_x))) {
    if (b_x == 0.0) {
      b_r = 0.0;
    } else {
      b_r = fmod(b_x, 6.2831853071795862);
      rEQ0 = (b_r == 0.0);
      if (!rEQ0) {
        q = fabs(b_x / 6.2831853071795862);
        rEQ0 = (fabs(q - floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        b_r = 0.0;
      } else {
        if (b_x < 0.0) {
          b_r += 6.2831853071795862;
        }
      }
    }
  } else {
    b_r = rtNaN;
  }

  return b_r;
}

/*
 * File trailer for mod1.c
 *
 * [EOF]
 */
