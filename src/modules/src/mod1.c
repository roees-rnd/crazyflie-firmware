/*
 * File: mod1.c
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
#include "mod1.h"

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
