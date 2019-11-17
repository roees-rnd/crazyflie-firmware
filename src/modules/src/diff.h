/*
 * File: diff.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef DIFF_H
#define DIFF_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "../modules/src/CyclicBuffer_addPos_types.h"
#include "../modules/src/rt_defines.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/rtwtypes.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void b_diff(const double b_x[4], double b_y[2]);
  extern void c_diff(const double b_x[4], double b_y[2]);
  extern double d_diff(const double b_x[2]);
  extern void diff(const double b_x[400], double b_y[398]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for diff.h
 *
 * [EOF]
 */
