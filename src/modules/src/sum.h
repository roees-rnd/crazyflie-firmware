/*
 * File: sum.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef SUM_H
#define SUM_H

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

  extern void b_sum(const double b_x[398], double b_y[199]);
  extern double c_sum(const boolean_T b_x[200]);
  extern double d_sum(const double b_x[2]);
  extern double e_sum(const double b_x[3]);
  extern double sum(const double b_x[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for sum.h
 *
 * [EOF]
 */
