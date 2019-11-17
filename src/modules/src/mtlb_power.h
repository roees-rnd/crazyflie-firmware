/*
 * File: power.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef MTLB_POWER_H
#define MTLB_POWER_H

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

  extern void b_power(const double a[2], double b_y[2]);
  extern void c_power(const double a[2], double b_y[2]);
  extern void mtlb_power(const double a[398], double b_y[398]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for power.h
 *
 * [EOF]
 */
