/*
 * File: getMaxPenDist.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef GETMAXPENDIST_H
#define GETMAXPENDIST_H

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

  extern void getMaxPenDist(const double Points[600], double i_s, double i_e,
    double *dmax, double *idx);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for getMaxPenDist.h
 *
 * [EOF]
 */
