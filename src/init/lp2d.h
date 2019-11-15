/*
 * File: lp2d.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef LP2D_H
#define LP2D_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "CyclicBuffer_addPos_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void lp2d(const double l[3], const double p[2], double *d, double D_l[3],
                   double D_p[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for lp2d.h
 *
 * [EOF]
 */
