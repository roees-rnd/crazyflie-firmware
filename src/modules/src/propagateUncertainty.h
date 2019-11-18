/*
 * File: propagateUncertainty.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef PROPAGATEUNCERTAINTY_H
#define PROPAGATEUNCERTAINTY_H

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

  extern void propagateUncertainty(const double b_x[2], const double X[4],
    double b_y[3], double Y[9]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for propagateUncertainty.h
 *
 * [EOF]
 */
