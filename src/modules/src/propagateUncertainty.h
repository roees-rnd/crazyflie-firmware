/*
 * File: propagateUncertainty.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef PROPAGATEUNCERTAINTY_H
#define PROPAGATEUNCERTAINTY_H

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
