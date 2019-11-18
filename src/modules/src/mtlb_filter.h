/*
 * File: mtlb_filter.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef FILTER_H
#define FILTER_H

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

  extern void mtlb_filter(double b[11], const double b_x[124], const double zi[10],
                     double b_y[124]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for mtlb_filter.h
 *
 * [EOF]
 */
