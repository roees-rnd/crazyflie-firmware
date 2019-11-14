/*
 * File: filtfilt.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef FILTFILT_H
#define FILTFILT_H

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

  extern void filtfilt(const double x_in[64], double y_out[64]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for filtfilt.h
 *
 * [EOF]
 */
