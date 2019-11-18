/*
 * File: correctBlockEkf.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef CORRECTBLOCKEKF_H
#define CORRECTBLOCKEKF_H

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

  extern void correctBlockEkf(Map_type *Map, const double b_r[5], const double
    H[10], const double Inn_z[2], const double Inn_Z[4], const double Inn_iZ[4]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for correctBlockEkf.h
 *
 * [EOF]
 */
