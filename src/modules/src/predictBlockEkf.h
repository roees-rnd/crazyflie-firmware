/*
 * File: predictBlockEkf.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef PREDICTBLOCKEKF_H
#define PREDICTBLOCKEKF_H

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

  extern void predictBlockEkf(Map_type *Map, const double b_r[3], const double
    F_r[9], const double b_U[9], const double F_u[9]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for predictBlockEkf.h
 *
 * [EOF]
 */
