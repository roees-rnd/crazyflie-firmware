/*
 * File: addToMap.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef ADDTOMAP_H
#define ADDTOMAP_H

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

  extern void addToMap(Map_type *Map, const double L[2], const double P_LL[4],
                       const double P_LX[66], const double usedInd[33], double
                       usedNum, const double b_r[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for addToMap.h
 *
 * [EOF]
 */
