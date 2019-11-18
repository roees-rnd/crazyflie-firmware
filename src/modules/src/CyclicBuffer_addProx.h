/*
 * File: CyclicBuffer_addProx.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef CYCLICBUFFER_ADDPROX_H
#define CYCLICBUFFER_ADDPROX_H

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

  extern boolean_T CyclicBuffer_addProx(proxBuff_type buff[16], double elemNum,
    double Val, double numDim, double b_TT, const OptSync_type *OptSync);
  extern void CyclicBuffer_addProx_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for CyclicBuffer_addProx.h
 *
 * [EOF]
 */
