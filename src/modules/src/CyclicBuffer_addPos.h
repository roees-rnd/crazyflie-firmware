/*
 * File: CyclicBuffer_addPos.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef CYCLICBUFFER_ADDPOS_H
#define CYCLICBUFFER_ADDPOS_H

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

  extern boolean_T CyclicBuffer_addPos(xyBuff_type *buff, double elemNum, const
    double Val[2], double numDim, double b_TT, const OptSync_type *OptSync);
  extern void CyclicBuffer_addPos_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for CyclicBuffer_addPos.h
 *
 * [EOF]
 */
