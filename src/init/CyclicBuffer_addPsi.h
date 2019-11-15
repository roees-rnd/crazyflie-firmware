/*
 * File: CyclicBuffer_addPsi.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:26:10
 */

#ifndef CYCLICBUFFER_ADDPSI_H
#define CYCLICBUFFER_ADDPSI_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "CyclicBuffer_addPsi_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern boolean_T CyclicBuffer_addPsi(psiBuff_type *buff, double elemNum,
    double Val, double numDim, double TT, const OptSync_type *OptSync);
  extern void CyclicBuffer_addPsi_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for CyclicBuffer_addPsi.h
 *
 * [EOF]
 */
