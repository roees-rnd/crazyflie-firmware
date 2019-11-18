/*
 * File: upSampAndFilt_AttPos.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef UPSAMPANDFILT_ATTPOS_H
#define UPSAMPANDFILT_ATTPOS_H

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

  extern void upSampAndFilt_AttPos(const psiBuff_type *psiBuff, const
    xyBuff_type *xyBuff, const OptSync_type *OptSync, boolean_T *shortTimeStep,
    double highFreqTimeGrid[64], double *wantedSyncTime_T0, double
    *wantedSyncTime_T1, double xUpSamp_f[64], double yUpSamp_f[64], double
    attUpSamp_f[64]);
  extern void upSampAndFilt_AttPos_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for upSampAndFilt_AttPos.h
 *
 * [EOF]
 */
