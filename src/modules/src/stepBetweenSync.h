/*
 * File: stepBetweenSync.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef STEPBETWEENSYNC_H
#define STEPBETWEENSYNC_H

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

  extern void d_wantedSyncTime_T0_prev_not_em(void);
  extern void stepBetweenSync(const proxBuff_type proxBuff[16], const double
    highFreqTimeGrid[64], double wantedSyncTime_T0, double wantedSyncTime_T1,
    const double xUpSamp_f[64], const double yUpSamp_f[64], const double
    attUpSamp_f[64], const OptSync_type *OptSync, boolean_T *done, double *RobTT,
    double RobXYT[3], double RobCov[9], Prox_type *prox);
  extern void stepBetweenSync_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for stepBetweenSync.h
 *
 * [EOF]
 */
