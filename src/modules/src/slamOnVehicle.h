/*
 * File: slamOnVehicle.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef SLAMONVEHICLE_H
#define SLAMONVEHICLE_H

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

  extern void FirstRun_not_empty_init(void);
  extern void slamOnVehicle(double RobTT, const double RobXYT[3], const double
    RobCov[9], const Prox_type *prox, Map_type *Map, Rob_type *Rob, const
    Sen_type Sen[17], Lmk_type Lmk[15], Obs_type Obs[15], const Opt_type *Opt,
    Raw_type Raw[20], double xyRob[400], double *pnum, boolean_T *LowFreqCycle,
    double RobPoseUpd[3], double RobCovUpd[9], double LmkIds[20], double
    LmkEndps[80], double LmkActions[20], struct0_T *Metry);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for slamOnVehicle.h
 *
 * [EOF]
 */
