/*
 * File: initSlam.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef INITSLAM_H
#define INITSLAM_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "../modules/src/CyclicBuffer_addPos_types.h"
#include "../modules/src/rt_defines.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/rtwtypes.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void initSlam(const char setupType[7], Map_type *Map, Rob_type *Rob,
                       Sen_type Sen[17], Lmk_type Lmk[15], Obs_type Obs[15],
                       Opt_type *Opt, Raw_type Raw[20], double xyRob[400],
                       double *pnum);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for initSlam.h
 *
 * [EOF]
 */
