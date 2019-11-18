/*
 * File: initSlam.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef INITSLAM_H
#define INITSLAM_H

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
