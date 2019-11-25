/*
 * File: myRansac.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 25-Nov-2019 10:24:09
 */

#ifndef MYRANSAC_H
#define MYRANSAC_H

/* Include Files */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "cf_feat_ext_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void mtlb_myRansac(const int points[400], double maxDistance, double
    sampleSize, float modelRANSAC[3], boolean_T isInliers[200]);
  extern void mtlb_myRansac_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for myRansac.h
 *
 * [EOF]
 */
