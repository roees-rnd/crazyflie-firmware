/*
 * File: cf_feat_ext.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 25-Nov-2019 10:24:09
 */

#ifndef CF_FEAT_EXT_H
#define CF_FEAT_EXT_H

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

  extern void cf_feat_ext(const int navXY[2], float Psi, const unsigned short
    ranges[16], const Opt_type *Opt, float modelRANSAC[3]);
  extern void mtlb_firstRun_not_empty_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for cf_feat_ext.h
 *
 * [EOF]
 */
