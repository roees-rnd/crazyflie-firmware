/*
 * File: batchSplitFit.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef BATCHSPLITFIT_H
#define BATCHSPLITFIT_H

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

  extern void b_batchSplitFit(const double orig[3], const double Points[400],
    double pnum, double sensStd, double Opt_SF_mm2m, double
    c_Opt_splitFit_MIN_NUM_OF_PNTS_, double Opt_splitFit_MIN_SEG_LEN, double
    Opt_splitFit_MAX_GAP, double Opt_splitFit_epsilon, double
    Opt_splitFit_alphaProx, boolean_T Opt_splitFit_onlyEndp, const double
    Opt_splitFit_v[3], double rhoTta[40], double covRhoTta[80], double tminmax
    [40], double b_avgDist[20], double nPoints[20], double *num_segments);
  extern void batchSplitFit(const double orig[3], const double Points[400],
    double pnum, double sensStd, double Opt_SF_mm2m, double
    c_Opt_splitFit_MIN_NUM_OF_PNTS_, double Opt_splitFit_MIN_SEG_LEN, double
    Opt_splitFit_MAX_GAP, double Opt_splitFit_epsilon, double
    Opt_splitFit_alphaProx, boolean_T Opt_splitFit_onlyEndp, const double
    Opt_splitFit_v[3], double rhoTta[40], double covRhoTta[80], double tminmax
    [40], double b_avgDist[20], double nPoints[20], double *num_segments, double
    Points_c[600]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for batchSplitFit.h
 *
 * [EOF]
 */
