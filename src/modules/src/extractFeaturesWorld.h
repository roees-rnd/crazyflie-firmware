/*
 * File: extractFeaturesWorld.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef EXTRACTFEATURESWORLD_H
#define EXTRACTFEATURESWORLD_H

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

  extern double extractFeaturesWorld(Raw_type Raw[20], const double
    RobPoseFreeze[3], const double RobPoseFreezeCov[9], double xyRob[400],
    double pnum, double sensStd, const double b_robPoseHist[400], double
    b_numPoses, double Opt_SF_mm2m, double c_Opt_splitFit_MIN_NUM_OF_PNTS_,
    double Opt_splitFit_MIN_SEG_LEN, double Opt_splitFit_MAX_GAP, double
    Opt_splitFit_epsilon, double Opt_splitFit_alphaProx, boolean_T
    Opt_splitFit_onlyEndp, const double Opt_splitFit_v[3]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for extractFeaturesWorld.h
 *
 * [EOF]
 */
