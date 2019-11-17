/*
 * File: matchFeatureExperiment.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef MATCHFEATUREEXPERIMENT_H
#define MATCHFEATUREEXPERIMENT_H

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

  extern void matchFeatureExperiment(const double Raw_endpWorld[4], const double
    Raw_covEndpointsWorld[16], const double Raw_J_s2r[12], Obs_type *Obs, double
    Opt_correct_MD2th);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for matchFeatureExperiment.h
 *
 * [EOF]
 */
