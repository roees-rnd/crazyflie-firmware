/*
 * File: correctKnownLmksExperiment.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef CORRECTKNOWNLMKSEXPERIMENT_H
#define CORRECTKNOWNLMKSEXPERIMENT_H

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

  extern double correctKnownLmksExperiment(Map_type *Map, Rob_type *Rob, const
    Sen_type *Sen, Raw_type *Raw, Lmk_type Lmk[15], Obs_type Obs[15], double
    Opt_correct_MD2th, double Opt_correct_isVisOrtThrsh, const double
    Opt_correct_isVisFarThrsh[2], double Opt_maxDistVis, double Opt_maxGap);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for correctKnownLmksExperiment.h
 *
 * [EOF]
 */
