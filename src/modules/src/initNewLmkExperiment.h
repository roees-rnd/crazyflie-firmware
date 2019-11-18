/*
 * File: initNewLmkExperiment.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef INITNEWLMKEXPERIMENT_H
#define INITNEWLMKEXPERIMENT_H

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

  extern double initNewLmkExperiment(Map_type *Map, double Rob_state_size, const
    double Rob_state_r[3], double Sen_sen, Raw_type *Raw, Lmk_type Lmk[15],
    Obs_type Obs[15], const char Opt_init_initType[6]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for initNewLmkExperiment.h
 *
 * [EOF]
 */
