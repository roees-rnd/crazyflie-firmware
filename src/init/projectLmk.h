/*
 * File: projectLmk.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef PROJECTLMK_H
#define PROJECTLMK_H

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

  extern void projectLmk(const double Map_x[303], const double Map_P[91809],
    const double Rob_state_x[3], const double Rob_state_r[3], const char
    b_Sen_type[7], double Sen_par_maxRange, const char b_Lmk_type[6], const
    double Lmk_state_r[2], const double Lmk_parEndp_t[2], const double
    Lmk_parEndp_e[4], const double Lmk_parEndp_nrm[2], Obs_type *Obs, const
    double Raw_endpWorld[4], const double Raw_nrm[2], double
    Opt_correct_isVisOrtThrsh, const double Opt_correct_isVisFarThrsh[2], double
    Opt_maxDistVis);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for projectLmk.h
 *
 * [EOF]
 */
