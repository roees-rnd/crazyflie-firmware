/*
 * File: getMetry.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef GETMETRY_H
#define GETMETRY_H

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

  extern void getMetry(double Map_numUsed, const double xyRob[400], const
                       Lmk_type Lmk[15], const Raw_type Raw[20], double
                       num_segments, double pnum, double b_dPosFault, double
                       b_oneOfTenCyc, struct1_T *Metry);
  extern void getMetry_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for getMetry.h
 *
 * [EOF]
 */
