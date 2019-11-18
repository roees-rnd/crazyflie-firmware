/*
 * File: processSlamInput.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef PROCESSSLAMINPUT_H
#define PROCESSSLAMINPUT_H

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

  extern void prevRobXYT_not_empty_init(void);
  extern void processSlamInput(double NavTT, double NavXYT[3], double prox_val
    [16], const double prox_TT[16], double MAX_RANGE, double MIN_RANGE, double
    Opt_SF_mm2m, double Rob_con_u[3], double rawRangeMat[16], double
    *dPosFault_out);
  extern void processSlamInput_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for processSlamInput.h
 *
 * [EOF]
 */
