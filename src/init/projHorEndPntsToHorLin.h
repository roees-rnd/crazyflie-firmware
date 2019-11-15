/*
 * File: projHorEndPntsToHorLin.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef PROJHORENDPNTSTOHORLIN_H
#define PROJHORENDPNTSTOHORLIN_H

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

  extern void projHorEndPntToHorLin(double rho, double theta, const double p[2],
    double projected_point[2], double *projection_length);
  extern void projHorEndPntsToHorLin(double rho, double theta, const double p1[2],
    const double p2[2], double seg[4], double b_t[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for projHorEndPntsToHorLin.h
 *
 * [EOF]
 */
