/*
 * File: epNom2epNew.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef EPNOM2EPNEW_H
#define EPNOM2EPNEW_H

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

  extern void epNom2epNew(const double b_rhoPhiWorld[2], const double endps[4],
    double epNew[4], double b_Jac[8]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for epNom2epNew.h
 *
 * [EOF]
 */
