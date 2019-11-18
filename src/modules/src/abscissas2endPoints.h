/*
 * File: abscissas2endPoints.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef ABSCISSAS2ENDPOINTS_H
#define ABSCISSAS2ENDPOINTS_H

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

  extern void abscissas2endPoints(const double rho_phi[2], const double absc[2],
    double endps[4], double ep_line[8], double ep_t[4]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for abscissas2endPoints.h
 *
 * [EOF]
 */
