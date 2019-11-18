/*
 * File: projRhoPhiAlpha2World.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef PROJRHOPHIALPHA2WORLD_H
#define PROJRHOPHIALPHA2WORLD_H

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

  extern void projRhoPhiAlpha2World(const double RobPosFreezeAlpha[3], const
    double b_rhoPhiRob[2], double RhoPhiWorld[2], double Wpos[6], double Wrp[4]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for projRhoPhiAlpha2World.h
 *
 * [EOF]
 */
