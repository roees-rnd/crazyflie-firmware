/*
 * File: projAbscissas2World.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef PROJABSCISSAS2WORLD_H
#define PROJABSCISSAS2WORLD_H

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

  extern void projAbscissas2World(const double RobPosFreeze[2], const double
    rhoTtaRob[2], const double tRob[2], double tWorld[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for projAbscissas2World.h
 *
 * [EOF]
 */
