/*
 * File: initCyclicBuffer.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef INITCYCLICBUFFER_H
#define INITCYCLICBUFFER_H

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

  extern void initCyclicBuffer(psiBuff_type *psiBuff, xyBuff_type *xyBuff,
    proxBuff_type proxBuff[16], OptSync_type *OptSync);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for initCyclicBuffer.h
 *
 * [EOF]
 */
