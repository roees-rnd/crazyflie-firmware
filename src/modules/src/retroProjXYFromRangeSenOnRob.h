/*
 * File: retroProjXYFromRangeSenOnRob.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef RETROPROJXYFROMRANGESENONROB_H
#define RETROPROJXYFROMRANGESENONROB_H

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

  extern void retroProjXYFromRangeSenOnRob(const double Rob_state_x[3], const
    double Sensor_position[2], double Sensor_orientationDegrees, double range,
    double xy[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for retroProjXYFromRangeSenOnRob.h
 *
 * [EOF]
 */
