/*
 * File: retroProjXYFromRangeSenOnRob.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

#ifndef RETROPROJXYFROMRANGESENONROB_H
#define RETROPROJXYFROMRANGESENONROB_H

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
