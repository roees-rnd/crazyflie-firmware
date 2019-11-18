/*
 * File: retroProjXYFromRangeSenOnRob.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "CyclicBuffer_addPos.h"
#include "CyclicBuffer_addProx.h"
#include "initCyclicBuffer.h"
#include "initSlam.h"
#include "slamOnVehicle.h"
#include "stepBetweenSync.h"
#include "upSampAndFilt_AttPos.h"
#include "retroProjXYFromRangeSenOnRob.h"

/* Function Definitions */

/*
 * Arguments    : const double Rob_state_x[3]
 *                const double Sensor_position[2]
 *                double Sensor_orientationDegrees
 *                double range
 *                double xy[2]
 * Return Type  : void
 */
void retroProjXYFromRangeSenOnRob(const double Rob_state_x[3], const double
  Sensor_position[2], double Sensor_orientationDegrees, double range, double xy
  [2])
{
  double s;
  double c;
  double sensor_ang_w;
  double b_c[4];
  double b_range[2];
  int i0;
  int i1;

  /* q2e(Rob.state.x(4:7)); rob_ang = rob_ang(3); */
  /*  rotates points in the xy-Cartesian plane counter-clockwise(left) through an angle ? about the origin of the Cartesian coordinate system. */
  s = sin(Rob_state_x[2]);
  c = cos(Rob_state_x[2]);

  /*  DEG2RAD  Degrees to radians conversion. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  ========== End of function - Start GPL license ========== */
  /*    # START GPL LICENSE */
  /* --------------------------------------------------------------------- */
  /*  */
  /*    This file is part of SLAMTB, a SLAM toolbox for Matlab. */
  /*  */
  /*    SLAMTB is free software: you can redistribute it and/or modify */
  /*    it under the terms of the GNU General Public License as published by */
  /*    the Free Software Foundation, either version 3 of the License, or */
  /*    (at your option) any later version. */
  /*  */
  /*    SLAMTB is distributed in the hope that it will be useful, */
  /*    but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*    GNU General Public License for more details. */
  /*  */
  /*    You should have received a copy of the GNU General Public License */
  /*    along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /* --------------------------------------------------------------------- */
  /*    SLAMTB is Copyright: */
  /*    Copyright (c) 2008-2010, Joan Sola @ LAAS-CNRS, */
  /*    Copyright (c) 2010-2013, Joan Sola, */
  /*    Copyright (c) 2014-    , Joan Sola @ IRI-UPC-CSIC, */
  /*    SLAMTB is Copyright 2009  */
  /*    by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol */
  /*    @ LAAS-CNRS. */
  /*    See on top of this file for its particular copyright. */
  /*    # END GPL LICENSE */
  sensor_ang_w = Rob_state_x[2] + Sensor_orientationDegrees * 3.1415926535897931
    / 180.0;
  b_c[0] = c;
  b_c[2] = -s;
  b_c[1] = s;
  b_c[3] = c;
  b_range[0] = range * cos(sensor_ang_w);
  b_range[1] = range * sin(sensor_ang_w);
  for (i0 = 0; i0 < 2; i0++) {
    s = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      s += b_c[i0 + (i1 << 1)] * Sensor_position[i1];
    }

    xy[i0] = b_range[i0] + (s + Rob_state_x[i0]);
  }
}

/*
 * File trailer for retroProjXYFromRangeSenOnRob.c
 *
 * [EOF]
 */
