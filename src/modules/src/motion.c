/*
 * File: motion.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/motion.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/predictBlockEkf.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Definitions */

/*
 * function Rob = motion(Rob, Tim)
 * MOTION Robot motion.
 *    ROB = MOTION(ROB, TIM) performs one EKF-prediction motion step to robot
 *    Rob in the global map Map, following the motion model in Rob.motion.
 *    Both Rob and Map are updated. The time information Tim is used only if
 *    the motion model requires it, but it has to be provided because MOTION
 *    is a generic method.
 *
 *    The following motion models are supported:
 *        'odometry'   uses function odo3()
 *        'constVel'   uses function constVel()
 *    Edit this file to add new motion models.
 *
 *    See also SIMMOTION, CONSTVEL, ODO3, UPDATEFRAME.
 * Arguments    : Map_type *Map
 *                Rob_type *Rob
 * Return Type  : void
 */
void motion(Map_type *Map, Rob_type *Rob)
{
  int i21;
  double b_r[3];
  int i;
  double b_Map[9];
  double s_init_x[3];
  RobState_type r11;
  double dv23[4];
  double d7;
  double dv24[9];
  static const signed char iv1[3] = { 0, 0, 1 };

  /*    Copyright 2009 David Marquez @ LAAS-CNRS. */
  /*  global Map */
  /*  Update rob and sen info from map */
  /*  MAP2ROB  Update Rob structure from the Map information. */
  /*    ROB = MAP2ROB(ROB) updates the structure ROB to reflect the information */
  /*    contained in the golbal map Map. */
  /*  */
  /*    See also UPDATEFRAME. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  global Map */
  /*  normalize quaternion - mean and cov */
  /*  % % [Map.x(Rob.frame.r(4:7)), NQ_q] = normvec(Map.x(Rob.frame.r(4:7)),1); */
  /*  % % Map.P(Rob.frame.r(4:7),Map.used) = NQ_q*Map.P(Rob.frame.r(4:7),Map.used); */
  /*  % % Map.P(Map.used,Rob.frame.r(4:7)) = Map.P(Map.used,Rob.frame.r(4:7))*NQ_q'; */
  /*  means */
  for (i21 = 0; i21 < 3; i21++) {
    b_r[i21] = Map->x[(int)Rob->state.r[i21] - 1];
  }

  for (i21 = 0; i21 < 3; i21++) {
    Rob->state.x[i21] = b_r[i21];
  }

  /*  % % Rob.frame.x = Map.x(Rob.frame.r); */
  /*  Rob.frame   = updateFrame(Rob.frame); */
  /*  covariances */
  for (i21 = 0; i21 < 3; i21++) {
    for (i = 0; i < 3; i++) {
      b_Map[i + 3 * i21] = Map->P[((int)Rob->state.r[i] + 303 * ((int)
        Rob->state.r[i21] - 1)) - 1];
    }
  }

  /*  Rob.frame.P = Map.P(Rob.frame.r,Rob.frame.r); */
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
  /*  robot state range */
  /*  switch Rob.motion */
  /*  */
  /*      case {'odometry'} */
  /*  motion model of the robot: mean and Jacobians */
  for (i = 0; i < 3; i++) {
    for (i21 = 0; i21 < 3; i21++) {
      Rob->state.P[i21 + 3 * i] = b_Map[i21 + 3 * i];
    }

    b_r[i] = Rob->state.r[i];
    s_init_x[i] = Rob->state.x[i];
  }

  /*  ODO2 2D Odometry evaluation. */
  /*    F = ODO2(F,U) performs one step on the pose F of a vehicle, given */
  /*    odometry increments U=[DXY;DTheta] in robot frame. */
  /*  */
  /*    [F,F_r,F_u] = ODO2(F,U) gives the full Jacobians wrt state and odometry  inputs. */
  /*  Input: */
  /*  s_init              - robot.state */
  /*  u                     - dx, dy, dt in robot coordinate system */
  r11 = Rob->state;
  dv23[0] = cos(Rob->state.x[2]);
  dv23[2] = -sin(Rob->state.x[2]);
  dv23[1] = sin(Rob->state.x[2]);
  dv23[3] = cos(Rob->state.x[2]);
  for (i21 = 0; i21 < 2; i21++) {
    d7 = 0.0;
    for (i = 0; i < 2; i++) {
      d7 += dv23[i21 + (i << 1)] * Rob->con.u[i];
    }

    r11.x[i21] = d7 + Rob->state.x[i21];
  }

  r11.x[2] = Rob->state.x[2] + Rob->con.u[2];

  /*  %% Jacobians */
  /*   */
  /*  syms x y a real */
  /*  syms dx dy da real */
  /*  Fi.x=[x;y;a]; */
  /*  Dx = [dx;dy]; */
  /*  Dv = [da]; */
  /*  u  = [Dx;Dv]; */
  /*   */
  /*  [F, F_r, F_u] = odo2(Fi.x,u); */
  /*   */
  /*  F_rs = jacobian(F,Fi.x); */
  /*  F_us = jacobian(F,u); */
  /*   */
  /*  simplify(F_rs-F_r) */
  /*  simplify(F_us-F_u) */
  Rob->state = r11;

  /*  update Map  - mean only */
  /*   update Map  - covariance only */
  b_Map[0] = 1.0;
  b_Map[3] = 0.0;
  b_Map[6] = -Rob->con.u[1] * cos(s_init_x[2]) - Rob->con.u[0] * sin(s_init_x[2]);
  b_Map[1] = 0.0;
  b_Map[4] = 1.0;
  b_Map[7] = Rob->con.u[0] * cos(s_init_x[2]) - Rob->con.u[1] * sin(s_init_x[2]);
  dv24[0] = cos(s_init_x[2]);
  dv24[3] = -sin(s_init_x[2]);
  dv24[6] = 0.0;
  dv24[1] = sin(s_init_x[2]);
  dv24[4] = cos(s_init_x[2]);
  dv24[7] = 0.0;
  for (i = 0; i < 3; i++) {
    Map->x[(int)r11.r[i] - 1] = r11.x[i];
    b_Map[2 + 3 * i] = iv1[i];
    dv24[2 + 3 * i] = iv1[i];
  }

  predictBlockEkf(Map, b_r, b_Map, Rob->con.U, dv24);

  /*   update Rob  - copy from Map */
  for (i21 = 0; i21 < 3; i21++) {
    for (i = 0; i < 3; i++) {
      b_Map[i + 3 * i21] = Map->P[((int)r11.r[i] + 303 * ((int)r11.r[i21] - 1))
        - 1];
      Rob->state.P[i + 3 * i21] = b_Map[i + 3 * i21];
    }
  }

  /*      case  {'constVel'} % constant velocity */
  /*  */
  /*          % motion model of the robot: mean and Jacobians */
  /*          [Map.x(r), F_x, F_u] = constVel(Map.x(r),Rob.con.u,Tim.dt); */
  /*  */
  /*          % update Rob and Map structures - mean only */
  /*          Rob = map2rob(Rob); */
  /*  */
  /*          % Covariances matrix update */
  /*          predictBlockEkf(r, F_x, Rob.con.U, F_u); */
  /*  */
  /*  */
  /*      case  {'odometry3D'}  % 3D odometry */
  /*  */
  /*          % motion model of the robot: mean and Jacobians */
  /*          [Rob.frame, F_x, F_u]   = odo3(Rob.frame,Rob.con.u); */
  /*  */
  /*          % update Rob and Map structures - mean only */
  /*          Map.x(Rob.frame.r) = Rob.frame.x; */
  /*  */
  /*          % Covariances matrix update */
  /*          predictBlockEkf(r, F_x, Rob.con.U, F_u); */
  /*  */
  /*          % update Rob.state.x - added by S */
  /*          Rob.state.x = Map.x(Rob.state.r); */
  /*  */
  /*      otherwise */
  /*  */
  /*          error('??? Unknown motion model ''%s''.',Rob.motion); */
  /*  end */
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
  /*    SLAMTB is Copyright 2009 */
  /*    by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol */
  /*    @ LAAS-CNRS. */
  /*    See on top of this file for its particular copyright. */
  /*    # END GPL LICENSE */
}

/*
 * File trailer for motion.c
 *
 * [EOF]
 */
