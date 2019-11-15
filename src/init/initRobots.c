/*
 * File: initRobots.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
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
#include "initRobots.h"

/* Function Definitions */

/*
 * INITROBOTS Initialize robots in Map.
 *    Rob = INITROBOTS(Rob) initializes all robots in Rob() in the global map
 *    Map. It does so by:
 *        getting a range of free states for the robot
 *        assigning it to the appropriate fields of Rob
 *        setting Rob's mean and cov. matrices in Map
 *        setting all Map.used positions in the range to true
 * Arguments    : struct0_T *Map
 *                Rob_type *Rob
 * Return Type  : void
 */
void initRobots(Map_type *Map, Rob_type *Rob)
{
  int i;
  boolean_T b_x[33];
  int count;
  double sr[3];
  boolean_T exitg1;
  double b_Rob[9];

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*    Copyright 2013 Joan Sola */
  /*  add to map and set state range */
  /*  ADDTOMAP  Add Gaussian to Map. */
  /*    ADDTOMAP(L,P_LL) adds the Gaussian N(L,P_LL) to the global EKF-map Map, */
  /*    at positions that are empty, and returns the range of positions where */
  /*    it has been added. ADDTOMAP adds mean L in Map.x and covariances P_LL */
  /*    in the block-diagonal of Map.P.  */
  /*  */
  /*    For example, in the case of a map which has all used states contiguous, */
  /*    the new state and covariance are appended at the end of the existing */
  /*    one: */
  /*  */
  /*            | x |        | P     0   | */
  /*        x = |   |    P = |           | */
  /*            | L |        | 0    P_LL | */
  /*  */
  /*  */
  /*    Map is a global structure, containing: */
  /*        .used   a vector of logicals indicating used positions  */
  /*        .x      the state vector  */
  /*        .P      the covariances matrix  */
  /*        .size   the Map's maximum size, numel(Map.x) */
  /*  */
  /*    ADDTOMAP(L,P_LL,P_LX) accepts the cross variance matrix between L and the */
  /*    currently used states in Map.x, so that the covariance is augmented */
  /*    with */
  /*  */
  /*            |   P     P_LX' | */
  /*        P = |               | */
  /*            | P_LX    P_LL  | */
  /*     */
  /*    ADDTOMAP(L,P_LL,P_LX,R) or ADDTOMAP(L,P_LL,[],R) permits forcing the */
  /*    range R as input. */
  /*  */
  /*    See also NEWRANGE, USEDRANGE. */
  /*  */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  global Map */
  /*  NEWRANGE  New map range. */
  /*    NEWRANGE(SIZE) returns a range vector with the first SIZE non-used */
  /*    positions in the global EKF-map Map. */
  /*  */
  /*    For example, the code */
  /*        global Map */
  /*        Map.used = [0 0 0 1 1 0 1 1 0 0 0 1 0 0]'; */
  /*        r = newRange(6) */
  /*  */
  /*    returns r = [1 2 3 6 9 10]' */
  /*  */
  /*    See also ADDTOMAP. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  global Map */
  for (i = 0; i < 33; i++) {
    b_x[i] = !Map->used[i];
  }

  /*  replacement for I = FIND(X,K,'first') - Matlab coder doesn't support variable sizing in original find */
  /*  returns at most the first K indices corresponding to  the nonzero entries of the array X.  K must be a positive integer,  */
  for (i = 0; i < 3; i++) {
    sr[i] = 0.0;
  }

  count = 1;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 33)) {
    if (b_x[i]) {
      sr[count - 1] = 1.0 + (double)i;
      count++;
      if (count > 3) {
        exitg1 = true;
      } else {
        i++;
      }
    } else {
      i++;
    }
  }

  /*  r = zeros(vecSize,1); */
  /*  if vecSize > 0 */
  /*      r = find(~Map.used,vecSize,'first'); */
  /*  else */
  /*      r = []; */
  /*  end */
  /*  test */
  /*  global Map */
  /*   */
  /*  Map.used = [0 0 0 1 1 0 1 1 0 0 0 1 0 0]'; */
  /*   */
  /*  nr = newRange(6) */
  /*  ur = usedRange() */
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
  /*  code before coder */
  /*  global Map */
  /*   */
  /*  % parse inputs */
  /*  if nargin == 2 */
  /*      P_LX = [] ; */
  /*  end */
  /*  if nargin <= 3 */
  /*      r = newRange(numel(L)); */
  /*  end */
  /*   */
  /*  % add to map */
  /*  Map.x(r)    = L; */
  /*  Map.P(r,r)  = P_LL; */
  /*  if(size(P_LX)~=0) */
  /*      mr = usedRange(); */
  /*      Map.P(r,mr) =  P_LX ; */
  /*      Map.P(mr,r) =  P_LX' ; */
  /*  end ; */
  /*   */
  /*  Map.used(r) = true; */
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
  /*  state range in map */
  for (i = 0; i < 3; i++) {
    Map->x[(int)sr[i] - 1] = Rob->state.x[i];
    for (count = 0; count < 3; count++) {
      b_Rob[count + 3 * i] = Rob->state.P[count + 3 * i];
      Map->P[((int)sr[count] + 33 * ((int)sr[i] - 1)) - 1] = b_Rob[count + 3 * i];
    }

    Map->used[(int)sr[i] - 1] = true;
    Rob->state.r[i] = sr[i];
  }

  /*  set frame range - model dependent */
  /*      switch Rob(rob).motion */
  /*           */
  /*          case {'constVel', 'odometry'} */
  /*               */
  /*              % frame is 7 first states */
  /*              %Rob(rob).frame.r = sr(1:7); */
  /*               */
  /*          otherwise */
  /*               */
  /*              error('??? Unknown motion model %s', Rob(rob).motion); */
  /*   */
  /*      end */
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
}

/*
 * File trailer for initRobots.c
 *
 * [EOF]
 */
