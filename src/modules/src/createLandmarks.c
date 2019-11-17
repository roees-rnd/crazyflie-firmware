/*
 * File: createLandmarks.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/createLandmarks.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Definitions */

/*
 * CREATELANDMARKS  Create Lmk structure array.
 *    Lmk = CREATELANDMARKS(Landmark) creates the structure array Lmk() to be
 *    used as SLAM data. The input Landmark{}  is a cell array of structures
 *    as specified by the user in userData.m. There must be one Landmark{}
 *    per each landmark type considered. See userData.m for details.
 * Arguments    : Lmk_type Lmk[15]
 * Return Type  : void
 */
void createLandmarks(Lmk_type Lmk[15])
{
  Lmk_type Lmk_;
  int i;
  static const char cv0[6] = { 'h', 'o', 'r', 'L', 'i', 'n' };

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /* DEBUG R */
  /*      tmp= cell(1,coder.const(NUM_LMKS)); */
  /*      for i=1:Opt.map.numLmks */
  /*          tmp{i}=i; */
  /*      end */
  /*       */
  /*      %% preallocation of output for coder */
  /*      Lmk = struct('lmk', tmp, 'id', 0, 'type',  'horLin', 'used', false,... */
  /*          'state', struct('r', zeros(2,1)), 'parEndp', struct('t', zeros(2,1),'e',zeros(4,1),'nrm',zeros(2,1)),... */
  /*          'nSearch', 0, 'nMatch', 0, 'nInlier', 0, 'link2prev', 0, 'link2next', 0); */
  /*       */
  /*       */
  Lmk_.lmk = 0.0;
  Lmk_.id = 0.0;
  for (i = 0; i < 6; i++) {
    Lmk_.type[i] = cv0[i];
  }

  Lmk_.used = false;
  for (i = 0; i < 2; i++) {
    Lmk_.state.r[i] = 0.0;
  }

  for (i = 0; i < 2; i++) {
    Lmk_.parEndp.t[i] = 0.0;
  }

  for (i = 0; i < 4; i++) {
    Lmk_.parEndp.e[i] = 0.0;
  }

  for (i = 0; i < 2; i++) {
    Lmk_.parEndp.nrm[i] = 0.0;
  }

  Lmk_.nSearch = 0.0;
  Lmk_.nMatch = 0.0;
  Lmk_.nInlier = 0.0;
  Lmk_.link2prev = 0.0;
  Lmk_.link2next = 0.0;
  for (i = 0; i < 15; i++) {
    Lmk_.lmk = 1.0 + (double)i;
  }

  for (i = 0; i < 15; i++) {
    Lmk[i] = Lmk_;
  }

  for (i = 0; i < 15; i++) {
    Lmk[i].lmk = 1.0 + (double)i;
  }

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
 * File trailer for createLandmarks.c
 *
 * [EOF]
 */
