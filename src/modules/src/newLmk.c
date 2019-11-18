/*
 * File: newLmk.c
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
#include "newLmk.h"

/* Function Definitions */

/*
 * NEWLMK  Get a new landmark.
 *    NEWLMK(Lmk) returns the index in structure array Lmk() of the first
 *    non-used landmark.
 *
 *    See also NEWRANGE, INITNEWLMKS.
 * Arguments    : Lmk_type Lmk[15]
 *                Map_type *Map
 * Return Type  : double
 */
double newLmk(Lmk_type Lmk[15], Map_type *Map)
{
  double b_lmk;
  double b_v;
  int exitg1;
  double currentInd;
  double nxtInd;
  boolean_T Case1;
  int i31;

  /*    Copyright 2009-2009 Joan sola @ LAAS-CNRS. */
  /*  lmk = find(~[Lmk.used],1,'first'); */
  /*  lmk =1; */
  /*  for i = 1:NUM_LMKS */
  /*      if Lmk(i).used == 0 */
  /*          lmk = i; */
  /*          return */
  /*      end */
  /*  end */
  if (Map->lmk0 == 0.0) {
    /* initialize the linked list: */
    Map->lmk0 = 1.0;
    b_lmk = 1.0;
    Map->lastChecked = 1.0;
    Lmk[0].link2prev = 1.0;
    Lmk[0].link2next = 1.0;
    for (i31 = 0; i31 < 2; i31++) {
      Lmk[0].state.r[i31] = 4.0 + (double)i31;
    }
  } else {
    b_v = Lmk[(int)Map->lmk0 - 1].link2next;
    do {
      exitg1 = 0;
      currentInd = Lmk[(int)b_v - 1].lmk;
      nxtInd = Lmk[(int)b_v - 1].link2next;
      Case1 = (Lmk[(int)b_v - 1].link2next <= Lmk[(int)b_v - 1].lmk);

      /*  this is the last used lmk (points back to first\earlier used lmk) */
      /*  there is a gap (e.g caused by smart delete), insert a link in between. */
      if (Case1 || (Lmk[(int)b_v - 1].link2next - Lmk[(int)b_v - 1].lmk > 1.0))
      {
        exitg1 = 1;
      } else {
        b_v = Lmk[(int)b_v - 1].link2next;
      }
    } while (exitg1 == 0);

    /*  take the next slot: */
    b_lmk = Lmk[(int)b_v - 1].lmk + 1.0;

    /*  update slot pointers (next and prev): */
    Lmk[(int)(Lmk[(int)b_v - 1].lmk + 1.0) - 1].lmk = Lmk[(int)b_v - 1].lmk +
      1.0;
    Lmk[(int)(currentInd + 1.0) - 1].link2prev = currentInd;
    Lmk[(int)(currentInd + 1.0) - 1].link2next = Lmk[(int)b_v - 1].link2next;
    Lmk[(int)(currentInd + 1.0) - 1].state.r[0] = 3.0 + (2.0 * (currentInd + 1.0)
      - 1.0);
    Lmk[(int)(currentInd + 1.0) - 1].state.r[1] = 3.0 + 2.0 * (currentInd + 1.0);

    /*  update the new neighbours: */
    Lmk[(int)nxtInd - 1].link2prev = currentInd + 1.0;
    Lmk[(int)currentInd - 1].link2next = currentInd + 1.0;
    if (Case1) {
      Map->lmk0 = currentInd + 1.0;
    }
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
  /*    SLAMTB is Copyright 2009 */
  /*    by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol */
  /*    @ LAAS-CNRS. */
  /*    See on top of this file for its particular copyright. */
  /*    # END GPL LICENSE */
  return b_lmk;
}

/*
 * File trailer for newLmk.c
 *
 * [EOF]
 */
