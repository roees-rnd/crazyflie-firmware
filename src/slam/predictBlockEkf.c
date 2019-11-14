/*
 * File: predictBlockEkf.c
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
#include "predictBlockEkf.h"

/* Function Definitions */

/*
 * Arguments    : Map_type *Map
 *                const double b_r[3]
 *                const double F_r[9]
 *                const double b_U[9]
 *                const double F_u[9]
 * Return Type  : void
 */
void predictBlockEkf(Map_type *Map, const double b_r[3], const double F_r[9],
                     const double b_U[9], const double F_u[9])
{
  int i22;
  int jj;
  double b_jj;
  int i23;
  double b_Map[9];
  double c_Map[3];
  double b_F_r[9];
  double b_F_u[9];
  double c_F_r[9];
  double c_F_u[9];
  int i24;

  /*  PREDICTBLOCKEKF  Covariance predict in block-defined EKF. */
  /*    PREDICTBLIOCKEKF(r,F_r,U,F_u) performs a covariance prediction step to */
  /*    global map Map by using the prediction Jacobian F_r, referring to range */
  /*    r in the map, and perturbation covariances matrix U and Jacobian F_u. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  m = Map.used; % caution: the range 'm' includes the range 'r' */
  i22 = (int)(Map->len + (1.0 - (b_r[2] + 1.0)));
  for (jj = 0; jj < i22; jj++) {
    b_jj = (b_r[2] + 1.0) + (double)jj;

    /*  update m which does not include 'r' (only mixed rob-lmk coefficients and not rob-rob coeffs). */
    if (Map->used[(int)b_jj - 1]) {
      for (i23 = 0; i23 < 3; i23++) {
        c_Map[i23] = Map->P[((int)b_r[i23] + 303 * ((int)b_jj - 1)) - 1] +
          Map->P[((int)b_jj + 303 * ((int)b_r[i23] - 1)) - 1];
      }

      for (i23 = 0; i23 < 3; i23++) {
        Map->P[((int)b_r[i23] + 303 * ((int)b_jj - 1)) - 1] = 0.0;
        for (i24 = 0; i24 < 3; i24++) {
          Map->P[((int)b_r[i23] + 303 * ((int)b_jj - 1)) - 1] += 0.5 * F_r[i23 +
            3 * i24] * c_Map[i24];
        }
      }

      for (i23 = 0; i23 < 3; i23++) {
        c_Map[i23] = Map->P[((int)b_r[i23] + 303 * ((int)b_jj - 1)) - 1];
      }

      for (i23 = 0; i23 < 3; i23++) {
        Map->P[((int)b_jj + 303 * ((int)b_r[i23] - 1)) - 1] = c_Map[i23];
      }
    }
  }

  for (i22 = 0; i22 < 3; i22++) {
    for (i23 = 0; i23 < 3; i23++) {
      b_Map[i23 + 3 * i22] = Map->P[((int)b_r[i23] + 303 * ((int)b_r[i22] - 1))
        - 1];
    }
  }

  for (i22 = 0; i22 < 3; i22++) {
    for (i23 = 0; i23 < 3; i23++) {
      b_F_r[i22 + 3 * i23] = 0.0;
      b_F_u[i22 + 3 * i23] = 0.0;
      for (i24 = 0; i24 < 3; i24++) {
        b_F_r[i22 + 3 * i23] += F_r[i22 + 3 * i24] * b_Map[i24 + 3 * i23];
        b_F_u[i22 + 3 * i23] += F_u[i22 + 3 * i24] * b_U[i24 + 3 * i23];
      }
    }

    for (i23 = 0; i23 < 3; i23++) {
      c_F_r[i22 + 3 * i23] = 0.0;
      c_F_u[i22 + 3 * i23] = 0.0;
      for (i24 = 0; i24 < 3; i24++) {
        c_F_r[i22 + 3 * i23] += b_F_r[i22 + 3 * i24] * F_r[i23 + 3 * i24];
        c_F_u[i22 + 3 * i23] += b_F_u[i22 + 3 * i24] * F_u[i23 + 3 * i24];
      }
    }
  }

  for (i22 = 0; i22 < 3; i22++) {
    for (i23 = 0; i23 < 3; i23++) {
      Map->P[((int)b_r[i23] + 303 * ((int)b_r[i22] - 1)) - 1] = c_F_r[i23 + 3 *
        i22] + c_F_u[i23 + 3 * i22];
    }
  }

  /*  update the rob-rob coeffs. */
  for (i22 = 0; i22 < 3; i22++) {
    for (i23 = 0; i23 < 3; i23++) {
      b_Map[i23 + 3 * i22] = 0.5 * (Map->P[((int)b_r[i23] + 303 * ((int)b_r[i22]
        - 1)) - 1] + Map->P[((int)b_r[i22] + 303 * ((int)b_r[i23] - 1)) - 1]);
    }
  }

  for (i22 = 0; i22 < 3; i22++) {
    for (i23 = 0; i23 < 3; i23++) {
      Map->P[((int)b_r[i23] + 303 * ((int)b_r[i22] - 1)) - 1] = b_Map[i23 + 3 *
        i22];
    }
  }

  /* Map.P(m,m) = (Map.P(m,m) + Map.P(m,m)')/2; */
  /*  [evec, eval]=eig(Map.P(m,m)); */
  /*  Map.P(m,m)=evec*abs(eval)*evec'; */
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
 * File trailer for predictBlockEkf.c
 *
 * [EOF]
 */
