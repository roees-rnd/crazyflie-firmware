/*
 * File: propagateUncertainty.c
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
#include "propagateUncertainty.h"

/* Function Definitions */

/*
 * PROPAGATEUNCERTAINTY  Non-linear propagation of Gaussian uncertainty.
 *    [y,Y] = PROPAGATEUNCERTAINTY(x,X,@f) propagates the Gaussian
 *    uncertainty N(x,X) through function f(), resulting in the Gaussian
 *    approximation N(y,Y).
 *
 *    The propagation is made by using the Jacobian of f(), thus the function
 *    in file f.m must have the form [y,F] = f(x), with F the Jacobian of f()
 *    wrt x.
 *
 *    See also Q2EG.
 * Arguments    : const double b_x[2]
 *                const double X[4]
 *                double b_y[3]
 *                double Y[9]
 * Return Type  : void
 */
void propagateUncertainty(const double b_x[2], const double X[4], double b_y[3],
  double Y[9])
{
  double a;
  double b;
  double Y_x[6];
  int i13;
  int i14;
  double b_Y_x[6];
  int i15;

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  RT2HMGLIN  Rho-theta to homogeneous line conversion. */
  /*    RT2HMGLIN(RT) transforms the line RT=[rho;theta] into a homogeneous */
  /*    line [a;b;c]. A homogeneous line is such that ax+by+c = 0 for any point */
  /*    [x;y] of the line. For homogeneous points [x;y;t] we have ax+by+ct = 0. */
  /*  */
  /*    [hm, HM_rt] = RT2HMGLIN(...) returns the Jacobian matrix. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  a = cos(b_x[1]);
  b = sin(b_x[1]);
  b_y[0] = a;
  b_y[1] = b;
  b_y[2] = -b_x[0];
  Y_x[0] = 0.0;
  Y_x[3] = -b;
  Y_x[1] = 0.0;
  Y_x[4] = a;
  for (i13 = 0; i13 < 2; i13++) {
    Y_x[2 + 3 * i13] = -1.0 + (double)i13;
  }

  /*  */
  /*   */
  /*  syms rho theta real */
  /*   */
  /*  rt = [rho;theta]; */
  /*   */
  /*  [hm,HMrt] = rt2hmgLin(rt) */
  /*   */
  /*  HMrt - jacobian(hm,rt) */
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
  for (i13 = 0; i13 < 3; i13++) {
    for (i14 = 0; i14 < 2; i14++) {
      b_Y_x[i13 + 3 * i14] = 0.0;
      for (i15 = 0; i15 < 2; i15++) {
        b_Y_x[i13 + 3 * i14] += Y_x[i13 + 3 * i15] * X[i15 + (i14 << 1)];
      }
    }

    for (i14 = 0; i14 < 3; i14++) {
      Y[i13 + 3 * i14] = 0.0;
      for (i15 = 0; i15 < 2; i15++) {
        Y[i13 + 3 * i14] += b_Y_x[i13 + 3 * i15] * Y_x[i14 + 3 * i15];
      }
    }
  }

  /*  test */
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
 * File trailer for propagateUncertainty.c
 *
 * [EOF]
 */
