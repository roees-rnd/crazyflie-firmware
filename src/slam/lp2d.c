/*
 * File: lp2d.c
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
#include "lp2d.h"

/* Function Definitions */

/*
 * LP2D  Line-point signed distance, in 2D.
 *    LP2D(HM,P) is the orthogonal signed distance from point P to the 2-D
 *    homogeneous line HM. The point P can be either an Euclidean 2-vector or
 *    a projective (homogeneous) 3-vector.
 *
 *    [d,D_hm,D_p) = LP2D(...) returns the Jacobians wrt HM and P.
 * Arguments    : const double l[3]
 *                const double p[2]
 *                double *d
 *                double D_l[3]
 *                double D_p[2]
 * Return Type  : void
 */
void lp2d(const double l[3], const double p[2], double *d, double D_l[3], double
          D_p[2])
{
  int i;
  double b_p[3];
  signed char I[4];
  double nn2;
  double nn;
  double d1;
  int i12;
  double b_I[6];

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  if numel(p) == 2 */
  /*  EUC2HMG Euclidean to Homogeneous point transform. */
  /*    EUC2HMG(E) transforms the Euclidean point E onto homogeneous space by */
  /*    appending 1 at the last coordinate. */
  /*  */
  /*    [h,H_e] = EUC2HMG(E) returns the Jacobian of the transformation. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  for (i = 0; i < 2; i++) {
    b_p[i] = p[i];
  }

  b_p[2] = 1.0;

  /*  Jac -- OK */
  for (i = 0; i < 4; i++) {
    I[i] = 0;
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
  /*  else */
  /*      P_p = 1; */
  /*  end */
  nn2 = 0.0;
  for (i = 0; i < 2; i++) {
    I[i + (i << 1)] = 1;
    nn2 += l[i] * l[i];
  }

  nn = sqrt(nn2);
  d1 = 0.0;
  for (i = 0; i < 3; i++) {
    d1 += l[i] * b_p[i];
  }

  *d = d1 / nn;

  /*  jac */
  nn2 *= nn;

  /*  SPLIT  Split vectors into scalars, or matrices into row vectors. */
  /*    [s1,s2,...,sn] = SPLIT(V), with V a vector, returns all its components */
  /*    in scalars s1 ... sn. It is an error if numel(V) < nargout. */
  /*  */
  /*    [v1,...,vn] = SPLIT(M), with M a matrix, returns its rows as separate */
  /*    vectors v1 ... vn. It is an error if size(M,2) < nargout. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  A is vector. We take row or column. */
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
  /*  SPLIT  Split vectors into scalars, or matrices into row vectors. */
  /*    [s1,s2,...,sn] = SPLIT(V), with V a vector, returns all its components */
  /*    in scalars s1 ... sn. It is an error if numel(V) < nargout. */
  /*  */
  /*    [v1,...,vn] = SPLIT(M), with M a matrix, returns its rows as separate */
  /*    vectors v1 ... vn. It is an error if size(M,2) < nargout. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  A is vector. We take row or column. */
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
  D_l[0] = b_p[0] / nn - d1 / nn2 * l[0];
  D_l[1] = b_p[1] / nn - d1 / nn2 * l[1];
  D_l[2] = 1.0 / nn;
  b_p[0] = l[0] / nn;
  b_p[1] = l[1] / nn;
  b_p[2] = l[2] / nn - d1 / nn;
  for (i = 0; i < 2; i++) {
    for (i12 = 0; i12 < 2; i12++) {
      b_I[i12 + 3 * i] = I[i12 + (i << 1)];
    }

    b_I[2 + 3 * i] = 0.0;
    D_p[i] = 0.0;
    for (i12 = 0; i12 < 3; i12++) {
      D_p[i] += b_p[i12] * b_I[i12 + 3 * i];
    }
  }

  /*  jac */
  /*  syms a b c u v w real */
  /*  l = [a;b;c]; */
  /*   */
  /*  %% Euclidean build */
  /*  p = [u;v]; */
  /*  d = lp2d(l,p) */
  /*   */
  /*  D_l = jacobian(d,l) */
  /*  D_p = jacobian(d,p) */
  /*   */
  /*  %% Euclidean test */
  /*  p = [u;v]; */
  /*  [d,D_l,D_p] = lp2d(l,p) */
  /*   */
  /*  D_l - jacobian(d,l) */
  /*  D_p - jacobian(d,p) */
  /*   */
  /*  %% Homog build */
  /*  p = [u;v;w]; */
  /*  d = lp2d(l,p) */
  /*   */
  /*  D_l = jacobian(d,l) */
  /*  D_p = jacobian(d,p) */
  /*   */
  /*  %% Homog test */
  /*  p = [u;v;w]; */
  /*  [d,D_l,D_p] = lp2d(l,p) */
  /*   */
  /*  simplify(D_l - jacobian(d,l)) */
  /*  simplify(D_p - jacobian(d,p)) */
  /*   */
  /*   */
  /*   */
  /*  % ========== End of function - Start GPL license ========== */
  /*   */
  /*   */
  /*  %   # START GPL LICENSE */
  /*   */
  /*  %--------------------------------------------------------------------- */
  /*  % */
  /*  %   This file is part of SLAMTB, a SLAM toolbox for Matlab. */
  /*  % */
  /*  %   SLAMTB is free software: you can redistribute it and/or modify */
  /*  %   it under the terms of the GNU General Public License as published by */
  /*  %   the Free Software Foundation, either version 3 of the License, or */
  /*  %   (at your option) any later version. */
  /*  % */
  /*  %   SLAMTB is distributed in the hope that it will be useful, */
  /*  %   but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  %   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  %   GNU General Public License for more details. */
  /*  % */
  /*  %   You should have received a copy of the GNU General Public License */
  /*  %   along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  % */
  /*  %--------------------------------------------------------------------- */
  /*   */
  /*  %   SLAMTB is Copyright: */
  /*  %   Copyright (c) 2008-2010, Joan Sola @ LAAS-CNRS, */
  /*  %   Copyright (c) 2010-2013, Joan Sola, */
  /*  %   Copyright (c) 2014-    , Joan Sola @ IRI-UPC-CSIC, */
  /*  %   SLAMTB is Copyright 2009  */
  /*  %   by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol */
  /*  %   @ LAAS-CNRS. */
  /*  %   See on top of this file for its particular copyright. */
  /*   */
  /*  %   # END GPL LICENSE */
  /*   */
}

/*
 * File trailer for lp2d.c
 *
 * [EOF]
 */
