/*
 * File: correctBlockEkf.c
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
#include "correctBlockEkf.h"

/* Function Definitions */

/*
 * Arguments    : Map_type *Map
 *                const double b_r[5]
 *                const double H[10]
 *                const double Inn_z[2]
 *                const double Inn_Z[4]
 *                const double Inn_iZ[4]
 * Return Type  : void
 */
void correctBlockEkf(Map_type *Map, const double b_r[5], const double H[10],
                     const double Inn_z[2], const double Inn_Z[4], const double
                     Inn_iZ[4])
{
  int ii;
  double kk;
  double K_[66];
  int i;
  int i29;
  signed char usedInd[33];
  double HinZ[10];
  int indI;
  double dv31[5];
  double Z12;
  double tmp;

  /*  CORRECTBLOCKEKF  Correct in block-defined EKF. */
  /*    CORRECTBLIOCKEKF(r,H,INN) performs a correction step to global map Map */
  /*    by using the observation Jacobian H, referring to range r in the map, */
  /*    and innovation INN. */
  /*  */
  /*    INN is a structure containing: */
  /*        .z      the innovation,         z  = y-h(x) */
  /*        .Z      its covariances matrix, Z  = HPH' + R */
  /*        .iZ     the inverse covariance, iZ = Z^-1. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  global Map */
  /*  global Debug */
  ii = 0;
  kk = 0.0;

  /*  kk=int32(0); */
  /*  Kalman gain */
  /*  K = Map.P(Map.used,r) * H' * Inn.iZ;   % K = PH'Z^-1 */
  /*  TODO: use smart size initialization and not hardcoded sizes. */
  memset(&K_[0], 0, 66U * sizeof(double));
  for (i = 0; i < 33; i++) {
    usedInd[i] = 0;
  }

  for (i29 = 0; i29 < 5; i29++) {
    for (i = 0; i < 2; i++) {
      HinZ[i29 + 5 * i] = 0.0;
      for (indI = 0; indI < 2; indI++) {
        HinZ[i29 + 5 * i] += H[indI + (i29 << 1)] * Inn_iZ[indI + (i << 1)];
      }
    }
  }

  /* for ii=1:(Map.numUsed*2+3) */
  while (kk < Map->numUsed * 2.0 + 3.0) {
    if (Map->used[ii]) {
      kk++;
      for (i29 = 0; i29 < 5; i29++) {
        dv31[i29] = 0.5 * (Map->P[((int)b_r[i29] + 33 * ii) - 1] + Map->P[ii +
                           33 * ((int)b_r[i29] - 1)]);
      }

      for (i29 = 0; i29 < 2; i29++) {
        K_[((int)kk + 33 * i29) - 1] = 0.0;
        for (i = 0; i < 5; i++) {
          K_[((int)kk + 33 * i29) - 1] += dv31[i] * HinZ[i + 5 * i29];
        }
      }

      usedInd[(int)kk - 1] = (signed char)(ii + 1);
    }

    ii++;

    /*  TODO: smark exit when reached number of lmks. */
  }

  /*  if  isempty(coder.target) */
  /*      a0=Map.x(3); */
  /*      p0=Map.x(1:2); */
  /*  end */
  /*  State and Covariance update: */
  /*  Map.x(Map.used)          = Map.x(Map.used)                          + K*Inn.z; */
  /*  Map.P(Map.used,Map.used) = Map.P(Map.used,Map.used) - K*Inn.Z*K'; */
  /*  and symetry preservation: */
  /*  Map.P(indI,indJ)=0.5*(Map.P(indI,indJ)+Map.P(indJ,indI))-tmp; */
  Z12 = 0.5 * (Inn_Z[2] + Inn_Z[1]);
  for (ii = 0; ii < (int)kk; ii++) {
    indI = usedInd[ii] - 1;
    tmp = 0.0;
    for (i29 = 0; i29 < 2; i29++) {
      tmp += K_[ii + 33 * i29] * Inn_z[i29];
    }

    Map->x[usedInd[ii] - 1] += tmp;
    for (i = 0; i <= ii; i++) {
      tmp = K_[i] * (K_[ii] * Inn_Z[0] + K_[33 + ii] * Z12) + K_[33 + i] *
        (K_[ii] * Z12 + K_[33 + ii] * Inn_Z[3]);
      if (1.0 + (double)i < 1.0 + (double)ii) {
        Map->P[indI + 33 * (usedInd[i] - 1)] = 0.5 * (Map->P[indI + 33 *
          (usedInd[i] - 1)] + Map->P[(usedInd[i] + 33 * indI) - 1]) - tmp;
        Map->P[(usedInd[i] + 33 * indI) - 1] = Map->P[indI + 33 * (usedInd[i] -
          1)];
      } else {
        Map->P[indI + 33 * indI] -= tmp;
      }
    }
  }

  /*  figure(33) */
  /*  [Xbefor,Ybefor] = cov2elli(Map.x(8:9),Map.P(8:9,8:9),3,20); */
  /*  hold on; plot(Xbefor,Ybefor,'g.-','LineWidth',2); */
  /*  mean and cov. updates */
  /*   M=K*Inn.Z*K'; */
  /*   if (abs(M(2,2))>0.1 || abs(M(1,1))>0.1),keyboard,end */
  /*  Regularization */
  /*  Map.P(Map.used,Map.used)  = Map.P(Map.used,Map.used) +eps*eye(size(Map.P(Map.used,Map.used) )); */
  /*  Force symmetry */
  /*  */
  /*  NOTE: this line of code has been moved to correctKnownLmks so that it is */
  /*  performed once per SLAM iteration, and not once per landmark correction. */
  /*  This is so done for speed reasons. */
  /*  */
  /*  Map.P(Map.used,Map.used) = (Map.P(Map.used,Map.used) + Map.P(Map.used,Map.used)')/2; */
  /*  figure(33) */
  /*   [Xcorr,Ycorr] = cov2elli(Map.x(8:9),Map.P(8:9,8:9),3,20); */
  /*  hold on; plot(Xcorr,Ycorr,'r.-','LineWidth',2); hold off */
  /*  legend('measured','in Map before','in Map after') */
  /*  if  isempty(coder.target) */
  /*      if Debug */
  /*          %a1=Map.x(4:7); */
  /*          a1=Map.x(3); */
  /*          p1=Map.x(1:2); */
  /*          d = norm(p1-p0); %d_rob_pos */
  /*          e0=(a0)*180/pi; %rob_orient (was q2e) */
  /*          e1=(a1)*180/pi; %rob_orient (was q2e) */
  /*           */
  /*          figure(88); */
  /*          aa=get(get(88,'CurrentAxes'),'children'); */
  /*          if ~isempty(aa) */
  /*              ind=aa(1).XData(end); */
  /*              if strcmp(aa(1).Tag,'ang') && strcmp(aa(2).Tag,'pos') */
  /*                  set(aa(1),'XData',[aa(1).XData ind+1],'YData',[aa(1).YData e1(end)-e0(end)]) */
  /*                  set(aa(2),'XData',[aa(2).XData ind+1],'YData',[aa(2).YData d]) */
  /*              elseif strcmp(aa(2).Tag,'ang') && strcmp(aa(1).Tag,'pos') */
  /*                  set(aa(2),'XData',[aa(2).XData ind+1],'YData',[aa(2).YData e1(end)-e0(end)]) */
  /*                  set(aa(1),'XData',[aa(1).XData ind+1],'YData',[aa(1).YData d]) */
  /*              else */
  /*                  assert(false); */
  /*              end */
  /*          else */
  /*              hold all;plot(1,e1(end)-e0(end),'Tag','ang'); */
  /*              plot(1,d,'Tag','pos'); legend('ang fix', 'pos fix') */
  /*          end */
  /*           */
  /*      end */
  /*  end */
  /*  % This is a way to understand the update: */
  /*  % the information from the innovation corresponding to x_rob, y_rob, theta_rob, rho_lmk, phi_lmk: */
  /*  (H' * Inn.iZ*Inn.z) */
  /*  % the uncertainty of x_rob, y_rob, theta_rob, rho_lmk-rob, phi_lmk-rob: */
  /*  Map.P(1:3,r) */
  /*  % Total update of rob state: */
  /*  Map.P(1:3,r)*(H' * Inn.iZ*Inn.z) */
  /*  % Each innovation component influence on the Total update of rob state: */
  /*  Map.P(1:3,r).*(H' * Inn.iZ*Inn.z)' */
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
 * File trailer for correctBlockEkf.c
 *
 * [EOF]
 */
