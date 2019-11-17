/*
 * File: projectLmk.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/projectLmk.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/norm.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Definitions */

/*
 * PROJECTLMK  Project landmark estimate into sensor's measurement space.
 *    Obs = PROJECTLMK(Rob,Sen,Lmk,Obs) projects the landmark Lmk into sensor
 *    Sen mounted on robot Rob, and updates the information of the
 *    observation structure Obs. The observation model is determined from
 *    Sen.type and Lmk.type. It is an error if no model exists for the chosen
 *    Sen-Lmk pair.
 *
 *    The updated fields in Obs are:
 *        .sid        % sensor ID
 *        .lid        % landmark ID
 *        .ltype      % landmark type
 *        .vis        % flag: true if landmark is visible
 *        .meas.R     % measurement noise cov. matrix
 *        .exp.e      % expectation's mean
 *        .exp.E      % expectation's covariances matrix
 *        .exp.um     % expectation's uncertainty measure
 *        .Jac.E_r    % Jacobian wrt robot frame
 *        .Jac.E_s    % Jacobian wrt sensor frame
 *        .Jac.E_l    % Jacobian wrt landmark state
 *
 *    See also OBSERVEKNOWNLMKS.
 * Arguments    : const double Map_x[303]
 *                const double Map_P[91809]
 *                const double Rob_state_x[3]
 *                const double Rob_state_r[3]
 *                const char b_Sen_type[7]
 *                double Sen_par_maxRange
 *                const char b_Lmk_type[6]
 *                const double Lmk_state_r[2]
 *                const double Lmk_parEndp_t[2]
 *                const double Lmk_parEndp_e[4]
 *                const double Lmk_parEndp_nrm[2]
 *                Obs_type *Obs
 *                const double Raw_endpWorld[4]
 *                const double Raw_nrm[2]
 *                double Opt_correct_isVisOrtThrsh
 *                const double Opt_correct_isVisFarThrsh[2]
 *                double Opt_maxDistVis
 * Return Type  : void
 */
void projectLmk(const double Map_x[303], const double Map_P[91809], const double
                Rob_state_x[3], const double Rob_state_r[3], const char
                b_Sen_type[7], double Sen_par_maxRange, const char b_Lmk_type[6],
                const double Lmk_state_r[2], const double Lmk_parEndp_t[2],
                const double Lmk_parEndp_e[4], const double Lmk_parEndp_nrm[2],
                Obs_type *Obs, const double Raw_endpWorld[4], const double
                Raw_nrm[2], double Opt_correct_isVisOrtThrsh, const double
                Opt_correct_isVisFarThrsh[2], double Opt_maxDistVis)
{
  double E_rf[9];
  int i28;
  boolean_T b_vis;
  double b_E_l[6];
  int i;
  double b_e[3];
  boolean_T b_bool;
  char t0_f1[7];
  static const char cv6[7] = { 'p', 's', 'e', 'u', 'd', 'o', 'S' };

  int exitg1;
  int c_bool;
  boolean_T guard1 = false;
  char t1_f1[6];
  static const char cv7[6] = { 'h', 'o', 'r', 'L', 'i', 'n' };

  double rslr[5];
  int d_bool;
  double dx;
  double b_Map_P[4];
  double c_E_l[6];
  int jy;
  double b_E[9];
  boolean_T guard2 = false;
  double b_Lmk_parEndp_e[2];
  int j;
  signed char ipiv[3];
  int c;
  double dy;
  int k;
  int ix;
  double segLengPow2;
  int ijA;
  static const signed char iv2[3] = { 0, 0, 1 };

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  global Map */
  /*  PREVIOUS TASKS */
  /*  get landmark range and mean */
  /*  lmk range in Map */
  /*  lmk mean */
  /*  Coder variables definition */
  memset(&E_rf[0], 0, 9U * sizeof(double));
  for (i28 = 0; i28 < 6; i28++) {
    b_E_l[i28] = 0.0;
  }

  b_vis = false;
  for (i = 0; i < 3; i++) {
    b_e[i] = 0.0;
  }

  /*  PROJECTION FUNCTION */
  /*  explore all sensor and landmark types */
  for (i28 = 0; i28 < 7; i28++) {
    t0_f1[i28] = cv6[i28];
  }

  b_bool = false;
  i = 0;
  do {
    exitg1 = 0;
    if (i + 1 < 8) {
      if (b_Sen_type[i] != t0_f1[i]) {
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (b_bool) {
    c_bool = 0;
  } else {
    c_bool = -1;
  }

  guard1 = false;
  switch (c_bool) {
   case 0:
    for (i28 = 0; i28 < 6; i28++) {
      t1_f1[i28] = cv7[i28];
    }

    b_bool = false;
    i = 0;
    do {
      exitg1 = 0;
      if (i + 1 < 7) {
        if (b_Lmk_type[i] != t1_f1[i]) {
          exitg1 = 1;
        } else {
          i++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_bool) {
      d_bool = 0;
    } else {
      d_bool = -1;
    }

    switch (d_bool) {
     case 0:
      dx = 0.0;
      for (i28 = 0; i28 < 2; i28++) {
        dx += Raw_nrm[i28] * Lmk_parEndp_nrm[i28];
      }

      if (dx >= Opt_correct_isVisOrtThrsh) {
        guard2 = false;
        if (fabs(Lmk_parEndp_t[1] - Lmk_parEndp_t[0]) >
            Opt_correct_isVisFarThrsh[0]) {
          guard2 = true;
        } else {
          for (i = 0; i < 2; i++) {
            b_Lmk_parEndp_e[i] = Lmk_parEndp_e[i] - Raw_endpWorld[i];
          }

          if (b_norm(b_Lmk_parEndp_e) < Opt_correct_isVisFarThrsh[1]) {
            guard2 = true;
          }
        }

        if (guard2) {
          /*  This function returns visible if center of robot is closer than maxRange */
          /*  to segment (defined by: lmkEndp1, lmkEndp2). */
          /*  Note: sensor direction is not used. (TODO) */
          /*  if distnace between robot and line defined by lmkRho, lmkPhi  is less */
          /*  than maxRange - there is a chance that lmk is visible */
          if (fabs((cos(Map_x[(int)Lmk_state_r[1] - 1]) * Rob_state_x[0] + sin
                    (Map_x[(int)Lmk_state_r[1] - 1]) * Rob_state_x[1]) - Map_x
                   [(int)Lmk_state_r[0] - 1]) < Sen_par_maxRange) {
            /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
            /*  Calculate Alpha: */
            /*  Alpha - point on lmk closest to robot coordinate. */
            /*  Alpha = argmin{  ||alpha*endp1+(1-alpha)*endp2  -robPose(1:2)||^2  } */
            /*  derivation: */
            /*  syms alpha x0 y0 x1 y1 xr yr real; */
            /*  L_tmp=alpha*[x0; y0]+(1-alpha)*[x1; y1]-[xr;yr] */
            /*  solve(diff(L_tmp'*L_tmp,alpha),alpha) */
            dx = Lmk_parEndp_e[0] - Lmk_parEndp_e[2];
            dy = Lmk_parEndp_e[1] - Lmk_parEndp_e[3];
            segLengPow2 = dx * dx + dy * dy;
            dy = (dx * (Lmk_parEndp_e[0] - Rob_state_x[0]) + dy *
                  (Lmk_parEndp_e[1] - Rob_state_x[1])) / segLengPow2;

            /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
            /* 300; */
            dx = Opt_maxDistVis / sqrt(segLengPow2);
            if ((dy < 1.0 + dx) && (dy > -dx)) {
              b_vis = true;
            }
          }

          /*  Change because coder doesn't support referencing a component from array of non-scalar values */
        }
      }

      if (!b_vis) {
        Obs->vis = false;
        Obs->updated = false;
      } else {
        /*  Change because coder doesn't support referencing a component from array of non-scalar values */
        /*  RT2HMGLIN  Rho-theta to homogeneous line conversion. */
        /*    RT2HMGLIN(RT) transforms the line RT=[rho;theta] into a homogeneous */
        /*    line [a;b;c]. A homogeneous line is such that ax+by+c = 0 for any point */
        /*    [x;y] of the line. For homogeneous points [x;y;t] we have ax+by+ct = 0. */
        /*  */
        /*    [hm, HM_rt] = RT2HMGLIN(...) returns the Jacobian matrix. */
        /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
        dx = cos(Map_x[(int)Lmk_state_r[1] - 1]);
        dy = sin(Map_x[(int)Lmk_state_r[1] - 1]);
        b_e[0] = dx;
        b_e[1] = dy;
        b_e[2] = -Map_x[(int)Lmk_state_r[0] - 1];
        b_E_l[0] = 0.0;
        b_E_l[3] = -dy;
        b_E_l[1] = 0.0;
        b_E_l[4] = dx;
        for (i28 = 0; i28 < 2; i28++) {
          b_E_l[2 + 3 * i28] = -1.0 + (double)i28;
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
        /*  needed to compute innovation distance in innovation.m */
        /*  if rho<0 */
        /*      phi=phi-pi; */
        /*  end */
        /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
        /* %% This is the calculation of : rhoPhi2xyTheta */
        /* %% this is dRho / dx,y,tta */
        /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
        /*  % tta phi rho x y are current (old) state values */
        /*  % t is for aux only (the distance along the new line) */
        /*  syms tta phi rho x y t rhoNew dtta dx dy real */
        /*   */
        /*  % assumptions: */
        /*  % 1. the relative angle between robot and lmk is constant. */
        /*  % 2. the relative distance between robot and lmk is constant. */
        /*   */
        /*  % unit vector in the direction of rho old: */
        /*  u_RhoOld=[cos(phi);sin(phi)]; */
        /*  % unit vector in the direction of rho new (because of assumption 1): */
        /*  u_rhoNew=[cos(dtta) -sin(dtta); sin(dtta) cos(dtta)]*u_RhoOld; */
        /*   */
        /*  % rhoGal - the distance (scalar) of robot (x,y) from line (rhoOld,phiOld): */
        /*  rhoGal=rho-(x*cos(phi)+y*sin(phi)); */
        /*   */
        /*   */
        /*  %Lhs=r*[cos(phi)-dtta*sin(phi);cos(phi)*dtta+sin(phi)]  */
        /*  Lhs=rhoNew*u_rhoNew; */
        /*   */
        /*  Rhs=[x+dx;y+dy]+rhoGal*u_rhoNew+t*[0 -1; 1 0]*u_rhoNew; */
        /*   */
        /*   */
        /*  [solr, solt]=solve(Lhs==Rhs,[rhoNew, t]); */
        /*  %% */
        /*  drdx=simplify(subs(subs(subs(diff(solr,dx),dx,0),dy,0),dtta,0)); */
        /*  drdy=simplify(subs(subs(subs(diff(solr,dy),dx,0),dy,0),dtta,0)); */
        /*  drdtta=simplify(subs(subs(subs(diff(solr,dtta),dx,0),dy,0),dtta,0)); */
        /*   */
        /*  phiNew=phi+dtta; */
        /*  dpdx=simplify(subs(subs(subs(diff(phiNew,dx),dx,0),dy,0),dtta,0)); */
        /*  dpdy=simplify(subs(subs(subs(diff(phiNew,dy),dx,0),dy,0),dtta,0)); */
        /*  dpdtta=simplify(subs(subs(subs(diff(phiNew,dtta),dx,0),dy,0),dtta,0)); */
        /*  J=[drdx drdy drdtta;dpdx dpdy dpdtta] */
        /*   */
        /*   */
        /*   */
        c_E_l[0] = -cos(Map_x[(int)Lmk_state_r[1] - 1]);
        c_E_l[2] = -sin(Map_x[(int)Lmk_state_r[1] - 1]);
        c_E_l[4] = -(Rob_state_x[1] * cos(Map_x[(int)Lmk_state_r[1] - 1]) -
                     Rob_state_x[0] * sin(Map_x[(int)Lmk_state_r[1] - 1]));
        for (i28 = 0; i28 < 3; i28++) {
          c_E_l[1 + (i28 << 1)] = -(double)iv2[i28];
        }

        for (i28 = 0; i28 < 3; i28++) {
          for (i = 0; i < 3; i++) {
            E_rf[i28 + 3 * i] = 0.0;
            for (jy = 0; jy < 2; jy++) {
              E_rf[i28 + 3 * i] += b_E_l[i28 + 3 * jy] * c_E_l[jy + (i << 1)];
            }
          }
        }

        /* *xyTheta2fullRobState; % jacobian of expectation wrt robot frame  (expectation is hom. line [3x1] , robot frame (x,y,z,q1,....q4) [7x1]  ) */
        guard1 = true;
      }
      break;

     default:
      guard1 = true;
      break;
    }
    break;

   default:
    /*  unknown Sensor type */
    guard1 = true;
    break;
  }

  if (guard1) {
    /*  sensor type */
    /*  COVARIANCES */
    for (i = 0; i < 3; i++) {
      rslr[i] = Rob_state_r[i];
    }

    /*  range of robot and landmark */
    /*  Expectation covariances matrix */
    /*  E = E_rsl*Map.P(rslr,rslr)*E_rsl' ; */
    for (i = 0; i < 2; i++) {
      rslr[i + 3] = Lmk_state_r[i];
      for (i28 = 0; i28 < 2; i28++) {
        b_Map_P[i28 + (i << 1)] = Map_P[((int)Lmk_state_r[i28] + 303 * ((int)
          Lmk_state_r[i] - 1)) - 1];
      }
    }

    for (i28 = 0; i28 < 3; i28++) {
      for (i = 0; i < 2; i++) {
        c_E_l[i28 + 3 * i] = 0.0;
        for (jy = 0; jy < 2; jy++) {
          c_E_l[i28 + 3 * i] += b_E_l[i28 + 3 * jy] * b_Map_P[jy + (i << 1)];
        }
      }

      for (i = 0; i < 3; i++) {
        b_E[i28 + 3 * i] = 0.0;
        for (jy = 0; jy < 2; jy++) {
          b_E[i28 + 3 * i] += c_E_l[i28 + 3 * jy] * b_E_l[i + 3 * jy];
        }
      }
    }

    /*  JPD : joint probability distribution */
    /*  UPDATE OBS STRUCTURE */
    for (i28 = 0; i28 < 6; i28++) {
      Obs->ltype[i28] = b_Lmk_type[i28];
    }

    /* Obs.meas.nrm = Lmk.parEndp.nrm; */
    Obs->vis = b_vis;
    for (i = 0; i < 3; i++) {
      Obs->exp.e[i] = b_e[i];
    }

    memcpy(&Obs->exp.E[0], &b_E[0], 9U * sizeof(double));
    for (i28 = 0; i28 < 5; i28++) {
      for (i = 0; i < 5; i++) {
        Obs->exp.E_rl_jpd[i + 5 * i28] = Map_P[((int)rslr[i] + 303 * ((int)
          rslr[i28] - 1)) - 1];
      }
    }

    for (i28 = 0; i28 < 3; i28++) {
      ipiv[i28] = (signed char)(1 + i28);
    }

    for (j = 0; j < 2; j++) {
      c = j << 2;
      jy = 0;
      ix = c;
      dx = fabs(b_E[c]);
      for (k = 2; k <= 3 - j; k++) {
        ix++;
        dy = fabs(b_E[ix]);
        if (dy > dx) {
          jy = k - 1;
          dx = dy;
        }
      }

      if (b_E[c + jy] != 0.0) {
        if (jy != 0) {
          ipiv[j] = (signed char)((j + jy) + 1);
          ix = j;
          i = j + jy;
          for (k = 0; k < 3; k++) {
            dx = b_E[ix];
            b_E[ix] = b_E[i];
            b_E[i] = dx;
            ix += 3;
            i += 3;
          }
        }

        i28 = (c - j) + 3;
        for (i = c + 1; i + 1 <= i28; i++) {
          b_E[i] /= b_E[c];
        }
      }

      i = c;
      jy = c + 3;
      for (k = 1; k <= 2 - j; k++) {
        dx = b_E[jy];
        if (b_E[jy] != 0.0) {
          ix = c + 1;
          i28 = (i - j) + 6;
          for (ijA = 4 + i; ijA + 1 <= i28; ijA++) {
            b_E[ijA] += b_E[ix] * -dx;
            ix++;
          }
        }

        jy += 3;
        i += 3;
      }
    }

    dx = b_E[0];
    b_vis = false;
    for (k = 0; k < 2; k++) {
      dx *= b_E[(k + 3 * (k + 1)) + 1];
      if (ipiv[k] > 1 + k) {
        b_vis = !b_vis;
      }
    }

    if (b_vis) {
      dx = -dx;
    }

    Obs->exp.um = dx;

    /*  uncertainty measure proportional to ellipsoid area */
    memcpy(&Obs->Jac.E_r[0], &E_rf[0], 9U * sizeof(double));
    for (i28 = 0; i28 < 6; i28++) {
      Obs->Jac.E_l[i28] = b_E_l[i28];
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
  /*    SLAMTB is Copyright 2009  */
  /*    by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol */
  /*    @ LAAS-CNRS. */
  /*    See on top of this file for its particular copyright. */
  /*    # END GPL LICENSE */
}

/*
 * File trailer for projectLmk.c
 *
 * [EOF]
 */
