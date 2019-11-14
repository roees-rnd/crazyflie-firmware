/*
 * File: matchFeatureExperiment.c
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
#include "matchFeatureExperiment.h"
#include "lp2d.h"

/* Function Definitions */

/*
 * global Debug
 *  MATCHFEATURE  Match feature.
 *   Obs = MATCHFEATURE(Sen,Raw,Obs) matches one feature in Raw to the predicted
 *   feature in Obs.
 * Arguments    : const double Raw_endpWorld[4]
 *                const double Raw_covEndpointsWorld[16]
 *                const double Raw_J_s2r[12]
 *                Obs_type *Obs
 *                double Opt_correct_MD2th
 * Return Type  : void
 */
void matchFeatureExperiment(const double Raw_endpWorld[4], const double
  Raw_covEndpointsWorld[16], const double Raw_J_s2r[12], Obs_type *Obs, double
  Opt_correct_MD2th)
{
  double a21;
  double HP_hm[3];
  double HP_p[2];
  double a22;
  double HQ_hm[3];
  double HQ_q[2];
  double b_z[2];
  int r1;
  double Z_e[6];
  double Z_y[8];
  double J_shm2rl[35];
  int r2;
  double b_Z_y[14];
  double b_J_z2rl[10];
  int k;
  double c_J_z2rl[4];
  double d_J_z2rl[10];
  double b_Z[4];
  double c_Z_y[4];
  double d_Z_y[8];
  static const signed char A[4] = { 1, 0, 0, 1 };

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /* [z, Z, iZ, MD2, Z_e] = innovation(Raw.endpWorld, Raw.covEndpointsWorld, Obs.exp.e, Obs.exp.E, @hms2hh); */
  /*  caculate innovation distances z: */
  /*  [z,Z_e,Z_y] = f(Obs.exp.e,Raw.endpWorld); */
  /*  HMS2HH  Orthogonal endpoints innovation for homogeneous line and segment. */
  /*    HMS2HH(HM,SEG) is a 2-vector with the orthogonal distances from the two */
  /*    extremes of the segment SEG to the homogeneous line HM. */
  /*  */
  /*    [HH,HH_hm,HH_s] = ... returns the Jacobians wrt HM and SEG. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  lp2d(Obs->exp.e, *(double (*)[2])&Raw_endpWorld[0], &a21, HP_hm, HP_p);
  lp2d(Obs->exp.e, *(double (*)[2])&Raw_endpWorld[2], &a22, HQ_hm, HQ_q);
  b_z[0] = a21;
  b_z[1] = a22;
  for (r1 = 0; r1 < 3; r1++) {
    Z_e[r1 << 1] = HP_hm[r1];
    Z_e[1 + (r1 << 1)] = HQ_hm[r1];
  }

  for (r1 = 0; r1 < 2; r1++) {
    Z_y[r1 << 1] = HP_p[r1];
    Z_y[(r1 + 2) << 1] = 0.0;
    Z_y[1 + (r1 << 1)] = 0.0;
    Z_y[1 + ((r1 + 2) << 1)] = HQ_q[r1];
  }

  /*  jac */
  /*  syms a b c u1 u2 v1 v2 real */
  /*  hm = [a;b;c]; */
  /*  seg = [u1;v1;u2;v2]; */
  /*   */
  /*  [hh,HH_hm,HH_s] = hms2hh(hm,seg) */
  /*   */
  /*  simplify(HH_hm - jacobian(hh,hm)) */
  /*  simplify(HH_s - jacobian(hh,seg)) */
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
  /*  Z_e - changes of innovation wrt hmg line. */
  /*  Z_y - changes of innovation wrt seg endpoints. */
  /*  Raw.J_s2r - changes of seg endpoints measurements wrt rob pose. */
  /*  Obs.Jac.E_l - changes of hml wrt landmark (rho, phi) */
  memset(&J_shm2rl[0], 0, 35U * sizeof(double));
  for (r1 = 0; r1 < 3; r1++) {
    for (r2 = 0; r2 < 4; r2++) {
      J_shm2rl[r2 + 7 * r1] = Raw_J_s2r[r2 + (r1 << 2)];
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      J_shm2rl[(r2 + 7 * (3 + r1)) + 4] = Obs->Jac.E_l[r2 + 3 * r1];
    }
  }

  /*  Caculate the covariance of expected measured segment and landmark: */
  /*  Obs.exp.E_rl_jpd - covariance of rob pose and lmk. */
  /*  P_s_hml=J_shm2rl*Obs.exp.E_rl_jpd*J_shm2rl'; */
  /*  J_z2rl = zeros(2,5); % for coder */
  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      b_Z_y[r2 + (r1 << 1)] = Z_y[r2 + (r1 << 1)];
    }
  }

  for (r1 = 0; r1 < 3; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      b_Z_y[r2 + ((r1 + 4) << 1)] = Z_e[r2 + (r1 << 1)];
    }
  }

  /*  Obs.Jac.J_z2rl=J_z2rl(1:2,1:5); */
  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 5; r2++) {
      b_J_z2rl[r1 + (r2 << 1)] = 0.0;
      for (k = 0; k < 7; k++) {
        b_J_z2rl[r1 + (r2 << 1)] += b_Z_y[r1 + (k << 1)] * J_shm2rl[k + 7 * r2];
      }
    }

    for (r2 = 0; r2 < 5; r2++) {
      d_J_z2rl[r1 + (r2 << 1)] = 0.0;
      for (k = 0; k < 5; k++) {
        d_J_z2rl[r1 + (r2 << 1)] += b_J_z2rl[r1 + (k << 1)] * Obs->
          exp.E_rl_jpd[k + 5 * r2];
      }
    }

    for (r2 = 0; r2 < 4; r2++) {
      d_Z_y[r1 + (r2 << 1)] = 0.0;
      for (k = 0; k < 4; k++) {
        d_Z_y[r1 + (r2 << 1)] += Z_y[r1 + (k << 1)] * Raw_covEndpointsWorld[k +
          (r2 << 2)];
      }
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      c_J_z2rl[r1 + (r2 << 1)] = 0.0;
      for (k = 0; k < 5; k++) {
        c_J_z2rl[r1 + (r2 << 1)] += d_J_z2rl[r1 + (k << 1)] * b_J_z2rl[r2 + (k <<
          1)];
      }

      c_Z_y[r1 + (r2 << 1)] = 0.0;
      for (k = 0; k < 4; k++) {
        c_Z_y[r1 + (r2 << 1)] += d_Z_y[r1 + (k << 1)] * Z_y[r2 + (k << 1)];
      }
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      b_Z[r2 + (r1 << 1)] = c_J_z2rl[r2 + (r1 << 1)] + c_Z_y[r2 + (r1 << 1)];
    }
  }

  /*  compute extra outputs */
  if (fabs(b_Z[1]) > fabs(b_Z[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = b_Z[r2] / b_Z[r1];
  a22 = b_Z[2 + r2] - a21 * b_Z[2 + r1];
  for (k = 0; k < 2; k++) {
    c_J_z2rl[k + (r1 << 1)] = (double)A[k] / b_Z[r1];
    c_J_z2rl[k + (r2 << 1)] = ((double)A[2 + k] - c_J_z2rl[k + (r1 << 1)] * b_Z
      [2 + r1]) / a22;
    c_J_z2rl[k + (r1 << 1)] -= c_J_z2rl[k + (r2 << 1)] * a21;
  }

  /*  better than inv(Z) -- ask Matlab! */
  a21 = 0.0;
  for (r1 = 0; r1 < 2; r1++) {
    HP_p[r1] = 0.0;
    for (r2 = 0; r2 < 2; r2++) {
      HP_p[r1] += b_z[r2] * c_J_z2rl[r2 + (r1 << 1)];
    }

    a21 += HP_p[r1] * b_z[r1];
  }

  /*  TODO: might there be a better distance measure? */
  /*  rt = hmgLin2rt(Obs.exp.e); */
  /*  AA=mvnrnd([Raw.endpWorld;Obs.exp.e],P_s_hml,7);%MM(r,[1 2 3 4 5]),15); */
  /*  mnX=min(min(AA(:,[1 3]))); */
  /*  mxX=max(max(AA(:,[1 3]))); */
  /*  mnY=min(min(AA(:,[2 4]))); */
  /*  mxY=max(max(AA(:,[2 4]))); */
  /*   */
  /*  figure;hold all; */
  /*  for ii=1:size(AA,1) */
  /*      clr=(0.5+[0.5 0.5 0.5].*rand(1,3)).^2; */
  /*      segEnp=AA(ii,1:4); */
  /*      segEnpLmk=AA(ii,1:4); */
  /*      rt = hmgLin2rt(AA(ii,5:7)); */
  /*      segEnpLmk([1 3])=3*(segEnpLmk([1 3])-mean(segEnpLmk([1 3])))+mean(segEnpLmk([1 3])); */
  /*      segEnpLmk([2 4])=3*(segEnpLmk([2 4])-mean(segEnpLmk([2 4])))+mean(segEnpLmk([2 4])); */
  /*      segEnpLmk=projHorEndPntsToHorLin(rt(1),rt(2),segEnpLmk(1:2),segEnpLmk(3:4)); */
  /*      plot(segEnp([1 3]),segEnp([2 4]),'s-','linewidth',1,'color',clr); */
  /*      plot(segEnpLmk([1 3]),segEnpLmk([2 4]),'o--','linewidth',1,'color',clr); */
  /*  end */
  /*  axis([mnX mxX mnY mxY]) */
  /*  axis equal */
  /*  CONSISTENCE \ MATCH */
  if (a21 < Opt_correct_MD2th) {
    /* && abs(min(z))<150 */
    for (r1 = 0; r1 < 2; r1++) {
      Obs->inn.z[r1] = b_z[r1];
    }

    for (r1 = 0; r1 < 4; r1++) {
      Obs->inn.Z[r1] = b_Z[r1];
      Obs->inn.iZ[r1] = c_J_z2rl[r1];
    }

    Obs->inn.MD2 = a21;

    /*  Jacobians of innovation - the chain rule */
    /* Z_e * Obs.Jac.E_r; %  DBDB R */
    /*      Obs.Jac.Z_s = Z_e * Obs.Jac.E_s; */
    for (r1 = 0; r1 < 2; r1++) {
      for (r2 = 0; r2 < 3; r2++) {
        Obs->Jac.Z_r[r1 + (r2 << 1)] = 0.0;
        for (k = 0; k < 4; k++) {
          Obs->Jac.Z_r[r1 + (r2 << 1)] += Z_y[r1 + (k << 1)] * Raw_J_s2r[k + (r2
            << 2)];
        }
      }

      for (r2 = 0; r2 < 2; r2++) {
        Obs->Jac.Z_l[r1 + (r2 << 1)] = 0.0;
        for (k = 0; k < 3; k++) {
          Obs->Jac.Z_l[r1 + (r2 << 1)] += Z_e[r1 + (k << 1)] * Obs->Jac.E_l[k +
            3 * r2];
        }
      }
    }

    for (r1 = 0; r1 < 4; r1++) {
      Obs->meas.y[r1] = Raw_endpWorld[r1];
    }

    memcpy(&Obs->meas.R[0], &Raw_covEndpointsWorld[0], sizeof(double) << 4);
    Obs->measured = true;
    Obs->matched = true;

    /*      if  isempty(coder.target) */
    /*          if Debug */
    /*              figure(99); */
    /*              aa=get(get(99,'CurrentAxes'),'children'); */
    /*              if ~isempty(aa) */
    /*                  ind=aa(1).XData(end); */
    /*                  if strcmp(aa(1).Tag,'detZ') && strcmp(aa(2).Tag,'normz') */
    /*                      set(aa(1),'XData',[aa(1).XData ind+1],'YData',[aa(1).YData det(Z)]) */
    /*                      set(aa(2),'XData',[aa(2).XData ind+1],'YData',[aa(2).YData norm(z)]) */
    /*                  elseif strcmp(aa(2).Tag,'detZ') && strcmp(aa(1).Tag,'normz') */
    /*                      set(aa(2),'XData',[aa(2).XData ind+1],'YData',[aa(2).YData det(Z)]) */
    /*                      set(aa(1),'XData',[aa(1).XData ind+1],'YData',[aa(1).YData norm(z)]) */
    /*                  else */
    /*                      assert(false); */
    /*                  end */
    /*              else */
    /*                  hold all;plot(1,det(Z),'Tag','detZ'); */
    /*                  plot(1,norm(z),'Tag','normz'); legend('det(Z)', 'norm(z)') */
    /*              end */
    /*          end */
    /*      end */
  } else {
    for (r1 = 0; r1 < 4; r1++) {
      Obs->meas.y[r1] = 0.0;
    }

    for (r1 = 0; r1 < 16; r1++) {
      Obs->meas.R[r1] = 0.0 * Raw_covEndpointsWorld[r1];
    }

    for (r1 = 0; r1 < 2; r1++) {
      Obs->inn.z[r1] = 0.0;
    }

    /*  innovation */
    /*  innovation cov */
    for (r1 = 0; r1 < 4; r1++) {
      Obs->inn.Z[r1] = 0.0;
      Obs->inn.iZ[r1] = 0.0;
    }

    /*  inverse inn. cov */
    Obs->inn.MD2 = 0.0;

    /*  Mahalanobis distance squared */
    Obs->vis = true;

    /*  for coder */
    Obs->measured = false;
    Obs->matched = false;
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
 * File trailer for matchFeatureExperiment.c
 *
 * [EOF]
 */
