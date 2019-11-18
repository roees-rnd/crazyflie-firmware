/*
 * File: correctKnownLmksExperiment.c
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
#include "correctKnownLmksExperiment.h"
#include "projHorEndPntsToHorLin.h"
#include "correctBlockEkf.h"
#include "matchFeatureExperiment.h"
#include "projectLmk.h"

/* Function Definitions */

/*
 * CORRECTKNOWNLMKS  Correct known landmarks.
 *    [Rob,Sen,Lmk,Obs] = correctKnownLmks(Rob, Sen, Raw, Lmk, Obs, Opt)
 *    performs EKF corrections to a selection of observations, updating Map,
 *    Rob, Sen, Lmk and Obs structures.
 *
 *    Input/output structures:
 *        Rob:  the robot
 *        Sen:  the sensor
 *        Raw:  the raw data issued from Sen
 *        Lmk:  the set of landmarks
 *        Obs:  the set of observations for the sensor Sen
 *        Opt:  the algorithm options
 *
 *    The selection of the landmarks to observe is based on active-search
 *    procedures and individual compatibility tests to reject outliers.
 *
 *    This function takes care of:
 *        1. Selection of observations to correct
 *        2. Feature matching
 *        3. EKF correction
 *        4. Correction of landmark parameters out of the EKF
 *        5. Landmark re-parametrization
 *        6. Landmark deletion in case of corruption
 *
 *    The algorithm is configurable through a number of options in structure
 *    Opt.correct. Edit USERDATA to access and modify these options.
 *
 *    See also PROJECTLMK, PROJEUCPNTINTOPINHOLEONROB, SMARTDELETELMK.
 * Arguments    : Map_type *Map
 *                Rob_type *Rob
 *                const Sen_type *Sen
 *                Raw_type *Raw
 *                Lmk_type Lmk[15]
 *                Obs_type Obs[15]
 *                double Opt_correct_MD2th
 *                double Opt_correct_isVisOrtThrsh
 *                const double Opt_correct_isVisFarThrsh[2]
 *                double Opt_maxDistVis
 *                double Opt_maxGap
 * Return Type  : double
 */
double correctKnownLmksExperiment(Map_type *Map, Rob_type *Rob, const Sen_type
  *Sen, Raw_type *Raw, Lmk_type Lmk[15], Obs_type Obs[15], double
  Opt_correct_MD2th, double Opt_correct_isVisOrtThrsh, const double
  Opt_correct_isVisFarThrsh[2], double Opt_maxDistVis, double Opt_maxGap)
{
  double corrLmkId;
  int i;
  double b_Map[3];
  int visInd_i;
  double c_Map[9];
  signed char visAndMatchInd[15];
  double b_lmk;
  int exitg1;
  Lmk_type expl_temp;
  double unusedU0[4];
  double t_cur[2];
  double b_Lmk;
  int minInd;
  double c_Lmk;
  double d_Lmk;
  double e_Lmk;
  double d_i;
  Obs_type b_Obs;
  double b_Rob[5];
  double c_Obs[10];
  boolean_T noHole;
  boolean_T b_bool;
  static const char cv4[7] = { 'p', 's', 'e', 'u', 'd', 'o', 'S' };

  int c_bool;
  static const char cv5[6] = { 'h', 'o', 'r', 'L', 'i', 'n' };

  int d_bool;
  double nrm_new[2];
  double e2[2];
  double seg_prev_lmk[4];

  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  steps in this function */
  /*  0- update Rob and Sen info from Map */
  /*  1- project all landmarks */
  /*  2- select landmarks to correct. For each one: */
  /*  3- do feature matching. If feature found: */
  /*  4- compute innovation. */
  /*  5- perform consistency test. If it is OK: */
  /*  6- do correction */
  /*  7- eventually reparametrize landmark */
  /*  8- eventually delete corrupted landmarks */
  /*  9- force covariance symmetry */
  /*  global Map */
  /*  global Debug */
  /*  */
  /*  if Debug, persistent itter; if isempty(itter), itter=1; end; end */
  corrLmkId = 0.0;

  /*  0. UPDATE ROB AND SEN INFO FROM MAP */
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
  for (i = 0; i < 3; i++) {
    b_Map[i] = Map->x[(int)Rob->state.r[i] - 1];
  }

  for (i = 0; i < 3; i++) {
    Rob->state.x[i] = b_Map[i];
  }

  /*  % % Rob.frame.x = Map.x(Rob.frame.r); */
  /*  Rob.frame   = updateFrame(Rob.frame); */
  /*  covariances */
  for (i = 0; i < 3; i++) {
    for (visInd_i = 0; visInd_i < 3; visInd_i++) {
      c_Map[visInd_i + 3 * i] = Map->P[((int)Rob->state.r[visInd_i] + 33 * ((int)
        Rob->state.r[i] - 1)) - 1];
    }
  }

  for (i = 0; i < 3; i++) {
    for (visInd_i = 0; visInd_i < 3; visInd_i++) {
      Rob->state.P[visInd_i + 3 * i] = c_Map[visInd_i + 3 * i];
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
  /*  Sen = map2sen(Sen); */
  /*  For coder - referencing a component from array of non-scalar values not supported by coder */
  /*  Lmk_used = false(1,length(Lmk)); */
  /*  for i =1: length(Lmk), Lmk_used(i) = Lmk(i).used; end */
  /*  Lmk_used_ind = find(Lmk_used); */
  /*  For coder - end */
  for (i = 0; i < 15; i++) {
    visAndMatchInd[i] = 0;
  }

  i = 0;

  /*  1. PROJECT ALL LMKS - get all expectations */
  /*  for lmk = find(Lmk_used) % Coder - FOR loop index expressions of unknown size are only supported if they are of the form A:B or A:B:C. */
  if (!(Map->numUsed == 0.0)) {
    b_lmk = Lmk[(int)Map->lmk0 - 1].link2next;
    do {
      exitg1 = 0;
      if (Lmk[(int)b_lmk - 1].used) {
        expl_temp = Lmk[(int)b_lmk - 1];
        projectLmk(Map->x, Map->P, Rob->state.x, Rob->state.r, Sen->type,
                   Sen->par.maxRange, expl_temp.type, expl_temp.state.r,
                   expl_temp.parEndp.t, expl_temp.parEndp.e,
                   expl_temp.parEndp.nrm, &Obs[(int)b_lmk - 1], Raw->endpWorld,
                   Raw->nrm, Opt_correct_isVisOrtThrsh,
                   Opt_correct_isVisFarThrsh, Opt_maxDistVis);
        if (Obs[(int)b_lmk - 1].vis) {
          /*  project raw to lmk: */
          projHorEndPntsToHorLin(Map->x[(int)Lmk[(int)b_lmk - 1].state.r[0] - 1],
            Map->x[(int)Lmk[(int)b_lmk - 1].state.r[1] - 1], *(double (*)[2])&
            Raw->endpWorld[0], *(double (*)[2])&Raw->endpWorld[2], unusedU0,
            t_cur);
          if ((Lmk[(int)b_lmk - 1].parEndp.t[1] < t_cur[1]) || rtIsNaN(t_cur[1]))
          {
            b_Lmk = Lmk[(int)b_lmk - 1].parEndp.t[1];
          } else {
            b_Lmk = t_cur[1];
          }

          if ((Lmk[(int)b_lmk - 1].parEndp.t[0] > t_cur[0]) || rtIsNaN(t_cur[0]))
          {
            c_Lmk = Lmk[(int)b_lmk - 1].parEndp.t[0];
          } else {
            c_Lmk = t_cur[0];
          }

          if (b_Lmk - c_Lmk > -Opt_maxGap) {
            /*  3. TRY TO MATCH FEATURE */
            matchFeatureExperiment(Raw->endpWorld, Raw->covEndpointsWorld,
              Raw->J_s2r, &Obs[(int)b_lmk - 1], Opt_correct_MD2th);
            if (Obs[(int)b_lmk - 1].matched) {
              i++;
              visAndMatchInd[i - 1] = (signed char)b_lmk;
            }
          }
        } else {
          Obs[(int)b_lmk - 1].measured = false;
          Obs[(int)b_lmk - 1].matched = false;
        }
      }

      if (b_lmk == Map->lmk0) {
        exitg1 = 1;
      } else {
        b_lmk = Lmk[(int)b_lmk - 1].link2next;
      }
    } while (exitg1 == 0);

    /*  --- all landmarks are now projected. */
    if (i > 0) {
      minInd = 0;
      b_lmk = -40000.0;
      if (i > 1) {
        for (visInd_i = 0; visInd_i < i; visInd_i++) {
          /*  Choose the best matched lmk */
          /*  project raw to lmk: */
          projHorEndPntsToHorLin(Map->x[(int)Lmk[visAndMatchInd[visInd_i] - 1].
            state.r[0] - 1], Map->x[(int)Lmk[visAndMatchInd[visInd_i] - 1].
            state.r[1] - 1], *(double (*)[2])&Raw->endpWorld[0], *(double (*)[2])
            &Raw->endpWorld[2], unusedU0, t_cur);
          if ((Lmk[visAndMatchInd[visInd_i] - 1].parEndp.t[1] < t_cur[1]) ||
              rtIsNaN(t_cur[1])) {
            d_Lmk = Lmk[visAndMatchInd[visInd_i] - 1].parEndp.t[1];
          } else {
            d_Lmk = t_cur[1];
          }

          if ((Lmk[visAndMatchInd[visInd_i] - 1].parEndp.t[0] > t_cur[0]) ||
              rtIsNaN(t_cur[0])) {
            e_Lmk = Lmk[visAndMatchInd[visInd_i] - 1].parEndp.t[0];
          } else {
            e_Lmk = t_cur[0];
          }

          d_i = d_Lmk - e_Lmk;
          if ((d_i > b_lmk) || (1 + visInd_i == 1)) {
            b_lmk = d_i;
            minInd = visInd_i;
          }
        }
      }

      /*  %  vis = logical([Obs.vis]); */
      /*  vis = false(1,length(Obs)); */
      /*   for i =1: length(Obs), vis(i) = Obs(i).vis; end % Coder - Referencing a component from array of non-scalar values is not supported for code generation. */
      /*  */
      /*  if any(vis) % Consider only visible observations */
      /*  */
      /*      %     % 2. SELECT LMKS TO CORRECT */
      /*      [lmksToObs,lmksToSkip] = selectLmksToObserve(Obs(vis),Opt.correct.nUpdates); */
      /*      % */
      /*      % lmks to skip, update Obs info */
      /*      [Obs(lmksToSkip).measured] = deal(false); */
      /*      [Obs(lmksToSkip).matched]  = deal(false); */
      /*      [Obs(lmksToSkip).updated]  = deal(false); */
      /*  for visInd_i = numberOfVisAndMatchedLmks:-1:1  % go through all visible landmarks */
      /*  */
      /*      lmk = visAndMatchInd(visInd_i); */
      /*  Update Lmk search counter */
      Lmk[visAndMatchInd[minInd] - 1].nSearch++;

      /*      if Obs(lmk).matched */
      corrLmkId = visAndMatchInd[minInd];

      /*  Update Lmk matched counter */
      Lmk[visAndMatchInd[minInd] - 1].nMatch++;

      /*  4. COMPUTE INNOVATIONS */
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
      for (i = 0; i < 3; i++) {
        b_Map[i] = Map->x[(int)Rob->state.r[i] - 1];
      }

      for (i = 0; i < 3; i++) {
        Rob->state.x[i] = b_Map[i];
      }

      /*  % % Rob.frame.x = Map.x(Rob.frame.r); */
      /*  Rob.frame   = updateFrame(Rob.frame); */
      /*  covariances */
      for (i = 0; i < 3; i++) {
        for (visInd_i = 0; visInd_i < 3; visInd_i++) {
          c_Map[visInd_i + 3 * i] = Map->P[((int)Rob->state.r[visInd_i] + 33 *
            ((int)Rob->state.r[i] - 1)) - 1];
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
      /*              Sen = map2sen(Sen); */
      /*  Update Lmk inlier counter */
      Lmk[visAndMatchInd[minInd] - 1].nInlier++;

      /*  6. LANDMARK CORRECTION AND 7. REPARAMETRIZATION */
      /*  fully correct landmark - EKF, reparam. and off-filter */
      b_Obs = Obs[visAndMatchInd[minInd] - 1];

      /*  CORRECTLMK  Correct landmark. */
      /*    [Rob,Sen,Lmk,Obs] = CORRECTLMK(Rob,Sen,Lmk,Obs,Opt) performs all */
      /*    landmark correction steps in EKF SLAM: stochastic EKF correction, */
      /*    landmark reparametrization, and non-stochastic landmark correction (for */
      /*    landmark parameters not maintained in the stochastic map). */
      /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
      /*  EKF correction */
      /*   EKFCORRECTLMK  Correct landmarks. */
      /*     [ROB,SEN,LMK,OBS] = EKFCORRECTLMK(ROB,SEN,LMK,OBS) returns the */
      /*     measurement update of robot and sensor based on EKF correction of */
      /*     subsisting maps. For each map, first we recuperate the previously */
      /*     stored Jacobians and the covariances matrix of the innovation, and */
      /*     then apply the correction equations to get the updated maps */
      /*        ROB:  the robot */
      /*        SEN:  the sensor */
      /*        LMK:  the set of landmarks */
      /*        OBS:  the observation structure for the sensor SEN */
      /*  */
      /*     See also CORRECTKNOWNLMKS, PROJECTLMK. */
      /*    Copyright 2009 David Marquez @ LAAS-CNRS. */
      /*  get landmark range */
      /*  lmk range in Map */
      /*  Rob-Sen-Lmk range and Jacobian of innovation wrt active states. */
      /*  if Sen.frameInMap */
      /*      rslr  = [Rob.frame.r ; Sen.frame.r ; lr]; % range of robot, sensor, and landmark */
      /*      Z_rsl = [Obs.Jac.Z_r Obs.Jac.Z_s Obs.Jac.Z_l]; */
      /*       */
      /*  else */
      /*  %     [~,Qe_tmp] = e2q(q2e(Rob.frame.q)); */
      /*  %     J1=[eye(3); Qe_tmp]; */
      /*  %     J2=[cos(Obs.meas.y(2)), 0*Rob.frame.x(2);... */
      /*  %         sin(Obs.meas.y(2)), 0*Rob.frame.x(1);... */
      /*  %         0 1]; */
      /*  %     Obs_Jac_Z_r=-(J1*J2)'; */
      /*       */
      /*  range of robot and landmark */
      /*  %     Z_rsl = [Obs_Jac_Z_r Obs.Jac.Z_l]; */
      /*  end */
      /*  correct map. See that Jac of innovation has its sign changed, as */
      /*  corresponds to the jacobian Z_x of z=y-h(x) wrt x in comparison of the */
      /*  Jacobian H_x of y=h(x) wrt x: it happens that H_x = -Z_x. */
      for (i = 0; i < 3; i++) {
        for (visInd_i = 0; visInd_i < 3; visInd_i++) {
          Rob->state.P[visInd_i + 3 * i] = c_Map[visInd_i + 3 * i];
        }

        b_Rob[i] = Rob->state.r[i];
      }

      for (i = 0; i < 2; i++) {
        b_Rob[i + 3] = Lmk[visAndMatchInd[minInd] - 1].state.r[i];
      }

      for (i = 0; i < 3; i++) {
        for (visInd_i = 0; visInd_i < 2; visInd_i++) {
          c_Obs[visInd_i + (i << 1)] = -Obs[visAndMatchInd[minInd] - 1]
            .Jac.Z_r[visInd_i + (i << 1)];
        }
      }

      for (i = 0; i < 2; i++) {
        for (visInd_i = 0; visInd_i < 2; visInd_i++) {
          c_Obs[visInd_i + ((i + 3) << 1)] = -Obs[visAndMatchInd[minInd] - 1].
            Jac.Z_l[visInd_i + (i << 1)];
        }
      }

      correctBlockEkf(Map, b_Rob, c_Obs, Obs[visAndMatchInd[minInd] - 1].inn.z,
                      Obs[visAndMatchInd[minInd] - 1].inn.Z,
                      Obs[visAndMatchInd[minInd] - 1].inn.iZ);

      /*  % Rob and Sen synchronized with Map */
      /*  Rob = map2rob(Rob); */
      /*  Sen = map2sen(Sen); */
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
      /*  Negative inverse distance correction */
      /*  Lmk = fixNegIdp(Lmk);  */
      /*  Transform to cheaper parametrization if possible */
      /*  if Opt.correct.reparametrize */
      /*      [Lmk,Obs] = reparametrizeLmk(Rob,Sen,Lmk,Obs,Opt); */
      /*  end */
      /*  Update off-filter parameters */
      /*  UPDATELMKPARAMS Update off-filter landmark parameters. */
      /*    Lmk = UPDATELMKPARAMS(Rob,Sen,Lmk,Obs,Opt) updates the internal params */
      /*    in Lmk.par, using fundamentally information from the last observation */
      /*    Obs, and following the options in Opt.  */
      /*  */
      /*    The function does nothing for punctual landmarks as they do not have */
      /*    internal parameters. It is useful on the ocntrary to update line */
      /*    endpoints in landmarks of the type ???Lin such as plkLin, aplLin, */
      /*    idpLin or hmgLin. */
      /*  */
      /*    This function should be called after EKFCORRECTLMK. */
      /*  */
      /*    See also EKFCORRECTLMK, UPDATEPLKLINENDPNTS. */
      /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
      noHole = false;
      b_bool = false;
      i = 0;
      do {
        exitg1 = 0;
        if (i + 1 < 8) {
          if (Sen->type[i] != cv4[i]) {
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

      switch (c_bool) {
       case 0:
        b_bool = false;
        i = 0;
        do {
          exitg1 = 0;
          if (i + 1 < 7) {
            if (Lmk[visAndMatchInd[minInd] - 1].type[i] != cv5[i]) {
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
          /*  update endpoints */
          /*  UPDATEIDPLINENDPNTS  Update IDP line endpoints. */
          /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
          /*  global Map */
          /*  Input:  */
          /*  rho,theta(rad) - scalars */
          /*  p1, p2 - 2D points (column vectors) */
          /*  Output: */
          /*  t   = [t1;t2]; where t1 & t2 scalars and t2 > t1 */
          /*  global Map */
          /*   */
          /*  % Plucker line */
          /*  l = Map.x(Lmk.state.r); */
          /*   */
          /*  % rays in sensor frame */
          /*  r1s = pix2PluckerRay(Sen.par.k,Obs.meas.y(1:2)); */
          /*  r2s = pix2PluckerRay(Sen.par.k,Obs.meas.y(3:4)); */
          /*   */
          /*  % rays in world frame */
          /*  r1 = fromFramePlucker(Rob.frame,fromFramePlucker(Sen.frame,r1s)); */
          /*  r2 = fromFramePlucker(Rob.frame,fromFramePlucker(Sen.frame,r2s)); */
          /*   */
          /*  % endpoints and abscissas */
          /*  [e1,t1] = intersectPlucker(l,r1); */
          /*  [e2,t2] = intersectPlucker(l,r2); */
          projHorEndPntToHorLin(Map->x[(int)Lmk[visAndMatchInd[minInd] - 1].
                                state.r[0] - 1], Map->x[(int)
                                Lmk[visAndMatchInd[minInd] - 1].state.r[1] - 1],
                                *(double (*)[2])&Obs[visAndMatchInd[minInd] - 1]
                                .meas.y[0], nrm_new, &b_lmk);
          projHorEndPntToHorLin(Map->x[(int)Lmk[visAndMatchInd[minInd] - 1].
                                state.r[0] - 1], Map->x[(int)
                                Lmk[visAndMatchInd[minInd] - 1].state.r[1] - 1],
                                *(double (*)[2])&Obs[visAndMatchInd[minInd] - 1]
                                .meas.y[2], e2, &d_i);

          /*  build segment and abscissas vector */
          if (d_i > b_lmk) {
            for (i = 0; i < 2; i++) {
              unusedU0[i] = nrm_new[i];
              unusedU0[i + 2] = e2[i];
            }

            t_cur[0] = b_lmk;
            t_cur[1] = d_i;
          } else {
            for (i = 0; i < 2; i++) {
              unusedU0[i] = e2[i];
              unusedU0[i + 2] = nrm_new[i];
            }

            t_cur[0] = d_i;
            t_cur[1] = b_lmk;
          }

          /*  project prev endp onto thre updated lmk */
          /*  Input:  */
          /*  rho,theta(rad) - scalars */
          /*  p1, p2 - 2D points (column vectors) */
          /*  Output: */
          /*  t   = [t1;t2]; where t1 & t2 scalars and t2 > t1 */
          /*  global Map */
          /*   */
          /*  % Plucker line */
          /*  l = Map.x(Lmk.state.r); */
          /*   */
          /*  % rays in sensor frame */
          /*  r1s = pix2PluckerRay(Sen.par.k,Obs.meas.y(1:2)); */
          /*  r2s = pix2PluckerRay(Sen.par.k,Obs.meas.y(3:4)); */
          /*   */
          /*  % rays in world frame */
          /*  r1 = fromFramePlucker(Rob.frame,fromFramePlucker(Sen.frame,r1s)); */
          /*  r2 = fromFramePlucker(Rob.frame,fromFramePlucker(Sen.frame,r2s)); */
          /*   */
          /*  % endpoints and abscissas */
          /*  [e1,t1] = intersectPlucker(l,r1); */
          /*  [e2,t2] = intersectPlucker(l,r2); */
          projHorEndPntToHorLin(Map->x[(int)Lmk[visAndMatchInd[minInd] - 1].
                                state.r[0] - 1], Map->x[(int)
                                Lmk[visAndMatchInd[minInd] - 1].state.r[1] - 1],
                                *(double (*)[2])&Lmk[visAndMatchInd[minInd] - 1]
                                .parEndp.e[0], nrm_new, &b_lmk);
          projHorEndPntToHorLin(Map->x[(int)Lmk[visAndMatchInd[minInd] - 1].
                                state.r[0] - 1], Map->x[(int)
                                Lmk[visAndMatchInd[minInd] - 1].state.r[1] - 1],
                                *(double (*)[2])&Lmk[visAndMatchInd[minInd] - 1]
                                .parEndp.e[2], e2, &d_i);

          /*  build segment and abscissas vector */
          if (d_i > b_lmk) {
            for (i = 0; i < 2; i++) {
              seg_prev_lmk[i] = nrm_new[i];
              seg_prev_lmk[i + 2] = e2[i];
            }

            nrm_new[0] = b_lmk;
            nrm_new[1] = d_i;
          } else {
            for (i = 0; i < 2; i++) {
              seg_prev_lmk[i] = e2[i];
              seg_prev_lmk[i + 2] = nrm_new[i];
            }

            nrm_new[0] = d_i;
            nrm_new[1] = b_lmk;
          }

          for (i = 0; i < 2; i++) {
            Lmk[visAndMatchInd[minInd] - 1].parEndp.t[i] = nrm_new[i];
          }

          for (i = 0; i < 4; i++) {
            Lmk[visAndMatchInd[minInd] - 1].parEndp.e[i] = seg_prev_lmk[i];
          }

          nrm_new[0] = seg_prev_lmk[3] - seg_prev_lmk[1];
          nrm_new[1] = -(seg_prev_lmk[2] - seg_prev_lmk[0]);
          b_lmk = 0.0;
          for (i = 0; i < 2; i++) {
            b_lmk += nrm_new[i] * nrm_new[i];
          }

          if (b_lmk > 0.1) {
            b_lmk = 0.0;
            for (i = 0; i < 2; i++) {
              b_lmk += nrm_new[i] * nrm_new[i];
            }

            b_lmk = sqrt(b_lmk);
            for (i = 0; i < 2; i++) {
              nrm_new[i] /= b_lmk;
            }
          } else {
            for (i = 0; i < 2; i++) {
              nrm_new[i] *= 0.0;
            }
          }

          b_lmk = 0.0;
          for (i = 0; i < 2; i++) {
            b_lmk += nrm_new[i] * Lmk[visAndMatchInd[minInd] - 1].parEndp.nrm[i];
          }

          if (b_lmk < 0.0) {
            b_lmk = -1.0;
          } else if (b_lmk > 0.0) {
            b_lmk = 1.0;
          } else {
            if (b_lmk == 0.0) {
              b_lmk = 0.0;
            }
          }

          for (i = 0; i < 2; i++) {
            Lmk[visAndMatchInd[minInd] - 1].parEndp.nrm[i] = nrm_new[i] * b_lmk;
          }

          /* Lmk(lmk).parEndp.nrm = Lmk(lmk).parEndp.e; */
          /*  detect hole */
          if ((t_cur[0] > Lmk[visAndMatchInd[minInd] - 1].parEndp.t[1] +
               Opt_maxGap) || (t_cur[1] + Opt_maxGap < Lmk[visAndMatchInd[minInd]
                               - 1].parEndp.t[0])) {
            /*  NEWLMK  Get a new landmark. */
            /*    NEWLMK(Lmk) returns the index in structure array Lmk() of the first */
            /*    non-used landmark. */
            /*  */
            /*    See also NEWRANGE, INITNEWLMKS. */
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
              b_lmk = 1.0;
            } else {
              b_lmk = Lmk[(int)Map->lmk0 - 1].link2next;

              /*  this is the last used lmk (points back to first\earlier used lmk) */
              /*  there is a gap (e.g caused by smart delete), insert a link in between. */
              while (!((Lmk[(int)b_lmk - 1].link2next <= Lmk[(int)b_lmk - 1].lmk)
                       || (Lmk[(int)b_lmk - 1].link2next - Lmk[(int)b_lmk - 1].
                           lmk > 1.0))) {
                b_lmk = Lmk[(int)b_lmk - 1].link2next;
              }

              /*  take the next slot: */
              b_lmk = Lmk[(int)b_lmk - 1].lmk + 1.0;

              /*  update slot pointers (next and prev): */
              /*  update the new neighbours: */
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
            for (i = 0; i < 2; i++) {
              Lmk[(int)b_lmk - 1].parEndp.t[i] = t_cur[i];
            }

            for (i = 0; i < 4; i++) {
              Lmk[(int)b_lmk - 1].parEndp.e[i] = unusedU0[i];
            }

            noHole = false;
          } else {
            /*  here we should see if the new abscissas make the segment longer or not. */
            /*  extend endpoint 1 */
            if (t_cur[0] < Lmk[visAndMatchInd[minInd] - 1].parEndp.t[0]) {
              Lmk[visAndMatchInd[minInd] - 1].parEndp.t[0] = t_cur[0];
              for (i = 0; i < 2; i++) {
                Lmk[visAndMatchInd[minInd] - 1].parEndp.e[i] = unusedU0[i];
              }
            }

            /*  extend endpoint 2 */
            if (t_cur[1] > Lmk[visAndMatchInd[minInd] - 1].parEndp.t[1]) {
              Lmk[visAndMatchInd[minInd] - 1].parEndp.t[1] = t_cur[1];
              for (i = 0; i < 2; i++) {
                Lmk[visAndMatchInd[minInd] - 1].parEndp.e[2 + i] = unusedU0[2 +
                  i];
              }
            }

            noHole = true;
          }

          /*  test - previous */
          /*  run slamtb and stop after creating SLAM structures */
          /*  test 1 */
          /*  lmk */
          /*  obs */
          /*  abscissas */
          /*  print */
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
          break;
        }
        break;
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
      /*  update flags and info */
      b_Obs.updated = true;
      b_Obs.matched = noHole;

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
      Obs[visAndMatchInd[minInd] - 1] = b_Obs;
      if (!Obs[visAndMatchInd[minInd] - 1].matched) {
        /*  this is an indication for 'initNewLmkExperiment' to do an */
        /*  initialization based on Lmk number 'lmk': */
        Raw->updated = visAndMatchInd[minInd];
      }

      /*  Obs(lmk).exp.um */
      /* %%%%%%%%%%%%%%%%%%%              plot update Lmk     %%%%%%%%%%%%%%%%%%%% */
      /*              if  isempty(coder.target) */
      /*                  if Debug > 3 */
      /*                      figure((330+itter)); */
      /*                      itter=itter+1; */
      /*                      %% transform measured seg=EndpointsWorld to rhoPhi */
      /*                      [rtm,RTp,RTq] = pp2rt(Raw.endpWorld(1:2),Raw.endpWorld(3:4)); */
      /*                      rt_endpWorld = [RTp,RTq]; */
      /*                      RTM =  rt_endpWorld*Raw.covEndpointsWorld*rt_endpWorld'; */
      /*                      [Xmeas,Ymeas] = cov2elli(rtm, RTM, 3,20); */
      /*                      plot(Xmeas,Ymeas,'r.-','LineWidth',2); hold on */
      /*                      %% transform expectation=hmLine to rhoPhi */
      /*                      [rte,RThm] = hmgLin2rt(Obs(lmk).exp.e); */
      /*                      RTE = RThm*Obs(lmk).exp.E*RThm'; */
      /*                      [Xexpect,Yexpect] = cov2elli(rte, RTE, 3,20); */
      /*                      plot(Xexpect,Yexpect,'g.-','LineWidth',2); */
      /*                      %% */
      /*                      lr = Lmk(lmk).state.r ;        % lmk range in Map */
      /*                      rtc  = Map.x(lr) ;                 % lmk mean */
      /*                      RTC = Map.P(lr,lr) ;            % lmk cov */
      /*                      [Xmap,Ymap] = cov2elli(rtc, RTC, 3,20); */
      /*                      plot(Xmap,Ymap,'b.-','LineWidth',2); hold off */
      /*                      legend('measured seg','expected hmLine', 'corrected rt'); hold off */
      /*  */
      /*                      %%%%%%%%%%%%%%%%%%%%       end  plot update Lmk     %%%%%%%%%%%%%%%%%%%% */
      /*                  end */
      /*              end */
      /*          break; %prevent matching 1 raw to multiple landmarks. TODO: match Raw to most probable of all landmarks. */
      /*      else */
      /*          Obs(lmk).updated = false; */
      /*      end % if matched */
    }

    /*  for lmk = lmkList */
    /*  8. LANDMARK DELETION -- loop all visible */
    /*  9. FORCE COVARIANCE SYMMETRY -R&S commented out because its already performed in "correctLmk" */
    /*  Map.P(Map.used,Map.used) = (Map.P(Map.used,Map.used) + Map.P(Map.used,Map.used)')/2; */
    /* tmpCov=Map.P(Map.used,Map.used); */
    /* [evec, eval]=eig(tmpCov); */
    /* Map.P(Map.used,Map.used)=evec*abs(eval)*evec'; */
    /*  end % if any(vis) */
  }

  /*  function */
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
  return corrLmkId;
}

/*
 * File trailer for correctKnownLmksExperiment.c
 *
 * [EOF]
 */
