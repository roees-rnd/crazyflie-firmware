/*
 * File: initNewLmkExperiment.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/initNewLmkExperiment.h"

#include "../modules/src/addToMap.h"
#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addPos_data.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/det.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/newLmk.h"
#include "../modules/src/propagateUncertainty.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Declarations */
static void getPlx(double P_LX[66], double usedInd[33], const double Map_P[91809],
                   double Map_numUsed, double Map_lmk0, const Lmk_type Lmk[15],
                   double Rob_state_size, const double Rob_state_r[3], const
                   double Jac_Wpos[6]);

/* Function Definitions */

/*
 * Arguments    : double P_LX[66]
 *                double usedInd[33]
 *                const double Map_P[91809]
 *                double Map_numUsed
 *                double Map_lmk0
 *                const Lmk_type Lmk[15]
 *                double Rob_state_size
 *                const double Rob_state_r[3]
 *                const double Jac_Wpos[6]
 * Return Type  : void
 */
static void getPlx(double P_LX[66], double usedInd[33], const double Map_P[91809],
                   double Map_numUsed, double Map_lmk0, const Lmk_type Lmk[15],
                   double Rob_state_size, const double Rob_state_r[3], const
                   double Jac_Wpos[6])
{
  int kk;
  int ii;
  double indLmkUsed;
  int i32;
  double b_Map_P[3];
  int i33;
  kk = -1;
  for (ii = 0; ii < 3; ii++) {
    /*  Map.numUsed */
    kk++;
    for (i32 = 0; i32 < 2; i32++) {
      P_LX[i32 + (kk << 1)] = 0.0;
    }

    for (i32 = 0; i32 < 3; i32++) {
      b_Map_P[i32] = Map_P[((int)Rob_state_r[i32] + 303 * ii) - 1];
    }

    for (i32 = 0; i32 < 2; i32++) {
      P_LX[i32 + (kk << 1)] = 0.0;
      for (i33 = 0; i33 < 3; i33++) {
        P_LX[i32 + (kk << 1)] += Jac_Wpos[i32 + (i33 << 1)] * b_Map_P[i33];
      }
    }

    usedInd[kk] = 1.0 + (double)ii;
  }

  kk++;
  indLmkUsed = Lmk[(int)Map_lmk0 - 1].link2next;
  while (kk + 1 <= Map_numUsed * 2.0 + Rob_state_size) {
    /* for ii=1:szMap% Map.numUsed */
    if (!Lmk[(int)indLmkUsed - 1].used) {
      /*  skip the new to be used lmk that is already in the linked list. */
      indLmkUsed = Lmk[(int)indLmkUsed - 1].link2next;
    } else {
      usedInd[kk] = Lmk[(int)indLmkUsed - 1].state.r[0];
      for (i32 = 0; i32 < 2; i32++) {
        P_LX[i32 + (kk << 1)] = 0.0;
      }

      for (i32 = 0; i32 < 3; i32++) {
        b_Map_P[i32] = Map_P[((int)Rob_state_r[i32] + 303 * ((int)Lmk[(int)
          indLmkUsed - 1].state.r[0] - 1)) - 1];
      }

      for (i32 = 0; i32 < 2; i32++) {
        P_LX[i32 + (kk << 1)] = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          P_LX[i32 + (kk << 1)] += Jac_Wpos[i32 + (i33 << 1)] * b_Map_P[i33];
        }
      }

      kk++;
      usedInd[kk] = Lmk[(int)indLmkUsed - 1].state.r[1];
      for (i32 = 0; i32 < 2; i32++) {
        P_LX[i32 + (kk << 1)] = 0.0;
      }

      for (i32 = 0; i32 < 3; i32++) {
        b_Map_P[i32] = Map_P[((int)Rob_state_r[i32] + 303 * ((int)Lmk[(int)
          indLmkUsed - 1].state.r[1] - 1)) - 1];
      }

      for (i32 = 0; i32 < 2; i32++) {
        P_LX[i32 + (kk << 1)] = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          P_LX[i32 + (kk << 1)] += Jac_Wpos[i32 + (i33 << 1)] * b_Map_P[i33];
        }
      }

      indLmkUsed = Lmk[(int)indLmkUsed - 1].link2next;
      kk++;

      /*          if Map.used(ii) */
      /*              kk = kk+1; */
      /*              P_LX(:,kk) = Jac_Wpos*Map.P(Rob.state.r,ii); */
      /*              usedInd(kk) = ii; */
      /*          end */
      /*  TODO: smark exit when reached number of lmks. */
    }
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
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
 * INITNEWLMK  Initialise one landmark.
 *    [LMK, OBS] = INITNEWLMK(ROB, SEN, RAW, LMK, OBS) initializes one new
 *    landmark. The landmark is selected by an active-search analysis of the
 *    Raw data belonging to the current sensor Sen in robot Rob. After
 *    successful initialization, structures Map, Lmk and Obs are updated.
 *
 *    Input/output structures:
 *        Rob:  the robot
 *        Sen:  the sensor
 *        Raw:  the raw datas issues from Sen
 *        Lmk:  the set of landmarks
 *        Obs:  the observation structure for the sensor Sen
 *        Opt:  the algorithm options
 *
 *    The algorithm can be configured through numerous options stored in
 *    structure Opt.init. Edit USERDATA to access and modify these options.
 * Arguments    : Map_type *Map
 *                double Rob_state_size
 *                const double Rob_state_r[3]
 *                double Sen_sen
 *                Raw_type *Raw
 *                Lmk_type Lmk[15]
 *                Obs_type Obs[15]
 *                const char Opt_init_initType[6]
 * Return Type  : double
 */
double initNewLmkExperiment(Map_type *Map, double Rob_state_size, const double
  Rob_state_r[3], double Sen_sen, Raw_type *Raw, Lmk_type Lmk[15], Obs_type Obs
  [15], const char Opt_init_initType[6])
{
  double initLmkId;
  double szMap;
  int i;
  double Rob_state_x[3];
  int noMatch;
  double Rob_state_P[9];
  int noVis;
  double lmkUpd;
  double b_r[2];
  double Jac_Wpos[6];
  static const signed char iv3[3] = { 0, 0, 1 };

  double b_Raw;
  double P_LX[66];
  double usedInd[33];
  double b_Map[2];
  double c_Map[4];
  double b_Jac_Wpos[6];
  int i30;
  double c_Raw[4];

  /*    Copyright 2009 Jean Marie Codol, David Marquez @ LAAS-CNRS */
  /*  0. UPDATE ROB AND SEN INFO FROM MAP */
  /*  global Map; */
  /*  P_LX = zeros(2,1003); */
  szMap = Rob_state_size + 2.0 * Map->numUsed;

  /* ; */
  /*  NUM_LMKS*2+3); */
  initLmkId = 0.0;

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
  /*  % % Rob.frame.x = Map.x(Rob.frame.r); */
  /*  Rob.frame   = updateFrame(Rob.frame); */
  /*  covariances */
  for (i = 0; i < 3; i++) {
    Rob_state_x[i] = Map->x[(int)Rob_state_r[i] - 1];
    for (noMatch = 0; noMatch < 3; noMatch++) {
      Rob_state_P[noMatch + 3 * i] = Map->P[((int)Rob_state_r[noMatch] + 303 *
        ((int)Rob_state_r[i] - 1)) - 1];
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
  /*  index to first free Idp lmk */
  if (!(Map->numUsed == NUM_LMKS)) {
    noVis = 1;
    noMatch = 1;
    for (i = 0; i < 15; i++) {
      if (Obs[i].vis) {
        noVis = 0;
      }

      if (Obs[i].matched && Obs[i].vis) {
        noMatch = 0;
      }
    }

    /*  Feature detection */
    /*  Vis = [Obs.vis]; */
    /*  noVis = ~any(Vis); */
    /*  Matched = [Obs.matched]; */
    /*  noMatch = ~any(Matched & Vis); */
    if (Raw->updated > 0.0) {
      lmkUpd = Raw->updated;
      Raw->updated = 0.0;
      for (noMatch = 0; noMatch < 2; noMatch++) {
        b_r[noMatch] = Lmk[(int)lmkUpd - 1].state.r[noMatch];
      }

      initLmkId = newLmk(Lmk, Map);

      /*    plot(Lmk(lmkUpd).parEndp.e([1 3]),Lmk(lmkUpd).parEndp.e([2 4]),'.-m');plot(Lmk(lmk).parEndp.e([1 3]),Lmk(lmk).parEndp.e([2 4]),'.-m');shg */
      /*  if rho<0 */
      /*      phi=phi-pi; */
      /*  end */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* %% This is the calculation of : rhoPhi2xyTheta */
      /* %% this is dRho / dx,y,tta */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      Jac_Wpos[0] = cos(Map->x[(int)b_r[1] - 1]);
      Jac_Wpos[2] = sin(Map->x[(int)b_r[1] - 1]);
      Jac_Wpos[4] = Rob_state_x[1] * cos(Map->x[(int)b_r[1] - 1]) - Rob_state_x
        [0] * sin(Map->x[(int)b_r[1] - 1]);
      for (noMatch = 0; noMatch < 3; noMatch++) {
        Jac_Wpos[1 + (noMatch << 1)] = iv3[noMatch];
      }

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
      memset(&P_LX[0], 0, 66U * sizeof(double));
      memset(&usedInd[0], 0, 33U * sizeof(double));
      getPlx(P_LX, usedInd, Map->P, Map->numUsed, Map->lmk0, Lmk, Rob_state_size,
             Rob_state_r, Jac_Wpos);

      /* [Rob.state.size + (lmk-1)*2+1;Rob.state.size + (lmk)*2]; */
      /*  add to Map and get lmk range in Map */
      for (i = 0; i < 2; i++) {
        b_Map[i] = Map->x[(int)b_r[i] - 1];
        for (noMatch = 0; noMatch < 3; noMatch++) {
          b_Jac_Wpos[i + (noMatch << 1)] = 0.0;
          for (i30 = 0; i30 < 3; i30++) {
            b_Jac_Wpos[i + (noMatch << 1)] += Jac_Wpos[i + (i30 << 1)] *
              Rob_state_P[i30 + 3 * noMatch];
          }
        }

        for (noMatch = 0; noMatch < 2; noMatch++) {
          b_Raw = 0.0;
          for (i30 = 0; i30 < 3; i30++) {
            b_Raw += b_Jac_Wpos[i + (i30 << 1)] * Jac_Wpos[noMatch + (i30 << 1)];
          }

          c_Map[i + (noMatch << 1)] = Map->P[((int)b_r[i] + 303 * ((int)
            b_r[noMatch] - 1)) - 1] + b_Raw;
        }
      }

      addToMap(Map, b_Map, c_Map, P_LX, usedInd, szMap, Lmk[(int)initLmkId - 1].
               state.r);

      /*  Fill Lmk structure */
      Lmk[(int)initLmkId - 1].lmk = initLmkId;
      Lmk[(int)initLmkId - 1].id = initLmkId;

      /* newIds(newId); */
      for (noMatch = 0; noMatch < 6; noMatch++) {
        Lmk[(int)initLmkId - 1].type[noMatch] = Opt_init_initType[noMatch];
      }

      Lmk[(int)initLmkId - 1].used = true;

      /* Lmk(lmk).sig     = newId; */
      Lmk[(int)initLmkId - 1].nSearch = 1.0;
      Lmk[(int)initLmkId - 1].nMatch = 1.0;
      Lmk[(int)initLmkId - 1].nInlier = 1.0;
      for (noMatch = 0; noMatch < 2; noMatch++) {
        Lmk[(int)initLmkId - 1].parEndp.nrm[noMatch] = Lmk[(int)lmkUpd - 1].
          parEndp.nrm[noMatch];
      }
    } else {
      if ((noVis != 0) || (noMatch != 0)) {
        initLmkId = newLmk(Lmk, Map);
        propagateUncertainty(Raw->rhoPhiWorld, Raw->covRhoPhiWorld, Rob_state_x,
                             Rob_state_P);

        /*              inn.z  = [0;0];  %updated in innovation */
        /*  used at the end of this function : Obs(lmk).exp.um   = det(inn.Z); */
        /*  TODO: why update Obs here when it will be overriden anyway in projectLmk (in correctKnownLmks) */
        /*  fill Obs struct before continuing */
        Obs[(int)initLmkId - 1].sen = Sen_sen;
        Obs[(int)initLmkId - 1].lmk = initLmkId;

        /*      Obs(lmk).sid      = Sen.id; */
        /*      Obs(lmk).lid      = newId;%     Obs(lmk).lid      = newIds(newId); */
        /*      Obs(lmk).stype    = Sen.type; */
        for (noMatch = 0; noMatch < 6; noMatch++) {
          Obs[(int)initLmkId - 1].ltype[noMatch] = Opt_init_initType[noMatch];
        }

        /*      Obs(lmk).meas     = meas; */
        for (noMatch = 0; noMatch < 3; noMatch++) {
          Obs[(int)initLmkId - 1].exp.e[noMatch] = Rob_state_x[noMatch];
        }

        memcpy(&Obs[(int)initLmkId - 1].exp.E[0], &Rob_state_P[0], 9U * sizeof
               (double));
        Obs[(int)initLmkId - 1].exp.um = det(Raw->covRhoPhiWorld);

        /*  uncertainty measure */
        for (noMatch = 0; noMatch < 4; noMatch++) {
          b_Raw = Raw->covRhoPhiWorld[noMatch];
          Obs[(int)initLmkId - 1].inn.Z[noMatch] = b_Raw;
        }

        /*      Obs(lmk).app.curr = app; */
        /*      Obs(lmk).app.pred = app; */
        Obs[(int)initLmkId - 1].vis = true;
        Obs[(int)initLmkId - 1].measured = true;
        Obs[(int)initLmkId - 1].matched = false;
        Obs[(int)initLmkId - 1].updated = false;

        /*     %% Change because coder doesn't support referencing a component from array of non-scalar values */
        /*  TODO: retro project lmk and update lmk covariances. */
        /*      % retro-project feature onto 3D space % comment out by R&S */
        /*       [l, L_rf, L_sf, L_obs, L_n, N] = retroProjLmk(Rob,Sen,Obs(lmk),Opt); */
        /*  if rho<0 */
        /*      phi=phi-pi; */
        /*  end */
        /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
        /* %% This is the calculation of : rhoPhi2xyTheta */
        /* %% this is dRho / dx,y,tta */
        /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
        Jac_Wpos[0] = cos(Raw->rhoPhiWorld[1]);
        Jac_Wpos[2] = sin(Raw->rhoPhiWorld[1]);
        Jac_Wpos[4] = Raw->robPoseFreeze[1] * cos(Raw->rhoPhiWorld[1]) -
          Raw->robPoseFreeze[0] * sin(Raw->rhoPhiWorld[1]);
        for (noMatch = 0; noMatch < 3; noMatch++) {
          Jac_Wpos[1 + (noMatch << 1)] = iv3[noMatch];
        }

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
        memset(&P_LX[0], 0, 66U * sizeof(double));
        memset(&usedInd[0], 0, 33U * sizeof(double));
        getPlx(P_LX, usedInd, Map->P, Map->numUsed, Map->lmk0, Lmk,
               Rob_state_size, Rob_state_r, Jac_Wpos);

        /*  TODO: move constants to userDataLinHor */
        /*      assert(all([Rob.state.size + (lmk-1)*2+1;Rob.state.size + (lmk)*2]==newInd)); */
        /* [Rob.state.size + (lmk-1)*2+1;Rob.state.size + (lmk)*2]; */
        /*  add to Map and get lmk range in Map */
        for (noMatch = 0; noMatch < 2; noMatch++) {
          for (i30 = 0; i30 < 3; i30++) {
            b_Jac_Wpos[noMatch + (i30 << 1)] = 0.0;
            for (noVis = 0; noVis < 3; noVis++) {
              b_Jac_Wpos[noMatch + (i30 << 1)] += Jac_Wpos[noMatch + (noVis << 1)]
                * Raw->robPoseCov[noVis + 3 * i30];
            }
          }

          for (i30 = 0; i30 < 2; i30++) {
            b_Raw = 0.0;
            for (noVis = 0; noVis < 3; noVis++) {
              b_Raw += b_Jac_Wpos[noMatch + (noVis << 1)] * Jac_Wpos[i30 +
                (noVis << 1)];
            }

            c_Raw[noMatch + (i30 << 1)] = Raw->covRhoPhiWorld[noMatch + (i30 <<
              1)] + b_Raw;
          }
        }

        for (noMatch = 0; noMatch < 2; noMatch++) {
          for (i30 = 0; i30 < 2; i30++) {
            c_Map[i30 + (noMatch << 1)] = 2.0 * c_Raw[i30 + (noMatch << 1)];
          }
        }

        addToMap(Map, Raw->rhoPhiWorld, c_Map, P_LX, usedInd, szMap, Lmk[(int)
                 initLmkId - 1].state.r);

        /*  Fill Lmk structure */
        Lmk[(int)initLmkId - 1].lmk = initLmkId;
        Lmk[(int)initLmkId - 1].id = initLmkId;

        /* newIds(newId); */
        for (noMatch = 0; noMatch < 6; noMatch++) {
          Lmk[(int)initLmkId - 1].type[noMatch] = Opt_init_initType[noMatch];
        }

        Lmk[(int)initLmkId - 1].used = true;

        /* Lmk(lmk).sig     = newId; */
        Lmk[(int)initLmkId - 1].nSearch = 1.0;
        Lmk[(int)initLmkId - 1].nMatch = 1.0;
        Lmk[(int)initLmkId - 1].nInlier = 1.0;

        /*  Init off-filter landmark params */
        /*     %% Change because coder doesn't support referencing a component from array of non-scalar values */
        for (noMatch = 0; noMatch < 2; noMatch++) {
          Lmk[(int)initLmkId - 1].parEndp.t[noMatch] = Raw->
            abscissasWorld[noMatch];
        }

        for (noMatch = 0; noMatch < 4; noMatch++) {
          Lmk[(int)initLmkId - 1].parEndp.e[noMatch] = Raw->endpWorld[noMatch];
        }

        for (noMatch = 0; noMatch < 2; noMatch++) {
          Lmk[(int)initLmkId - 1].parEndp.nrm[noMatch] = Raw->nrm[noMatch];
        }

        /*  hold on; plot([Lmk(lmk).par.endp(1).e(1) Lmk(lmk).par.endp(2).e(1)], [Lmk(lmk).par.endp(1).e(2) Lmk(lmk).par.endp(2).e(2)], 'ys-','LineWidth',4) */
        /*  % % SIGMA = Map.P(8:11,8:11); */
        /*  % % SIGMA_=0.5*(SIGMA+SIGMA'); */
        /*  % % R = mvnrnd(Map.x(8:11),SIGMA_,100); */
        /*  if false */
        /*      testInitCov(Lmk(lmk),Opt); */
        /*      title(['init, nmatch=' num2str(Lmk(lmk).nMatch) ' (id=' num2str(Lmk(lmk).id) ')']); */
        /*  */
        /*      plotSamLineCov(Raw.rhoPhiWorld, Raw.covRhoPhiWorld, Raw.endpWorld(1:2), Opt) */
        /*      axis equal; */
        /*      title('Only SaM covariance') */
        /*  end */
        /*      fprintf('Initialized landmark ''%d''.\n',Lmk(lmk).id) */
      }
    }
  } else {
    /*  Lmk structure array full. Unable to initialize new landmark. */
  }

  return initLmkId;
}

/*
 * File trailer for initNewLmkExperiment.c
 *
 * [EOF]
 */
