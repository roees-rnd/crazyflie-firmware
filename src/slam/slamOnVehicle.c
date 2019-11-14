/*
 * File: slamOnVehicle.c
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
#include "initNewLmkExperiment.h"
#include "norm.h"
#include "diff.h"
#include "correctKnownLmksExperiment.h"
#include "refreshMapCntlUnit.h"
#include "getMetry.h"
#include "extractFeaturesWorld.h"
#include "retroProjXYFromRangeSenOnRob.h"
#include "motion.h"
#include "processSlamInput.h"
#include "CyclicBuffer_addPos_rtwutil.h"
#include "CyclicBuffer_addPos_data.h"

/* Variable Definitions */
static boolean_T FirstRun_not_empty;
static double xyWorld_prev[32];
static double LmkIds_p[20];
static double LmkEndps_p[80];
static double LmkActions_p[20];
static double numActions;
static double oneOfTenCyc;
static double robPoseHist[400];
static double numPoses;
static double oneOfFive;
static double UM_lmkID;

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void FirstRun_not_empty_init(void)
{
  FirstRun_not_empty = false;
}

/*
 * declare globals
 * Arguments    : double RobTT
 *                const double RobXYT[3]
 *                const double RobCov[9]
 *                const Prox_type *prox
 *                Map_type *Map
 *                Rob_type *Rob
 *                const Sen_type Sen[17]
 *                Lmk_type Lmk[15]
 *                Obs_type Obs[15]
 *                const Opt_type *Opt
 *                Raw_type Raw[20]
 *                double xyRob[400]
 *                double *pnum
 *                boolean_T *LowFreqCycle
 *                double RobPoseUpd[3]
 *                double RobCovUpd[9]
 *                double LmkIds[20]
 *                double LmkEndps[80]
 *                double LmkActions[20]
 *                struct1_T *Metry
 * Return Type  : void
 */
void slamOnVehicle(double RobTT, const double RobXYT[3], const double RobCov[9],
                   const Prox_type *prox, Map_type *Map, Rob_type *Rob, const
                   Sen_type Sen[17], Lmk_type Lmk[15], Obs_type Obs[15], const
                   Opt_type *Opt, Raw_type Raw[20], double xyRob[400], double
                   *pnum, boolean_T *LowFreqCycle, double RobPoseUpd[3], double
                   RobCovUpd[9], double LmkIds[20], double LmkEndps[80], double
                   LmkActions[20], struct1_T *Metry)
{
  int i19;
  double b_prox[16];
  double b_RobXYT[3];
  double rawRangeMat[16];
  double b_dPosFault;
  boolean_T zeroCmd;
  double num_segments;
  boolean_T flagSavePos;
  int b_pnum;
  int b_sen;
  double b_Map;
  Sen_type expl_temp;
  double xyWorld[2];
  double corrLmkId;
  double i;
  boolean_T guard1 = false;
  double dv20[4];
  double d6;
  double b_xyRob[400];
  unsigned short u1;
  double b_xyWorld[2];
  boolean_T exitg1;
  Raw_type r10;
  boolean_T guard2 = false;
  static const char cv2[7] = { 'n', 'o', '_', 't', 'y', 'p', 'e' };

  (void)RobCov;

  /*  Slam is called when vechicle state!= STOP && vehicle pitch == 0 */
  /*  Output is on Low freq only */
  /* persistent robPoseHist fid numPoses */
  /*  for debug: */
  /*  type definition for coder */
  /*  drawing buffer length */
  /*  I. Specify user-defined options - EDIT USER DATA FILE userData.m */
  if (!FirstRun_not_empty) {
    /* Coder : Non-constant expression or empty matrix. This expression must be constant because its value determines the size or class of some expression. */
    /* pnum = 0; % number of accumulated points in current multi scan */
    FirstRun_not_empty = true;
    memset(&LmkIds_p[0], 0, 20U * sizeof(double));
    memset(&LmkEndps_p[0], 0, 80U * sizeof(double));
    memset(&LmkActions_p[0], 0, 20U * sizeof(double));
    numActions = 0.0;
    oneOfTenCyc = 0.0;
    oneOfFive = 0.0;
    memset(&robPoseHist[0], 0, 400U * sizeof(double));
    numPoses = 0.0;
    UM_lmkID = NUM_LMKS;

    /*  fid = fopen('testFile_L.txt','w');numPoses = 0; */
    memset(&xyWorld_prev[0], 0, sizeof(double) << 5);
  } else {
    if (oneOfTenCyc >= Opt->stepsToSendMap - 1.0) {
      oneOfTenCyc = 0.0;
    } else {
      oneOfTenCyc++;
    }

    if (oneOfFive >= Opt->stepsToMerge - 1.0) {
      oneOfFive = 0.0;
    } else {
      oneOfFive++;
    }
  }

  /*  Init out */
  *LowFreqCycle = false;
  memset(&LmkIds[0], 0, 20U * sizeof(double));
  memset(&LmkEndps[0], 0, 80U * sizeof(double));
  memset(&LmkActions[0], 0, 20U * sizeof(double));
  for (i19 = 0; i19 < 3; i19++) {
    b_RobXYT[i19] = RobXYT[i19];
  }

  memcpy(&b_prox[0], &prox->val[0], sizeof(double) << 4);
  processSlamInput(RobTT, b_RobXYT, b_prox, prox->TT, Sen[0].par.maxRange, Sen[0]
                   .par.minRange, Opt->SF_mm2m, Rob->con.u, rawRangeMat,
                   &b_dPosFault);

  /*  instead of ~ should be Rob.con.U -> currently we don't overwrite it because we don't get RobCov from Nav */
  if ((fabs(Rob->con.u[0]) < Opt->minCmd_x) && (fabs(Rob->con.u[1]) <
       Opt->minCmd_x) && (fabs(Rob->con.u[2]) < Opt->minCmd_t)) {
    zeroCmd = true;
  } else {
    zeroCmd = false;
  }

  num_segments = 0.0;
  if (!zeroCmd) {
    /*  2. ESTIMATION */
    motion(Map, Rob);
    flagSavePos = true;
    for (b_sen = 0; b_sen < 16; b_sen++) {
      /*  for each range generate point in world */
      if (rawRangeMat[b_sen] > 0.0) {
        expl_temp = Sen[b_sen];
        retroProjXYFromRangeSenOnRob(Rob->state.x, expl_temp.position,
          expl_temp.orientationDegrees, rawRangeMat[b_sen], xyWorld);

        /*              hold all;plot(xyWorld(1),xyWorld(2),'color',clr(sen,:),'markersize',5,'marker','o'); */
        guard1 = false;
        if (*pnum > 1.0) {
          for (i19 = 0; i19 < 2; i19++) {
            b_xyWorld[i19] = xyWorld[i19] - xyWorld_prev[b_sen + (i19 << 4)];
          }

          if (norm(b_xyWorld) < Opt->pntVicThresh) {
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          (*pnum)++;
          b_pnum = (int)*pnum - 1;
          for (i19 = 0; i19 < 2; i19++) {
            xyRob[b_pnum + 200 * i19] = xyWorld[i19];
            xyWorld_prev[b_sen + (i19 << 4)] = xyWorld[i19];
          }

          if (flagSavePos) {
            numPoses++;
            for (i19 = 0; i19 < 2; i19++) {
              robPoseHist[((int)numPoses + 200 * i19) - 1] = Rob->state.x[i19];
            }

            flagSavePos = false;
          }

          /*  hold all;plot(xyWorld(1),xyWorld(2),'.m'); */
          /*              if sen==1 */
          /*                  numPoses=numPoses+1; */
          /*                  xyRob(numPoses,:) = xyWorld; */
          /*                  robPoseHist(numPoses,:)=Rob.state.x'; */
          /*              end */
        }
      }
    }

    /*  end process sensors */
    *LowFreqCycle = (*pnum >= Opt->NSamplesPseudoscan);
    if (*LowFreqCycle) {
      for (i19 = 0; i19 < 200; i19++) {
        for (b_sen = 0; b_sen < 2; b_sen++) {
          b_xyRob[b_sen + (i19 << 1)] = xyRob[i19 + 200 * b_sen];
        }
      }

      for (i19 = 0; i19 < 3; i19++) {
        b_RobXYT[i19] = Opt->splitFit.v[i19];
      }

      num_segments = extractFeaturesWorld(Raw, Rob->state.x, Rob->state.P,
        b_xyRob, *pnum, Sen[0].par.stdErr, robPoseHist, numPoses, Opt->SF_mm2m,
        Opt->splitFit.MIN_NUM_OF_PNTS_ON_LINE, Opt->splitFit.MIN_SEG_LEN,
        Opt->splitFit.MAX_GAP, Opt->splitFit.epsilon, Opt->splitFit.alphaProx,
        Opt->splitFit.onlyEndp, b_RobXYT);

      /* ,xyRob_P(:,:,1:pnum)); */
      /*  [Raw, num_segments]= extractFeaturesWorld(Raw, Rob(1).state.x',Rob(1).state.P, xyRob',numPoses,Sen(1).par.stdErr,Opt);%,xyRob_P(:,:,1:pnum)); */
      /*  for i=1:num_segments */
      /*      fprintf(fid,[num2str(Raw(i).nSamples) ... */
      /*          ', ' num2str(Raw(i).endpWorld(1)) ', ' num2str(Raw(i).endpWorld(2))... */
      /*          ', ' num2str(Raw(i).endpWorld(3)) ', ' num2str(Raw(i).endpWorld(4)) ', ']); */
      /*  end */
      /*  if num_segments>0 */
      /*      fprintf(fid,'\n'); */
      /*      dlmwrite('testFile_robPoseAndProx_L.txt',robPoseHist(1:numPoses,:)','-append','delimiter',','); */
      /*      dlmwrite('testFile_robPoseAndProx_L.txt',xyRob(1:numPoses,:)','-append','delimiter',','); */
      /*  end */
      i = Rob->sensors[16];

      /* Pseudosensor = Rob.sensors(end) */
      for (b_pnum = 0; b_pnum < (int)num_segments; b_pnum++) {
        /* reformatRaw */
        /*  l=eig(Raw(i).covRhoPhiWorld);% sqrt(diff(Raw(i).endpWorld([1 3]))^2+diff(Raw(i).endpWorld([2 4]))^2);% */
        if (Raw[b_pnum].nSamples > 0.0) {
          r10 = Raw[b_pnum];
          corrLmkId = correctKnownLmksExperiment(Map, Rob, &Sen[(int)i - 1],
            &r10, Lmk, Obs, Opt->correct.MD2th, Opt->correct.isVisOrtThrsh,
            Opt->correct.isVisFarThrsh, Opt->maxDistVis, Opt->maxGap);
          Raw[b_pnum] = r10;
        } else {
          corrLmkId = 0.0;
        }

        /*              if min(eig(Map.P(Map.used,Map.used)))<-0.0001 */
        /*                  disp(['Cr:minEig=' num2str(min(eig(Map.P(Map.used,Map.used)))) ', lmk=' num2str(corrLmkId)]) */
        /*                  keyboard; */
        /*              else */
        /*                  disp(['Cr:minEig=' num2str(min(eig(Map.P(Map.used,Map.used)))) ', lmk=' num2str(corrLmkId)]) */
        /*              end */
        guard1 = false;
        guard2 = false;
        if (corrLmkId > 0.0) {
          if ((20.0 < numActions + 1.0) || rtIsNaN(numActions + 1.0)) {
            numActions = 20.0;
          } else {
            numActions++;
          }

          LmkIds_p[(int)numActions - 1] = corrLmkId;
          for (i19 = 0; i19 < 4; i19++) {
            LmkEndps_p[i19 + (((int)numActions - 1) << 2)] = Lmk[(int)corrLmkId
              - 1].parEndp.e[i19];
          }

          LmkActions_p[(int)numActions - 1] = 3.0;

          /*                  fprintf(1,'crct: '); */
          guard1 = true;
        } else if ((Raw[b_pnum].nSamples == -1.0) || (fabs(d_diff(Raw[b_pnum].
                      abscissasWorld)) < Opt->segMinLenForInit)) {
          guard2 = true;
        } else {
          for (b_sen = 0; b_sen < 2; b_sen++) {
            xyWorld[b_sen] = Raw[b_pnum].endpWorld[b_sen] - Map->x[b_sen];
          }

          if (b_norm(xyWorld) > Opt->segMaxDistForInit) {
            guard2 = true;
          } else {
            guard1 = true;
          }
        }

        if (guard2) {
          if ((20.0 < numActions + 1.0) || rtIsNaN(numActions + 1.0)) {
            numActions = 20.0;
          } else {
            numActions++;
          }

          UM_lmkID++;
          LmkIds_p[(int)numActions - 1] = UM_lmkID;
          for (i19 = 0; i19 < 4; i19++) {
            LmkEndps_p[i19 + (((int)numActions - 1) << 2)] = Raw[b_pnum].
              endpWorld[i19];
          }

          LmkActions_p[(int)numActions - 1] = 4.0;

          /*                  fprintf(1,'cntu: '); */
          /*                  fprintf(1,['det cov rho, phi = ' num2str(det(Raw(i).covRhoPhiWorld)) ', l1=' num2str(max(l)) ', l2=' num2str(min(l)) '.\n']) */
          /* fprintf(1,['l = ' num2str(l) '.\n']) */
        }

        if (guard1) {
          r10 = Raw[b_pnum];
          corrLmkId = initNewLmkExperiment(Map, Rob->state.size, Rob->state.r,
            Sen[(int)i - 1].sen, &r10, Lmk, Obs, Opt->init.initType);
          Raw[b_pnum] = r10;

          /*              if min(eig(Map.P(Map.used,Map.used)))<-0.0001 */
          /*                  disp(['In:minEig=' num2str(min(eig(Map.P(Map.used,Map.used)))) ', lmk=' num2str(corrLmkId)]) */
          /*                  keyboard; */
          /*              else */
          /*                  disp(['In:minEig=' num2str(min(eig(Map.P(Map.used,Map.used)))) ', lmk=' num2str(corrLmkId)]) */
          /*              end */
          /*             %% If there are number of segments - Rob pose and cov overwrite itself after each update */
          /* [LmkIds(i), LmkEndps(:,i), LmkActions_p(i)] = calcSlamOutput(Lmk, Obs(1,:),NUM_LMKS); */
          if (corrLmkId > 0.0) {
            if ((20.0 < numActions + 1.0) || rtIsNaN(numActions + 1.0)) {
              numActions = 20.0;
            } else {
              numActions++;
            }

            LmkIds_p[(int)numActions - 1] = corrLmkId;
            for (i19 = 0; i19 < 4; i19++) {
              LmkEndps_p[i19 + (((int)numActions - 1) << 2)] = Lmk[(int)
                corrLmkId - 1].parEndp.e[i19];
            }

            LmkActions_p[(int)numActions - 1] = 1.0;

            /*                  fprintf(1,'Init: '); */
          }

          /*              fprintf(1,['det cov rho, phi = ' num2str(det(Raw(i).covRhoPhiWorld)) ', l1=' num2str(floor(max(l))) ', l2=' num2str(min(l)) '.\n']) */
          /* fprintf(1,['l = ' num2str(l) '.\n']) */
          for (i19 = 0; i19 < 7; i19++) {
            Raw[b_pnum].type[i19] = cv2[i19];
          }
        }
      }

      *pnum = 0.0;
      numPoses = 0.0;
    }
  }

  if (oneOfTenCyc == 0.0) {
    /*  Communication cycle */
    if ((Map->numUsed < Opt->numRefreshMap) || rtIsNaN(Opt->numRefreshMap)) {
      b_Map = Map->numUsed;
    } else {
      b_Map = Opt->numRefreshMap;
    }

    i19 = (int)(b_Map + (1.0 - (numActions + 1.0)));
    for (b_pnum = 0; b_pnum < i19; b_pnum++) {
      i = (numActions + 1.0) + (double)b_pnum;

      /*  fill up the 'map update buffer' for map refresh */
      refreshMapCntlUnit(Map, Lmk, &corrLmkId, dv20, &d6);
      LmkIds_p[(int)i - 1] = corrLmkId;
      for (b_sen = 0; b_sen < 4; b_sen++) {
        LmkEndps_p[b_sen + (((int)i - 1) << 2)] = dv20[b_sen];
      }

      LmkActions_p[(int)i - 1] = d6;
    }

    numActions = 0.0;
    for (b_pnum = 0; b_pnum < (int)Opt->numRefreshMap; b_pnum++) {
      LmkIds[b_pnum] = LmkIds_p[b_pnum];
      for (i19 = 0; i19 < 4; i19++) {
        LmkEndps[i19 + (b_pnum << 2)] = LmkEndps_p[i19 + (b_pnum << 2)];
      }

      LmkActions[b_pnum] = LmkActions_p[b_pnum];
      LmkActions_p[b_pnum] = 0.0;
    }

    corrLmkId = Opt->numRefreshMap + 1.0;
    b_pnum = 0;
    exitg1 = false;
    while ((!exitg1) && (b_pnum <= (int)(20.0 + (1.0 - corrLmkId)) - 1)) {
      i = corrLmkId + (double)b_pnum;
      if (LmkActions_p[(int)i - 1] > 0.0) {
        LmkActions_p[(int)(i - Opt->numRefreshMap) - 1] = LmkActions_p[(int)i -
          1];
        LmkActions_p[(int)i - 1] = 0.0;
        LmkIds_p[(int)(i - Opt->numRefreshMap) - 1] = LmkIds_p[(int)i - 1];
        b_sen = (int)(i - Opt->numRefreshMap);
        for (i19 = 0; i19 < 4; i19++) {
          dv20[i19] = LmkEndps_p[i19 + (((int)i - 1) << 2)];
        }

        for (i19 = 0; i19 < 4; i19++) {
          LmkEndps_p[i19 + ((b_sen - 1) << 2)] = dv20[i19];
        }

        numActions++;
        b_pnum++;
      } else {
        exitg1 = true;
      }
    }
  }

  for (b_pnum = 0; b_pnum < 3; b_pnum++) {
    RobPoseUpd[b_pnum] = Rob->state.x[b_pnum];
  }

  for (i19 = 0; i19 < 9; i19++) {
    RobCovUpd[i19] = Rob->state.P[i19];
  }

  if (zeroCmd) {
    getMetry(Map->numUsed, xyRob, Lmk, Raw, 0.0, -1.0, b_dPosFault, oneOfTenCyc,
             Metry);
  } else if (*LowFreqCycle) {
    getMetry(Map->numUsed, xyRob, Lmk, Raw, num_segments,
             Opt->NSamplesPseudoscan, b_dPosFault, oneOfTenCyc, Metry);
  } else {
    getMetry(Map->numUsed, xyRob, Lmk, Raw, num_segments, *pnum, b_dPosFault,
             oneOfTenCyc, Metry);
  }

  corrLmkId = rt_roundd_snf(UM_lmkID);
  if (corrLmkId < 65536.0) {
    if (corrLmkId >= 0.0) {
      u1 = (unsigned short)corrLmkId;
    } else {
      u1 = 0;
    }
  } else if (corrLmkId >= 65536.0) {
    u1 = MAX_uint16_T;
  } else {
    u1 = 0;
  }

  Metry->Raw_num_segments = u1;

  /*  Temporary; */
  /*              tmpCov=Map.P(Map.used,Map.used); */
  /*              [evec, eval]=eig(tmpCov); */
  /*              goodInds = diag(eval)>1e-6; */
  /*              Map.P(Map.used,Map.used)=evec*diag(diag(eval).*(goodInds)+(~goodInds)*1e-6)*evec'; */
  /*  4. DATA LOGGING */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  TODO: do something here to collect data for post-processing or */
  /*  plotting. Think about collecting data in files using fopen, fwrite, */
  /*  etc., instead of creating large Matlab variables for data logging. */
  /*  V. Post-processing */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Enter post-processing code here */
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
 * File trailer for slamOnVehicle.c
 *
 * [EOF]
 */
