/*
 * File: extractFeaturesWorld.c
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
#include "extractFeaturesWorld.h"
#include "sqrt.h"
#include "sum.h"
#include "power.h"
#include "diff.h"
#include "abs.h"
#include "sign.h"
#include "mldivide.h"
#include "mean.h"
#include "epNom2epNew.h"
#include "abscissas2endPoints.h"
#include "projAbscissas2World.h"
#include "projRhoPhiAlpha2World.h"
#include "batchSplitFit.h"
#include "CyclicBuffer_addPos_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : Raw_type Raw[20]
 *                const double RobPoseFreeze[3]
 *                const double RobPoseFreezeCov[9]
 *                double xyRob[400]
 *                double pnum
 *                double sensStd
 *                const double b_robPoseHist[400]
 *                double b_numPoses
 *                double Opt_SF_mm2m
 *                double c_Opt_splitFit_MIN_NUM_OF_PNTS_
 *                double Opt_splitFit_MIN_SEG_LEN
 *                double Opt_splitFit_MAX_GAP
 *                double Opt_splitFit_epsilon
 *                double Opt_splitFit_alphaProx
 *                boolean_T Opt_splitFit_onlyEndp
 *                const double Opt_splitFit_v[3]
 * Return Type  : double
 */
double extractFeaturesWorld(Raw_type Raw[20], const double RobPoseFreeze[3],
  const double RobPoseFreezeCov[9], double xyRob[400], double pnum, double
  sensStd, const double b_robPoseHist[400], double b_numPoses, double
  Opt_SF_mm2m, double c_Opt_splitFit_MIN_NUM_OF_PNTS_, double
  Opt_splitFit_MIN_SEG_LEN, double Opt_splitFit_MAX_GAP, double
  Opt_splitFit_epsilon, double Opt_splitFit_alphaProx, boolean_T
  Opt_splitFit_onlyEndp, const double Opt_splitFit_v[3])
{
  double num_segments;
  double num_segments_r;
  double b_rhoPhiRob[40];
  double b_covRhoPhi[80];
  double b_abscissas[40];
  double b_avgDist[20];
  double b_nSamples[20];
  double b_num_segments;
  double Points_c[600];
  int count;
  int i25;
  boolean_T b_Points_c[200];
  double covRhoPhi_r[80];
  int i;
  double d8;
  double unusedU0[40];
  double abscissas_r[40];
  double unusedU1[20];
  double unusedU2[20];
  double lngIndx;
  double lngVal;
  int i26;
  double b_covRhoPhi_r[4];
  double dv25[2];
  double dv26[2];
  double dv27[2];
  double dv28[2];
  static const char cv3[7] = { 'p', 's', 'e', 'u', 'd', 'o', 'S' };

  double b_Raw[3];
  double c_Raw[2];
  double Jac_Wpos[6];
  double Jac_Wrt[4];
  double unusedU3[8];
  double b_Jac_Wpos[6];
  double b_unusedU3[8];
  double d_Raw[16];
  static const signed char b[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  double dv29[2];
  double nrms[3];
  double c_robPoseHist[2];
  double dv30[3];
  num_segments_r = 0.0;

  /*  [rhoPhiRob, covRhoPhi, abscissas, avgDist, nSamples] = batchSplitFit_coder([0, 0], xyRob,pnum,10,Opt); */
  batchSplitFit(RobPoseFreeze, xyRob, pnum, sensStd, Opt_SF_mm2m,
                c_Opt_splitFit_MIN_NUM_OF_PNTS_, Opt_splitFit_MIN_SEG_LEN,
                Opt_splitFit_MAX_GAP, Opt_splitFit_epsilon,
                Opt_splitFit_alphaProx, Opt_splitFit_onlyEndp, Opt_splitFit_v,
                b_rhoPhiRob, b_covRhoPhi, b_abscissas, b_avgDist, b_nSamples,
                &b_num_segments, Points_c);
  num_segments = b_num_segments;
  count = 0;
  for (i25 = 0; i25 < 200; i25++) {
    b_Points_c[i25] = (Points_c[2 + 3 * i25] > 0.0);
  }

  if (c_sum(b_Points_c) > 8.0) {
    for (i = 0; i < (int)pnum; i++) {
      if (Points_c[2 + 3 * i] > 0.0) {
        count++;
        for (i25 = 0; i25 < 2; i25++) {
          xyRob[i25 + ((count - 1) << 1)] = Points_c[i25 + 3 * i] +
            RobPoseFreeze[i25];
        }
      }
    }

    Opt_splitFit_epsilon *= 2.0;
    c_Opt_splitFit_MIN_NUM_OF_PNTS_ = rt_roundd_snf
      (c_Opt_splitFit_MIN_NUM_OF_PNTS_ * 2.0 / 3.0);
    b_batchSplitFit(RobPoseFreeze, xyRob, count, sensStd, Opt_SF_mm2m,
                    c_Opt_splitFit_MIN_NUM_OF_PNTS_, Opt_splitFit_MIN_SEG_LEN,
                    Opt_splitFit_MAX_GAP, Opt_splitFit_epsilon,
                    Opt_splitFit_alphaProx, true, Opt_splitFit_v, unusedU0,
                    covRhoPhi_r, abscissas_r, unusedU1, unusedU2,
                    &num_segments_r);
    lngIndx = 1.0;
    lngVal = -1000.0;
    for (count = 1; count - 1 < (int)(num_segments_r + -1.0); count++) {
      for (i25 = 0; i25 < 2; i25++) {
        for (i26 = 0; i26 < 2; i26++) {
          b_covRhoPhi_r[i26 + (i25 << 1)] = covRhoPhi_r[(i25 + (i26 << 1)) +
            (count << 2)];
        }
      }

      c_diff(b_covRhoPhi_r, dv25);
      c_power(dv25, dv26);
      d8 = sum(dv26);
      b_sqrt(&d8);
      if (d8 > lngVal) {
        lngIndx = 2.0 + (double)(count - 1);
        for (i25 = 0; i25 < 2; i25++) {
          for (i26 = 0; i26 < 2; i26++) {
            b_covRhoPhi_r[i26 + (i25 << 1)] = covRhoPhi_r[(i25 + (i26 << 1)) +
              (count << 2)];
          }
        }

        c_diff(b_covRhoPhi_r, dv27);
        c_power(dv27, dv28);
        lngVal = sum(dv28);
        b_sqrt(&lngVal);
      }
    }

    for (i25 = 0; i25 < 2; i25++) {
      for (i26 = 0; i26 < 2; i26++) {
        b_covRhoPhi_r[i26 + (i25 << 1)] = covRhoPhi_r[(i26 + (i25 << 1)) +
          (((int)lngIndx - 1) << 2)];
        covRhoPhi_r[i26 + (i25 << 1)] = b_covRhoPhi_r[i26 + (i25 << 1)];
      }
    }

    if ((1.0 < num_segments_r) || rtIsNaN(num_segments_r)) {
      num_segments_r = 1.0;
    }
  } else {
    memcpy(&covRhoPhi_r[0], &b_covRhoPhi[0], 80U * sizeof(double));
  }

  /*  hold all;plot(xyRob(1,1:pnum),xyRob(2,1:pnum),'om'); */
  /*        Conversion of segments endpoints from world frame to robot frame %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  if ((20.0 < b_num_segments) || rtIsNaN(b_num_segments)) {
    d8 = 20.0;
  } else {
    d8 = b_num_segments;
  }

  for (i = 0; i < (int)d8; i++) {
    /*          rd =Raw.data(i); */
    /* disp(['Det cov rhoPhi = ' num2str(det(Raw(i).covRhoPhi)) '']) */
    for (i25 = 0; i25 < 2; i25++) {
      Raw[i].rhoPhiRob[i25] = b_rhoPhiRob[i25 + (i << 1)];
      for (i26 = 0; i26 < 2; i26++) {
        Raw[i].covRhoPhi[i26 + (i25 << 1)] = b_covRhoPhi[(i26 + (i25 << 1)) + (i
          << 2)];
      }

      Raw[i].abscissas[i25] = b_abscissas[i25 + (i << 1)];
    }

    Raw[i].nSamples = b_nSamples[i];

    /* disp(['num samples per seg: ' num2str(nSamples(i))]) */
    Raw[i].length = b_abscissas[1 + (i << 1)] - b_abscissas[i << 1];

    /* disp(['seg length: ' num2str(Raw(i).length)]) */
    for (i25 = 0; i25 < 3; i25++) {
      Raw[i].robPoseFreeze[i25] = RobPoseFreeze[i25];
    }

    memcpy(&Raw[i].robPoseCov[0], &RobPoseFreezeCov[0], 9U * sizeof(double));
    Raw[i].avgDist = b_avgDist[i];

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%  Transform everything to world %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    for (i25 = 0; i25 < 7; i25++) {
      Raw[i].type[i25] = cv3[i25];
    }

    /*  IMPORTANT CHANGE: for looping over Raws in main slamtb. Required for init and correct. */
    /*  Transform the segment to world c.s. */
    /*  The following is a system coordinate transformation of [rho, phi] */
    /*  from local robot coordinate to world (Map) coordiates: */
    b_Raw[2] = 0.0;
    for (count = 0; count < 2; count++) {
      b_Raw[count] = Raw[i].robPoseFreeze[count];
      c_Raw[count] = Raw[i].rhoPhiRob[count];
    }

    projRhoPhiAlpha2World(b_Raw, c_Raw, Raw[i].rhoPhiWorld, Jac_Wpos, Jac_Wrt);

    /*  This is solely the line fit covariance of [rho, phi] (from SaM) transformed to world coordinates. */
    for (i25 = 0; i25 < 4; i25++) {
      Raw[i].covRhoPhiWorld[i25] = 0.0;
    }

    for (i25 = 0; i25 < 2; i25++) {
      for (i26 = 0; i26 < 2; i26++) {
        b_covRhoPhi_r[i25 + (i26 << 1)] = 0.0;
        for (count = 0; count < 2; count++) {
          b_covRhoPhi_r[i25 + (i26 << 1)] += Jac_Wrt[i25 + (count << 1)] * Raw[i]
            .covRhoPhi[count + (i26 << 1)];
        }
      }

      for (i26 = 0; i26 < 2; i26++) {
        Raw[i].covRhoPhiWorld[i25 + (i26 << 1)] = 0.0;
        for (count = 0; count < 2; count++) {
          Raw[i].covRhoPhiWorld[i25 + (i26 << 1)] += b_covRhoPhi_r[i25 + (count <<
            1)] * Jac_Wrt[i26 + (count << 1)];
        }
      }
    }

    for (i25 = 0; i25 < 4; i25++) {
      Raw[i].covRhoPhiWorldForInit[i25] = 0.0;
    }

    projAbscissas2World(*(double (*)[2])&Raw[i].robPoseFreeze[0], Raw[i].
                        rhoPhiRob, Raw[i].abscissas, Raw[i].abscissasWorld);
    abscissas2endPoints(Raw[i].rhoPhiWorld, Raw[i].abscissasWorld, b_covRhoPhi_r,
                        unusedU3, Jac_Wrt);
    epNom2epNew(Raw[i].rhoPhiWorld, b_covRhoPhi_r, Jac_Wrt, unusedU3);

    /*  calculate the influence of rhoPhi uncertainty on endpoints. */
    for (i25 = 0; i25 < 2; i25++) {
      for (i26 = 0; i26 < 3; i26++) {
        b_Jac_Wpos[i25 + (i26 << 1)] = 0.0;
        for (count = 0; count < 3; count++) {
          b_Jac_Wpos[i25 + (i26 << 1)] += Jac_Wpos[i25 + (count << 1)] * Raw[i].
            robPoseCov[i26 + 3 * count];
        }
      }

      for (i26 = 0; i26 < 2; i26++) {
        Raw[i].covRhoPhiWorldForInit[i25 + (i26 << 1)] = 0.0;
        for (count = 0; count < 3; count++) {
          Raw[i].covRhoPhiWorldForInit[i25 + (i26 << 1)] += b_Jac_Wpos[i25 +
            (count << 1)] * Jac_Wpos[i26 + (count << 1)];
        }
      }

      Raw[i].endpWorld[i25] = b_covRhoPhi_r[i25 << 1];
      Raw[i].endpWorld[i25 + 2] = b_covRhoPhi_r[1 + (i25 << 1)];
    }

    /*  [epWorld1'; epWorld2']; */
    for (i25 = 0; i25 < 4; i25++) {
      for (i26 = 0; i26 < 2; i26++) {
        b_unusedU3[i25 + (i26 << 2)] = 0.0;
        for (count = 0; count < 2; count++) {
          b_unusedU3[i25 + (i26 << 2)] += unusedU3[i25 + (count << 2)] * Raw[i].
            covRhoPhiWorld[count + (i26 << 1)];
        }
      }

      for (i26 = 0; i26 < 4; i26++) {
        lngIndx = 0.0;
        for (count = 0; count < 2; count++) {
          lngIndx += b_unusedU3[i25 + (count << 2)] * unusedU3[i26 + (count << 2)];
        }

        Raw[i].covEndpointsWorld[i25 + (i26 << 2)] = lngIndx + Raw[i].avgDist *
          (double)b[i25 + (i26 << 2)];
      }
    }

    for (i25 = 0; i25 < 4; i25++) {
      for (i26 = 0; i26 < 4; i26++) {
        d_Raw[i26 + (i25 << 2)] = (Raw[i].covEndpointsWorld[i26 + (i25 << 2)] +
          Raw[i].covEndpointsWorld[i25 + (i26 << 2)]) / 2.0;
      }
    }

    for (i25 = 0; i25 < 4; i25++) {
      for (i26 = 0; i26 < 4; i26++) {
        Raw[i].covEndpointsWorld[i26 + (i25 << 2)] = d_Raw[i26 + (i25 << 2)];
      }
    }

    /*  J=ds/dr_xyt*dr_xyt/dr_xyzq */
    /*  J=[[ 1, 0, s(2) - robPose_v(2)];... */
    /*  [ 0, 1, robPose_v(1) - s(1)];... */
    /*  [ 1, 0, s(4) - robPose_v(2)];... */
    /*  [ 0, 1, robPose_v(1) - s(3)]]; */
    Raw[i].J_s2r[0] = 1.0;
    Raw[i].J_s2r[4] = 0.0;
    Raw[i].J_s2r[8] = -Raw[i].endpWorld[1] + RobPoseFreeze[1];
    Raw[i].J_s2r[1] = 0.0;
    Raw[i].J_s2r[5] = 1.0;
    Raw[i].J_s2r[9] = -RobPoseFreeze[0] + Raw[i].endpWorld[0];
    Raw[i].J_s2r[2] = 1.0;
    Raw[i].J_s2r[6] = 0.0;
    Raw[i].J_s2r[10] = -Raw[i].endpWorld[3] + RobPoseFreeze[1];
    Raw[i].J_s2r[3] = 0.0;
    Raw[i].J_s2r[7] = 1.0;
    Raw[i].J_s2r[11] = -RobPoseFreeze[0] + Raw[i].endpWorld[2];

    /*  % Constant parameters: */
    /*  syms xr yr xe ye real; */
    /*   */
    /*  % Robor disposition (deriviative parameters): */
    /*  syms dxr dyr dttar real; */
    /*   */
    /*  % Robot location: */
    /*  r_w_init=[xr;yr]; */
    /*  dr_w=[dxr;dyr]; */
    /*   */
    /*  % Endpoint location: */
    /*  e_w=[xe;ye]; */
    /*   */
    /*  % seg with relation to r */
    /*  s_wrt_r=e_w-r_w_init; */
    /*   */
    /*  % seg with relation to r rotated by dttar: % TODO: check sign (direction) */
    /*  s_wrt_r_rot=[cos(dttar) -sin(dttar); sin(dttar) cos(dttar)]*s_wrt_r; */
    /*   */
    /*  % new endpoint in world: */
    /*  endpNew = s_wrt_r_rot+r_w_init+dr_w; */
    /*   */
    /*  %test 2: */
    /*  J2_1=diff(endpNew,dxr); */
    /*  J2_2=diff(endpNew,dyr); */
    /*  J2_3=subs(diff(endpNew,dttar),dttar,0); */
    /*  J=subs(subs([J2_1 J2_2 J2_3],cos(dttar),1),sin(dttar),dttar) */
    /*   */
    /*  % Result: */
    /*  J=[[ 1, 0, ye - yr];... */
    /*      [ 0, 1, xr - xe]]; */
    mean(b_covRhoPhi_r, c_Raw);
    dv29[0] = cos(Raw[i].rhoPhiWorld[1]);
    dv29[1] = sin(Raw[i].rhoPhiWorld[1]);
    for (i25 = 0; i25 < 2; i25++) {
      c_robPoseHist[i25] = b_robPoseHist[200 * i25] - c_Raw[i25];
    }

    nrms[0] = mldivide(dv29, c_robPoseHist);
    b_sign(&nrms[0]);
    mean(b_covRhoPhi_r, c_Raw);
    dv29[0] = cos(Raw[i].rhoPhiWorld[1]);
    dv29[1] = sin(Raw[i].rhoPhiWorld[1]);
    for (i25 = 0; i25 < 2; i25++) {
      c_robPoseHist[i25] = b_robPoseHist[((int)b_numPoses + 200 * i25) - 1] -
        c_Raw[i25];
    }

    nrms[1] = mldivide(dv29, c_robPoseHist);
    b_sign(&nrms[1]);
    mean(b_covRhoPhi_r, c_Raw);
    dv29[0] = cos(Raw[i].rhoPhiWorld[1]);
    dv29[1] = sin(Raw[i].rhoPhiWorld[1]);
    i25 = (int)rt_roundd_snf(b_numPoses / 2.0);
    for (i26 = 0; i26 < 2; i26++) {
      c_robPoseHist[i26] = b_robPoseHist[(i25 + 200 * i26) - 1] - c_Raw[i26];
    }

    nrms[2] = mldivide(dv29, c_robPoseHist);
    b_sign(&nrms[2]);
    b_abs(nrms, dv30);
    if (e_sum(dv30) == 3.0) {
      lngIndx = sin(Raw[i].rhoPhiWorld[1]);
      Raw[i].nrm[0] = nrms[0] * cos(Raw[i].rhoPhiWorld[1]);
      Raw[i].nrm[1] = nrms[0] * lngIndx;
    } else {
      for (count = 0; count < 2; count++) {
        Raw[i].nrm[count] = 0.0;
      }
    }

    /*      figure(78);hold all;plot(Raw(i).endpWorld([1 3]),Raw(i).endpWorld([2,4]),'or-');shg */
    /* disp(['direction : ']);disp(num2str(sign([cos(Raw(i).rhoPhiWorld(2));sin(Raw(i).rhoPhiWorld(2))]\(Raw(i).robPoseFreeze(1:2)'-mean(endps,1)'))*[cos(Raw(i).rhoPhiWorld(2));sin(Raw(i).rhoPhiWorld(2))])) */
    /*      plot(mean(endps(:,1))+[0;nrm(1)*10],mean(endps(:,2))+[0;nrm(2)*10],'r') */
    /*  % %                     figure(77);    */
    /*      if abs(1-Raw(i).rhoPhiWorld(1)/norm(Raw(i).endpWorld(3:4)))<1e-2 */
    /*          hold all;  plot(endps(:,1),endps(:,2),'-o','linewidth',2,'color',[0.5 1 0.8],'MarkerFaceColor','m'); */
    /*          disp(['1~' num2str(Raw(i).rhoPhiWorld(1)/norm(Raw(i).endpWorld(3:4)))]) */
    /*      end */
    /*  % %                     [X1,Y1] = cov2elli( endps(1,:),Raw.data.covEndpointsWorld(1:2,1:2),3,10); plot(X1,Y1,'g:','LineWidth',3); */
    /*  % %                     [X2,Y2] = cov2elli( endps(2,:),Raw.data.covEndpointsWorld(3:4,3:4),3,10); plot(X2,Y2,'g--','LineWidth',3); */
  }

  count = 0;
  lngIndx = b_num_segments + num_segments_r;
  i25 = (int)(lngIndx + (1.0 - (b_num_segments + 1.0)));
  for (i = 0; i < i25; i++) {
    lngVal = (b_num_segments + 1.0) + (double)i;
    Raw[(int)lngVal - 1].nSamples = -1.0;
    for (i26 = 0; i26 < 2; i26++) {
      Raw[(int)lngVal - 1].endpWorld[i26] = covRhoPhi_r[i26 + (count << 2)] +
        RobPoseFreeze[i26];
      Raw[(int)lngVal - 1].endpWorld[i26 + 2] = covRhoPhi_r[2 + (i26 + (count <<
        2))] + RobPoseFreeze[i26];
    }

    count++;
    num_segments++;
  }

  for (i = 0; i < (int)(20.0 + (1.0 - (lngIndx + 1.0))); i++) {
    Raw[(int)((lngIndx + 1.0) + (double)i) - 1].nSamples = -1.0;
  }

  return num_segments;
}

/*
 * File trailer for extractFeaturesWorld.c
 *
 * [EOF]
 */
