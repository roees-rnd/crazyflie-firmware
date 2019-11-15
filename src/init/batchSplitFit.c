/*
 * File: batchSplitFit.c
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
#include "batchSplitFit.h"
#include "getMaxPenDist.h"
#include "sqrt.h"
#include "sum.h"
#include "diff.h"
#include "SaM.h"
#include "sort1.h"
#include "mod.h"
#include "mod1.h"
#include "mtlb_power.h"

/* Function Declarations */
static void calcRhoTta(const double X[2], double rhoTta[2], double J[4]);
static double rt_atan2d_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : const double X[2]
 *                double rhoTta[2]
 *                double J[4]
 * Return Type  : void
 */
static void calcRhoTta(const double X[2], double rhoTta[2], double J[4])
{
  double b_y;
  int i5;
  b_y = 0.0;
  for (i5 = 0; i5 < 2; i5++) {
    b_y += X[i5] * X[i5];
  }

  J[0] = -X[0] / rt_powd_snf(X[0] * X[0] + X[1] * X[1], 1.5);
  J[2] = -X[1] / rt_powd_snf(X[0] * X[0] + X[1] * X[1], 1.5);
  J[1] = -X[1] / (X[0] * X[0] + X[1] * X[1]);
  J[3] = X[0] / (X[0] * X[0] + X[1] * X[1]);
  rhoTta[0] = 1.0 / sqrt(b_y);
  rhoTta[1] = rt_atan2d_snf(X[1], X[0]);
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double b_y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    b_y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    b_y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      b_y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      b_y = -(RT_PI / 2.0);
    } else {
      b_y = 0.0;
    }
  } else {
    b_y = atan2(u0, u1);
  }

  return b_y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double b_y;
  double d4;
  double d5;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    b_y = rtNaN;
  } else {
    d4 = fabs(u0);
    d5 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d4 == 1.0) {
        b_y = 1.0;
      } else if (d4 > 1.0) {
        if (u1 > 0.0) {
          b_y = rtInf;
        } else {
          b_y = 0.0;
        }
      } else if (u1 > 0.0) {
        b_y = 0.0;
      } else {
        b_y = rtInf;
      }
    } else if (d5 == 0.0) {
      b_y = 1.0;
    } else if (d5 == 1.0) {
      if (u1 > 0.0) {
        b_y = u0;
      } else {
        b_y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      b_y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      b_y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      b_y = rtNaN;
    } else {
      b_y = pow(u0, u1);
    }
  }

  return b_y;
}

/*
 * Arguments    : const double orig[3]
 *                const double Points[400]
 *                double pnum
 *                double sensStd
 *                double Opt_SF_mm2m
 *                double c_Opt_splitFit_MIN_NUM_OF_PNTS_
 *                double Opt_splitFit_MIN_SEG_LEN
 *                double Opt_splitFit_MAX_GAP
 *                double Opt_splitFit_epsilon
 *                double Opt_splitFit_alphaProx
 *                boolean_T Opt_splitFit_onlyEndp
 *                const double Opt_splitFit_v[3]
 *                double rhoTta[40]
 *                double covRhoTta[80]
 *                double tminmax[40]
 *                double b_avgDist[20]
 *                double nPoints[20]
 *                double *num_segments
 * Return Type  : void
 */
void b_batchSplitFit(const double orig[3], const double Points[400], double pnum,
                     double sensStd, double Opt_SF_mm2m, double
                     c_Opt_splitFit_MIN_NUM_OF_PNTS_, double
                     Opt_splitFit_MIN_SEG_LEN, double Opt_splitFit_MAX_GAP,
                     double Opt_splitFit_epsilon, double Opt_splitFit_alphaProx,
                     boolean_T Opt_splitFit_onlyEndp, const double
                     Opt_splitFit_v[3], double rhoTta[40], double covRhoTta[80],
                     double tminmax[40], double b_avgDist[20], double nPoints[20],
                     double *num_segments)
{
  double angs[200];
  double Points_c[600];
  double H[400];
  int i;
  double srtAngs[200];
  int iidx[200];
  double mx;
  double mn;
  double maxDiff1;
  int prevInd;
  int idxMax0;
  double srtAngs_i;
  int i6;
  double segments[600];
  double b_Points_c[400];
  double dv4[398];
  int i7;
  double dv5[398];
  double d[199];
  boolean_T b_segments[200];
  double nn;
  int b_prevInd[2];
  double c[4];
  double b_z[2];
  double dv6[2];
  double sumx;
  double sumx2;
  double sumxy;
  double sumy;
  double sumy2;
  double HH_inv[4];
  double b_sumx[2];
  double b_c;
  double b_y;
  double HH[4];
  double c2[2];
  double c_y[4];
  double b_HH[4];
  double c_c[4];

  /*  =Opt.NSamplesPseudoscan; */
  memset(&rhoTta[0], 0, 40U * sizeof(double));
  memset(&covRhoTta[0], 0, 80U * sizeof(double));
  memset(&tminmax[0], 0, 40U * sizeof(double));
  memset(&nPoints[0], 0, 20U * sizeof(double));
  memset(&b_avgDist[0], 0, 20U * sizeof(double));
  *num_segments = 0.0;
  memset(&angs[0], 0, 200U * sizeof(double));
  memset(&Points_c[0], 0, 600U * sizeof(double));
  memset(&H[0], 0, 400U * sizeof(double));
  if (!(pnum < 2.0)) {
    if (!(pnum < 200.0)) {
      pnum = 200.0;
    }

    /*  showGraphs=false; */
    /*  showGraphsCov = false; */
    /*  if showGraphs */
    /*      figure(11711);clf */
    /*      ax=axes('xlim',[min(Points(1,1:pnum)) max(Points(1,1:pnum))],'ylim',[ min(Points(2,1:pnum)) max(Points(2,1:pnum))]); */
    /*      hold all; */
    /*  end */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  Sort points radially: */
    for (i = 0; i < (int)pnum; i++) {
      angs[i] = rt_atan2d_snf(Points[1 + (i << 1)] - orig[1], Points[i << 1] -
        orig[0]);
    }

    memcpy(&srtAngs[0], &angs[0], 200U * sizeof(double));
    sort(srtAngs, iidx);
    mx = -300.0;
    mn = 300.0;
    maxDiff1 = -300.0;
    prevInd = -1;
    idxMax0 = 1;
    srtAngs_i = 0.0;
    for (i = 0; i < 200; i++) {
      if (!(iidx[i] > pnum)) {
        if (!((mx > srtAngs[i]) || rtIsNaN(srtAngs[i]))) {
          mx = srtAngs[i];
        }

        if (!((mn < srtAngs[i]) || rtIsNaN(srtAngs[i]))) {
          mn = srtAngs[i];
        }

        if ((prevInd > 0) && (srtAngs[i] - srtAngs_i > maxDiff1)) {
          maxDiff1 = srtAngs[i] - srtAngs_i;
          idxMax0 = iidx[i];
        }

        prevInd = iidx[i];
        srtAngs_i = srtAngs[i];
      }
    }

    /*  [mx, ~]=max(srtAngs); */
    /*  [mn, ~]=min(srtAngs); */
    /*  % Maximum of diff(sorted_angles): */
    /*  [maxDiff1, idxMax0]=max(diff(srtAngs)); */
    /*  Difference around pi: */
    /*  Find neutral angle (direction with biggest angular gap): */
    if (maxDiff1 > (mn + 6.2831853071795862) - mx) {
      mx = srtAngs[idxMax0 - 1] + 0.5 * maxDiff1;
    } else {
      mx = floatmod(0.5 * ((mn + 6.2831853071795862) + mx) + 3.1415926535897931)
        - 3.1415926535897931;
    }

    /*  if showGraphs */
    /*      u=[cos(neutralAng);sin(neutralAng)]*1000; */
    /*      plot([orig(1), orig(1)+u(1)],[orig(2),orig(2)+u(2)],'-ok','MarkerFaceColor',[0 0 0],'markersize',9) */
    /*      plot(Points(1,inds(idxMax0)),Points(2,inds(idxMax0)),'om','MarkerFaceColor',[0 0 0],'markersize',9) */
    /*      plot(Points(1,inds(idxMax1)),Points(2,inds(idxMax1)),'ob','MarkerFaceColor',[0 0 0],'markersize',9) */
    /*  end */
    /*  Transform range from [-pi,pi] to [0,2*pi] where neutralAng is now 0 */
    /*  degrees: */
    for (i6 = 0; i6 < 200; i6++) {
      srtAngs[i6] = angs[i6] - mx;
    }

    b_mod(srtAngs, angs);
    b_sort(angs, iidx);
    idxMax0 = 0;
    for (i = 0; i < 200; i++) {
      if (!(iidx[i] > pnum)) {
        for (i6 = 0; i6 < 2; i6++) {
          Points_c[i6 + 3 * idxMax0] = Points[i6 + ((iidx[i] - 1) << 1)] -
            orig[i6];
        }

        Points_c[2 + 3 * idxMax0] = idxMax0 + 1;
        idxMax0++;
      }
    }

    /* figure;hold all;k=1;for iii=1:maxNumPoints,if inds_(iii)>pnum,continue,end,plot(Points(1,inds_(iii)),Points(2,inds_(iii)),'o'),k=k+1;pause(0.1);,end */
    /*  the std is parabolic with minimum at x1, with sdt multiplied by b[2], and */
    /*  multplied by b[3] at x2 */
    /*  % x1=400; */
    /*  % x2=900; */
    /*  % abc=[2*x1 1 0;x1^2 x1 1;x2^2 x2 1]; */
    /*  % b=[0;1;4]; */
    /*  % v=abc\b; */
    /*  Points(1:2,1:pnum)=Points(1:2,inds_); */
    /*  Points_c=cat(1,Points(:,1:pnum),1:pnum); */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  for i=1:size(Points_c,2)*showGraphs */
    /*      plot(Points_c(1,i),Points_c(2,i),'ob','markersize',11); axis equal; drawnow;%pause(0.1); */
    /*  end */
    /*  Split */
    memcpy(&segments[0], &Points_c[0], 600U * sizeof(double));
    SaM(segments, 1.0, pnum, Opt_splitFit_epsilon);

    /*  Merge: */
    mx = -1.0;
    mn = -1.0;
    for (i = 1; i - 1 < (int)((pnum - 1.0) + -1.0); i++) {
      if (segments[2 + 3 * i] > 0.0) {
        if (mn > 0.0) {
          getMaxPenDist(Points_c, mn, Points_c[2 + 3 * i], &maxDiff1, &srtAngs_i);
          if (maxDiff1 < Opt_splitFit_epsilon) {
            segments[2 + 3 * ((int)mx - 1)] = 0.0;
            mx = Points_c[2 + 3 * i];
          } else {
            mn = mx;
            mx = Points_c[2 + 3 * i];
          }
        } else {
          mn = mx;
          mx = Points_c[2 + 3 * i];
        }
      }
    }

    /*  Detect gaps in segments: */
    for (i6 = 0; i6 < 200; i6++) {
      for (i7 = 0; i7 < 2; i7++) {
        b_Points_c[i7 + (i6 << 1)] = Points_c[i7 + 3 * i6];
      }
    }

    diff(b_Points_c, dv4);
    mtlb_power(dv4, dv5);
    b_sum(dv5, d);
    c_sqrt(d);
    for (i = 0; i < (int)(pnum - 1.0); i++) {
      if (d[i] > Opt_splitFit_MAX_GAP) {
        segments[2 + 3 * i] = Points_c[2 + 3 * i];
        segments[2 + 3 * (i + 1)] = Points_c[2 + 3 * (i + 1)];
      }
    }

    for (i6 = 0; i6 < 200; i6++) {
      b_segments[i6] = (segments[2 + 3 * i6] > 0.0);
    }

    *num_segments = c_sum(b_segments) - 1.0;

    /* why is the minus 1??? */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  Delete segments outside tolerances: */
    nn = 0.0;
    idxMax0 = 0;
    mx = 0.0;
    prevInd = 0;
    while (*num_segments - nn > 0.0) {
      idxMax0++;
      if (!(segments[2 + 3 * idxMax0] == 0.0)) {
        nn++;
        if (segments[2 + 3 * idxMax0] - (double)(prevInd + 1) <
            c_Opt_splitFit_MIN_NUM_OF_PNTS_) {
          segments[2 + 3 * prevInd] = 0.0;
          mx++;
        } else {
          b_prevInd[0] = prevInd + 1;
          b_prevInd[1] = idxMax0 + 1;
          for (i6 = 0; i6 < 2; i6++) {
            for (i7 = 0; i7 < 2; i7++) {
              c[i7 + (i6 << 1)] = segments[i7 + 3 * (b_prevInd[i6] - 1)];
            }
          }

          b_diff(c, b_z);
          b_power(b_z, dv6);
          maxDiff1 = d_sum(dv6);
          b_sqrt(&maxDiff1);
          if (maxDiff1 < Opt_splitFit_MIN_SEG_LEN) {
            segments[2 + 3 * prevInd] = 0.0;
            mx++;
          } else {
            segments[2 + 3 * prevInd] = segments[2 + 3 * idxMax0];
          }
        }

        prevInd = idxMax0;
      }
    }

    *num_segments -= mx;

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  if showGraphs */
    /*      plot(reshape(segmentsMat([1 4],:),[],1),reshape(segmentsMat([2 5],:),[],1),'ro','markersize',8,'linewidth',3) */
    /*  end */
    nn = 0.0;
    if (Opt_splitFit_onlyEndp) {
      for (i = 0; i < (int)*num_segments; i++) {
        do {
          nn++;
        } while (segments[2 + 3 * ((int)nn - 1)] == 0.0);

        for (i6 = 0; i6 < 2; i6++) {
          covRhoTta[i6 + (i << 2)] = segments[i6 + 3 * ((int)nn - 1)];
          covRhoTta[2 + (i6 + (i << 2))] = segments[i6 + 3 * ((int)segments[2 +
            3 * ((int)nn - 1)] - 1)];
        }
      }
    } else {
      for (i = 0; i < (int)*num_segments; i++) {
        do {
          nn++;
        } while (segments[2 + 3 * ((int)nn - 1)] == 0.0);

        /* [rho(i), tta(i)] = fitLineRhoAlpha(Points(1,cur_seg_idxs),Points(2,cur_seg_idxs)); */
        sumx = 0.0;
        sumx2 = 0.0;
        sumxy = 0.0;
        sumy = 0.0;
        sumy2 = 0.0;
        nPoints[i] = (segments[2 + 3 * ((int)nn - 1)] - nn) + 1.0;
        i6 = (int)(segments[2 + 3 * ((int)nn - 1)] + (1.0 - nn));
        for (idxMax0 = 0; idxMax0 < i6; idxMax0++) {
          mn = nn + (double)idxMax0;
          for (i7 = 0; i7 < 2; i7++) {
            H[((int)mn + 200 * i7) - 1] = Points_c[i7 + 3 * ((int)mn - 1)];
          }

          sumx += Points_c[3 * ((int)mn - 1)];
          sumx2 += Points_c[3 * ((int)mn - 1)] * Points_c[3 * ((int)mn - 1)];
          sumy += Points_c[1 + 3 * ((int)mn - 1)];
          sumy2 += Points_c[1 + 3 * ((int)mn - 1)] * Points_c[1 + 3 * ((int)mn -
            1)];
          sumxy += Points_c[1 + 3 * ((int)mn - 1)] * Points_c[3 * ((int)mn - 1)];
          Points_c[2 + 3 * ((int)mn - 1)] = 0.0;
        }

        mx = (sumx * sumx + sumy * sumy) / (nPoints[i] * nPoints[i]);
        b_sqrt(&mx);
        mn = (Opt_splitFit_v[0] * (mx * mx) + Opt_splitFit_v[1] * mx) +
          Opt_splitFit_v[2];
        mx = sumy2 * sumx2 - sumxy * sumxy;
        if (!(mx > 1.0E-6)) {
          mx = 1.0E-6;
        }

        mx = 1.0 / mx;
        HH_inv[0] = sumy2 * mx;
        HH_inv[2] = -sumxy * mx;
        HH_inv[1] = -sumxy * mx;
        HH_inv[3] = sumx2 * mx;
        b_sumx[0] = sumx;
        b_sumx[1] = sumy;
        for (i6 = 0; i6 < 2; i6++) {
          b_z[i6] = 0.0;
          for (i7 = 0; i7 < 2; i7++) {
            b_z[i6] += HH_inv[i6 + (i7 << 1)] * b_sumx[i7];
          }
        }

        mx = b_z[0] + b_z[1];
        srtAngs_i = mx * mx;
        mx = (1.0 - Opt_splitFit_alphaProx) * sensStd * mn;
        b_c = mx * mx;
        b_y = 0.0;
        for (i6 = 0; i6 < 2; i6++) {
          b_y += b_z[i6] * b_z[i6];
        }

        mx = sensStd * Opt_splitFit_alphaProx * mn;
        mx *= mx;
        calcRhoTta(b_z, *(double (*)[2])&rhoTta[i << 1], HH);
        b_z[0] = sin(rhoTta[1 + (i << 1)]);
        b_z[1] = -cos(rhoTta[1 + (i << 1)]);
        c2[0] = cos(rhoTta[1 + (i << 1)]);
        c2[1] = sin(rhoTta[1 + (i << 1)]);
        maxDiff1 = 0.0;
        for (i6 = 0; i6 < 2; i6++) {
          maxDiff1 += -H[((int)nn + 200 * i6) - 1] * b_z[i6];
        }

        tminmax[i << 1] = maxDiff1;
        tminmax[1 + (i << 1)] = tminmax[i << 1];
        i6 = (int)(segments[2 + 3 * ((int)nn - 1)] + (1.0 - nn));
        for (idxMax0 = 0; idxMax0 < i6; idxMax0++) {
          mn = nn + (double)idxMax0;
          maxDiff1 = 0.0;
          for (i7 = 0; i7 < 2; i7++) {
            maxDiff1 += H[((int)mn + 200 * i7) - 1] * c2[i7];
          }

          b_avgDist[i] += fabs(maxDiff1 - rhoTta[i << 1]);
          maxDiff1 = 0.0;
          for (i7 = 0; i7 < 2; i7++) {
            maxDiff1 += -H[((int)mn + 200 * i7) - 1] * b_z[i7];
          }

          if ((tminmax[i << 1] < maxDiff1) || rtIsNaN(maxDiff1)) {
          } else {
            tminmax[i << 1] = maxDiff1;
          }

          if ((tminmax[1 + (i << 1)] > maxDiff1) || rtIsNaN(maxDiff1)) {
          } else {
            tminmax[1 + (i << 1)] = maxDiff1;
          }
        }

        b_avgDist[i] /= nPoints[i];
        c[0] = srtAngs_i * (sumx * sumx) * b_c * Opt_SF_mm2m;
        c[2] = srtAngs_i * (sumx * sumy) * b_c * Opt_SF_mm2m;
        c[1] = srtAngs_i * (sumx * sumy) * b_c * Opt_SF_mm2m;
        c[3] = srtAngs_i * (sumy * sumy) * b_c * Opt_SF_mm2m;
        c_y[0] = b_y * sumx2 * mx * Opt_SF_mm2m;
        c_y[2] = b_y * sumxy * mx * Opt_SF_mm2m;
        c_y[1] = b_y * sumxy * mx * Opt_SF_mm2m;
        c_y[3] = b_y * sumy2 * mx * Opt_SF_mm2m;
        for (i6 = 0; i6 < 2; i6++) {
          for (i7 = 0; i7 < 2; i7++) {
            covRhoTta[(i7 + (i6 << 1)) + (i << 2)] = 0.0;
            b_HH[i6 + (i7 << 1)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              b_HH[i6 + (i7 << 1)] += HH[i6 + (idxMax0 << 1)] * HH_inv[idxMax0 +
                (i7 << 1)];
            }

            c_c[i7 + (i6 << 1)] = c[i7 + (i6 << 1)] + c_y[i7 + (i6 << 1)];
          }
        }

        for (i6 = 0; i6 < 2; i6++) {
          for (i7 = 0; i7 < 2; i7++) {
            c[i6 + (i7 << 1)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              c[i6 + (i7 << 1)] += b_HH[i6 + (idxMax0 << 1)] * c_c[idxMax0 + (i7
                << 1)];
            }
          }

          for (i7 = 0; i7 < 2; i7++) {
            c_y[i6 + (i7 << 1)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              c_y[i6 + (i7 << 1)] += c[i6 + (idxMax0 << 1)] * HH_inv[idxMax0 +
                (i7 << 1)];
            }
          }

          for (i7 = 0; i7 < 2; i7++) {
            covRhoTta[(i6 + (i7 << 1)) + (i << 2)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              covRhoTta[(i6 + (i7 << 1)) + (i << 2)] += c_y[i6 + (idxMax0 << 1)]
                * HH[i7 + (idxMax0 << 1)];
            }
          }
        }

        /*      if  isempty(coder.target) */
        /*          if showGraphs */
        /*              plot(Points(1,cur_seg_idxs),Points(2,cur_seg_idxs),'o','markersize',5,'linewidth',2,'MarkerFaceColor',0.5+0.5*[0.9 0.9 0.6]) */
        /*              plot(segmentsMat([1 4],i),segmentsMat([2 5],i),'k') */
        /*              text_xy = mean([Points(1,cur_seg_idxs);Points(2,cur_seg_idxs)],2); */
        /*              text(text_xy(1),text_xy(2),['\rho = ' num2str(rho) ', \theta = ' num2str((180/pi)*tta)]); */
        /*          end */
        /*      end */
      }
    }

    /*  if showGraphsCov */
    /*      figure(1234); */
    /*      aa=get(get(1234,'CurrentAxes'),'children'); */
    /*      if ~isempty(aa) */
    /*          ind=aa(1).XData(end); */
    /*          if strcmp(aa(1).Tag,'ang') */
    /*              set(aa(1),'XData',[aa(1).XData (ind+1)*ones(1,length(dets))],'YData',[aa(1).YData dets']) */
    /*          else */
    /*              assert(false); */
    /*          end */
    /*      else */
    /*          hold all; */
    /*          plot(ones(length(dets)),dets,'Tag','ang'); */
    /*      end */
    /*       */
    /*      figure(12345); */
    /*      aa=get(get(12345,'CurrentAxes'),'children'); */
    /*      if ~isempty(aa) */
    /*          ind=aa(1).XData(end); */
    /*          if strcmp(aa(1).Tag,'detZ') */
    /*              set(aa(1),'XData',[aa(1).XData (ind+1)*ones(1,length(dets))],'YData',[aa(1).YData detsRhoPhi']) */
    /*          else */
    /*              assert(false); */
    /*          end */
    /*      else */
    /*          hold all; */
    /*          plot(ones(length(dets)),detsRhoPhi,'Tag','detZ');legend('det([a;b])', 'det([rho;phi])') */
    /*      end */
    /*       */
    /*  end */
  }
}

/*
 * Arguments    : const double orig[3]
 *                const double Points[400]
 *                double pnum
 *                double sensStd
 *                double Opt_SF_mm2m
 *                double c_Opt_splitFit_MIN_NUM_OF_PNTS_
 *                double Opt_splitFit_MIN_SEG_LEN
 *                double Opt_splitFit_MAX_GAP
 *                double Opt_splitFit_epsilon
 *                double Opt_splitFit_alphaProx
 *                boolean_T Opt_splitFit_onlyEndp
 *                const double Opt_splitFit_v[3]
 *                double rhoTta[40]
 *                double covRhoTta[80]
 *                double tminmax[40]
 *                double b_avgDist[20]
 *                double nPoints[20]
 *                double *num_segments
 *                double Points_c[600]
 * Return Type  : void
 */
void batchSplitFit(const double orig[3], const double Points[400], double pnum,
                   double sensStd, double Opt_SF_mm2m, double
                   c_Opt_splitFit_MIN_NUM_OF_PNTS_, double
                   Opt_splitFit_MIN_SEG_LEN, double Opt_splitFit_MAX_GAP, double
                   Opt_splitFit_epsilon, double Opt_splitFit_alphaProx,
                   boolean_T Opt_splitFit_onlyEndp, const double Opt_splitFit_v
                   [3], double rhoTta[40], double covRhoTta[80], double tminmax
                   [40], double b_avgDist[20], double nPoints[20], double
                   *num_segments, double Points_c[600])
{
  double angs[200];
  double H[400];
  int i;
  double srtAngs[200];
  int iidx[200];
  double mx;
  double mn;
  double maxDiff1;
  int prevInd;
  int idxMax0;
  double srtAngs_i;
  int i2;
  double segments[600];
  double b_Points_c[400];
  double dv1[398];
  int i3;
  double dv2[398];
  double d[199];
  boolean_T b_segments[200];
  double nn;
  int b_prevInd[2];
  double c[4];
  double b_z[2];
  double dv3[2];
  double sumx;
  double sumx2;
  double sumxy;
  double sumy;
  double sumy2;
  double HH_inv[4];
  double b_sumx[2];
  double b_c;
  double b_y;
  double HH[4];
  double c2[2];
  double c_y[4];
  double b_HH[4];
  double c_c[4];

  /*  =Opt.NSamplesPseudoscan; */
  memset(&rhoTta[0], 0, 40U * sizeof(double));
  memset(&covRhoTta[0], 0, 80U * sizeof(double));
  memset(&tminmax[0], 0, 40U * sizeof(double));
  memset(&nPoints[0], 0, 20U * sizeof(double));
  memset(&b_avgDist[0], 0, 20U * sizeof(double));
  *num_segments = 0.0;
  memset(&angs[0], 0, 200U * sizeof(double));
  memset(&Points_c[0], 0, 600U * sizeof(double));
  memset(&H[0], 0, 400U * sizeof(double));
  if (!(pnum < 2.0)) {
    if (!(pnum < 200.0)) {
      pnum = 200.0;
    }

    /*  showGraphs=false; */
    /*  showGraphsCov = false; */
    /*  if showGraphs */
    /*      figure(11711);clf */
    /*      ax=axes('xlim',[min(Points(1,1:pnum)) max(Points(1,1:pnum))],'ylim',[ min(Points(2,1:pnum)) max(Points(2,1:pnum))]); */
    /*      hold all; */
    /*  end */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  Sort points radially: */
    for (i = 0; i < (int)pnum; i++) {
      angs[i] = rt_atan2d_snf(Points[1 + (i << 1)] - orig[1], Points[i << 1] -
        orig[0]);
    }

    memcpy(&srtAngs[0], &angs[0], 200U * sizeof(double));
    sort(srtAngs, iidx);
    mx = -300.0;
    mn = 300.0;
    maxDiff1 = -300.0;
    prevInd = -1;
    idxMax0 = 1;
    srtAngs_i = 0.0;
    for (i = 0; i < 200; i++) {
      if (!(iidx[i] > pnum)) {
        if (!((mx > srtAngs[i]) || rtIsNaN(srtAngs[i]))) {
          mx = srtAngs[i];
        }

        if (!((mn < srtAngs[i]) || rtIsNaN(srtAngs[i]))) {
          mn = srtAngs[i];
        }

        if ((prevInd > 0) && (srtAngs[i] - srtAngs_i > maxDiff1)) {
          maxDiff1 = srtAngs[i] - srtAngs_i;
          idxMax0 = iidx[i];
        }

        prevInd = iidx[i];
        srtAngs_i = srtAngs[i];
      }
    }

    /*  [mx, ~]=max(srtAngs); */
    /*  [mn, ~]=min(srtAngs); */
    /*  % Maximum of diff(sorted_angles): */
    /*  [maxDiff1, idxMax0]=max(diff(srtAngs)); */
    /*  Difference around pi: */
    /*  Find neutral angle (direction with biggest angular gap): */
    if (maxDiff1 > (mn + 6.2831853071795862) - mx) {
      mx = srtAngs[idxMax0 - 1] + 0.5 * maxDiff1;
    } else {
      mx = floatmod(0.5 * ((mn + 6.2831853071795862) + mx) + 3.1415926535897931)
        - 3.1415926535897931;
    }

    /*  if showGraphs */
    /*      u=[cos(neutralAng);sin(neutralAng)]*1000; */
    /*      plot([orig(1), orig(1)+u(1)],[orig(2),orig(2)+u(2)],'-ok','MarkerFaceColor',[0 0 0],'markersize',9) */
    /*      plot(Points(1,inds(idxMax0)),Points(2,inds(idxMax0)),'om','MarkerFaceColor',[0 0 0],'markersize',9) */
    /*      plot(Points(1,inds(idxMax1)),Points(2,inds(idxMax1)),'ob','MarkerFaceColor',[0 0 0],'markersize',9) */
    /*  end */
    /*  Transform range from [-pi,pi] to [0,2*pi] where neutralAng is now 0 */
    /*  degrees: */
    for (i2 = 0; i2 < 200; i2++) {
      srtAngs[i2] = angs[i2] - mx;
    }

    b_mod(srtAngs, angs);
    b_sort(angs, iidx);
    idxMax0 = 0;
    for (i = 0; i < 200; i++) {
      if (!(iidx[i] > pnum)) {
        for (i2 = 0; i2 < 2; i2++) {
          Points_c[i2 + 3 * idxMax0] = Points[i2 + ((iidx[i] - 1) << 1)] -
            orig[i2];
        }

        Points_c[2 + 3 * idxMax0] = idxMax0 + 1;
        idxMax0++;
      }
    }

    /* figure;hold all;k=1;for iii=1:maxNumPoints,if inds_(iii)>pnum,continue,end,plot(Points(1,inds_(iii)),Points(2,inds_(iii)),'o'),k=k+1;pause(0.1);,end */
    /*  the std is parabolic with minimum at x1, with sdt multiplied by b[2], and */
    /*  multplied by b[3] at x2 */
    /*  % x1=400; */
    /*  % x2=900; */
    /*  % abc=[2*x1 1 0;x1^2 x1 1;x2^2 x2 1]; */
    /*  % b=[0;1;4]; */
    /*  % v=abc\b; */
    /*  Points(1:2,1:pnum)=Points(1:2,inds_); */
    /*  Points_c=cat(1,Points(:,1:pnum),1:pnum); */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  for i=1:size(Points_c,2)*showGraphs */
    /*      plot(Points_c(1,i),Points_c(2,i),'ob','markersize',11); axis equal; drawnow;%pause(0.1); */
    /*  end */
    /*  Split */
    memcpy(&segments[0], &Points_c[0], 600U * sizeof(double));
    SaM(segments, 1.0, pnum, Opt_splitFit_epsilon);

    /*  Merge: */
    mx = -1.0;
    mn = -1.0;
    for (i = 1; i - 1 < (int)((pnum - 1.0) + -1.0); i++) {
      if (segments[2 + 3 * i] > 0.0) {
        if (mn > 0.0) {
          getMaxPenDist(Points_c, mn, Points_c[2 + 3 * i], &maxDiff1, &srtAngs_i);
          if (maxDiff1 < Opt_splitFit_epsilon) {
            segments[2 + 3 * ((int)mx - 1)] = 0.0;
            mx = Points_c[2 + 3 * i];
          } else {
            mn = mx;
            mx = Points_c[2 + 3 * i];
          }
        } else {
          mn = mx;
          mx = Points_c[2 + 3 * i];
        }
      }
    }

    /*  Detect gaps in segments: */
    for (i2 = 0; i2 < 200; i2++) {
      for (i3 = 0; i3 < 2; i3++) {
        b_Points_c[i3 + (i2 << 1)] = Points_c[i3 + 3 * i2];
      }
    }

    diff(b_Points_c, dv1);
    mtlb_power(dv1, dv2);
    b_sum(dv2, d);
    c_sqrt(d);
    for (i = 0; i < (int)(pnum - 1.0); i++) {
      if (d[i] > Opt_splitFit_MAX_GAP) {
        segments[2 + 3 * i] = Points_c[2 + 3 * i];
        segments[2 + 3 * (i + 1)] = Points_c[2 + 3 * (i + 1)];
      }
    }

    for (i2 = 0; i2 < 200; i2++) {
      b_segments[i2] = (segments[2 + 3 * i2] > 0.0);
    }

    *num_segments = c_sum(b_segments) - 1.0;

    /* why is the minus 1??? */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  Delete segments outside tolerances: */
    nn = 0.0;
    idxMax0 = 0;
    mx = 0.0;
    prevInd = 0;
    while (*num_segments - nn > 0.0) {
      idxMax0++;
      if (!(segments[2 + 3 * idxMax0] == 0.0)) {
        nn++;
        if (segments[2 + 3 * idxMax0] - (double)(prevInd + 1) <
            c_Opt_splitFit_MIN_NUM_OF_PNTS_) {
          segments[2 + 3 * prevInd] = 0.0;
          mx++;
        } else {
          b_prevInd[0] = prevInd + 1;
          b_prevInd[1] = idxMax0 + 1;
          for (i2 = 0; i2 < 2; i2++) {
            for (i3 = 0; i3 < 2; i3++) {
              c[i3 + (i2 << 1)] = segments[i3 + 3 * (b_prevInd[i2] - 1)];
            }
          }

          b_diff(c, b_z);
          b_power(b_z, dv3);
          maxDiff1 = d_sum(dv3);
          b_sqrt(&maxDiff1);
          if (maxDiff1 < Opt_splitFit_MIN_SEG_LEN) {
            segments[2 + 3 * prevInd] = 0.0;
            mx++;
          } else {
            segments[2 + 3 * prevInd] = segments[2 + 3 * idxMax0];
          }
        }

        prevInd = idxMax0;
      }
    }

    *num_segments -= mx;

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  if showGraphs */
    /*      plot(reshape(segmentsMat([1 4],:),[],1),reshape(segmentsMat([2 5],:),[],1),'ro','markersize',8,'linewidth',3) */
    /*  end */
    nn = 0.0;
    if (Opt_splitFit_onlyEndp) {
      for (i = 0; i < (int)*num_segments; i++) {
        do {
          nn++;
        } while (segments[2 + 3 * ((int)nn - 1)] == 0.0);

        for (i2 = 0; i2 < 2; i2++) {
          covRhoTta[i2 + (i << 2)] = segments[i2 + 3 * ((int)nn - 1)];
          covRhoTta[2 + (i2 + (i << 2))] = segments[i2 + 3 * ((int)segments[2 +
            3 * ((int)nn - 1)] - 1)];
        }
      }
    } else {
      for (i = 0; i < (int)*num_segments; i++) {
        do {
          nn++;
        } while (segments[2 + 3 * ((int)nn - 1)] == 0.0);

        /* [rho(i), tta(i)] = fitLineRhoAlpha(Points(1,cur_seg_idxs),Points(2,cur_seg_idxs)); */
        sumx = 0.0;
        sumx2 = 0.0;
        sumxy = 0.0;
        sumy = 0.0;
        sumy2 = 0.0;
        nPoints[i] = (segments[2 + 3 * ((int)nn - 1)] - nn) + 1.0;
        i2 = (int)(segments[2 + 3 * ((int)nn - 1)] + (1.0 - nn));
        for (idxMax0 = 0; idxMax0 < i2; idxMax0++) {
          mn = nn + (double)idxMax0;
          for (i3 = 0; i3 < 2; i3++) {
            H[((int)mn + 200 * i3) - 1] = Points_c[i3 + 3 * ((int)mn - 1)];
          }

          sumx += Points_c[3 * ((int)mn - 1)];
          sumx2 += Points_c[3 * ((int)mn - 1)] * Points_c[3 * ((int)mn - 1)];
          sumy += Points_c[1 + 3 * ((int)mn - 1)];
          sumy2 += Points_c[1 + 3 * ((int)mn - 1)] * Points_c[1 + 3 * ((int)mn -
            1)];
          sumxy += Points_c[1 + 3 * ((int)mn - 1)] * Points_c[3 * ((int)mn - 1)];
          Points_c[2 + 3 * ((int)mn - 1)] = 0.0;
        }

        mx = (sumx * sumx + sumy * sumy) / (nPoints[i] * nPoints[i]);
        b_sqrt(&mx);
        mn = (Opt_splitFit_v[0] * (mx * mx) + Opt_splitFit_v[1] * mx) +
          Opt_splitFit_v[2];
        mx = sumy2 * sumx2 - sumxy * sumxy;
        if (!(mx > 1.0E-6)) {
          mx = 1.0E-6;
        }

        mx = 1.0 / mx;
        HH_inv[0] = sumy2 * mx;
        HH_inv[2] = -sumxy * mx;
        HH_inv[1] = -sumxy * mx;
        HH_inv[3] = sumx2 * mx;
        b_sumx[0] = sumx;
        b_sumx[1] = sumy;
        for (i2 = 0; i2 < 2; i2++) {
          b_z[i2] = 0.0;
          for (i3 = 0; i3 < 2; i3++) {
            b_z[i2] += HH_inv[i2 + (i3 << 1)] * b_sumx[i3];
          }
        }

        mx = b_z[0] + b_z[1];
        srtAngs_i = mx * mx;
        mx = (1.0 - Opt_splitFit_alphaProx) * sensStd * mn;
        b_c = mx * mx;
        b_y = 0.0;
        for (i2 = 0; i2 < 2; i2++) {
          b_y += b_z[i2] * b_z[i2];
        }

        mx = sensStd * Opt_splitFit_alphaProx * mn;
        mx *= mx;
        calcRhoTta(b_z, *(double (*)[2])&rhoTta[i << 1], HH);
        b_z[0] = sin(rhoTta[1 + (i << 1)]);
        b_z[1] = -cos(rhoTta[1 + (i << 1)]);
        c2[0] = cos(rhoTta[1 + (i << 1)]);
        c2[1] = sin(rhoTta[1 + (i << 1)]);
        maxDiff1 = 0.0;
        for (i2 = 0; i2 < 2; i2++) {
          maxDiff1 += -H[((int)nn + 200 * i2) - 1] * b_z[i2];
        }

        tminmax[i << 1] = maxDiff1;
        tminmax[1 + (i << 1)] = tminmax[i << 1];
        i2 = (int)(segments[2 + 3 * ((int)nn - 1)] + (1.0 - nn));
        for (idxMax0 = 0; idxMax0 < i2; idxMax0++) {
          mn = nn + (double)idxMax0;
          maxDiff1 = 0.0;
          for (i3 = 0; i3 < 2; i3++) {
            maxDiff1 += H[((int)mn + 200 * i3) - 1] * c2[i3];
          }

          b_avgDist[i] += fabs(maxDiff1 - rhoTta[i << 1]);
          maxDiff1 = 0.0;
          for (i3 = 0; i3 < 2; i3++) {
            maxDiff1 += -H[((int)mn + 200 * i3) - 1] * b_z[i3];
          }

          if ((tminmax[i << 1] < maxDiff1) || rtIsNaN(maxDiff1)) {
          } else {
            tminmax[i << 1] = maxDiff1;
          }

          if ((tminmax[1 + (i << 1)] > maxDiff1) || rtIsNaN(maxDiff1)) {
          } else {
            tminmax[1 + (i << 1)] = maxDiff1;
          }
        }

        b_avgDist[i] /= nPoints[i];
        c[0] = srtAngs_i * (sumx * sumx) * b_c * Opt_SF_mm2m;
        c[2] = srtAngs_i * (sumx * sumy) * b_c * Opt_SF_mm2m;
        c[1] = srtAngs_i * (sumx * sumy) * b_c * Opt_SF_mm2m;
        c[3] = srtAngs_i * (sumy * sumy) * b_c * Opt_SF_mm2m;
        c_y[0] = b_y * sumx2 * mx * Opt_SF_mm2m;
        c_y[2] = b_y * sumxy * mx * Opt_SF_mm2m;
        c_y[1] = b_y * sumxy * mx * Opt_SF_mm2m;
        c_y[3] = b_y * sumy2 * mx * Opt_SF_mm2m;
        for (i2 = 0; i2 < 2; i2++) {
          for (i3 = 0; i3 < 2; i3++) {
            covRhoTta[(i3 + (i2 << 1)) + (i << 2)] = 0.0;
            b_HH[i2 + (i3 << 1)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              b_HH[i2 + (i3 << 1)] += HH[i2 + (idxMax0 << 1)] * HH_inv[idxMax0 +
                (i3 << 1)];
            }

            c_c[i3 + (i2 << 1)] = c[i3 + (i2 << 1)] + c_y[i3 + (i2 << 1)];
          }
        }

        for (i2 = 0; i2 < 2; i2++) {
          for (i3 = 0; i3 < 2; i3++) {
            c[i2 + (i3 << 1)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              c[i2 + (i3 << 1)] += b_HH[i2 + (idxMax0 << 1)] * c_c[idxMax0 + (i3
                << 1)];
            }
          }

          for (i3 = 0; i3 < 2; i3++) {
            c_y[i2 + (i3 << 1)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              c_y[i2 + (i3 << 1)] += c[i2 + (idxMax0 << 1)] * HH_inv[idxMax0 +
                (i3 << 1)];
            }
          }

          for (i3 = 0; i3 < 2; i3++) {
            covRhoTta[(i2 + (i3 << 1)) + (i << 2)] = 0.0;
            for (idxMax0 = 0; idxMax0 < 2; idxMax0++) {
              covRhoTta[(i2 + (i3 << 1)) + (i << 2)] += c_y[i2 + (idxMax0 << 1)]
                * HH[i3 + (idxMax0 << 1)];
            }
          }
        }

        /*      if  isempty(coder.target) */
        /*          if showGraphs */
        /*              plot(Points(1,cur_seg_idxs),Points(2,cur_seg_idxs),'o','markersize',5,'linewidth',2,'MarkerFaceColor',0.5+0.5*[0.9 0.9 0.6]) */
        /*              plot(segmentsMat([1 4],i),segmentsMat([2 5],i),'k') */
        /*              text_xy = mean([Points(1,cur_seg_idxs);Points(2,cur_seg_idxs)],2); */
        /*              text(text_xy(1),text_xy(2),['\rho = ' num2str(rho) ', \theta = ' num2str((180/pi)*tta)]); */
        /*          end */
        /*      end */
      }
    }

    /*  if showGraphsCov */
    /*      figure(1234); */
    /*      aa=get(get(1234,'CurrentAxes'),'children'); */
    /*      if ~isempty(aa) */
    /*          ind=aa(1).XData(end); */
    /*          if strcmp(aa(1).Tag,'ang') */
    /*              set(aa(1),'XData',[aa(1).XData (ind+1)*ones(1,length(dets))],'YData',[aa(1).YData dets']) */
    /*          else */
    /*              assert(false); */
    /*          end */
    /*      else */
    /*          hold all; */
    /*          plot(ones(length(dets)),dets,'Tag','ang'); */
    /*      end */
    /*       */
    /*      figure(12345); */
    /*      aa=get(get(12345,'CurrentAxes'),'children'); */
    /*      if ~isempty(aa) */
    /*          ind=aa(1).XData(end); */
    /*          if strcmp(aa(1).Tag,'detZ') */
    /*              set(aa(1),'XData',[aa(1).XData (ind+1)*ones(1,length(dets))],'YData',[aa(1).YData detsRhoPhi']) */
    /*          else */
    /*              assert(false); */
    /*          end */
    /*      else */
    /*          hold all; */
    /*          plot(ones(length(dets)),detsRhoPhi,'Tag','detZ');legend('det([a;b])', 'det([rho;phi])') */
    /*      end */
    /*       */
    /*  end */
  }
}

/*
 * File trailer for batchSplitFit.c
 *
 * [EOF]
 */
