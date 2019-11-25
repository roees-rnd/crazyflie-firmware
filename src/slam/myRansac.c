/*
 * File: myRansac.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 25-Nov-2019 10:24:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "cf_feat_ext.h"
#include "init.h"
#include "myRansac.h"

/* Variable Definitions */
static unsigned char rand_couple[62];

/* Function Definitions */

/*
 * Arguments    : const int points[400]
 *                double maxDistance
 *                double sampleSize
 *                float modelRANSAC[3]
 *                boolean_T isInliers[200]
 * Return Type  : void
 */
void mtlb_myRansac(const int points[400], double maxDistance, double sampleSize,
                   float modelRANSAC[3], boolean_T isInliers[200])
{
  int i;
  double tmp;
  boolean_T flag;
  unsigned char i0;
  unsigned char i1;
  float sumx;
  float sumx2;
  float sumy;
  float sumy2;
  int b_i;
  float sumxy;
  int sample[4];
  float det_HH;
  float b_sumy2[4];
  float b_sumx[2];
  boolean_T singular;
  float c_sumy2[2];
  int b_i0;
  float theta0[3];
  double nInliers;
  boolean_T x[200];
  double dist[200];

  /*  epsilon:      estimated fraction of outliers [0,1] */
  for (i = 0; i < 3; i++) {
    modelRANSAC[i] = 0.0F;
  }

  /*  if nargin<3 */
  /*      P = 0.99; */
  /*      sampleSize = ceil(log(1-P)/log(1-(1-epsilon)^3)); */
  /*  end */
  tmp = 0.0;
  flag = false;
  memset(&isInliers[0], 0, 200U * sizeof(boolean_T));
  for (i = 0; i < (int)sampleSize; i++) {
    /*  RANSAC loop */
    i0 = rand_couple[i << 1];
    i1 = rand_couple[1 + (i << 1)];
    while (i0 > 200) {
      i0 = (unsigned char)(i0 - 200);
    }

    while (i1 > 200) {
      i1 = (unsigned char)(i1 - 200);
    }

    for (b_i = 0; b_i < 2; b_i++) {
      sample[b_i << 1] = points[(i0 + 200 * b_i) - 1];
      sample[1 + (b_i << 1)] = points[(i1 + 200 * b_i) - 1];
    }

    /*  line est */
    /*  ax + by = 1 */
    /*  a=cos(phi)/rho, b=sin(phi)/rho */
    sumx = 0.0F;
    sumx2 = 0.0F;
    sumy = 0.0F;
    sumy2 = 0.0F;
    sumxy = 0.0F;
    for (b_i = 0; b_i < 2; b_i++) {
      sumx += (float)sample[b_i];
      sumx2 += (float)sample[b_i] * (float)sample[b_i];
      sumy += (float)sample[2 + b_i];
      sumy2 += (float)sample[2 + b_i] * (float)sample[2 + b_i];
      sumxy += (float)sample[2 + b_i] * (float)sample[b_i];
    }

    /* HH = [sumx2 sumxy; sumxy sumy2]; */
    det_HH = sumy2 * sumx2 - sumxy * sumxy;
    if (det_HH < 1.0E-6) {
      /*  line with rho close to zero */
      singular = true;
      det_HH = 1.0E+6F;
    } else {
      singular = false;
      det_HH = 1.0F / det_HH;
    }

    b_sumy2[0] = sumy2 * det_HH;
    b_sumy2[2] = -sumxy * det_HH;
    b_sumy2[1] = -sumxy * det_HH;
    b_sumy2[3] = sumx2 * det_HH;
    b_sumx[0] = sumx;
    b_sumx[1] = sumy;
    theta0[2] = 1.0F;
    det_HH = 0.0F;
    for (b_i = 0; b_i < 2; b_i++) {
      c_sumy2[b_i] = 0.0F;
      for (b_i0 = 0; b_i0 < 2; b_i0++) {
        c_sumy2[b_i] += b_sumy2[b_i + (b_i0 << 1)] * b_sumx[b_i0];
      }

      det_HH += c_sumy2[b_i] * c_sumy2[b_i];
      theta0[b_i] = c_sumy2[b_i];
    }

    det_HH = (float)sqrt(det_HH);
    for (b_i = 0; b_i < 3; b_i++) {
      theta0[b_i] /= det_HH;
    }

    if (!singular) {
      for (b_i = 0; b_i < 200; b_i++) {
        nInliers = (float)fabs((theta0[0] * (float)points[b_i] + theta0[1] *
          (float)points[200 + b_i]) - theta0[2]);
        x[b_i] = (nInliers < maxDistance);
        dist[b_i] = nInliers;
      }

      nInliers = x[0];
      for (b_i = 0; b_i < 199; b_i++) {
        nInliers += (double)x[b_i + 1];
      }

      if ((nInliers > 30.000001907348633) && (nInliers > tmp)) {
        /*  number of Inliers threshold */
        /*  take sample with biggest number of inliers */
        for (b_i = 0; b_i < 200; b_i++) {
          isInliers[b_i] = (dist[b_i] < maxDistance);
        }

        tmp = nInliers;
        flag = true;
      }
    }
  }

  if (flag) {
    /*  line est */
    /*  ax + by = 1 */
    /*  a=cos(phi)/rho, b=sin(phi)/rho */
    sumx = 0.0F;
    sumx2 = 0.0F;
    sumy = 0.0F;
    sumy2 = 0.0F;
    sumxy = 0.0F;
    for (b_i = 0; b_i < 200; b_i++) {
      if (isInliers[b_i]) {
        sumx += (float)points[b_i];
        sumx2 += (float)points[b_i] * (float)points[b_i];
        sumy += (float)points[200 + b_i];
        sumy2 += (float)points[200 + b_i] * (float)points[200 + b_i];
        sumxy += (float)points[200 + b_i] * (float)points[b_i];
      }
    }

    /* HH = [sumx2 sumxy; sumxy sumy2]; */
    det_HH = sumy2 * sumx2 - sumxy * sumxy;
    if (det_HH < 1.0E-6) {
      /*  line with rho close to zero */
      det_HH = 1.0E+6F;
    } else {
      det_HH = 1.0F / det_HH;
    }

    b_sumy2[0] = sumy2 * det_HH;
    b_sumy2[2] = -sumxy * det_HH;
    b_sumy2[1] = -sumxy * det_HH;
    b_sumy2[3] = sumx2 * det_HH;
    b_sumx[0] = sumx;
    b_sumx[1] = sumy;
    modelRANSAC[2] = 1.0F;
    det_HH = 0.0F;
    for (b_i = 0; b_i < 2; b_i++) {
      c_sumy2[b_i] = 0.0F;
      for (b_i0 = 0; b_i0 < 2; b_i0++) {
        c_sumy2[b_i] += b_sumy2[b_i + (b_i0 << 1)] * b_sumx[b_i0];
      }

      det_HH += c_sumy2[b_i] * c_sumy2[b_i];
      modelRANSAC[b_i] = c_sumy2[b_i];
    }

    det_HH = (float)sqrt(det_HH);
    for (b_i = 0; b_i < 3; b_i++) {
      modelRANSAC[b_i] /= det_HH;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void mtlb_myRansac_init(void)
{
  int i1;
  static const unsigned char uv0[62] = { 105U, 59U, 115U, 19U, 191U, 82U, 178U,
    7U, 165U, 132U, 186U, 156U, 94U, 178U, 169U, 25U, 50U, 64U, 113U, 90U, 198U,
    125U, 38U, 196U, 46U, 24U, 182U, 69U, 9U, 33U, 172U, 73U, 23U, 62U, 157U,
    139U, 8U, 96U, 72U, 165U, 138U, 174U, 16U, 134U, 31U, 146U, 166U, 152U, 179U,
    56U, 76U, 149U, 75U, 11U, 123U, 45U, 187U, 160U, 12U, 91U, 131U, 83U };

  for (i1 = 0; i1 < 62; i1++) {
    rand_couple[i1] = uv0[i1];
  }
}

/*
 * File trailer for myRansac.c
 *
 * [EOF]
 */
