/*
 * File: cf_feat_ext.c
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
static boolean_T firstRun_not_empty;
static int xyMeasWorld[400];
static unsigned short xyMeasLastInd;
static int xyPosWorld[30];
static unsigned short xyPosLastInd;
static int xyMeasWorldLastInputs[32];
static float sin_sen[16];
static float cos_sen[16];
static double freq_reduce;

/* Function Declarations */
static double mtlb_rt_remd_snf(double u0, double u1);
static float mtlb_rt_roundf_snf(float u);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double mtlb_rt_remd_snf(double u0, double u1)
{
  double y;
  double b_u1;
  double q;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    y = rtNaN;
  } else {
    if (u1 < 0.0) {
      b_u1 = ceil(u1);
    } else {
      b_u1 = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      q = fabs(u0 / u1);
      if (fabs(q - floor(q + 0.5)) <= DBL_EPSILON * q) {
        y = 0.0 * u0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : float u
 * Return Type  : float
 */
static float mtlb_rt_roundf_snf(float u)
{
  float y;
  if ((float)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (float)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (float)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : const int navXY[2]
 *                float Psi
 *                const unsigned short ranges[16]
 *                const Opt_type *Opt
 *                float modelRANSAC[3]
 * Return Type  : void
 */
void cf_feat_ext(const int navXY[2], float Psi, const unsigned short ranges[16],
                 const Opt_type *Opt, float modelRANSAC[3])
{
  int i;
  int jcol;
  float sin_rob;
  float cos_rob;
  int ibmat;
  int itilerow;
  float a;
  double x;
  float z1[2];
  unsigned int u0;
  double absx;
  boolean_T isInliers[200];
  signed char n;
  int xyMeasWorld_[2];

  /* t */
  if (!firstRun_not_empty) {
    firstRun_not_empty = true;
    for (jcol = 0; jcol < 32; jcol++) {
      xyMeasWorldLastInputs[jcol] = 30000;
    }

    memset(&xyMeasWorld[0], 0, 400U * sizeof(int));
    for (jcol = 0; jcol < 2; jcol++) {
      ibmat = jcol * 15;
      for (itilerow = 0; itilerow < 15; itilerow++) {
        xyPosWorld[ibmat + itilerow] = navXY[jcol];
      }
    }

    xyMeasLastInd = Opt->xyMeasLen;
    xyPosLastInd = Opt->xyPosLen;
    for (itilerow = 0; itilerow < 16; itilerow++) {
      x = 22.5 * (double)itilerow;
      x = mtlb_rt_remd_snf(x, 360.0);
      absx = x;
      if (x > 180.0) {
        if (x > 0.0) {
          x -= 360.0;
        } else {
          x += 360.0;
        }

        absx = fabs(x);
      }

      if (absx <= 45.0) {
        x *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (x > 0.0) {
          x = 0.017453292519943295 * (x - 90.0);
          n = 1;
        } else {
          x = 0.017453292519943295 * (x + 90.0);
          n = -1;
        }
      } else if (x > 0.0) {
        x = 0.017453292519943295 * (x - 180.0);
        n = 2;
      } else {
        x = 0.017453292519943295 * (x + 180.0);
        n = -2;
      }

      if (n == 0) {
        x = sin(x);
      } else if (n == 1) {
        x = cos(x);
      } else if (n == -1) {
        x = -cos(x);
      } else {
        x = -sin(x);
      }

      sin_sen[itilerow] = -(float)x;
      x = 22.5 * (double)itilerow;
      x = mtlb_rt_remd_snf(x, 360.0);
      absx = x;
      if (x > 180.0) {
        if (x > 0.0) {
          x -= 360.0;
        } else {
          x += 360.0;
        }

        absx = fabs(x);
      }

      if (absx <= 45.0) {
        x *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (x > 0.0) {
          x = 0.017453292519943295 * (x - 90.0);
          n = 1;
        } else {
          x = 0.017453292519943295 * (x + 90.0);
          n = -1;
        }
      } else if (x > 0.0) {
        x = 0.017453292519943295 * (x - 180.0);
        n = 2;
      } else {
        x = 0.017453292519943295 * (x + 180.0);
        n = -2;
      }

      if (n == 0) {
        x = cos(x);
      } else if (n == 1) {
        x = -sin(x);
      } else if (n == -1) {
        x = sin(x);
      } else {
        x = -cos(x);
      }

      cos_sen[itilerow] = -(float)x;
    }

    freq_reduce = 1.0;

    /* t = int32(zeros(coder.Constant(Opt.xyMeasLen),1)); */
  }

  for (i = 0; i < 3; i++) {
    modelRANSAC[i] = 0.0F;
  }

  sin_rob = (float)sin(Psi);
  cos_rob = (float)cos(Psi);

  /*  project range measurements to world: */
  for (i = 0; i < 16; i++) {
    if ((ranges[i] < 10) || (ranges[i] > 1200)) {
    } else {
      a = mtlb_rt_roundf_snf((float)ranges[i] * (cos_rob * cos_sen[i] - sin_rob *
        sin_sen[i]));
      if (a < 2.14748365E+9F) {
        if (a >= -2.14748365E+9F) {
          jcol = (int)a;
        } else {
          jcol = MIN_int32_T;
        }
      } else if (a >= 2.14748365E+9F) {
        jcol = MAX_int32_T;
      } else {
        jcol = 0;
      }

      ibmat = navXY[0];
      if ((jcol < 0) && (ibmat < MIN_int32_T - jcol)) {
        jcol = MIN_int32_T;
      } else if ((jcol > 0) && (ibmat > MAX_int32_T - jcol)) {
        jcol = MAX_int32_T;
      } else {
        jcol += ibmat;
      }

      xyMeasWorld_[0] = jcol;
      a = mtlb_rt_roundf_snf((float)ranges[i] * (cos_rob * sin_sen[i] +
        cos_sen[i] * sin_rob));
      if (a < 2.14748365E+9F) {
        if (a >= -2.14748365E+9F) {
          jcol = (int)a;
        } else {
          jcol = MIN_int32_T;
        }
      } else if (a >= 2.14748365E+9F) {
        jcol = MAX_int32_T;
      } else {
        jcol = 0;
      }

      ibmat = navXY[1];
      if ((jcol < 0) && (ibmat < MIN_int32_T - jcol)) {
        jcol = MIN_int32_T;
      } else if ((jcol > 0) && (ibmat > MAX_int32_T - jcol)) {
        jcol = MAX_int32_T;
      } else {
        jcol += ibmat;
      }

      xyMeasWorld_[1] = jcol;
      for (itilerow = 0; itilerow < 2; itilerow++) {
        jcol = xyMeasWorldLastInputs[i + (itilerow << 4)];
        ibmat = xyMeasWorld_[itilerow];
        if ((jcol >= 0) && (ibmat < jcol - MAX_int32_T)) {
          jcol = MAX_int32_T;
        } else if ((jcol < 0) && (ibmat > jcol - MIN_int32_T)) {
          jcol = MIN_int32_T;
        } else {
          jcol -= ibmat;
        }

        a = (float)jcol;
        z1[itilerow] = a * a;
      }

      if (z1[0] + z1[1] > Opt->measHistMinDist_p2) {
        u0 = xyMeasLastInd + 1U;
        if (u0 > 65535U) {
          u0 = 65535U;
        }

        if (xyMeasLastInd == Opt->xyMeasLen) {
          xyMeasLastInd = 1;
        } else {
          xyMeasLastInd = (unsigned short)u0;
        }

        for (jcol = 0; jcol < 2; jcol++) {
          xyMeasWorldLastInputs[i + (jcol << 4)] = xyMeasWorld_[jcol];
          xyMeasWorld[(xyMeasLastInd + 200 * jcol) - 1] = xyMeasWorld_[jcol];
        }
      }
    }
  }

  /*  If current position is too close to last saved position, then dont save current position */
  for (itilerow = 0; itilerow < 2; itilerow++) {
    jcol = xyPosWorld[(xyPosLastInd + 15 * itilerow) - 1];
    ibmat = navXY[itilerow];
    if ((jcol >= 0) && (ibmat < jcol - MAX_int32_T)) {
      jcol = MAX_int32_T;
    } else if ((jcol < 0) && (ibmat > jcol - MIN_int32_T)) {
      jcol = MIN_int32_T;
    } else {
      jcol -= ibmat;
    }

    a = (float)jcol;
    z1[itilerow] = a * a;
  }

  if (z1[0] + z1[1] > Opt->posHistMinStep_p2) {
    u0 = xyPosLastInd + 1U;
    if (u0 > 65535U) {
      u0 = 65535U;
    }

    if (xyPosLastInd == Opt->xyPosLen) {
      xyPosLastInd = 1;
    } else {
      xyPosLastInd = (unsigned short)u0;
    }

    for (jcol = 0; jcol < 2; jcol++) {
      xyPosWorld[(xyPosLastInd + 15 * jcol) - 1] = navXY[jcol];
    }
  }

  if (freq_reduce == Opt->nCyc_lowFreq) {
    freq_reduce = 0.0;
    mtlb_myRansac(xyMeasWorld, 40.0, 12.0, modelRANSAC, isInliers);

    /*      kk=uint8(0); */
    /*      for i=1:Opt.xyMeasLen */
    /*          if isInliers(i) */
    /*              kk = kk + 1; */
    /*              t(kk)=int32(single(xyMeasWorld(i,2))*modelRANSAC(1)-single(xyMeasWorld(i,1))*modelRANSAC(2)); */
    /*          end */
    /*      end */
  } else {
    freq_reduce++;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void mtlb_firstRun_not_empty_init(void)
{
  firstRun_not_empty = false;
}

/*
 * File trailer for cf_feat_ext.c
 *
 * [EOF]
 */
