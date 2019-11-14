/*
 * File: getMetry.c
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
#include "getMetry.h"
#include "CyclicBuffer_addPos_rtwutil.h"

/* Variable Definitions */
static double indLmk;

/* Function Definitions */

/*
 * Arguments    : double Map_numUsed
 *                const double xyRob[400]
 *                const Lmk_type Lmk[15]
 *                const Raw_type Raw[20]
 *                double num_segments
 *                double pnum
 *                double b_dPosFault
 *                double b_oneOfTenCyc
 *                struct1_T *Metry
 * Return Type  : void
 */
void getMetry(double Map_numUsed, const double xyRob[400], const Lmk_type Lmk[15],
              const Raw_type Raw[20], double num_segments, double pnum, double
              b_dPosFault, double b_oneOfTenCyc, struct1_T *Metry)
{
  int ix;
  int i;
  double b_y[400];
  int ixstart;
  double mtmp;
  double maxval[2];
  boolean_T exitg1;
  int b_ix;
  double d2;
  unsigned short u0;
  short i16;
  signed char i17;
  if (indLmk >= Map_numUsed) {
    indLmk = 1.0;
  } else {
    indLmk++;
  }

  for (ix = 0; ix < 400; ix++) {
    b_y[ix] = fabs(xyRob[ix]);
  }

  for (i = 0; i < 2; i++) {
    ix = i * 200;
    ixstart = i * 200 + 1;
    mtmp = b_y[ix];
    if (rtIsNaN(b_y[ix])) {
      b_ix = ixstart;
      exitg1 = false;
      while ((!exitg1) && (b_ix + 1 <= ix + 200)) {
        ixstart = b_ix + 1;
        if (!rtIsNaN(b_y[b_ix])) {
          mtmp = b_y[b_ix];
          exitg1 = true;
        } else {
          b_ix++;
        }
      }
    }

    if (ixstart < ix + 200) {
      while (ixstart + 1 <= ix + 200) {
        if (b_y[ixstart] > mtmp) {
          mtmp = b_y[ixstart];
        }

        ixstart++;
      }
    }

    maxval[i] = mtmp;
  }

  ixstart = 1;
  mtmp = maxval[0];
  if (rtIsNaN(maxval[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 3)) {
      ixstart = 2;
      if (!rtIsNaN(maxval[1])) {
        mtmp = maxval[1];
        exitg1 = true;
      } else {
        ix = 3;
      }
    }
  }

  if ((ixstart < 2) && (maxval[1] > mtmp)) {
    mtmp = maxval[1];
  }

  Metry->Raw_faultyMeas = (mtmp > 10000.0);
  if (pnum > 1.0) {
    mtmp = pnum;
  } else {
    mtmp = 1.0;
  }

  for (ix = 0; ix < 2; ix++) {
    d2 = rt_roundd_snf(xyRob[200 * ix]);
    if (d2 < 32768.0) {
      if (d2 >= -32768.0) {
        i16 = (short)d2;
      } else {
        i16 = MIN_int16_T;
      }
    } else if (d2 >= 32768.0) {
      i16 = MAX_int16_T;
    } else {
      i16 = 0;
    }

    Metry->Raw_xyRob_1[ix] = i16;
    d2 = rt_roundd_snf(xyRob[((int)mtmp + 200 * ix) - 1]);
    if (d2 < 32768.0) {
      if (d2 >= -32768.0) {
        i16 = (short)d2;
      } else {
        i16 = MIN_int16_T;
      }
    } else if (d2 >= 32768.0) {
      i16 = MAX_int16_T;
    } else {
      i16 = 0;
    }

    Metry->Raw_xyRob_pnum[ix] = i16;
  }

  d2 = rt_roundd_snf(num_segments);
  if (d2 < 65536.0) {
    if (d2 >= 0.0) {
      u0 = (unsigned short)d2;
    } else {
      u0 = 0;
    }
  } else if (d2 >= 65536.0) {
    u0 = MAX_uint16_T;
  } else {
    u0 = 0;
  }

  Metry->Raw_num_segments = u0;
  d2 = rt_roundd_snf(pnum);
  if (d2 < 128.0) {
    if (d2 >= -128.0) {
      i17 = (signed char)d2;
    } else {
      i17 = MIN_int8_T;
    }
  } else if (d2 >= 128.0) {
    i17 = MAX_int8_T;
  } else {
    i17 = 0;
  }

  Metry->Raw_pnum = i17;
  d2 = rt_roundd_snf(Map_numUsed);
  if (d2 < 65536.0) {
    if (d2 >= 0.0) {
      u0 = (unsigned short)d2;
    } else {
      u0 = 0;
    }
  } else if (d2 >= 65536.0) {
    u0 = MAX_uint16_T;
  } else {
    u0 = 0;
  }

  Metry->Lmk_numLmks = u0;
  Metry->Lmk_indLmk = indLmk;
  for (i = 0; i < 4; i++) {
    d2 = rt_roundd_snf(Lmk[(int)indLmk - 1].parEndp.e[i]);
    if (d2 < 2.147483648E+9) {
      if (d2 >= -2.147483648E+9) {
        ix = (int)d2;
      } else {
        ix = MIN_int32_T;
      }
    } else if (d2 >= 2.147483648E+9) {
      ix = MAX_int32_T;
    } else {
      ix = 0;
    }

    Metry->Lmk_ParEndp[i] = ix;
    d2 = rt_roundd_snf(Raw[0].endpWorld[i]);
    if (d2 < 2.147483648E+9) {
      if (d2 >= -2.147483648E+9) {
        ix = (int)d2;
      } else {
        ix = MIN_int32_T;
      }
    } else if (d2 >= 2.147483648E+9) {
      ix = MAX_int32_T;
    } else {
      ix = 0;
    }

    Metry->Raw_EndpW1[i] = ix;
    d2 = rt_roundd_snf(Raw[1].endpWorld[i]);
    if (d2 < 2.147483648E+9) {
      if (d2 >= -2.147483648E+9) {
        ix = (int)d2;
      } else {
        ix = MIN_int32_T;
      }
    } else if (d2 >= 2.147483648E+9) {
      ix = MAX_int32_T;
    } else {
      ix = 0;
    }

    Metry->Raw_EndpW2[i] = ix;
    d2 = rt_roundd_snf(Raw[2].endpWorld[i]);
    if (d2 < 2.147483648E+9) {
      if (d2 >= -2.147483648E+9) {
        ix = (int)d2;
      } else {
        ix = MIN_int32_T;
      }
    } else if (d2 >= 2.147483648E+9) {
      ix = MAX_int32_T;
    } else {
      ix = 0;
    }

    Metry->Raw_EndpW3[i] = ix;
  }

  Metry->slamVersion = 1911131524;
  Metry->dPosFault = b_dPosFault;
  Metry->updateCU = (b_oneOfTenCyc == 0.0);

  /*  update control unit */
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void getMetry_init(void)
{
  indLmk = 1.0;
}

/*
 * File trailer for getMetry.c
 *
 * [EOF]
 */
