/*
 * File: processSlamInput.c
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
#include "processSlamInput.h"

/* Variable Definitions */
static double prevRobXYT[3];
static boolean_T prevRobXYT_not_empty;
static double prev_TT[16];
static double dPosFault;

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void prevRobXYT_not_empty_init(void)
{
  prevRobXYT_not_empty = false;
}

/*
 * Arguments    : double NavTT
 *                double NavXYT[3]
 *                double prox_val[16]
 *                const double prox_TT[16]
 *                double MAX_RANGE
 *                double MIN_RANGE
 *                double Opt_SF_mm2m
 *                double Rob_con_u[3]
 *                double rawRangeMat[16]
 *                double *dPosFault_out
 * Return Type  : void
 */
void processSlamInput(double NavTT, double NavXYT[3], double prox_val[16], const
                      double prox_TT[16], double MAX_RANGE, double MIN_RANGE,
                      double Opt_SF_mm2m, double Rob_con_u[3], double
                      rawRangeMat[16], double *dPosFault_out)
{
  int ii;
  double b_x;
  double dv21[4];
  double b_NavXYT[2];
  double dv22[2];
  int i20;

  /*  NavXYT contains [dx,dy=0,dt ] in robot coordinate system */
  NavXYT[0] *= Opt_SF_mm2m;
  NavXYT[1] *= Opt_SF_mm2m;

  /*  -1* this is due to opposite z direction in LLP */
  /*  -1* this is due to opposite z direction in LLP */
  if (!prevRobXYT_not_empty) {
    for (ii = 0; ii < 3; ii++) {
      prevRobXYT[ii] = NavXYT[ii];
    }

    prevRobXYT_not_empty = true;
  }

  if (fabs(fabs(NavXYT[2] - prevRobXYT[2]) - 6.283) < 0.1) {
    b_x = NavXYT[2] - prevRobXYT[2];
    if (b_x < 0.0) {
      b_x = -1.0;
    } else if (b_x > 0.0) {
      b_x = 1.0;
    } else {
      if (b_x == 0.0) {
        b_x = 0.0;
      }
    }

    prevRobXYT[2] += b_x * 2.0 * 3.1415926535897931;
  }

  /* ( (abs(NavXYT(1)-prevRobXYT(1))<60) && (abs(NavXYT(2)-prevRobXYT(2))<60) && ( abs(NavXYT(3)-prevRobXYT(3))<0.1  || (abs(abs(NavXYT(3)-prevRobXYT(3))-6.283)<0.100) ) ); */
  /*  noJump = ( (sqrt((NavXYT(1:2)-prevRobXYT(1:2))*(NavXYT(1:2)-prevRobXYT(1:2))')<4) && ( abs(NavXYT(3)-prevRobXYT(3))<0.1  || (abs(abs(NavXYT(3)-prevRobXYT(3))-6.283)<0.100) ) ); */
  /*  transform back to previous state axis-system of primitive navigation */
  /*  solution: */
  /*  calculate delta (x,y) in previous state axis-system */
  /*  dxy = [NavXYT(1);NavXYT(2)]; */
  dv21[0] = cos(prevRobXYT[2]);
  dv21[2] = sin(prevRobXYT[2]);
  dv21[1] = -sin(prevRobXYT[2]);
  dv21[3] = cos(prevRobXYT[2]);
  b_NavXYT[0] = NavXYT[0] - prevRobXYT[0];
  b_NavXYT[1] = NavXYT[1] - prevRobXYT[1];
  for (ii = 0; ii < 2; ii++) {
    dv22[ii] = 0.0;
    for (i20 = 0; i20 < 2; i20++) {
      dv22[ii] += dv21[ii + (i20 << 1)] * b_NavXYT[i20];
    }

    Rob_con_u[ii] = dv22[ii];
  }

  Rob_con_u[2] = NavXYT[2] - prevRobXYT[2];

  /*  Rob_con_u=[dxy; NavXYT(3)]; */
  /*     %% fill rawRangeMat */
  /*  milisec */
  for (ii = 0; ii < 16; ii++) {
    b_x = prox_val[ii];
    rawRangeMat[ii] = -1.0;
    if ((prox_val[ii] >= MAX_RANGE) || (prox_val[ii] < MIN_RANGE) || (fabs
         (prev_TT[ii] - prox_TT[ii]) < 1.0)) {
      b_x = -1.0;
    }

    if (fabs(NavTT - prox_TT[ii]) < 10000.0) {
      rawRangeMat[ii] = b_x;
    }

    prox_val[ii] = b_x;
  }

  *dPosFault_out = dPosFault;
  memcpy(&prev_TT[0], &prox_TT[0], sizeof(double) << 4);
  for (ii = 0; ii < 3; ii++) {
    prevRobXYT[ii] = NavXYT[ii];
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void processSlamInput_init(void)
{
  memset(&prev_TT[0], 0, sizeof(double) << 4);
  dPosFault = 0.0;
}

/*
 * File trailer for processSlamInput.c
 *
 * [EOF]
 */
