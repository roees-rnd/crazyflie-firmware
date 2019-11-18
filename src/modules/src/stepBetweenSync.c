/*
 * File: stepBetweenSync.c
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

/* Variable Definitions */
static double wantedSyncTime_T0_prev;
static boolean_T c_wantedSyncTime_T0_prev_not_em;
static double i_end;
static double i_tmp;
static double lastInd[16];
static boolean_T doneSearch[16];

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void d_wantedSyncTime_T0_prev_not_em(void)
{
  c_wantedSyncTime_T0_prev_not_em = false;
}

/*
 * Arguments    : const proxBuff_type proxBuff[16]
 *                const double highFreqTimeGrid[64]
 *                double wantedSyncTime_T0
 *                double wantedSyncTime_T1
 *                const double xUpSamp_f[64]
 *                const double yUpSamp_f[64]
 *                const double attUpSamp_f[64]
 *                const OptSync_type *OptSync
 *                boolean_T *done
 *                double *RobTT
 *                double RobXYT[3]
 *                double RobCov[9]
 *                Prox_type *prox
 * Return Type  : void
 */
void stepBetweenSync(const proxBuff_type proxBuff[16], const double
                     highFreqTimeGrid[64], double wantedSyncTime_T0, double
                     wantedSyncTime_T1, const double xUpSamp_f[64], const double
                     yUpSamp_f[64], const double attUpSamp_f[64], const
                     OptSync_type *OptSync, boolean_T *done, double *RobTT,
                     double RobXYT[3], double RobCov[9], Prox_type *prox)
{
  boolean_T firstRun;
  int ii;
  boolean_T guard1 = false;
  double i_strt;
  int i;
  boolean_T exitg1;
  double timeBin_idx_0;
  int ind;
  signed char found[16];
  double indTmp;
  firstRun = false;
  *done = false;
  if (!c_wantedSyncTime_T0_prev_not_em) {
    firstRun = true;
    c_wantedSyncTime_T0_prev_not_em = true;
  }

  memset(&prox->val[0], 0, sizeof(double) << 4);
  memset(&prox->TT[0], 0, sizeof(double) << 4);
  *RobTT = 0.0;
  for (ii = 0; ii < 3; ii++) {
    RobXYT[ii] = 0.0;
  }

  memset(&RobCov[0], 0, 9U * sizeof(double));
  guard1 = false;
  if (firstRun || (wantedSyncTime_T0_prev < wantedSyncTime_T0)) {
    wantedSyncTime_T0_prev = wantedSyncTime_T0;
    i_strt = -1.0;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)OptSync->timeGridLen - 1)) {
      if ((highFreqTimeGrid[i] >= wantedSyncTime_T0) > 0.5) {
        i_strt = 1.0 + (double)i;
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (i_strt == -1.0) {
      *done = true;
    } else {
      ind = -1;
      ii = (int)((1.0 + (-1.0 - OptSync->timeGridLen)) / -1.0);
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i <= ii - 1)) {
        timeBin_idx_0 = OptSync->timeGridLen + -(double)i;
        if ((highFreqTimeGrid[(int)timeBin_idx_0 - 1] < wantedSyncTime_T1) > 0.5)
        {
          ind = (int)timeBin_idx_0;
          exitg1 = true;
        } else {
          i++;
        }
      }

      i_end = ind;
      i_tmp = i_strt;
      memset(&lastInd[0], 0, sizeof(double) << 4);
      for (ii = 0; ii < 16; ii++) {
        doneSearch[ii] = false;
      }

      for (i = 0; i < (int)OptSync->numSen; i++) {
        lastInd[i] = proxBuff[i].oldInd;
      }

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    if (i_tmp > i_end) {
      *done = true;
    } else {
      timeBin_idx_0 = highFreqTimeGrid[(int)i_tmp - 1] + -OptSync->fastTimeStep /
        2.0;
      i_strt = highFreqTimeGrid[(int)i_tmp - 1] + OptSync->fastTimeStep / 2.0;
      for (i = 0; i < 16; i++) {
        found[i] = 0;
      }

      /*  For each sensor, find if there is a range meas inside timeBin: */
      for (ii = 0; ii < (int)OptSync->numSen; ii++) {
        indTmp = lastInd[ii];
        if (!(proxBuff[ii].numel == 0.0)) {
          exitg1 = false;
          while ((!exitg1) && (!doneSearch[ii])) {
            if (proxBuff[ii].Mat[(int)indTmp + 49] < timeBin_idx_0) {
              indTmp++;
              if (indTmp > OptSync->BufLen) {
                indTmp = 1.0;
              }

              if (indTmp == proxBuff[ii].newInd) {
                doneSearch[ii] = true;
              }
            } else {
              if (proxBuff[ii].Mat[(int)indTmp + 49] >= i_strt) {
                found[ii] = 0;

                /*  didnt find range meas for this timeBin (maybe next timeBin) */
              } else {
                found[ii] = (signed char)indTmp;
                lastInd[ii] = indTmp + 1.0;
                if (lastInd[ii] > OptSync->BufLen) {
                  lastInd[ii] = 1.0;
                }
              }

              exitg1 = true;
            }
          }
        }
      }

      for (i = 0; i < (int)OptSync->numSen; i++) {
        if (found[i] != 0) {
          prox->val[i] = proxBuff[i].Mat[found[i] - 1];
          prox->TT[i] = proxBuff[i].Mat[found[i] + 49];
        }
      }

      *RobTT = highFreqTimeGrid[(int)i_tmp - 1];
      RobXYT[0] = xUpSamp_f[(int)i_tmp - 1];
      RobXYT[1] = yUpSamp_f[(int)i_tmp - 1];
      RobXYT[2] = attUpSamp_f[(int)i_tmp - 1];
      i_tmp++;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void stepBetweenSync_init(void)
{
  int i18;
  wantedSyncTime_T0_prev = 0.0;
  i_tmp = 0.0;
  i_end = 0.0;
  memset(&lastInd[0], 0, sizeof(double) << 4);
  for (i18 = 0; i18 < 16; i18++) {
    doneSearch[i18] = false;
  }
}

/*
 * File trailer for stepBetweenSync.c
 *
 * [EOF]
 */
