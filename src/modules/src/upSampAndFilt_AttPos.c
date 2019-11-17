/*
 * File: upSampAndFilt_AttPos.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/upSampAndFilt_AttPos.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/filtfilt.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/interp1.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"

/* Variable Definitions */
static double roundedT1_prev;

/* Function Definitions */

/*
 * Arguments    : const psiBuff_type *psiBuff
 *                const xyBuff_type *xyBuff
 *                const OptSync_type *OptSync
 *                boolean_T *shortTimeStep
 *                double highFreqTimeGrid[64]
 *                double *wantedSyncTime_T0
 *                double *wantedSyncTime_T1
 *                double xUpSamp_f[64]
 *                double yUpSamp_f[64]
 *                double attUpSamp_f[64]
 * Return Type  : void
 */
void upSampAndFilt_AttPos(const psiBuff_type *psiBuff, const xyBuff_type *xyBuff,
  const OptSync_type *OptSync, boolean_T *shortTimeStep, double
  highFreqTimeGrid[64], double *wantedSyncTime_T0, double *wantedSyncTime_T1,
  double xUpSamp_f[64], double yUpSamp_f[64], double attUpSamp_f[64])
{
  double attTot[100];
  double kk;
  int i;
  int low_i;
  double OdomXY_Tot[60];
  double xUpSamp[64];
  double yUpSamp[64];
  double varargin_1[50];
  double varargin_2[50];
  int exitg1;
  double maxx;
  double attUpSamp[64];
  int low_ip1;
  int high_i;
  double b_r;
  int mid_i;

  /* ,'extern','HeaderFile','/home/roees/SLAM_Indoor/CNRS_EKF_SLAM_Toolbox_main/HighLevel/externHeaderFile.h'); % This needs to be corrected */
  *shortTimeStep = false;
  memset(&attTot[0], 0, 100U * sizeof(double));
  kk = psiBuff->oldInd;
  for (i = 0; i < (int)OptSync->BufLen; i++) {
    for (low_i = 0; low_i < 2; low_i++) {
      attTot[i + 50 * low_i] = psiBuff->Mat[((int)kk + 50 * low_i) - 1];
    }

    kk++;
    if (kk > OptSync->BufLen) {
      kk = 1.0;
    }
  }

  kk = xyBuff->newInd;
  for (i = 0; i < 20; i++) {
    for (low_i = 0; low_i < 3; low_i++) {
      OdomXY_Tot[(20 * low_i - i) + 19] = xyBuff->Mat[((int)kk + 50 * low_i) - 1];
    }

    kk--;
    if (kk < 1.0) {
      kk = OptSync->BufLen;
    }
  }

  kk = floor(OptSync->numOdomSmpl / 2.0);
  *wantedSyncTime_T0 = OdomXY_Tot[(int)kk + 39];
  *wantedSyncTime_T1 = OdomXY_Tot[40 + (int)kk];

  /*  Upsample attitude and position: */
  kk = floor(OdomXY_Tot[40 + (int)kk] * OptSync->fastTimeStepFreq) *
    OptSync->fastTimeStep;
  for (low_i = 0; low_i < 64; low_i++) {
    highFreqTimeGrid[low_i] = kk + OptSync->timeGrid[low_i];
  }

  if (roundedT1_prev == kk) {
    /*  Trash assertion (for coder purposes only) */
    /*  Trash assertion (for coder purposes only) */
    memcpy(&xUpSamp_f[0], &highFreqTimeGrid[0], sizeof(double) << 6);
    memcpy(&yUpSamp_f[0], &highFreqTimeGrid[0], sizeof(double) << 6);
    memcpy(&attUpSamp_f[0], &highFreqTimeGrid[0], sizeof(double) << 6);

    /*  Trash assertion (for coder purposes only) */
    *shortTimeStep = true;
  } else {
    roundedT1_prev = kk;
    interp1(*(double (*)[20])&OdomXY_Tot[40], *(double (*)[20])&OdomXY_Tot[0],
            highFreqTimeGrid, xUpSamp);
    interp1(*(double (*)[20])&OdomXY_Tot[40], *(double (*)[20])&OdomXY_Tot[20],
            highFreqTimeGrid, yUpSamp);
    memcpy(&varargin_1[0], &attTot[50], 50U * sizeof(double));
    memcpy(&varargin_2[0], &attTot[0], 50U * sizeof(double));
    i = 1;
    do {
      exitg1 = 0;
      if (i < 51) {
        if (rtIsNaN(attTot[i + 49])) {
          exitg1 = 1;
        } else {
          i++;
        }
      } else {
        if (attTot[51] < attTot[50]) {
          for (i = 0; i < 25; i++) {
            kk = varargin_1[i];
            varargin_1[i] = varargin_1[49 - i];
            varargin_1[49 - i] = kk;
            kk = varargin_2[i];
            varargin_2[i] = varargin_2[49 - i];
            varargin_2[49 - i] = kk;
          }
        }

        kk = varargin_1[0];
        maxx = varargin_1[49];
        for (i = 0; i < 64; i++) {
          if (rtIsNaN(highFreqTimeGrid[i])) {
            attUpSamp[i] = rtNaN;
          } else if (highFreqTimeGrid[i] > maxx) {
            attUpSamp[i] = varargin_2[49] + (highFreqTimeGrid[i] - maxx) / (maxx
              - varargin_1[48]) * (varargin_2[49] - varargin_2[48]);
          } else if (highFreqTimeGrid[i] < kk) {
            attUpSamp[i] = varargin_2[0] + (highFreqTimeGrid[i] - kk) /
              (varargin_1[1] - kk) * (varargin_2[1] - varargin_2[0]);
          } else {
            low_i = 1;
            low_ip1 = 2;
            high_i = 50;
            while (high_i > low_ip1) {
              mid_i = (low_i + high_i) >> 1;
              if (highFreqTimeGrid[i] >= varargin_1[mid_i - 1]) {
                low_i = mid_i;
                low_ip1 = mid_i + 1;
              } else {
                high_i = mid_i;
              }
            }

            b_r = (highFreqTimeGrid[i] - varargin_1[low_i - 1]) /
              (varargin_1[low_i] - varargin_1[low_i - 1]);
            if (b_r == 0.0) {
              attUpSamp[i] = varargin_2[low_i - 1];
            } else if (b_r == 1.0) {
              attUpSamp[i] = varargin_2[low_i];
            } else if (varargin_2[low_i - 1] == varargin_2[low_i]) {
              attUpSamp[i] = varargin_2[low_i - 1];
            } else {
              attUpSamp[i] = (1.0 - b_r) * varargin_2[low_i - 1] + b_r *
                varargin_2[low_i];
            }
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /*  Filter attitude and position: */
    filtfilt(xUpSamp, xUpSamp_f);
    filtfilt(yUpSamp, yUpSamp_f);
    filtfilt(attUpSamp, attUpSamp_f);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void upSampAndFilt_AttPos_init(void)
{
  roundedT1_prev = 0.0;
}

/*
 * File trailer for upSampAndFilt_AttPos.c
 *
 * [EOF]
 */
