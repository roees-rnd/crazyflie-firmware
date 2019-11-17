/*
 * File: interp1.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/interp1.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Definitions */

/*
 * Arguments    : const double varargin_1[20]
 *                const double varargin_2[20]
 *                const double varargin_3[64]
 *                double Vq[64]
 * Return Type  : void
 */
void interp1(const double varargin_1[20], const double varargin_2[20], const
             double varargin_3[64], double Vq[64])
{
  double b_y[20];
  double b_x[20];
  int k;
  int exitg1;
  double xtmp;
  double maxx;
  int low_i;
  int low_ip1;
  int high_i;
  double b_r;
  int mid_i;
  memcpy(&b_y[0], &varargin_2[0], 20U * sizeof(double));
  memcpy(&b_x[0], &varargin_1[0], 20U * sizeof(double));
  k = 1;
  do {
    exitg1 = 0;
    if (k < 21) {
      if (rtIsNaN(varargin_1[k - 1])) {
        exitg1 = 1;
      } else {
        k++;
      }
    } else {
      if (varargin_1[1] < varargin_1[0]) {
        for (k = 0; k < 10; k++) {
          xtmp = b_x[k];
          b_x[k] = b_x[19 - k];
          b_x[19 - k] = xtmp;
          xtmp = b_y[k];
          b_y[k] = b_y[19 - k];
          b_y[19 - k] = xtmp;
        }
      }

      xtmp = b_x[0];
      maxx = b_x[19];
      for (k = 0; k < 64; k++) {
        if (rtIsNaN(varargin_3[k])) {
          Vq[k] = rtNaN;
        } else if (varargin_3[k] > maxx) {
          Vq[k] = b_y[19] + (varargin_3[k] - maxx) / (maxx - b_x[18]) * (b_y[19]
            - b_y[18]);
        } else if (varargin_3[k] < xtmp) {
          Vq[k] = b_y[0] + (varargin_3[k] - xtmp) / (b_x[1] - xtmp) * (b_y[1] -
            b_y[0]);
        } else {
          low_i = 1;
          low_ip1 = 2;
          high_i = 20;
          while (high_i > low_ip1) {
            mid_i = (low_i + high_i) >> 1;
            if (varargin_3[k] >= b_x[mid_i - 1]) {
              low_i = mid_i;
              low_ip1 = mid_i + 1;
            } else {
              high_i = mid_i;
            }
          }

          b_r = (varargin_3[k] - b_x[low_i - 1]) / (b_x[low_i] - b_x[low_i - 1]);
          if (b_r == 0.0) {
            Vq[k] = b_y[low_i - 1];
          } else if (b_r == 1.0) {
            Vq[k] = b_y[low_i];
          } else if (b_y[low_i - 1] == b_y[low_i]) {
            Vq[k] = b_y[low_i - 1];
          } else {
            Vq[k] = (1.0 - b_r) * b_y[low_i - 1] + b_r * b_y[low_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/*
 * File trailer for interp1.c
 *
 * [EOF]
 */
