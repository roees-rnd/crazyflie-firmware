/*
 * File: sort1.c
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
#include "sort1.h"
#include "sortIdx.h"

/* Function Declarations */
static void c_sort(double b_x[200], int idx[200]);

/* Function Definitions */

/*
 * Arguments    : double b_x[200]
 *                int idx[200]
 * Return Type  : void
 */
static void c_sort(double b_x[200], int idx[200])
{
  int i;
  double xwork[200];
  double x4[4];
  int nNaNs;
  unsigned char idx4[4];
  int ib;
  int k;
  signed char perm[4];
  int iwork[200];
  int i2;
  int i3;
  int i4;
  memset(&idx[0], 0, 200U * sizeof(int));
  for (i = 0; i < 4; i++) {
    x4[i] = 0.0;
    idx4[i] = 0;
  }

  memset(&xwork[0], 0, 200U * sizeof(double));
  nNaNs = 0;
  ib = 0;
  for (k = 0; k < 200; k++) {
    if (rtIsNaN(b_x[k])) {
      idx[199 - nNaNs] = k + 1;
      xwork[199 - nNaNs] = b_x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (unsigned char)(k + 1);
      x4[ib - 1] = b_x[k];
      if (ib == 4) {
        i = k - nNaNs;
        if (x4[0] >= x4[1]) {
          ib = 1;
          i2 = 2;
        } else {
          ib = 2;
          i2 = 1;
        }

        if (x4[2] >= x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        if (x4[ib - 1] >= x4[i3 - 1]) {
          if (x4[i2 - 1] >= x4[i3 - 1]) {
            perm[0] = (signed char)ib;
            perm[1] = (signed char)i2;
            perm[2] = (signed char)i3;
            perm[3] = (signed char)i4;
          } else if (x4[i2 - 1] >= x4[i4 - 1]) {
            perm[0] = (signed char)ib;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)i2;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)ib;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)i2;
          }
        } else if (x4[ib - 1] >= x4[i4 - 1]) {
          if (x4[i2 - 1] >= x4[i4 - 1]) {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)ib;
            perm[2] = (signed char)i2;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)ib;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)i2;
          }
        } else {
          perm[0] = (signed char)i3;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)ib;
          perm[3] = (signed char)i2;
        }

        idx[i - 3] = idx4[perm[0] - 1];
        idx[i - 2] = idx4[perm[1] - 1];
        idx[i - 1] = idx4[perm[2] - 1];
        idx[i] = idx4[perm[3] - 1];
        b_x[i - 3] = x4[perm[0] - 1];
        b_x[i - 2] = x4[perm[1] - 1];
        b_x[i - 1] = x4[perm[2] - 1];
        b_x[i] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    for (i = 0; i < 4; i++) {
      perm[i] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] >= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] >= x4[1]) {
      if (x4[1] >= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] >= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] >= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] >= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 200; k - 199 <= ib; k++) {
      idx[(k - nNaNs) - ib] = idx4[perm[k - 200] - 1];
      b_x[(k - nNaNs) - ib] = x4[perm[k - 200] - 1];
    }
  }

  i = nNaNs >> 1;
  for (k = 1; k <= i; k++) {
    ib = (unsigned char)idx[(k - nNaNs) + 199];
    idx[(k - nNaNs) + 199] = (unsigned char)idx[200 - k];
    idx[200 - k] = ib;
    b_x[(k - nNaNs) + 199] = xwork[200 - k];
    b_x[200 - k] = xwork[(k - nNaNs) + 199];
  }

  if ((nNaNs & 1) != 0) {
    b_x[(i - nNaNs) + 200] = xwork[(i - nNaNs) + 200];
  }

  memset(&iwork[0], 0, 200U * sizeof(int));
  if (200 - nNaNs > 1) {
    b_merge_block(idx, b_x, 0, 200 - nNaNs, 2, iwork, xwork);
  }

  if ((nNaNs > 0) && (200 - nNaNs > 0)) {
    for (k = 200; k - 199 <= nNaNs; k++) {
      xwork[k - 200] = b_x[k - nNaNs];
      iwork[k - 200] = idx[k - nNaNs];
    }

    for (k = 199 - nNaNs; k + 1 > 0; k--) {
      b_x[nNaNs + k] = b_x[k];
      idx[nNaNs + k] = idx[k];
    }

    for (k = 0; k + 1 <= nNaNs; k++) {
      b_x[k] = xwork[k];
      idx[k] = iwork[k];
    }
  }
}

/*
 * Arguments    : double b_x[200]
 *                int idx[200]
 * Return Type  : void
 */
void b_sort(double b_x[200], int idx[200])
{
  c_sort(b_x, idx);
}

/*
 * Arguments    : double b_x[200]
 *                int idx[200]
 * Return Type  : void
 */
void sort(double b_x[200], int idx[200])
{
  int i;
  double xwork[200];
  double x4[4];
  int nNaNs;
  unsigned char idx4[4];
  int ib;
  int k;
  signed char perm[4];
  int i2;
  int iwork[200];
  int i3;
  int i4;
  memset(&idx[0], 0, 200U * sizeof(int));
  for (i = 0; i < 4; i++) {
    x4[i] = 0.0;
    idx4[i] = 0;
  }

  memset(&xwork[0], 0, 200U * sizeof(double));
  nNaNs = -199;
  ib = 0;
  for (k = 0; k < 200; k++) {
    if (rtIsNaN(b_x[k])) {
      idx[-nNaNs] = k + 1;
      xwork[-nNaNs] = b_x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (unsigned char)(k + 1);
      x4[ib - 1] = b_x[k];
      if (ib == 4) {
        i = (k - nNaNs) - 202;
        if (x4[0] <= x4[1]) {
          ib = 1;
          i2 = 2;
        } else {
          ib = 2;
          i2 = 1;
        }

        if (x4[2] <= x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        if (x4[ib - 1] <= x4[i3 - 1]) {
          if (x4[i2 - 1] <= x4[i3 - 1]) {
            perm[0] = (signed char)ib;
            perm[1] = (signed char)i2;
            perm[2] = (signed char)i3;
            perm[3] = (signed char)i4;
          } else if (x4[i2 - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)ib;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)i2;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)ib;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)i2;
          }
        } else if (x4[ib - 1] <= x4[i4 - 1]) {
          if (x4[i2 - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)ib;
            perm[2] = (signed char)i2;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)ib;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)i2;
          }
        } else {
          perm[0] = (signed char)i3;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)ib;
          perm[3] = (signed char)i2;
        }

        idx[i] = idx4[perm[0] - 1];
        idx[i + 1] = idx4[perm[1] - 1];
        idx[i + 2] = idx4[perm[2] - 1];
        idx[i + 3] = idx4[perm[3] - 1];
        b_x[i] = x4[perm[0] - 1];
        b_x[i + 1] = x4[perm[1] - 1];
        b_x[i + 2] = x4[perm[2] - 1];
        b_x[i + 3] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    for (i = 0; i < 4; i++) {
      perm[i] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 1; k <= ib; k++) {
      idx[(k - nNaNs) - ib] = idx4[perm[k - 1] - 1];
      b_x[(k - nNaNs) - ib] = x4[perm[k - 1] - 1];
    }
  }

  i = (nNaNs + 199) >> 1;
  for (k = 1; k <= i; k++) {
    ib = (unsigned char)idx[k - nNaNs];
    idx[k - nNaNs] = (unsigned char)idx[200 - k];
    idx[200 - k] = ib;
    b_x[k - nNaNs] = xwork[200 - k];
    b_x[200 - k] = xwork[k - nNaNs];
  }

  if (((nNaNs + 199) & 1) != 0) {
    b_x[(i - nNaNs) + 1] = xwork[(i - nNaNs) + 1];
  }

  if (1 - nNaNs > 1) {
    memset(&iwork[0], 0, 200U * sizeof(int));
    merge_block(idx, b_x, 0, 1 - nNaNs, 2, iwork, xwork);
  }
}

/*
 * File trailer for sort1.c
 *
 * [EOF]
 */
