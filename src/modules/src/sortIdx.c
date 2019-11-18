/*
 * File: sortIdx.c
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
#include "sortIdx.h"

/* Function Declarations */
static void b_merge(int idx[200], double b_x[200], int offset, int np, int nq,
                    int iwork[200], double xwork[200]);
static void merge(int idx[200], double b_x[200], int offset, int np, int nq, int
                  iwork[200], double xwork[200]);

/* Function Definitions */

/*
 * Arguments    : int idx[200]
 *                double b_x[200]
 *                int offset
 *                int np
 *                int nq
 *                int iwork[200]
 *                double xwork[200]
 * Return Type  : void
 */
static void b_merge(int idx[200], double b_x[200], int offset, int np, int nq,
                    int iwork[200], double xwork[200])
{
  int n;
  int qend;
  int p;
  int iout;
  int exitg1;
  if ((np == 0) || (nq == 0)) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork[qend] = idx[offset + qend];
      xwork[qend] = b_x[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] >= xwork[n]) {
        idx[iout] = iwork[p];
        b_x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[n];
        b_x[iout] = xwork[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = iout - p;
          while (p + 1 <= np) {
            idx[(n + p) + 1] = iwork[p];
            b_x[(n + p) + 1] = xwork[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : int idx[200]
 *                double b_x[200]
 *                int offset
 *                int np
 *                int nq
 *                int iwork[200]
 *                double xwork[200]
 * Return Type  : void
 */
static void merge(int idx[200], double b_x[200], int offset, int np, int nq, int
                  iwork[200], double xwork[200])
{
  int n;
  int qend;
  int p;
  int iout;
  int exitg1;
  if ((np == 0) || (nq == 0)) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork[qend] = idx[offset + qend];
      xwork[qend] = b_x[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[n]) {
        idx[iout] = iwork[p];
        b_x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[n];
        b_x[iout] = xwork[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = iout - p;
          while (p + 1 <= np) {
            idx[(n + p) + 1] = iwork[p];
            b_x[(n + p) + 1] = xwork[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : int idx[200]
 *                double b_x[200]
 *                int offset
 *                int n
 *                int preSortLevel
 *                int iwork[200]
 *                double xwork[200]
 * Return Type  : void
 */
void b_merge_block(int idx[200], double b_x[200], int offset, int n, int
                   preSortLevel, int iwork[200], double xwork[200])
{
  int nPairs;
  int bLen;
  int tailOffset;
  int nTail;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        b_merge(idx, b_x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      b_merge(idx, b_x, offset + (nTail - 1) * tailOffset, bLen, bLen, iwork,
              xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    b_merge(idx, b_x, offset, bLen, n - bLen, iwork, xwork);
  }
}

/*
 * Arguments    : int idx[200]
 *                double b_x[200]
 *                int offset
 *                int n
 *                int preSortLevel
 *                int iwork[200]
 *                double xwork[200]
 * Return Type  : void
 */
void merge_block(int idx[200], double b_x[200], int offset, int n, int
                 preSortLevel, int iwork[200], double xwork[200])
{
  int nPairs;
  int bLen;
  int tailOffset;
  int nTail;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, b_x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      merge(idx, b_x, offset + (nTail - 1) * tailOffset, bLen, bLen, iwork,
            xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    merge(idx, b_x, offset, bLen, n - bLen, iwork, xwork);
  }
}

/*
 * File trailer for sortIdx.c
 *
 * [EOF]
 */
