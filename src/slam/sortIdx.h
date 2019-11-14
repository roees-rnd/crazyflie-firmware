/*
 * File: sortIdx.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef SORTIDX_H
#define SORTIDX_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "CyclicBuffer_addPos_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void b_merge_block(int idx[200], double b_x[200], int offset, int n,
    int preSortLevel, int iwork[200], double xwork[200]);
  extern void merge_block(int idx[200], double b_x[200], int offset, int n, int
    preSortLevel, int iwork[200], double xwork[200]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for sortIdx.h
 *
 * [EOF]
 */
