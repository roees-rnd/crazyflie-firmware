/*
 * File: CyclicBuffer_addProx.c
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

/* Variable Definitions */
static double b_numThrows;

/* Function Definitions */

/*
 * Arguments    : proxBuff_type buff[16]
 *                double elemNum
 *                double Val
 *                double numDim
 *                double b_TT
 *                const OptSync_type *OptSync
 * Return Type  : boolean_T
 */
boolean_T CyclicBuffer_addProx(proxBuff_type buff[16], double elemNum, double
  Val, double numDim, double b_TT, const OptSync_type *OptSync)
{
  boolean_T added;
  double nxtInd;
  int i;
  b_TT += OptSync->prox_TTfix;
  added = true;
  if (buff[(int)elemNum - 1].newTT + OptSync->minTTstep_prox >= b_TT) {
    b_numThrows++;
    added = false;
  } else {
    b_numThrows = 0.0;
    nxtInd = buff[(int)elemNum - 1].newInd + 1.0;
    if (buff[(int)elemNum - 1].newInd + 1.0 > OptSync->BufLen) {
      nxtInd = 1.0;
    }

    /*  check if looped over oldest data: */
    if (buff[(int)elemNum - 1].oldInd == nxtInd) {
      buff[(int)elemNum - 1].oldInd++;
      if (buff[(int)elemNum - 1].oldInd > OptSync->BufLen) {
        buff[(int)elemNum - 1].oldInd = 1.0;
      }

      buff[(int)elemNum - 1].oldTT = buff[(int)elemNum - 1].Mat[((int)buff[(int)
        elemNum - 1].oldInd + 50 * ((int)(numDim + 1.0) - 1)) - 1];
    }

    for (i = 0; i < (int)numDim; i++) {
      buff[(int)elemNum - 1].Mat[((int)nxtInd + 50 * i) - 1] = Val;
    }

    buff[(int)elemNum - 1].Mat[((int)nxtInd + 50 * ((int)(numDim + 1.0) - 1)) -
      1] = b_TT;
    buff[(int)elemNum - 1].newInd = nxtInd;
    buff[(int)elemNum - 1].newTT = b_TT;
    if (buff[(int)elemNum - 1].numel == 0.0) {
      buff[(int)elemNum - 1].oldInd = 1.0;
      buff[(int)elemNum - 1].oldTT = b_TT;
      buff[(int)elemNum - 1].numel = 1.0;
    } else if ((buff[(int)elemNum - 1].numel + 1.0 < OptSync->BufLen) || rtIsNaN
               (OptSync->BufLen)) {
      buff[(int)elemNum - 1].numel++;
    } else {
      buff[(int)elemNum - 1].numel = OptSync->BufLen;
    }
  }

  return added;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void CyclicBuffer_addProx_init(void)
{
  b_numThrows = 0.0;
}

/*
 * File trailer for CyclicBuffer_addProx.c
 *
 * [EOF]
 */
