/*
 * File: CyclicBuffer_addPos.c
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
static double numThrows;

/* Function Definitions */

/*
 * Arguments    : xyBuff_type *buff
 *                double elemNum
 *                const double Val[2]
 *                double numDim
 *                double b_TT
 *                const OptSync_type *OptSync
 * Return Type  : boolean_T
 */
boolean_T CyclicBuffer_addPos(xyBuff_type *buff, double elemNum, const double
  Val[2], double numDim, double b_TT, const OptSync_type *OptSync)
{
  boolean_T added;
  double nxtInd;
  int i;
  (void)elemNum;
  b_TT += OptSync->xy_TTfix;
  added = true;
  if (buff->newTT + OptSync->minTTstep_xy >= b_TT) {
    numThrows++;
    added = false;
  } else {
    numThrows = 0.0;
    nxtInd = buff->newInd + 1.0;
    if (buff->newInd + 1.0 > OptSync->BufLen) {
      nxtInd = 1.0;
    }

    /*  check if looped over oldest data: */
    if (buff->oldInd == nxtInd) {
      buff->oldInd++;
      if (buff->oldInd > OptSync->BufLen) {
        buff->oldInd = 1.0;
      }

      buff->oldTT = buff->Mat[((int)buff->oldInd + 50 * ((int)(numDim + 1.0) - 1))
        - 1];
    }

    for (i = 0; i < (int)numDim; i++) {
      buff->Mat[((int)nxtInd + 50 * i) - 1] = Val[i];
    }

    buff->Mat[((int)nxtInd + 50 * ((int)(numDim + 1.0) - 1)) - 1] = b_TT;
    buff->newInd = nxtInd;
    buff->newTT = b_TT;
    if (buff->numel == 0.0) {
      buff->oldInd = 1.0;
      buff->oldTT = b_TT;
      buff->numel = 1.0;
    } else if ((buff->numel + 1.0 < OptSync->BufLen) || rtIsNaN(OptSync->BufLen))
    {
      buff->numel++;
    } else {
      buff->numel = OptSync->BufLen;
    }
  }

  return added;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void CyclicBuffer_addPos_init(void)
{
  numThrows = 0.0;
}

/*
 * File trailer for CyclicBuffer_addPos.c
 *
 * [EOF]
 */
