/*
 * File: CyclicBuffer_addPsi.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:26:10
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "CyclicBuffer_addPsi.h"

/* Variable Definitions */
static double numThrows;

/* Function Definitions */

/*
 * Arguments    : psiBuff_type *buff
 *                double elemNum
 *                double Val
 *                double numDim
 *                double TT
 *                const OptSync_type *OptSync
 * Return Type  : boolean_T
 */
boolean_T CyclicBuffer_addPsi(psiBuff_type *buff, double elemNum, double Val,
  double numDim, double TT, const OptSync_type *OptSync)
{
  boolean_T added;
  double nxtInd;
  int i;
  (void)elemNum;
  TT += OptSync->psi_TTfix;
  added = true;
  if (buff->newTT + OptSync->minTTstep_psi >= TT) {
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
      buff->Mat[((int)nxtInd + 50 * i) - 1] = Val;
    }

    buff->Mat[((int)nxtInd + 50 * ((int)(numDim + 1.0) - 1)) - 1] = TT;
    buff->newInd = nxtInd;
    buff->newTT = TT;
    if (buff->numel == 0.0) {
      buff->oldInd = 1.0;
      buff->oldTT = TT;
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
void CyclicBuffer_addPsi_init(void)
{
  numThrows = 0.0;
}

/*
 * File trailer for CyclicBuffer_addPsi.c
 *
 * [EOF]
 */
