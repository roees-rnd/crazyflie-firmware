/*
 * File: refreshMapCntlUnit.c
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
#include "refreshMapCntlUnit.h"

/* Function Definitions */

/*
 * persistent lastChecked;
 * Arguments    : Map_type *Map
 *                const Lmk_type Lmk[15]
 *                double *lmkId
 *                double lmkEndp[4]
 *                double *LmkAction
 * Return Type  : void
 */
void refreshMapCntlUnit(Map_type *Map, const Lmk_type Lmk[15], double *lmkId,
  double lmkEndp[4], double *LmkAction)
{
  int i;
  *LmkAction = 0.0;
  *lmkId = 0.0;
  for (i = 0; i < 4; i++) {
    lmkEndp[i] = 0.0;
  }

  if (!(Map->numUsed < 1.0)) {
    Map->lastChecked = Lmk[(int)Map->lastChecked - 1].link2next;
    *lmkId = Map->lastChecked;
    for (i = 0; i < 4; i++) {
      lmkEndp[i] = Lmk[(int)Map->lastChecked - 1].parEndp.e[i];
    }

    *LmkAction = 3.0;
  }
}

/*
 * File trailer for refreshMapCntlUnit.c
 *
 * [EOF]
 */
