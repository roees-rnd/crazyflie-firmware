/*
 * File: initCyclicBuffer.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/initCyclicBuffer.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Definitions */

/*
 * Arguments    : psiBuff_type *psiBuff
 *                xyBuff_type *xyBuff
 *                proxBuff_type proxBuff[16]
 *                OptSync_type *OptSync
 * Return Type  : void
 */
void initCyclicBuffer(psiBuff_type *psiBuff, xyBuff_type *xyBuff, proxBuff_type
                      proxBuff[16], OptSync_type *OptSync)
{
  int i;
  static const proxBuff_type r0 = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0 },                      /* Mat */
    1.0,                               /* oldInd */
    0.0,                               /* newInd */
    0.0,                               /* numel */
    1.0,                               /* oldTT */
    1.0                                /* newTT */
  };

  memset(&psiBuff->Mat[0], 0, 100U * sizeof(double));
  psiBuff->oldInd = 0.0;
  psiBuff->newInd = 0.0;
  psiBuff->numel = 0.0;
  psiBuff->oldTT = 1.0;
  psiBuff->newTT = 1.0;
  memset(&xyBuff->Mat[0], 0, 150U * sizeof(double));
  xyBuff->oldInd = 0.0;
  xyBuff->newInd = 0.0;
  xyBuff->numel = 0.0;
  xyBuff->oldTT = 1.0;
  xyBuff->newTT = 1.0;
  for (i = 0; i < 16; i++) {
    proxBuff[i] = r0;
  }

  OptSync->BufLen = 50.0;

  /*  [ms] (50[Hz]) */
  OptSync->fastTimeStep = 30.0;
  OptSync->fastTimeStepFreq = 0.033333333333333333;

  /*  [samples] */
  OptSync->numOdomSmpl = 20.0;

  /*  [samples] */
  /* [ms] */
  OptSync->slowTimeStep = 100.0;

  /*  [ms] */
  OptSync->numSen = 16.0;
  OptSync->timeGridLen = 64.0;
  OptSync->minTTstep_xy = 31.0;
  OptSync->minTTstep_psi = 0.0;
  OptSync->minTTstep_prox = 0.0;
  OptSync->xy_TTfix = 0.0;
  OptSync->psi_TTfix = 0.0;
  OptSync->prox_TTfix = 0.0;
  for (i = 0; i < 64; i++) {
    OptSync->timeGrid[i] = 0.0;
    OptSync->timeGrid[i] = -1000.0 + ((1.0 + (double)i) - 1.0) * 30.0;
  }

  /*  OptSync.timeGrid_indOfSmallestNeg = floor((slowTimeStep*numLeft+fastTimeStep)/fastTimeStep-0.00001); */
}

/*
 * File trailer for initCyclicBuffer.c
 *
 * [EOF]
 */
