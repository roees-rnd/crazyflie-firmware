/*
 * File: CyclicBuffer_addPos_initialize.c
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
#include "CyclicBuffer_addPos_initialize.h"
#include "getMetry.h"
#include "processSlamInput.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void CyclicBuffer_addPos_initialize(void)
{
  rt_InitInfAndNaN(8U);
  d_wantedSyncTime_T0_prev_not_em();
  prevRobXYT_not_empty_init();
  FirstRun_not_empty_init();
  CyclicBuffer_addPos_init();
  CyclicBuffer_addProx_init();
  processSlamInput_init();
  getMetry_init();
  stepBetweenSync_init();
  upSampAndFilt_AttPos_init();
}

/*
 * File trailer for CyclicBuffer_addPos_initialize.c
 *
 * [EOF]
 */
