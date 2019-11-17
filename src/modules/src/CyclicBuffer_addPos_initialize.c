/*
 * File: CyclicBuffer_addPos_initialize.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/CyclicBuffer_addPos_initialize.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/getMetry.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/processSlamInput.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

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
