/*
 * File: CyclicBuffer_addPsi_initialize.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:26:10
 */

/* Include Files */
#include "../modules/src/CyclicBuffer_addPsi_initialize.h"

#include "../modules/src/CyclicBuffer_addPsi.h"
#include "../modules/src/rt_nonfinite.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void CyclicBuffer_addPsi_initialize(void)
{
  rt_InitInfAndNaN(8U);
  CyclicBuffer_addPsi_init();
}

/*
 * File trailer for CyclicBuffer_addPsi_initialize.c
 *
 * [EOF]
 */
