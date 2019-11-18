/*
 * File: CyclicBuffer_addPsi_initialize.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:30:18
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "CyclicBuffer_addPsi.h"
#include "CyclicBuffer_addPsi_initialize.h"

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
