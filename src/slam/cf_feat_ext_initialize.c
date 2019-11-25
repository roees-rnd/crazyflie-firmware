/*
 * File: cf_feat_ext_initialize.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 25-Nov-2019 10:24:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "cf_feat_ext.h"
#include "init.h"
#include "cf_feat_ext_initialize.h"
#include "myRansac.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void cf_feat_ext_initialize(void)
{
  rt_InitInfAndNaN(8U);
  mtlb_firstRun_not_empty_init();
  mtlb_myRansac_init();
}

/*
 * File trailer for cf_feat_ext_initialize.c
 *
 * [EOF]
 */
