/*
 * File: init.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 25-Nov-2019 10:24:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "cf_feat_ext.h"
#include "init.h"

/* Function Definitions */

/*
 * Arguments    : unsigned short xyPosLen
 *                unsigned short xyMeasLen
 *                Opt_type *Opt
 * Return Type  : void
 */
void init(unsigned short xyPosLen, unsigned short xyMeasLen, Opt_type *Opt)
{
  Opt->measHistMinDist_p2 = 1600.0F;
  Opt->posHistMinStep_p2 = 10000.0F;
  Opt->xyPosLen = xyPosLen;
  Opt->xyMeasLen = xyMeasLen;
  Opt->nCyc_lowFreq = 40;
}

/*
 * File trailer for init.c
 *
 * [EOF]
 */
