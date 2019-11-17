/*
 * File: getMaxPenDist.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/getMaxPenDist.h"

#include "../modules/src/CyclicBuffer_addPos.h"
#include "../modules/src/CyclicBuffer_addProx.h"
#include "../modules/src/initCyclicBuffer.h"
#include "../modules/src/initSlam.h"
#include "../modules/src/rt_nonfinite.h"
#include "../modules/src/slamOnVehicle.h"
#include "../modules/src/stepBetweenSync.h"
#include "../modules/src/upSampAndFilt_AttPos.h"

/* Function Definitions */

/*
 * Arguments    : const double Points[600]
 *                double i_s
 *                double i_e
 *                double *dmax
 *                double *idx
 * Return Type  : void
 */
void getMaxPenDist(const double Points[600], double i_s, double i_e, double
                   *dmax, double *idx)
{
  int i4;
  int ii;
  double b_ii;
  double d;
  double a;
  *idx = -1.0;
  *dmax = 0.0;
  i4 = (int)((i_e - 1.0) + (1.0 - (i_s + 1.0)));
  for (ii = 0; ii < i4; ii++) {
    b_ii = (i_s + 1.0) + (double)ii;

    /*  find the distance between a Point Pp and a line segment between P1, P2. */
    d = Points[1 + 3 * ((int)i_e - 1)] - Points[1 + 3 * ((int)i_s - 1)];
    a = Points[3 * ((int)i_e - 1)] - Points[3 * ((int)i_s - 1)];
    d = fabs((((Points[1 + 3 * ((int)i_e - 1)] - Points[1 + 3 * ((int)i_s - 1)])
               * Points[3 * ((int)b_ii - 1)] - (Points[3 * ((int)i_e - 1)] -
                Points[3 * ((int)i_s - 1)]) * Points[1 + 3 * ((int)b_ii - 1)]) +
              Points[3 * ((int)i_e - 1)] * Points[1 + 3 * ((int)i_s - 1)]) -
             Points[1 + 3 * ((int)i_e - 1)] * Points[3 * ((int)i_s - 1)]) / sqrt
      (d * d + a * a);
    if (d > *dmax) {
      *idx = b_ii;
      *dmax = d;
    }
  }
}

/*
 * File trailer for getMaxPenDist.c
 *
 * [EOF]
 */
