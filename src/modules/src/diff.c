/*
 * File: diff.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/diff.h"

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
 * Arguments    : const double b_x[4]
 *                double b_y[2]
 * Return Type  : void
 */
void b_diff(const double b_x[4], double b_y[2])
{
  int ix;
  int iy;
  int s;
  ix = 2;
  iy = 0;
  for (s = 0; s < 2; s++) {
    b_y[iy] = b_x[ix] - b_x[ix - 2];
    ix++;
    iy++;
  }
}

/*
 * Arguments    : const double b_x[4]
 *                double b_y[2]
 * Return Type  : void
 */
void c_diff(const double b_x[4], double b_y[2])
{
  int ixStart;
  int iyStart;
  int b_r;
  ixStart = 1;
  iyStart = 0;
  for (b_r = 0; b_r < 2; b_r++) {
    b_y[iyStart] = b_x[ixStart] - b_x[ixStart - 1];
    ixStart += 2;
    iyStart++;
  }
}

/*
 * Arguments    : const double b_x[2]
 * Return Type  : double
 */
double d_diff(const double b_x[2])
{
  return b_x[1] - b_x[0];
}

/*
 * Arguments    : const double b_x[400]
 *                double b_y[398]
 * Return Type  : void
 */
void diff(const double b_x[400], double b_y[398])
{
  int ix;
  int iy;
  int s;
  int ixLead;
  int iyLead;
  double work;
  int m;
  double tmp1;
  ix = 2;
  iy = 0;
  for (s = 0; s < 2; s++) {
    ixLead = ix;
    iyLead = iy;
    work = b_x[ix - 2];
    for (m = 0; m < 199; m++) {
      tmp1 = work;
      work = b_x[ixLead];
      tmp1 = b_x[ixLead] - tmp1;
      ixLead += 2;
      b_y[iyLead] = tmp1;
      iyLead += 2;
    }

    ix++;
    iy++;
  }
}

/*
 * File trailer for diff.c
 *
 * [EOF]
 */
