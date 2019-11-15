/*
 * File: det.c
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
#include "det.h"

/* Function Definitions */

/*
 * Arguments    : const double b_x[4]
 * Return Type  : double
 */
double det(const double b_x[4])
{
  double b_y;
  int ix;
  double c_x[4];
  signed char ipiv[2];
  int iy;
  boolean_T isodd;
  int k;
  double temp;
  for (ix = 0; ix < 4; ix++) {
    c_x[ix] = b_x[ix];
  }

  for (ix = 0; ix < 2; ix++) {
    ipiv[ix] = (signed char)(1 + ix);
  }

  ix = 0;
  if (fabs(b_x[1]) > fabs(b_x[0])) {
    ix = 1;
  }

  if (b_x[ix] != 0.0) {
    if (ix != 0) {
      ipiv[0] = 2;
      ix = 0;
      iy = 1;
      for (k = 0; k < 2; k++) {
        temp = c_x[ix];
        c_x[ix] = c_x[iy];
        c_x[iy] = temp;
        ix += 2;
        iy += 2;
      }
    }

    c_x[1] /= c_x[0];
  }

  if (c_x[2] != 0.0) {
    c_x[3] += c_x[1] * -c_x[2];
  }

  b_y = c_x[0] * c_x[3];
  isodd = false;
  if (ipiv[0] > 1) {
    isodd = true;
  }

  if (isodd) {
    b_y = -b_y;
  }

  return b_y;
}

/*
 * File trailer for det.c
 *
 * [EOF]
 */
