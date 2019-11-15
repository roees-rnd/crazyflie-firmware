/*
 * File: projAbscissas2World.c
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
#include "projAbscissas2World.h"

/* Function Definitions */

/*
 * tRob=ds.t;
 * Arguments    : const double RobPosFreeze[2]
 *                const double rhoTtaRob[2]
 *                const double tRob[2]
 *                double tWorld[2]
 * Return Type  : void
 */
void projAbscissas2World(const double RobPosFreeze[2], const double rhoTtaRob[2],
  const double tRob[2], double tWorld[2])
{
  double dv8[2];
  double b_y;
  int i;

  /*  preallocation for coder */
  dv8[0] = -sin(rhoTtaRob[1]);
  dv8[1] = cos(rhoTtaRob[1]);
  b_y = 0.0;
  for (i = 0; i < 2; i++) {
    b_y += dv8[i] * RobPosFreeze[i];
  }

  for (i = 0; i < 2; i++) {
    tWorld[i] = tRob[i] + b_y;
  }
}

/*
 * File trailer for projAbscissas2World.c
 *
 * [EOF]
 */
