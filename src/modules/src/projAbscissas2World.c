/*
 * File: projAbscissas2World.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/projAbscissas2World.h"

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
