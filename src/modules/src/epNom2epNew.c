/*
 * File: epNom2epNew.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/epNom2epNew.h"

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
 * Both inputs are at world coordinate system.
 *  endps = [x1 y1; x2 y2]
 *
 *  epNew - is the projection of endps onto the line defined by 'rhoPhiWorld'.
 *  Jac - is d(epNew)/d(rhoPhiWorld).
 * Arguments    : const double b_rhoPhiWorld[2]
 *                const double endps[4]
 *                double epNew[4]
 *                double b_Jac[8]
 * Return Type  : void
 */
void epNom2epNew(const double b_rhoPhiWorld[2], const double endps[4], double
                 epNew[4], double b_Jac[8])
{
  double rhoDirec[2];
  double b_x;
  int i11;
  double b;
  double c_x;
  double d_x;
  rhoDirec[0] = cos(b_rhoPhiWorld[1]);
  rhoDirec[1] = sin(b_rhoPhiWorld[1]);
  b_x = 0.0;
  for (i11 = 0; i11 < 2; i11++) {
    b_x += endps[i11 << 1] * rhoDirec[i11];
  }

  b = b_rhoPhiWorld[0] - b_x;
  b_x = 0.0;
  for (i11 = 0; i11 < 2; i11++) {
    b_x += endps[1 + (i11 << 1)] * rhoDirec[i11];
    epNew[i11 << 1] = endps[i11 << 1] + rhoDirec[i11] * b;
  }

  b = b_rhoPhiWorld[0] - b_x;
  for (i11 = 0; i11 < 2; i11++) {
    epNew[1 + (i11 << 1)] = endps[1 + (i11 << 1)] + rhoDirec[i11] * b;
  }

  b_x = cos(b_rhoPhiWorld[1]);
  b = cos(b_rhoPhiWorld[1]);
  c_x = cos(b_rhoPhiWorld[1]);
  d_x = cos(b_rhoPhiWorld[1]);
  b_Jac[0] = cos(b_rhoPhiWorld[1]);
  b_Jac[4] = (2.0 * endps[0] * cos(b_rhoPhiWorld[1]) * sin(b_rhoPhiWorld[1]) -
              endps[2] * (2.0 * (b_x * b_x) - 1.0)) - b_rhoPhiWorld[0] * sin
    (b_rhoPhiWorld[1]);
  b_Jac[1] = sin(b_rhoPhiWorld[1]);
  b_Jac[5] = (b_rhoPhiWorld[0] * cos(b_rhoPhiWorld[1]) - endps[0] * (2.0 * (b *
    b) - 1.0)) - 2.0 * endps[2] * cos(b_rhoPhiWorld[1]) * sin(b_rhoPhiWorld[1]);
  b_Jac[2] = cos(b_rhoPhiWorld[1]);
  b_Jac[6] = (2.0 * endps[1] * cos(b_rhoPhiWorld[1]) * sin(b_rhoPhiWorld[1]) -
              endps[3] * (2.0 * (c_x * c_x) - 1.0)) - b_rhoPhiWorld[0] * sin
    (b_rhoPhiWorld[1]);
  b_Jac[3] = sin(b_rhoPhiWorld[1]);
  b_Jac[7] = (b_rhoPhiWorld[0] * cos(b_rhoPhiWorld[1]) - endps[1] * (2.0 * (d_x *
    d_x) - 1.0)) - 2.0 * endps[3] * cos(b_rhoPhiWorld[1]) * sin(b_rhoPhiWorld[1]);

  /*  syms rho phi endpx1 endpx2 endpy1 endpy2 real */
  /*   */
  /*  [epNew, Jac]=epNom2epNew([rho phi],[endpx1 endpy1; endpx2 endpy2]) */
  /*   */
  /*  Jac-simplify(jacobian(epNew([1 3 2 4]'),[rho phi])) */
  /*   */
}

/*
 * File trailer for epNom2epNew.c
 *
 * [EOF]
 */
