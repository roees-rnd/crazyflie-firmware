/*
 * File: projRhoPhiAlpha2World.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:29:35
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
#include "projRhoPhiAlpha2World.h"

/* Function Definitions */

/*
 * translation transformation of line param.(RhoPhi) from some point (RobPosFreeze) to [0,0]
 *    [meanRhoPhiWorld, covRhoPhiWorld] = projRhoPhi2World(RobPosFreeze,  rhoTtaRob, covRhoTtaRob) returns also the Jacobians:
 *  Wpos: wrt the RobPosFreeze [xRobFreeze yRobFreeze]
 *  Wrp: wrt the rhoPhiRob [rho; phi];
 *
 *    See also projAbscissas2World
 * Arguments    : const double RobPosFreezeAlpha[3]
 *                const double b_rhoPhiRob[2]
 *                double RhoPhiWorld[2]
 *                double Wpos[6]
 *                double Wrp[4]
 * Return Type  : void
 */
void projRhoPhiAlpha2World(const double RobPosFreezeAlpha[3], const double
  b_rhoPhiRob[2], double RhoPhiWorld[2], double Wpos[6], double Wrp[4])
{
  double dv7[2];
  double d0;
  int i8;
  static const signed char iv0[3] = { 0, 0, 1 };

  /*  NO ROTATION!!! Only translation of c.sys. origin , from RobPosFreeze to World's origin (initial robot position) */
  /*  Alpha - the angular error in robot orientation (in [rad]) */
  dv7[0] = cos(b_rhoPhiRob[1]);
  dv7[1] = sin(b_rhoPhiRob[1]);
  d0 = 0.0;
  for (i8 = 0; i8 < 2; i8++) {
    d0 += dv7[i8] * RobPosFreezeAlpha[i8];
  }

  RhoPhiWorld[0] = b_rhoPhiRob[0] + d0;
  RhoPhiWorld[1] = b_rhoPhiRob[1] + RobPosFreezeAlpha[2];
  Wpos[0] = cos(b_rhoPhiRob[1]);
  Wpos[2] = sin(b_rhoPhiRob[1]);
  Wpos[4] = 0.0;
  for (i8 = 0; i8 < 3; i8++) {
    Wpos[1 + (i8 << 1)] = iv0[i8];
  }

  /*    [ drho_w_dxpos(i)  drho_w_dypos(i);   dphi_w_dxpos(i)  dphi_w_dypos(i) ]; */
  Wrp[0] = 1.0;
  Wrp[2] = RobPosFreezeAlpha[1] * cos(b_rhoPhiRob[1]) - RobPosFreezeAlpha[0] *
    sin(b_rhoPhiRob[1]);
  for (i8 = 0; i8 < 2; i8++) {
    Wrp[1 + (i8 << 1)] = i8;
  }

  /*      [drho_w_drho_r(i)  drho_w_dphi_r(i);   dphi_w_drho_r(i)  dphi_w_dphi_r(i) ] */
  /*  %% jac */
  /*   */
  /*  syms xRobFreeze yRobFreeze AlphaFreeze rho phi real */
  /*  RobPosFreeze = [xRobFreeze yRobFreeze AlphaFreeze]; */
  /*  rhoPhiRob = [rho; phi]; */
  /*  [RhoPhiWorld, Wpos,  Wrp] = projRhoPhiAlpha2World(RobPosFreeze, rhoPhiRob) */
  /*   */
  /*  simplify(Wpos-jacobian(RhoPhiWorld,RobPosFreeze)) */
  /*  simplify(Wrp-jacobian(RhoPhiWorld,rhoPhiRob)) */
}

/*
 * File trailer for projRhoPhiAlpha2World.c
 *
 * [EOF]
 */
