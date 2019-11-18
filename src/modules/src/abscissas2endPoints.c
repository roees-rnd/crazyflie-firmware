/*
 * File: abscissas2endPoints.c
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
#include "abscissas2endPoints.h"

/* Function Definitions */

/*
 * Arguments    : const double rho_phi[2]
 *                const double absc[2]
 *                double endps[4]
 *                double ep_line[8]
 *                double ep_t[4]
 * Return Type  : void
 */
void abscissas2endPoints(const double rho_phi[2], const double absc[2], double
  endps[4], double ep_line[8], double ep_t[4])
{
  double dv9[4];
  double b_rho_phi[2];
  double dv10[4];
  double c_rho_phi[2];
  int i9;
  double dv11[2];
  double dv12[2];
  int i10;

  /*  jacobians */
  /*   */
  /*  syms rho phi t real */
  /*  rho_phi = [rho; phi ] */
  /*  [p,P_line,P_t]=t2endPoint(rho_phi,t) */
  /*  simplify(P_line -jacobian(p,rho_phi)) */
  /*  simplify(P_t -jacobian(p,t)) */
  /*  dpdrho = */
  /*  */
  /*   cos(phi) */
  /*   sin(phi) */
  /*  */
  /*  */
  /*  dpdphi = */
  /*  */
  /*   - t*cos(phi) - rho*sin(phi) */
  /*     rho*cos(phi) - t*sin(phi) */
  /*  jacobians */
  /*   */
  /*  syms rho phi t real */
  /*  rho_phi = [rho; phi ] */
  /*  [p,P_line,P_t]=t2endPoint(rho_phi,t) */
  /*  simplify(P_line -jacobian(p,rho_phi)) */
  /*  simplify(P_t -jacobian(p,t)) */
  /*  dpdrho = */
  /*  */
  /*   cos(phi) */
  /*   sin(phi) */
  /*  */
  /*  */
  /*  dpdphi = */
  /*  */
  /*   - t*cos(phi) - rho*sin(phi) */
  /*     rho*cos(phi) - t*sin(phi) */
  dv9[0] = cos(rho_phi[1]);
  dv9[2] = -sin(rho_phi[1]);
  dv9[1] = sin(rho_phi[1]);
  dv9[3] = cos(rho_phi[1]);
  b_rho_phi[0] = rho_phi[0];
  b_rho_phi[1] = absc[0];
  dv10[0] = cos(rho_phi[1]);
  dv10[2] = -sin(rho_phi[1]);
  dv10[1] = sin(rho_phi[1]);
  dv10[3] = cos(rho_phi[1]);
  c_rho_phi[0] = rho_phi[0];
  c_rho_phi[1] = absc[1];
  for (i9 = 0; i9 < 2; i9++) {
    dv11[i9] = 0.0;
    dv12[i9] = 0.0;
    for (i10 = 0; i10 < 2; i10++) {
      dv11[i9] += dv9[i9 + (i10 << 1)] * b_rho_phi[i10];
      dv12[i9] += dv10[i9 + (i10 << 1)] * c_rho_phi[i10];
    }

    endps[i9 << 1] = dv11[i9];
    endps[1 + (i9 << 1)] = dv12[i9];
  }

  ep_line[0] = cos(rho_phi[1]);
  ep_line[1] = sin(rho_phi[1]);
  ep_line[4] = -absc[0] * cos(rho_phi[1]) - rho_phi[0] * sin(rho_phi[1]);
  ep_line[5] = rho_phi[0] * cos(rho_phi[1]) - absc[0] * sin(rho_phi[1]);
  ep_line[2] = cos(rho_phi[1]);
  ep_line[3] = sin(rho_phi[1]);
  ep_line[6] = -absc[1] * cos(rho_phi[1]) - rho_phi[0] * sin(rho_phi[1]);
  ep_line[7] = rho_phi[0] * cos(rho_phi[1]) - absc[1] * sin(rho_phi[1]);
  ep_t[0] = -sin(rho_phi[1]);
  ep_t[1] = cos(rho_phi[1]);
  ep_t[2] = -sin(rho_phi[1]);
  ep_t[3] = cos(rho_phi[1]);
}

/*
 * File trailer for abscissas2endPoints.c
 *
 * [EOF]
 */
