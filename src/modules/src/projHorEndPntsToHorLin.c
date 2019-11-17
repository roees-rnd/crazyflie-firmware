/*
 * File: projHorEndPntsToHorLin.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

/* Include Files */
#include "../modules/src/projHorEndPntsToHorLin.h"

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
 * Arguments    : double rho
 *                double theta
 *                const double p[2]
 *                double projected_point[2]
 *                double *projection_length
 * Return Type  : void
 */
void projHorEndPntToHorLin(double rho, double theta, const double p[2], double
  projected_point[2], double *projection_length)
{
  double vec_from_origin_to_p[2];
  double line_dir[2];
  int k;

  /*  rho,theta(rad) - 2D line param */
  /*  p - 2D point */
  /*  line_dir is always pi/2 to the left from the line normal (  pi/2 counterclockwise ) */
  projected_point[0] = rho * cos(theta);
  projected_point[1] = rho * sin(theta);
  vec_from_origin_to_p[0] = p[0] - projected_point[0];
  vec_from_origin_to_p[1] = p[1] - projected_point[1];
  line_dir[0] = -sin(theta);
  line_dir[1] = cos(theta);

  /*  line_dir is pi/2 counterclockwise from the line normal  */
  *projection_length = 0.0;
  for (k = 0; k < 2; k++) {
    *projection_length += vec_from_origin_to_p[k] * line_dir[k];
  }

  for (k = 0; k < 2; k++) {
    projected_point[k] += *projection_length * line_dir[k];
  }
}

/*
 * Input:
 *  rho,theta(rad) - scalars
 *  p1, p2 - 2D points (column vectors)
 *  Output:
 *  t   = [t1;t2]; where t1 & t2 scalars and t2 > t1
 * Arguments    : double rho
 *                double theta
 *                const double p1[2]
 *                const double p2[2]
 *                double seg[4]
 *                double b_t[2]
 * Return Type  : void
 */
void projHorEndPntsToHorLin(double rho, double theta, const double p1[2], const
  double p2[2], double seg[4], double b_t[2])
{
  double e1[2];
  double t1;
  double e2[2];
  double t2;
  int i;

  /*  global Map */
  /*   */
  /*  % Plucker line */
  /*  l = Map.x(Lmk.state.r); */
  /*   */
  /*  % rays in sensor frame */
  /*  r1s = pix2PluckerRay(Sen.par.k,Obs.meas.y(1:2)); */
  /*  r2s = pix2PluckerRay(Sen.par.k,Obs.meas.y(3:4)); */
  /*   */
  /*  % rays in world frame */
  /*  r1 = fromFramePlucker(Rob.frame,fromFramePlucker(Sen.frame,r1s)); */
  /*  r2 = fromFramePlucker(Rob.frame,fromFramePlucker(Sen.frame,r2s)); */
  /*   */
  /*  % endpoints and abscissas */
  /*  [e1,t1] = intersectPlucker(l,r1); */
  /*  [e2,t2] = intersectPlucker(l,r2); */
  projHorEndPntToHorLin(rho, theta, p1, e1, &t1);
  projHorEndPntToHorLin(rho, theta, p2, e2, &t2);

  /*  build segment and abscissas vector */
  if (t2 > t1) {
    for (i = 0; i < 2; i++) {
      seg[i] = e1[i];
      seg[i + 2] = e2[i];
    }

    b_t[0] = t1;
    b_t[1] = t2;
  } else {
    for (i = 0; i < 2; i++) {
      seg[i] = e2[i];
      seg[i + 2] = e1[i];
    }

    b_t[0] = t2;
    b_t[1] = t1;
  }
}

/*
 * File trailer for projHorEndPntsToHorLin.c
 *
 * [EOF]
 */
