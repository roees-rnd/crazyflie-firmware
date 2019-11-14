/*
 * File: SaM.c
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
#include "SaM.h"
#include "getMaxPenDist.h"

/* Function Definitions */

/*
 * The Ramer�Douglas�Peucker algorithm (RDP) is an algorithm for reducing
 *  the number of points in a curve that is approximated by a series of
 *  points. The initial form of the algorithm was independently suggested
 *  in 1972 by Urs Ramer and 1973 by David Douglas and Thomas Peucker and
 *  several others in the following decade. This algorithm is also known
 *  under the names Douglas�Peucker algorithm, iterative end-point fit
 *  algorithm and split-and-merge algorithm. [Source Wikipedia]
 *
 *  Input:
 *            Points: List of Points 2xN
 *            epsilon: distance dimension, specifies the similarity between
 *            the original curve and the approximated (smaller the epsilon,
 *            the curves more similar)
 *  Output:
 *            Points: List of Points for the approximated curve 2xM (M<=N)
 *
 *
 *  -------------------------------------------------------
 *  Code: Reza Ahmadzadeh (2017)
 *  -------------------------------------------------------
 * Arguments    : double Points[600]
 *                double i_s
 *                double i_e
 *                double b_epsilon
 * Return Type  : void
 */
void SaM(double Points[600], double i_s, double i_e, double b_epsilon)
{
  double dmax;
  double idx;
  int i27;
  int i;
  getMaxPenDist(Points, i_s, i_e, &dmax, &idx);

  /*  dmax = 0; */
  /*  edx = size(Points,2); */
  /*  for ii = 2:edx-1 */
  /*      d = penDistance(Points(1:2,ii),Points(1:2,1),Points(1:2,edx)); */
  /*      if d > dmax */
  /*          idx = ii; */
  /*          dmax = d; */
  /*      end */
  /*  end */
  if (dmax > b_epsilon) {
    /*  recursive call */
    if (idx - i_s > 1.0) {
      SaM(Points, i_s, idx, b_epsilon);

      /* (:,1:idx),epsilon); */
      /* else */
      /* recResult1=Points(:,1:idx); */
    }

    if (i_e - idx > 1.0) {
      SaM(Points, idx, i_e, b_epsilon);

      /* (:,idx:edx),epsilon); */
      /* else */
      /* recResult2 = Points(:,idx:edx); */
    }

    /*      for i=i_s:(idx-1) */
    /*          result(3,i)=recResult1(3,i); */
    /*      end */
    /*      for i=idx:edx */
    /*          result(3,i)=recResult2(3,i); */
    /*      end */
    /*      ind_new = union(ind1,ind2); */
  } else {
    i27 = (int)((i_e - 1.0) + (1.0 - (i_s + 1.0)));
    for (i = 0; i < i27; i++) {
      Points[2 + 3 * ((int)((i_s + 1.0) + (double)i) - 1)] = 0.0;
    }

    /*      ind_new = [1 edx]; */
  }

  /*  If max distance is greater than epsilon, recursively simplify */
  /*      function d = penDistance(Pp, P1, P2) */
  /*          % find the distance between a Point Pp and a line segment between P1, P2. */
  /*          d = abs((P2(2,1)-P1(2,1))*Pp(1,1) - (P2(1,1)-P1(1,1))*Pp(2,1) + P2(1,1)*P1(2,1) - P2(2,1)*P1(1,1)) ... */
  /*              / sqrt((P2(2,1)-P1(2,1))^2 + (P2(1,1)-P1(1,1))^2); */
  /*      end */
  /*  ind= union(ind,ind_new) */
}

/*
 * File trailer for SaM.c
 *
 * [EOF]
 */
