/*
 * File: CyclicBuffer_addPsi_types.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 18-Nov-2019 16:30:18
 */

#ifndef CYCLICBUFFER_ADDPSI_TYPES_H
#define CYCLICBUFFER_ADDPSI_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_OptSync_type
#define typedef_OptSync_type

typedef struct {
  double BufLen;
  double fastTimeStep;
  double fastTimeStepFreq;
  double numOdomSmpl;
  double slowTimeStep;
  double numSen;
  double timeGridLen;
  double timeGrid[64];
  double minTTstep_xy;
  double minTTstep_psi;
  double minTTstep_prox;
  double xy_TTfix;
  double psi_TTfix;
  double prox_TTfix;
} OptSync_type;

#endif                                 /*typedef_OptSync_type*/

#ifndef typedef_psiBuff_type
#define typedef_psiBuff_type

typedef struct {
  double Mat[100];
  double oldInd;
  double newInd;
  double numel;
  double oldTT;
  double newTT;
} psiBuff_type;

#endif                                 /*typedef_psiBuff_type*/
#endif

/*
 * File trailer for CyclicBuffer_addPsi_types.h
 *
 * [EOF]
 */
