/*
 * File: CyclicBuffer_addPos_types.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 13-Nov-2019 15:25:03
 */

#ifndef CYCLICBUFFER_ADDPOS_TYPES_H
#define CYCLICBUFFER_ADDPOS_TYPES_H

/* Include Files */
#include "../modules/src/rtwtypes.h"

/* Type Definitions */
#ifndef typedef_LmkParEndp_type
#define typedef_LmkParEndp_type

typedef struct {
  double t[2];
  double e[4];
  double nrm[2];
} LmkParEndp_type;

#endif                                 /*typedef_LmkParEndp_type*/

#ifndef typedef_LmkState_type
#define typedef_LmkState_type

typedef struct {
  double r[2];
} LmkState_type;

#endif                                 /*typedef_LmkState_type*/

#ifndef typedef_Lmk_type
#define typedef_Lmk_type

typedef struct {
  double lmk;
  double id;
  char type[6];
  boolean_T used;
  LmkState_type state;
  LmkParEndp_type parEndp;
  double nSearch;
  double nMatch;
  double nInlier;
  double link2prev;
  double link2next;
} Lmk_type;

#endif                                 /*typedef_Lmk_type*/

#ifndef typedef_Map_type
#define typedef_Map_type

typedef struct {
  boolean_T used[303];
  double x[303];
  double P[91809];
  double len;
  double numUsed;
  double lmk0;
  double lastChecked;
} Map_type;

#endif                                 /*typedef_Map_type*/

#ifndef typedef_ObsExp_type
#define typedef_ObsExp_type

typedef struct {
  double e[3];
  double E[9];
  double um;
  double E_rl_jpd[25];
} ObsExp_type;

#endif                                 /*typedef_ObsExp_type*/

#ifndef typedef_ObsInn_type
#define typedef_ObsInn_type

typedef struct {
  double z[2];
  double Z[4];
  double iZ[4];
  double MD2;
} ObsInn_type;

#endif                                 /*typedef_ObsInn_type*/

#ifndef typedef_ObsJac_type
#define typedef_ObsJac_type

typedef struct {
  double E_r[9];
  double E_l[6];
  double Z_r[6];
  double Z_l[4];
  double J_z2rl[10];
} ObsJac_type;

#endif                                 /*typedef_ObsJac_type*/

#ifndef typedef_ObsMeas_type
#define typedef_ObsMeas_type

typedef struct {
  double y[4];
  double R[16];
} ObsMeas_type;

#endif                                 /*typedef_ObsMeas_type*/

#ifndef typedef_Obs_type
#define typedef_Obs_type

typedef struct {
  double sen;
  double lmk;
  char ltype[6];
  ObsMeas_type meas;
  ObsExp_type exp;
  ObsInn_type inn;
  boolean_T vis;
  boolean_T measured;
  boolean_T matched;
  boolean_T updated;
  ObsJac_type Jac;
} Obs_type;

#endif                                 /*typedef_Obs_type*/

#ifndef typedef_OptCorrectLines_type
#define typedef_OptCorrectLines_type

typedef struct {
  char innType[6];
  boolean_T extPolicy;
} OptCorrectLines_type;

#endif                                 /*typedef_OptCorrectLines_type*/

#ifndef typedef_OptCorrect_type
#define typedef_OptCorrect_type

typedef struct {
  double nUpdates;
  double MD2th;
  double linTestIdp;
  double isVisOrtThrsh;
  double isVisFarThrsh[2];
  OptCorrectLines_type lines;
} OptCorrect_type;

#endif                                 /*typedef_OptCorrect_type*/

#ifndef typedef_OptInitLines_type
#define typedef_OptInitLines_type

typedef struct {
  double minLength;
} OptInitLines_type;

#endif                                 /*typedef_OptInitLines_type*/

#ifndef typedef_OptInit_type
#define typedef_OptInit_type

typedef struct {
  double nbrInits[2];
  char initType[6];
  OptInitLines_type lines;
} OptInit_type;

#endif                                 /*typedef_OptInit_type*/

#ifndef typedef_OptMap_type
#define typedef_OptMap_type

typedef struct {
  double numLmks;
  double lmkSize;
} OptMap_type;

#endif                                 /*typedef_OptMap_type*/

#ifndef typedef_OptSplitFit_type
#define typedef_OptSplitFit_type

typedef struct {
  double MIN_NUM_OF_PNTS_ON_LINE;
  double MIN_SEG_LEN;
  double MAX_GAP;
  double epsilon;
  double alphaProx;
  boolean_T onlyEndp;
  double v[3];
} OptSplitFit_type;

#endif                                 /*typedef_OptSplitFit_type*/

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

#ifndef typedef_Opt_type
#define typedef_Opt_type

typedef struct {
  OptMap_type map;
  OptCorrect_type correct;
  OptInit_type init;
  double maxDistVis;
  double SF_mm2m;
  double minCmd_x;
  double minCmd_t;
  double pntVicThresh;
  double numRefreshMap;
  double NSamplesPseudoscan;
  double maxGap;
  double stepsToSendMap;
  double stepsToMerge;
  double segMinLenForInit;
  double segMaxDistForInit;
  OptSplitFit_type splitFit;
} Opt_type;

#endif                                 /*typedef_Opt_type*/

#ifndef typedef_Prox_type
#define typedef_Prox_type

typedef struct {
  double val[16];
  double TT[16];
} Prox_type;

#endif                                 /*typedef_Prox_type*/

#ifndef typedef_Raw_type
#define typedef_Raw_type

typedef struct {
  double rhoPhiRob[2];
  double covRhoPhi[4];
  double abscissas[2];
  double nSamples;
  double length;
  double robPoseFreeze[3];
  double robPoseCov[9];
  double avgDist;
  char type[7];
  double rhoPhiWorld[2];
  double covRhoPhiWorld[4];
  double covRhoPhiWorldForInit[4];
  double abscissasWorld[2];
  double endpWorld[4];
  double covEndpointsWorld[16];
  double J_s2r[12];
  double updated;
  double nrm[2];
} Raw_type;

#endif                                 /*typedef_Raw_type*/

#ifndef typedef_RobCon_type
#define typedef_RobCon_type

typedef struct {
  double u[3];
  double U[9];
} RobCon_type;

#endif                                 /*typedef_RobCon_type*/

#ifndef typedef_RobState_type
#define typedef_RobState_type

typedef struct {
  double x[3];
  double P[9];
  double size;
  double r[3];
} RobState_type;

#endif                                 /*typedef_RobState_type*/

#ifndef typedef_Rob_type
#define typedef_Rob_type

typedef struct {
  RobCon_type con;
  RobState_type state;
  double sensors[17];
} Rob_type;

#endif                                 /*typedef_Rob_type*/

#ifndef typedef_SenPar_type
#define typedef_SenPar_type

typedef struct {
  double maxRange;
  double minRange;
  double stdErr;
  double meanErr;
} SenPar_type;

#endif                                 /*typedef_SenPar_type*/

#ifndef typedef_Sen_type
#define typedef_Sen_type

typedef struct {
  double sen;
  double id;
  char name[5];
  char type[7];
  double robot;
  double position[2];
  double orientationDegrees;
  SenPar_type par;
} Sen_type;

#endif                                 /*typedef_Sen_type*/

#ifndef typedef_proxBuff_type
#define typedef_proxBuff_type

typedef struct {
  double Mat[100];
  double oldInd;
  double newInd;
  double numel;
  double oldTT;
  double newTT;
} proxBuff_type;

#endif                                 /*typedef_proxBuff_type*/

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

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  boolean_T used[33];
  double x[33];
  double P[1089];
  double len;
  double numUsed;
  double lmk0;
  double lastChecked;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  boolean_T Raw_faultyMeas;
  short Raw_xyRob_1[2];
  short Raw_xyRob_pnum[2];
  unsigned short Raw_num_segments;
  signed char Raw_pnum;
  unsigned short Lmk_numLmks;
  double Lmk_indLmk;
  int Lmk_ParEndp[4];
  int Raw_EndpW1[4];
  int Raw_EndpW2[4];
  int Raw_EndpW3[4];
  double slamVersion;
  double dPosFault;
  boolean_T updateCU;
} struct1_T;

#endif                                 /*typedef_struct1_T*/

#ifndef typedef_xyBuff_type
#define typedef_xyBuff_type

typedef struct {
  double Mat[150];
  double oldInd;
  double newInd;
  double numel;
  double oldTT;
  double newTT;
} xyBuff_type;

#endif                                 /*typedef_xyBuff_type*/
#endif

/*
 * File trailer for CyclicBuffer_addPos_types.h
 *
 * [EOF]
 */
