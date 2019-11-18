/*
 * File: createObservations.c
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
#include "createObservations.h"

/* Function Definitions */

/*
 * Arguments    : Obs_type Obs[15]
 * Return Type  : void
 */
void createObservations(Obs_type Obs[15])
{
  int i;
  static const Obs_type r3 = { 17.0,   /* sen */
    0.0,                               /* lmk */
    { 'h', 'o', 'r', 'L', 'i', 'n' },  /* ltype */
    { { 0.0, 0.0, 0.0, 0.0 },          /* y */
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0 }                     /* R */
    },                                 /* meas */
    { { 0.0, 0.0, 0.0 },               /* e */
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },/* E */
      0.0,                             /* um */
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }/* E_rl_jpd */
    },                                 /* exp */
    { { 0.0, 0.0 },                    /* z */
      { 0.0, 0.0, 0.0, 0.0 },          /* Z */
      { 0.0, 0.0, 0.0, 0.0 },          /* iZ */
      0.0                              /* MD2 */
    },                                 /* inn */
    false,                             /* vis */
    false,                             /* measured */
    false,                             /* matched */
    false,                             /* updated */
    { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },/* E_r */
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },/* E_l */
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },/* Z_r */
      { 0.0, 0.0, 0.0, 0.0 },          /* Z_l */
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }/* J_z2rl */
    }                                  /* Jac */
  };

  /*  CREATEOBSERVATIONS Create Obs structure array. */
  /*    Rob = CREATEOBSERVATIONS(Sen,Lmk) creates the Obs() structure array to */
  /*    be used as SLAM data, from the information contained in Sen() and */
  /*    Rob(). See the toolbox documentation for details on this structure. */
  /*    Copyright 2008-2009 Joan Sola @ LAAS-CNRS. */
  /*  DEBUG R */
  /*  %     tmp= cell(1,Opt.map.numLmks); */
  /*  %     for i=1:Opt.map.numLmks */
  /*  %         tmp{i}=i; */
  /*  %     end */
  /*  %     %% preallocation of output for coder */
  /*  %     Obs = struct('sen', numel(Sen), 'lmk', tmp,...% 'sid', 0,...      'lid', [],...'stype', 'proxHor', */
  /*  %         'ltype', 'horLin',... */
  /*  %         'meas', struct('y', zeros(4,1), 'R', zeros(4,4)), 'exp', struct('e', zeros(3,1), 'E', zeros(3,3), 'um', 0,'E_rl_jpd',zeros(5,5)),... */
  /*  %         'inn', struct('z', zeros(2,1), 'Z', zeros(2,2), 'iZ', zeros(2,2), 'MD2', 0), 'vis', false,... */
  /*  %         'measured', false,  'matched', false, 'updated', false,... */
  /*  %         'Jac', struct('E_r', zeros(3,3), 'E_l', zeros(3,2), 'Z_r', zeros(2,3), 'Z_l', zeros(2,2), 'J_z2rl',  zeros(2,5)));% , 'E_s', [] , 'Z_s', [] */
  /*  %     % Obs = repmat(Obs, [1 Opt.map.numLmks]); */
  /*  %     % %% end of preallocation */
  /*  %     % */
  /*  %     % for sen = numel(Sen):numel(Sen) */
  /*  %     % % for sen = [1 2 4] */
  /* % 'sid', 0,...      'lid', [],...'stype', 'proxHor', */
  /*  , 'E_s', [] , 'Z_s', [] */
  for (i = 0; i < 15; i++) {
    Obs[i] = r3;
  }

  for (i = 0; i < 15; i++) {
    Obs[i].lmk = 1.0 + (double)i;

    /*          S = Sen(sen); */
    /*          O.sen       = sen;        % sensor index */
    Obs[i].lmk = 1.0 + (double)i;

    /*  landmark index. */
    /*          O.sid       = S.id;       % sensor id */
    /*          O.lid       = -1;         % lmk id */
    /*          O.stype     = S.type;     % sensor type */
    /*          O.ltype     =  'horLin';         % lmk type */
    /*          O.meas.y    = zeros(4,1);         % observation */
    /*          O.meas.R    = zeros(4,4);  %S.par.pixCov; % observation cov */
    /*          O.nom.n     = [];         % expected non-observable */
    /*          O.nom.N     = [];         % expected non-observable cov */
    /*          O.exp.e     = zeros(3,1);         % expectation mean */
    /*          O.exp.E     = zeros(3,3);         % expectation cov */
    /*          O.exp.um    = 0;         % expectation uncertainty measure */
    /*          O.exp.E_rl_jpd=zeros(5,5);  % R */
    /*          O.inn.z     = zeros(2,1);         % innovation */
    /*          O.inn.Z     = zeros(2,2);         % innovation cov */
    /*          O.inn.iZ    = zeros(2,2);         % inverse inn. cov */
    /*          O.inn.MD2   = 0;          % Mahalanobis distance squared */
    /*          O.app.pred  = [];         % predicted appearence */
    /*          O.app.curr  = [];         % current appearence */
    /*          O.app.sc    = 0;          % match score */
    /*          O.par       = [];         % other params */
    /*          O.vis       = false;      % lmk is visible? */
    /*          O.measured  = false;      % lmk has been measured? */
    /*          O.matched   = false;      % lmk has been matched? */
    /*          O.updated   = false;      % lmk has been updated? */
    /*          O.Jac.E_r   = zeros(3,3);        % Jac of expectation wrt robot state - Obs function */
    /*          O.Jac.E_s   = [];         % Jac of expectation wrt sensor */
    /*          O.Jac.E_l   = zeros(3,2);     % Jac of expectation wrt landmark */
    /*          O.Jac.Z_r   = zeros(2,3);         % Jac of innovation wrt robot state - Inn function */
    /*          O.Jac.Z_s   = [];         % Jac of innovation wrt sensor */
    /*          O.Jac.Z_l   = zeros(2,2);         % Jac of innovation wrt landmark */
    /*          O.Jac.J_z2rl= zeros(2,5); % R */
    /*          Obs(lmk) = O; */
  }

  /*  ========== End of function - Start GPL license ========== */
  /*    # START GPL LICENSE */
  /* --------------------------------------------------------------------- */
  /*  */
  /*    This file is part of SLAMTB, a SLAM toolbox for Matlab. */
  /*  */
  /*    SLAMTB is free software: you can redistribute it and/or modify */
  /*    it under the terms of the GNU General Public License as published by */
  /*    the Free Software Foundation, either version 3 of the License, or */
  /*    (at your option) any later version. */
  /*  */
  /*    SLAMTB is distributed in the hope that it will be useful, */
  /*    but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*    GNU General Public License for more details. */
  /*  */
  /*    You should have received a copy of the GNU General Public License */
  /*    along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /* --------------------------------------------------------------------- */
  /*    SLAMTB is Copyright: */
  /*    Copyright (c) 2008-2010, Joan Sola @ LAAS-CNRS, */
  /*    Copyright (c) 2010-2013, Joan Sola, */
  /*    Copyright (c) 2014-    , Joan Sola @ IRI-UPC-CSIC, */
  /*    SLAMTB is Copyright 2009 */
  /*    by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol */
  /*    @ LAAS-CNRS. */
  /*    See on top of this file for its particular copyright. */
  /*    # END GPL LICENSE */
}

/*
 * File trailer for createObservations.c
 *
 * [EOF]
 */
