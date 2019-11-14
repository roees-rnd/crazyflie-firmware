/*
 * File: initSlam.c
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
#include "createObservations.h"
#include "initRobots.h"
#include "createLandmarks.h"

/* Function Definitions */

/*
 * global Map
 *  part of userDataLinHor + createSlamStructures
 * Arguments    : const char setupType[7]
 *                struct0_T *Map
 *                Rob_type *Rob
 *                Sen_type Sen[17]
 *                Lmk_type Lmk[15]
 *                Obs_type Obs[15]
 *                Opt_type *Opt
 *                Raw_type Raw[20]
 *                double xyRob[400]
 *                double *pnum
 * Return Type  : void
 */
void initSlam(const char setupType[7], Map_type *Map, Rob_type *Rob, Sen_type
              Sen[17], Lmk_type Lmk[15], Obs_type Obs[15], Opt_type *Opt,
              Raw_type Raw[20], double xyRob[400], double *pnum)
{
  static Opt_type r1 = { { 15.0,       /* numLmks */
      2.0                              /* lmkSize */
    },                                 /* map */
    { 50.0,                            /* nUpdates */
      9.0,                             /* MD2th */
      0.1,                             /* linTestIdp */
      0.766,                           /* isVisOrtThrsh */
      { 150.0, 400.0 },                /* isVisFarThrsh */
      { { 'o', 'r', 't', 'D', 's', 't' },/* innType */
        false                          /* extPolicy */
      }                                /* lines */
    },                                 /* correct */
    { { 1.0, 1.0 },                    /* nbrInits */
      { 'h', 'o', 'r', 'L', 'i', 'n' },/* initType */
      { 10.0                           /* minLength */
      }                                /* lines */
    },                                 /* init */
    18000.0,                           /* maxDistVis */
    1.0,                               /* SF_mm2m */
    0.02,                              /* minCmd_x */
    0.017,                             /* minCmd_t */
    20.0,                              /* pntVicThresh */
    3.0,                               /* numRefreshMap */
    70.0,                              /* NSamplesPseudoscan */
    100.0,                             /* maxGap */
    10.0,                              /* stepsToSendMap */
    5.0,                               /* stepsToMerge */
    50.0,                              /* segMinLenForInit */
    4600.0,                            /* segMaxDistForInit */
    { 12.0,                            /* MIN_NUM_OF_PNTS_ON_LINE */
      25.0,                            /* MIN_SEG_LEN */
      300.0,                           /* MAX_GAP */
      100.0,                           /* epsilon */
      0.5,                             /* alphaProx */
      false,                           /* onlyEndp */
      { 6.0E-6, -0.0048, 1.46 }        /* v */
    }                                  /* splitFit */
  };

  static const Sen_type rv0[17] = { { 1.0,/* sen */
      1.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      180.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 2.0,                          /* sen */
      2.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      202.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 3.0,                          /* sen */
      3.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      225.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 4.0,                          /* sen */
      4.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      247.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 5.0,                          /* sen */
      5.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      270.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 6.0,                          /* sen */
      6.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      292.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 7.0,                          /* sen */
      7.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      315.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 8.0,                          /* sen */
      8.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      337.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 9.0,                          /* sen */
      9.0,                             /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      360.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 10.0,                         /* sen */
      10.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      382.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 11.0,                         /* sen */
      11.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      405.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 12.0,                         /* sen */
      12.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      427.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 13.0,                         /* sen */
      13.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      450.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 14.0,                         /* sen */
      14.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      472.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 15.0,                         /* sen */
      15.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      495.0,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 16.0,                         /* sen */
      16.0,                            /* id */
      { 'P', 'r', 'o', 'x', 'X' },     /* name */
      { 'p', 'r', 'o', 'x', 'H', 'o', 'r' },/* type */
      1.0,                             /* robot */
      { 0.0, 0.0 },                    /* position */
      517.5,                           /* orientationDegrees */
      { 15000.0,                       /* maxRange */
        200.0,                         /* minRange */
        20.0,                          /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    }, { 17.0,                         /* sen */
      16.0,                            /* id */
      { 'p', 'd', 'S', 'e', 'n' },     /* name */
      { 'p', 's', 'e', 'u', 'd', 'o', 'S' },/* type */
      1.0,                             /* robot */
      { 15.0, 10.0 },                  /* position */
      0.0,                             /* orientationDegrees */
      { 15100.0,                       /* maxRange */
        1.0,                           /* minRange */
        0.0,                           /* stdErr */
        0.0                            /* meanErr */
      }                                /* par */
    } };

  int i;
  static const double dv0[9] = { 0.0036, 0.0, 0.0, 0.0, 0.0036, 0.0, 0.0, 0.0,
    3.0461741978670861E-6 };

  static const Raw_type r2 = { { 0.0, 0.0 },/* rhoPhiRob */
    { 0.0, 0.0, 0.0, 0.0 },            /* covRhoPhi */
    { 0.0, 0.0 },                      /* abscissas */
    0.0,                               /* nSamples */
    0.0,                               /* length */
    { 0.0, 0.0, 0.0 },                 /* robPoseFreeze */
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },/* robPoseCov */
    0.0,                               /* avgDist */
    { 'n', 'o', '_', 't', 'y', 'p', 'e' },/* type */
    { 0.0, 0.0 },                      /* rhoPhiWorld */
    { 0.0, 0.0, 0.0, 0.0 },            /* covRhoPhiWorld */
    { 0.0, 0.0, 0.0, 0.0 },            /* covRhoPhiWorldForInit */
    { 0.0, 0.0 },                      /* abscissasWorld */
    { 0.0, 0.0, 0.0, 0.0 },            /* endpWorld */
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0 },                           /* covEndpointsWorld */
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },/* J_s2r */
    0.0,                               /* updated */
    { 0.0, 0.0 }                       /* nrm */
  };

  (void)setupType;
  *Opt = r1;
  memcpy(&Sen[0], &rv0[0], 17U * sizeof(Sen_type));

  /*  NUM_LMKS=coder.Constant(NUM_LMKS_); */
  /*  Use 1e-3 for working in meters. */
  /*  Sensor = placeSensors_16(SF_mm2m,20,15000); */
  memset(&xyRob[0], 0, 400U * sizeof(double));
  *pnum = 0.0;

  /*  switch setupType */
  /*      case '2cfRPLIDAR' */
  /*          if coder.target('MATLAB') */
  /*              Sensor = placeSensors_2cf_onrplidar(SF_mm2m,40,2500); */
  /*          end */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 8; */
  /*          MIN_SEG_LEN=35; */
  /*          MAX_GAP=700; */
  /*          epsilon=50; */
  /*          NSamplesPseudoscan=60; */
  /*          NoiseSF=0.004;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 1; */
  /*      case '1cfFreeFlight_optiTrk' */
  /*          if coder.target('MATLAB') */
  /*              Sensor = placeSensors_1cf_freeflight(SF_mm2m,50,2500); */
  /*          end */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 10; */
  /*          MIN_SEG_LEN=25; */
  /*          MAX_GAP=50; */
  /*          epsilon=50; */
  /*          NSamplesPseudoscan=50; */
  /*          NoiseSF=0.5;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 1; */
  /*      case '1cfFreeFlight' */
  /*          if coder.target('MATLAB') */
  /*              Sensor = placeSensors_1cf_freeflight(SF_mm2m,40,2500); */
  /*          end */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 10; */
  /*          MIN_SEG_LEN=25; */
  /*          MAX_GAP=200; */
  /*          epsilon=100; */
  /*          NSamplesPseudoscan=150; */
  /*          NoiseSF=0.05;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 50; */
  /*          MD2th = 9; */
  /*          pntVicThresh = 20; */
  /*      case '1cfFreeFlight_cf3cf6' */
  /*          if coder.target('MATLAB') */
  /*              Sensor = placeSensors_1cf_freeflight(SF_mm2m,40,2500); */
  /*          end */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 10; */
  /*          MIN_SEG_LEN=25; */
  /*          MAX_GAP=50; */
  /*          epsilon=50; */
  /*          NSamplesPseudoscan=150; */
  /*          NoiseSF=0.05;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 5; */
  /*      case '1cfFreeFlight_lessSensorNoise' */
  /*          if coder.target('MATLAB') */
  /*              Sensor = placeSensors_1cf_freeflight(SF_mm2m,20,2500); */
  /*          end */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 10; */
  /*          MIN_SEG_LEN=25; */
  /*          MAX_GAP=100; */
  /*          epsilon=60; */
  /*          NSamplesPseudoscan=150; */
  /*          NoiseSF=0.05;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 50; */
  /*          MD2th = 9; */
  /*      case '3cfRPLIDAR_CF231' */
  /*          if coder.target('MATLAB') */
  /*              Sensor = placeSensors_3cf_onrplidar(SF_mm2m,20,2500); */
  /*          end */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 10; */
  /*          MIN_SEG_LEN=25; */
  /*          MAX_GAP=100; */
  /*          epsilon=60; */
  /*          NSamplesPseudoscan=150; */
  /*          NoiseSF=0.05;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 50; */
  /*          MD2th = 9; */
  /*      case {'3cfRPLIDAR_CF213','3cfRPLIDAR_CF213_noSync'} */
  /*          Sensor = placeSensors_3cf_213_onrplidar(SF_mm2m,20,3000); */
  /*          MIN_NUM_OF_PNTS_ON_LINE= 10; */
  /*          MIN_SEG_LEN=25; */
  /*          MAX_GAP=300; */
  /*          epsilon=250; */
  /*          NSamplesPseudoscan=90; */
  /*          NoiseSF=0.05;%0.328; */
  /*          dxStdSF = 60; */
  /*          daStdSF = 50; */
  /*          MD2th = 9; */
  /*          pntVicThresh = 10; */
  /*      case '16_sens' */
  /*  300*5; */
  /* 250; */
  /* 0.328; */
  /*      otherwise */
  /*          Sensor = placeSensors_3cf_213_onrplidar(SF_mm2m,20,3000); */
  /*          assert(false); */
  /*  end */
  /*   % (control) odo linear error model (std) */
  /*  (control) odo ang error model (std) [degrees] */
  /*  Estimation options  */
  /*  DEBUG R */
  /*     % options for the map */
  /*          % number of 3d landmarks */
  /*          % Size of landmark */
  /*     % options for lmk correction */
  /*          % max simultaneus updates - used in correctKnownLmksExperiment -> selectLmksToObserve % TODO probably just observe all the available Lmks */
  /*           % Threshold on Mahalanobis distance squared % how many standard deviations away Point is from the mean of line % TODO: assess what is the real needed threshold. */
  /*         % threshold on IDP linearity test */
  /*      % options for line corrections */
  /*     % innovation type for lines */
  /*          % line extending policy ? */
  /*     % Options for initialization */
  /*       % number of inits [firstFrame, otherFrames] */
  /*    % Type of lmk to use for init */
  /*      % lines options */
  /* % minimum segment length */
  /*  % maximum dist from lmk in order to be visible (used in isLmkVis). */
  /* %1e-4*SF_mm2m,... */
  /* 1e-6,... */
  /*  number of ladnmarks to send update for */
  /*   % adapted for coder: had to define class SizeOpt to convince matlab coder that the size is constant */
  /*  % maximum gap allowed inside a landmark (lmk to seg). */
  /*  % changed from 30 to 100 and got much better resualts!! (maybe...) */
  /*  .*[avgDistEst^2, avgDistEst, 1]*sensStd */
  /*  Create robots and controls */
  /*  Create sensors */
  /*  Rob = createRobots(Robot); */
  /*  control */
  /* [dxy_cur'; ang_vel]; */
  /*  state */
  /* [cur_pos; cur_orient]; % state */
  /* numel(Rob.state.x);   % state size */
  /*  Install sensors in robots   [Rob,Sen] = installSensors(Rob,Sen); */
  /*  Create Landmarks and non-observables */
  createLandmarks(Lmk);

  /*  Create Map - empty */
  /*  coder.const( DB DB DB */
  /*  Initialize robots and sensors in Map */
  memset(&Map->x[0], 0, 33U * sizeof(double));
  for (i = 0; i < 33; i++) {
    Map->used[i] = false;
  }

  memset(&Map->P[0], 0, 1089U * sizeof(double));
  Map->len = 33.0;
  Map->numUsed = 0.0;
  Map->lmk0 = 0.0;
  Map->lastChecked = 0.0;
  for (i = 0; i < 3; i++) {
    Rob->con.u[i] = 0.0;
  }

  memcpy(&Rob->con.U[0], &dv0[0], 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    Rob->state.x[i] = 0.0;
  }

  memset(&Rob->state.P[0], 0, 9U * sizeof(double));
  Rob->state.size = 3.0;
  for (i = 0; i < 3; i++) {
    Rob->state.r[i] = 0.0;
  }

  for (i = 0; i < 17; i++) {
    Rob->sensors[i] = 1.0 + (double)i;
  }

  initRobots(Map, Rob);

  /*  Sen = initSensors(Sen); */
  /*  Create Observations (matrix: [ line=sensor , colums=landmark ]) */
  createObservations(Obs);

  /*  DEBUG R */
  /* rd.covRhoPhi = zeros(2); */
  for (i = 0; i < 20; i++) {
    Raw[i] = r2;
  }
}

/*
 * File trailer for initSlam.c
 *
 * [EOF]
 */
