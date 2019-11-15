/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * main.c - Containing the main function.
 */



//
// Created by makeruser on 7/22/19.
//
#include "initSlam.h"
#include "stdio.h"
#include "CyclicBuffer_addProx.h"
#include "initCyclicBuffer.h"
#include "CyclicBuffer_addPos.h"
#include "CyclicBuffer_addPsi.h"
#include "upSampAndFilt_AttPos.h"
#include "stepBetweenSync.h"
#include "slamOnVehicle.h"
#include <stdlib.h>
#include <assert.h>


/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "config.h"
#include "platform.h"
#include "system.h"
#include "usec_time.h"

#include "led.h"

/* ST includes */
#include "stm32fxxx.h"


#ifndef NULL
#define NULL   ((void *) 0)
#endif

#define MAX 300


int main (int argc, char *argv[])
{

    FILE * fid_sequence;
    FILE * fid_cbaprox_ins;
    FILE * fid_cbapos_ins;
    FILE * fid_cbapsi_ins;
    FILE * fid_initPose;

    FILE * fid_cbaprox_outs;
    FILE * fid_cbapos_outs;
    FILE * fid_cbapsi_outs;
    FILE * fid_stpbsync_outs;
    FILE * fid_usafap_outs;
    FILE * fid_sov_outs;
    int stepNum = 1;

    char buf[MAX];
    char buf2[MAX];
    char * tmpAgrIn1;
    char * tmpAgrIn2;
    char * tmpAgrIn3;
    char * tmpAgrIn4;
    char * tmpAgrIn5;
    char * tmpAgrIn6;
    const char sep[2] = ",";

    // initSlam inputs:
    char setupType[7];
    Map_type Map;
    Rob_type Rob;
    Sen_type Sen[17];
    Lmk_type Lmk[15];
    Obs_type Obs[15];
    Opt_type Opt;
    Raw_type Raw[20];
    double xyRob[400];
    double pnum;
    double xyIn[2];

    //upSampAndFilt_AttPos outputs:
    boolean_T shortTimeStep;
    double highFreqTimeGrid[64];
    double wantedSyncTime_T0;
    double wantedSyncTime_T1;
    double xUpSamp_f[64];
    double yUpSamp_f[64];
    double attUpSamp_f[64];

    // stepBetweenSync outputs:
    boolean_T done;
    double RobTT;
    double RobXYT[3];
    double RobCov[9];
    Prox_type prox;

    // Synchronization buffers and configuration options:
    psiBuff_type psiBuff;
    xyBuff_type xyBuff;
    proxBuff_type proxBuff[16];
    OptSync_type OptSync;

    // slamOnVehicle outputs:
    boolean_T LowFreqCycle;
    double RobPoseUpd[3];
    double RobCovUpd[9];
    double LmkIds[20];
    double LmkEndps[80];
    double LmkActions[20];
    struct1_T Metry;

    uint64_t begin_time[10000];
    uint64_t end_time[10000];
    boolean_T isLowFreqCyc[10000];
    int num_clks=0;
    // Call initialization functions:
    initSlam(setupType, &Map, &Rob,Sen,  Lmk,  Obs,  &Opt, Raw, xyRob, &pnum);
    initCyclicBuffer(&psiBuff, &xyBuff, proxBuff, &OptSync);

    // Load initial Position and update states:
    fid_initPose = fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/initial_pose.txt","r");
    fgets(buf, MAX, fid_initPose);
    tmpAgrIn1 = strtok(buf, sep);
    tmpAgrIn2 = strtok(NULL, sep);
    tmpAgrIn3 = strtok(NULL, sep);
    Map.x[0]=atof(tmpAgrIn1);
    Map.x[1]=atof(tmpAgrIn2);
    Map.x[2]=atof(tmpAgrIn3);
    Rob.state.x[0]=Map.x[0];
    Rob.state.x[1]=Map.x[1];
    Rob.state.x[2]=Map.x[2];
    fclose(fid_initPose);

    initUsecTimer();


    // Open input files:
    fid_sequence =      fopen ("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/fid_sequence_log.txt", "r");
    fid_cbaprox_ins =   fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/cbaProx_ins.txt","r");
    fid_cbapos_ins =    fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/cbaPos_ins.txt","r");
    fid_cbapsi_ins =    fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/cbaPsi_ins.txt","r");

    // Prepare output files:
    fid_cbaprox_outs =  fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/cbaProx_outs_new.txt","w");
    fid_cbapos_outs =   fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/cbaPos_outs_new.txt","w");
    fid_cbapsi_outs =   fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/cbaPsi_outs_new.txt","w");
    fid_stpbsync_outs = fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/stepBsync_outs_new.txt","w");
    fid_usafap_outs =   fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/usafap_outs_new.txt","w");
    fid_sov_outs =      fopen("/home/bitcraze/projects/crazyflie-firmware/src/slam/unitTestFiles/sov_outs_new.txt","w");


    // fprintf(fid_sequence, "%s %s %s %d", "We", "are", "in", 2019);


    while (!feof(fid_sequence)) {
        fgets(buf, MAX, fid_sequence);

        if (strstr(buf, "CyclicBuffer_addProx") != NULL) {
            fgets(buf2, MAX, fid_cbaprox_ins);
            tmpAgrIn1 = strtok(buf2, sep);if (tmpAgrIn1 == NULL) {break;}
            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
            //printf("add Prox\n");
            CyclicBuffer_addProx(proxBuff, atof(tmpAgrIn2), atof(tmpAgrIn3), atof(tmpAgrIn4), atof(tmpAgrIn5),
                                 &OptSync);
        } else if (strstr(buf, "CyclicBuffer_addPos") != NULL) {
            fgets(buf2, MAX, fid_cbapos_ins);
            tmpAgrIn1 = strtok(buf2, sep);if (tmpAgrIn1 == NULL) {break;}
            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
            tmpAgrIn6 = strtok(NULL, sep);if (tmpAgrIn6 == NULL) {break;}
            //printf("add Pos\n");
            xyIn[0]=atof(tmpAgrIn3);
            xyIn[1]=atof(tmpAgrIn4);
//            boolean_T CyclicBuffer_addPos(xyBuff_type *buff, double elemNum, const double
//            Val[2], double numDim, double b_TT, const OptSync_type *OptSync)
            CyclicBuffer_addPos(&xyBuff, atof(tmpAgrIn2), xyIn, atof(tmpAgrIn5), atof(tmpAgrIn6),
                                 &OptSync);
        } else if (strstr(buf, "CyclicBuffer_addPsi") != NULL) {
            fgets(buf2, MAX, fid_cbapsi_ins);
            tmpAgrIn1 = strtok(buf2, sep);if (tmpAgrIn1 == NULL) {break;}
            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
            //printf("add Psi\n");
            CyclicBuffer_addPsi(&psiBuff, atof(tmpAgrIn2), atof(tmpAgrIn3), atof(tmpAgrIn4), atof(tmpAgrIn5),
                                &OptSync);
        } else if (strstr(buf, "upSampAndFilt_AttPos") != NULL) {
            //printf("Att Pos\n");
            //void upSampAndFilt_AttPos(const psiBuff_type *psiBuff, const xyBuff_type *xyBuff,
            //                          const OptSync_type *OptSync, boolean_T *shortTimeStep, double
            //                          highFreqTimeGrid[64], double *wantedSyncTime_T0, double *wantedSyncTime_T1,
            //                          double xUpSamp_f[64], double yUpSamp_f[64], double attUpSamp_f[64])
            upSampAndFilt_AttPos(&psiBuff, &xyBuff, &OptSync, &shortTimeStep, highFreqTimeGrid, &wantedSyncTime_T0,
                                 &wantedSyncTime_T1, xUpSamp_f, yUpSamp_f, attUpSamp_f);
            fprintf(fid_usafap_outs, "%d, ", stepNum);
            fprintf(fid_usafap_outs, "%d, ",shortTimeStep);
            for( int i=0; i<64; i++) {
                fprintf(fid_usafap_outs, "%d, ", (int)highFreqTimeGrid[i]);
            }
            fprintf(fid_usafap_outs, "%d, ",(int)wantedSyncTime_T0);
            fprintf(fid_usafap_outs, "%d, ",(int)wantedSyncTime_T1);

            for( int i=0; i<64; i++) {
                fprintf(fid_usafap_outs, "%.16f, ", xUpSamp_f[i]);
            }
            for( int i=0; i<64; i++) {
                fprintf(fid_usafap_outs, "%.16f, ", yUpSamp_f[i]);
            }
            for( int i=0; i<64; i++) {
                fprintf(fid_usafap_outs, "%.16f, ", attUpSamp_f[i]);
            }
            fprintf(fid_usafap_outs, "\n");
        } else if (strstr(buf, "stepBetweenSync") != NULL) {
            //printf("step between sync\n");
            //void stepBetweenSync(const proxBuff_type proxBuff[16], const double
            //highFreqTimeGrid[64], double wantedSyncTime_T0, double
            //                     wantedSyncTime_T1, const double xUpSamp_f[64], const double
            //                     yUpSamp_f[64], const double attUpSamp_f[64], const
            //                     OptSync_type *OptSync, boolean_T *done, double *RobTT,
            //                     double RobXYT[3], double RobCov[9], Prox_type *prox)

            stepBetweenSync(proxBuff, highFreqTimeGrid, wantedSyncTime_T0, wantedSyncTime_T1,
                    xUpSamp_f, yUpSamp_f, attUpSamp_f, &OptSync, &done, &RobTT, RobXYT, RobCov, &prox);

            fprintf(fid_stpbsync_outs, "%d, ", (int)stepNum);
            fprintf(fid_stpbsync_outs, "%d, ",(int)done);
            fprintf(fid_stpbsync_outs, "%d, ",(int)RobTT);
            fprintf(fid_stpbsync_outs, "%.16f, ",RobXYT[0]);
            fprintf(fid_stpbsync_outs, "%.16f, ",RobXYT[1]);
            fprintf(fid_stpbsync_outs, "%.16f, ",RobXYT[2]);
            for (int i=0; i<9; i++) {
                fprintf(fid_stpbsync_outs, "%.16f, ", RobCov[i]);
            }
            for (int i=0; i<16; i++) {
                fprintf(fid_stpbsync_outs, "%.16f, ", prox.TT[i]);
                fprintf(fid_stpbsync_outs, "%.16f, ", prox.val[i]);
            }
            fprintf(fid_stpbsync_outs, "\n");


        } else if (strstr(buf, "slamOnVehicle") != NULL) {
            //printf("slamOnVehicle\n");
            //void slamOnVehicle(double RobTT, const double RobXYT[3], const double RobCov[9],
            //                   const Prox_type *prox, Map_type *Map, Rob_type *Rob, const
            //                   Sen_type Sen[17], Lmk_type Lmk[150], Obs_type Obs[150], const
            //                   Opt_type *Opt, Raw_type Raw[20], double xyRob[400], double
            //                   *pnum, boolean_T *LowFreqCycle, double RobPoseUpd[3], double
            //                   RobCovUpd[9], double LmkIds[20], double LmkEndps[80], double
            //                   LmkActions[20], struct0_T *Metry)
            begin_time[num_clks]=usecTimestamp();
            slamOnVehicle(RobTT, RobXYT, RobCov,
                           &prox, &Map, &Rob, Sen, Lmk, Obs, &Opt, Raw, xyRob, &pnum, &LowFreqCycle,
                           RobPoseUpd, RobCovUpd, LmkIds, LmkEndps, LmkActions, &Metry);
            end_time[num_clks]=usecTimestamp();
            isLowFreqCyc[num_clks] = LowFreqCycle;
            num_clks++;
            fprintf(fid_sov_outs, "%d, ", stepNum);
            fprintf(fid_sov_outs, "%d, ", LowFreqCycle);
            fprintf(fid_sov_outs, "%.16f, %.16f, %.16f, ",RobPoseUpd[0],RobPoseUpd[1],RobPoseUpd[2]);
            for (int i=0; i<9; i++) {
                fprintf(fid_sov_outs, "%.16f, ", RobCovUpd[i]);
            }
            fprintf(fid_sov_outs, "\n");

        } else {
            assert(false);
        }
        stepNum++;
        // printf("string is: %s\n", buf);


    }
    fclose(fid_sequence);
    fclose(fid_cbaprox_ins);
    fclose(fid_cbapos_ins);
    fclose(fid_cbapsi_ins);

    fclose(fid_cbaprox_outs);
    fclose(fid_cbapos_outs);
    fclose(fid_cbapsi_outs);
    fclose(fid_stpbsync_outs);
    fclose(fid_usafap_outs);
    fclose(fid_sov_outs);

    for (int i=0; i<num_clks; i++){
        printf("dtime=%f", 1000*(double)(end_time[i]-begin_time[i]));
        if (isLowFreqCyc[i]) {
            printf(" ---\n");
        }else{
            printf("\n");
        }
    }

    return 0;
}

