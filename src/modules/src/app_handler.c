/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2019, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* app_handler.c: App layer handling function implementation */

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"

#include "app.h"

#ifndef APP_STACKSIZE
#define APP_STACKSIZE 500
#endif

#ifndef APP_PRIORITY
#define APP_PRIORITY 3
#endif

#include "ff.h"  // for reading files

//
////
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

#include "debug.h"



#define MAX 300

//static FIL fid_sequence;
//static FIL fid_cbaprox_ins;
//static FIL fid_cbapos_ins;
//static FIL fid_cbapsi_ins;
static FIL fid_initPose;

static FIL fid_cbaprox_outs;
//static FIL fid_cbapos_outs;
//static FIL fid_cbapsi_outs;
//static FIL fid_stpbsync_outs;
//static FIL fid_usafap_outs;
//static FIL fid_sov_outs;
//static FIL fid_timing;

#include "cfassert.h"
#include "sleepus.h"
//static FATFS FatFs;

static bool isInit = false;

static void appTask(void *param);

void __attribute__((weak)) appInit()
{
  if (isInit) return;

  xTaskCreate(appTask, "app", APP_STACKSIZE, NULL,
              APP_PRIORITY, NULL);
  isInit = true;

}

static void appTask(void *param)
{
  systemWaitStart();

  appMain();

  while(1) {
    vTaskDelay(portMAX_DELAY);
  }
}



void appMain()
{

	DEBUG_PRINT("running appMain function!\n");

//    int stepNum = 1;

    TCHAR buf[MAX];
//    TCHAR buf2[MAX];
//    TCHAR * tmpAgrIn1;
//    TCHAR * tmpAgrIn2;
//    TCHAR * tmpAgrIn3;
//    TCHAR * tmpAgrIn4;
//    TCHAR * tmpAgrIn5;
//    TCHAR * tmpAgrIn6;
//    const char sep[2] = ",";

    // initSlam inputs:
//    TCHAR setupType[7];
//    Map_type Map;
//    Rob_type Rob;
//    Sen_type Sen[17];
//    Lmk_type Lmk[15];
//    Obs_type Obs[15];
//    Opt_type Opt;
//    Raw_type Raw[20];
//    double xyRob[400];
//    double pnum;
//    double xyIn[2];

    //upSampAndFilt_AttPos outputs:
//    boolean_T shortTimeStep;
//    double highFreqTimeGrid[64];
//    double wantedSyncTime_T0;
//    double wantedSyncTime_T1;
//    double xUpSamp_f[64];
//    double yUpSamp_f[64];
//    double attUpSamp_f[64];

    // stepBetweenSync outputs:
//    boolean_T done;
//    double RobTT;
//    double RobXYT[3];
//    double RobCov[9];
//    Prox_type prox;

    // Synchronization buffers and configuration options:
//    psiBuff_type psiBuff;
//    xyBuff_type xyBuff;
//    proxBuff_type proxBuff[16];
//    OptSync_type OptSync;

    // slamOnVehicle outputs:
//    boolean_T LowFreqCycle;
//    double RobPoseUpd[3];
//    double RobCovUpd[9];
//    double LmkIds[20];
//    double LmkEndps[80];
//    double LmkActions[20];
//    struct1_T Metry;

//    uint64_t begin_time[10000];
//    uint64_t end_time[10000];
//    boolean_T isLowFreqCyc[10000];
//    int num_clks=0;
//    // Call initialization functions:
//    initSlam(setupType, &Map, &Rob,Sen,  Lmk,  Obs,  &Opt, Raw, xyRob, &pnum);
//    initCyclicBuffer(&psiBuff, &xyBuff, proxBuff, &OptSync);
//
////    /* try to mount drives before creating the tasks */
////	if (f_mount(&FatFs, "", 1) == FR_OK) {
////		DEBUG_PRINT("-- mount SD-Card [OK].\n");
////	}else{
////		while (true){
////			DEBUG_PRINT("-- mount SD-Card [FAIL].\n");
////		}
////	}
//
//
//    // Load initial Position and update states:
    f_open(&fid_initPose, "initial_pose.txt",FA_READ);
    f_gets(buf, MAX, &fid_initPose);
//    tmpAgrIn1 = strtok(buf, sep);
//    tmpAgrIn2 = strtok(NULL, sep);
//    tmpAgrIn3 = strtok(NULL, sep);
//    Map.x[0]=atof(tmpAgrIn1);
//    Map.x[1]=atof(tmpAgrIn2);
//    Map.x[2]=atof(tmpAgrIn3);
//    Rob.state.x[0]=Map.x[0];
//    Rob.state.x[1]=Map.x[1];
//    Rob.state.x[2]=Map.x[2];
    f_close(&fid_initPose);
//
//    initUsecTimer();
//
//
//    // Open input files:
//    f_open (&fid_sequence, "fid_sequence_log.txt", FA_READ);
//    f_open(&fid_cbaprox_ins, "cbaProx_ins.txt",FA_READ);
//    f_open(&fid_cbapos_ins, "cbaPos_ins.txt",FA_READ);
//    f_open(&fid_cbapsi_ins, "cbaPsi_ins.txt",FA_READ);
//
//    // Prepare output files:
    if (f_open(&fid_cbaprox_outs, "cbaProx_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
    	DEBUG_PRINT("f_open cbaProx_outs_new.txt success!\n");
    }else{
    	DEBUG_PRINT("f_open cbaProx_outs_new.txt fails :-(\n");
    }
//    f_open(&fid_cbapos_outs, "cbaPos_outs_new.txt",FA_WRITE);
//    f_open(&fid_cbapsi_outs, "cbaPsi_outs_new.txt",FA_WRITE);
//    f_open(&fid_stpbsync_outs, "stepBsync_outs_new.txt",FA_WRITE);
//    f_open(&fid_usafap_outs, "usafap_outs_new.txt",FA_WRITE);
//    f_open(&fid_sov_outs, "sov_outs_new.txt",FA_WRITE);
//
//    f_open(&fid_timing, "sov_outs_new.txt",FA_WRITE);
//
//
//
//    // f_printf(&fid_sequence, "%s %s %s %d", "We", "are", "in", 2019);
//
//
//    while (!f_eof(&fid_sequence)) {
//    	f_gets(buf, MAX, &fid_sequence);
//
//        if (strstr(buf, "CyclicBuffer_addProx") != NULL) {
//            f_gets(buf2, MAX, &fid_cbaprox_ins);
//            tmpAgrIn1 = strtok((char *)buf2, sep);if (tmpAgrIn1 == NULL) {break;}
//            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
//            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
//            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
//            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
//            //printf("add Prox\n");
//            CyclicBuffer_addProx(proxBuff, atof(tmpAgrIn2), atof(tmpAgrIn3), atof(tmpAgrIn4), atof(tmpAgrIn5),
//                                 &OptSync);
//        } else if (strstr(buf, "CyclicBuffer_addPos") != NULL) {
//            f_gets(buf2, MAX, &fid_cbapos_ins);
//            tmpAgrIn1 = strtok((char *)buf2, sep);if (tmpAgrIn1 == NULL) {break;}
//            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
//            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
//            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
//            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
//            tmpAgrIn6 = strtok(NULL, sep);if (tmpAgrIn6 == NULL) {break;}
//            //printf("add Pos\n");
//            xyIn[0]=atof(tmpAgrIn3);
//            xyIn[1]=atof(tmpAgrIn4);
////            boolean_T CyclicBuffer_addPos(xyBuff_type *buff, double elemNum, const double
////            Val[2], double numDim, double b_TT, const OptSync_type *OptSync)
//            CyclicBuffer_addPos(&xyBuff, atof(tmpAgrIn2), xyIn, atof(tmpAgrIn5), atof(tmpAgrIn6),
//                                 &OptSync);
//        } else if (strstr(buf, "CyclicBuffer_addPsi") != NULL) {
//            f_gets(buf2, MAX, &fid_cbapsi_ins);
//            tmpAgrIn1 = strtok((char *)buf2, sep);if (tmpAgrIn1 == NULL) {break;}
//            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
//            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
//            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
//            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
//            //printf("add Psi\n");
//            CyclicBuffer_addPsi(&psiBuff, atof(tmpAgrIn2), atof(tmpAgrIn3), atof(tmpAgrIn4), atof(tmpAgrIn5),
//                                &OptSync);
//        } else if (strstr(buf, "upSampAndFilt_AttPos") != NULL) {
//            //printf("Att Pos\n");
//            //void upSampAndFilt_AttPos(const psiBuff_type *psiBuff, const xyBuff_type *xyBuff,
//            //                          const OptSync_type *OptSync, boolean_T *shortTimeStep, double
//            //                          highFreqTimeGrid[64], double *wantedSyncTime_T0, double *wantedSyncTime_T1,
//            //                          double xUpSamp_f[64], double yUpSamp_f[64], double attUpSamp_f[64])
//            upSampAndFilt_AttPos(&psiBuff, &xyBuff, &OptSync, &shortTimeStep, highFreqTimeGrid, &wantedSyncTime_T0,
//                                 &wantedSyncTime_T1, xUpSamp_f, yUpSamp_f, attUpSamp_f);
//            f_printf(&fid_usafap_outs, "%d, ", stepNum);
//            f_printf(&fid_usafap_outs, "%d, ",shortTimeStep);
//            for( int i=0; i<64; i++) {
//                f_printf(&fid_usafap_outs, "%d, ", (int)highFreqTimeGrid[i]);
//            }
//            f_printf(&fid_usafap_outs, "%d, ",(int)wantedSyncTime_T0);
//            f_printf(&fid_usafap_outs, "%d, ",(int)wantedSyncTime_T1);
//
//            for( int i=0; i<64; i++) {
//                f_printf(&fid_usafap_outs, "%.16f, ", xUpSamp_f[i]);
//            }
//            for( int i=0; i<64; i++) {
//                f_printf(&fid_usafap_outs, "%.16f, ", yUpSamp_f[i]);
//            }
//            for( int i=0; i<64; i++) {
//                f_printf(&fid_usafap_outs, "%.16f, ", attUpSamp_f[i]);
//            }
//            f_printf(&fid_usafap_outs, "\n");
//        } else if (strstr(buf, "stepBetweenSync") != NULL) {
//            //printf("step between sync\n");
//            //void stepBetweenSync(const proxBuff_type proxBuff[16], const double
//            //highFreqTimeGrid[64], double wantedSyncTime_T0, double
//            //                     wantedSyncTime_T1, const double xUpSamp_f[64], const double
//            //                     yUpSamp_f[64], const double attUpSamp_f[64], const
//            //                     OptSync_type *OptSync, boolean_T *done, double *RobTT,
//            //                     double RobXYT[3], double RobCov[9], Prox_type *prox)
//
//            stepBetweenSync(proxBuff, highFreqTimeGrid, wantedSyncTime_T0, wantedSyncTime_T1,
//                    xUpSamp_f, yUpSamp_f, attUpSamp_f, &OptSync, &done, &RobTT, RobXYT, RobCov, &prox);
//
//            f_printf(&fid_stpbsync_outs, "%d, ", (int)stepNum);
//            f_printf(&fid_stpbsync_outs, "%d, ",(int)done);
//            f_printf(&fid_stpbsync_outs, "%d, ",(int)RobTT);
//            f_printf(&fid_stpbsync_outs, "%.16f, ",RobXYT[0]);
//            f_printf(&fid_stpbsync_outs, "%.16f, ",RobXYT[1]);
//            f_printf(&fid_stpbsync_outs, "%.16f, ",RobXYT[2]);
//            for (int i=0; i<9; i++) {
//                f_printf(&fid_stpbsync_outs, "%.16f, ", RobCov[i]);
//            }
//            for (int i=0; i<16; i++) {
//                f_printf(&fid_stpbsync_outs, "%.16f, ", prox.TT[i]);
//                f_printf(&fid_stpbsync_outs, "%.16f, ", prox.val[i]);
//            }
//            f_printf(&fid_stpbsync_outs, "\n");
//
//
//        } else if (strstr(buf, "slamOnVehicle") != NULL) {
//            //printf("slamOnVehicle\n");
//            //void slamOnVehicle(double RobTT, const double RobXYT[3], const double RobCov[9],
//            //                   const Prox_type *prox, Map_type *Map, Rob_type *Rob, const
//            //                   Sen_type Sen[17], Lmk_type Lmk[150], Obs_type Obs[150], const
//            //                   Opt_type *Opt, Raw_type Raw[20], double xyRob[400], double
//            //                   *pnum, boolean_T *LowFreqCycle, double RobPoseUpd[3], double
//            //                   RobCovUpd[9], double LmkIds[20], double LmkEndps[80], double
//            //                   LmkActions[20], struct0_T *Metry)
//            begin_time[num_clks]=usecTimestamp();
//            slamOnVehicle(RobTT, RobXYT, RobCov,
//                           &prox, &Map, &Rob, Sen, Lmk, Obs, &Opt, Raw, xyRob, &pnum, &LowFreqCycle,
//                           RobPoseUpd, RobCovUpd, LmkIds, LmkEndps, LmkActions, &Metry);
//            end_time[num_clks]=usecTimestamp();
//            isLowFreqCyc[num_clks] = LowFreqCycle;
//            num_clks++;
//            f_printf(&fid_sov_outs, "%d, ", stepNum);
//            f_printf(&fid_sov_outs, "%d, ", LowFreqCycle);
//            f_printf(&fid_sov_outs, "%.16f, %.16f, %.16f, ",RobPoseUpd[0],RobPoseUpd[1],RobPoseUpd[2]);
//            for (int i=0; i<9; i++) {
//                f_printf(&fid_sov_outs, "%.16f, ", RobCovUpd[i]);
//            }
//            f_printf(&fid_sov_outs, "\n");
//
//        } else {
//            ASSERT(false);
//        }
//        stepNum++;
//        // printf("string is: %s\n", buf);
//
//
//    }
//    f_close(&fid_sequence);
//    f_close(&fid_cbaprox_ins);
//    f_close(&fid_cbapos_ins);
//    f_close(&fid_cbapsi_ins);
//
//    f_close(&fid_cbaprox_outs);
//    f_close(&fid_cbapos_outs);
//    f_close(&fid_cbapsi_outs);
//    f_close(&fid_stpbsync_outs);
//    f_close(&fid_usafap_outs);
//    f_close(&fid_sov_outs);
//
//    for (int i=0; i<num_clks; i++){
//        f_printf(&fid_timing, "dtime=%f", 1000*(double)(end_time[i]-begin_time[i]));
//        if (isLowFreqCyc[i]) {
//            f_printf(&fid_timing, " ---\n");
//        }else{
//            f_printf(&fid_timing, "\n");
//        }
//    }
//    f_close(&fid_timing);

    f_printf(&fid_cbaprox_outs, "buf = %s", buf);
    f_close(&fid_cbaprox_outs);

    DEBUG_PRINT("Done unit-test\n");

    while (1){
    	vTaskDelay(M2T(100));
    	DEBUG_PRINT("buf = %s\n", buf);
    }

    return;
}

