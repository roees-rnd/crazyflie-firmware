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
#define APP_STACKSIZE 1000
#endif

#ifndef APP_PRIORITY
#define APP_PRIORITY 3
#endif

#include "ff.h"  // for reading files

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

static FIL fid_sequence;
static FIL fid_cbaprox_ins;
static FIL fid_cbapos_ins;
static FIL fid_cbapsi_ins;
static FIL fid_initPose;

static FIL fid_cbaprox_outs;
static FIL fid_cbapos_outs;
static FIL fid_cbapsi_outs;
static FIL fid_stpbsync_outs;
static FIL fid_usafap_outs;
static FIL fid_sov_outs;
static FIL fid_timing;

//
//
//// initSlam inputs:
//static TCHAR setupType[7];
//static Map_type Map;
//static Rob_type Rob;
//static Sen_type Sen[17];
//static Lmk_type Lmk[15];
//static Obs_type Obs[15];
//static Opt_type Opt;
//static Raw_type Raw[20];
//static double xyRob[400];
//static double pnum;
//static double xyIn[2];
//
////upSampAndFilt_AttPos outputs:
//static boolean_T shortTimeStep;
//static double highFreqTimeGrid[64];
//static double wantedSyncTime_T0;
//static double wantedSyncTime_T1;
//static double xUpSamp_f[64];
//static double yUpSamp_f[64];
//static double attUpSamp_f[64];
//
//// stepBetweenSync outputs:
//static boolean_T done;
//static double RobTT;
//static double RobXYT[3];
//static double RobCov[9];
//static Prox_type prox;
//
//// Synchronization buffers and configuration options:
//static psiBuff_type psiBuff;
//static xyBuff_type xyBuff;
//static proxBuff_type proxBuff[16];
//static OptSync_type OptSync;

#include "cfassert.h"
#include "sleepus.h"

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

    int stepNum = 1;

    TCHAR buf[MAX];
    TCHAR buf2[MAX];
    TCHAR * tmpAgrIn1;
    TCHAR * tmpAgrIn2;
    TCHAR * tmpAgrIn3;
    TCHAR * tmpAgrIn4;
    TCHAR * tmpAgrIn5;
    TCHAR * tmpAgrIn6;
    const char sep[2] = ",";

    // initSlam inputs:
    TCHAR setupType[7];
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
    struct0_T Metry;

    uint64_t begin_time;
    uint64_t end_time;

    // Call initialization functions:
    initSlam(setupType, &Map, &Rob,Sen,  Lmk,  Obs,  &Opt, Raw, xyRob, &pnum);
    initCyclicBuffer(&psiBuff, &xyBuff, proxBuff, &OptSync);


//    // Load initial Position and update states:
    if (f_open(&fid_initPose, "initial_pose.txt",FA_READ) == FR_OK){
    	DEBUG_PRINT("f_open initial_pose.txt success!\n");
    }else{
    	while (1)
		DEBUG_PRINT("f_open initial_pose.txt fails :-(\n");
    }
    f_gets(buf, MAX, &fid_initPose);
    tmpAgrIn1 = strtok(buf, sep);
    tmpAgrIn2 = strtok(NULL, sep);
    tmpAgrIn3 = strtok(NULL, sep);
//    DEBUG_PRINT("x,y,psi = %s, %s, %s\n", tmpAgrIn1, tmpAgrIn2, tmpAgrIn1);

    Map.x[0]=atof(tmpAgrIn1);
    Map.x[1]=atof(tmpAgrIn2);
    Map.x[2]=atof(tmpAgrIn3);
    Rob.state.x[0]=Map.x[0];
    Rob.state.x[1]=Map.x[1];
    Rob.state.x[2]=Map.x[2];
    f_close(&fid_initPose);

//    DEBUG_PRINT("mapx1,mapx2,mapx3 = %f, %f, %f\n", Map.x[0], Map.x[1], Map.x[2]);
//    initUsecTimer();


    // Open input files:
    if (f_open (&fid_sequence, "fid_sequence_log.txt", FA_READ) == FR_OK){
    	DEBUG_PRINT("f_open fid_sequence_log.txt success!\n");
    }else{
    	while (1)
		DEBUG_PRINT("f_open fid_sequence_log.txt fails :-(\n");
    }
    if (f_open(&fid_cbaprox_ins, "cbaProx_ins.txt",FA_READ) == FR_OK){
    	DEBUG_PRINT("f_open cbaProx_ins.txt success!\n");
    }else{
    	while (1)
		DEBUG_PRINT("f_open cbaProx_ins.txt fails :-(\n");
    }
    if (f_open(&fid_cbapos_ins, "cbaPos_ins.txt",FA_READ) == FR_OK){
    	DEBUG_PRINT("f_open cbaPos_ins.txt success!\n");
    }else{
    	while (1)
		DEBUG_PRINT("f_open cbaPos_ins.txt fails :-(\n");
    }
    if (f_open(&fid_cbapsi_ins, "cbaPsi_ins.txt",FA_READ) == FR_OK){
    	DEBUG_PRINT("f_open cbaPsi_ins.txt success!\n");
    }else{
    	while (1)
		DEBUG_PRINT("f_open cbaPsi_ins.txt fails :-(\n");
    }



    // Prepare output files:
    if (f_open(&fid_cbaprox_outs, "cbaProx_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
    	DEBUG_PRINT("f_open cbaProx_outs_new.txt success!\n");
    }else{
    	while (1)
    	DEBUG_PRINT("f_open cbaProx_outs_new.txt fails :-(\n");

    }
    if (f_open(&fid_cbapos_outs, "cbaPos_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
		DEBUG_PRINT("f_open cbaPos_outs_new.txt success!\n");
	}else{
		while (1)
		DEBUG_PRINT("f_open cbaPos_outs_new.txt fails :-(\n");
	}
    if (f_open(&fid_cbapsi_outs, "cbaPsi_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
		DEBUG_PRINT("f_open cbaPsi_outs_new.txt success!\n");
	}else{
		DEBUG_PRINT("f_open cbaPsi_outs_new.txt fails :-(\n");
		while (1);
	}
    if (f_open(&fid_stpbsync_outs, "stepBsync_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
		DEBUG_PRINT("f_open stepBsync_outs_new.txt success!\n");
	}else{
		DEBUG_PRINT("f_open stepBsync_outs_new.txt fails :-(\n");
		while (1);
	}
    if (f_open(&fid_usafap_outs, "usafap_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
		DEBUG_PRINT("f_open usafap_outs_new.txt success!\n");
	}else{
		DEBUG_PRINT("f_open usafap_outs_new.txt fails :-(\n");
		while (1);
	}
    if (f_open(&fid_sov_outs, "sov_outs_new.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
		DEBUG_PRINT("f_open sov_outs_new.txt success!\n");
	}else{
		DEBUG_PRINT("f_open sov_outs_new.txt fails :-(\n");
		while (1);
	}
    if (f_open(&fid_timing, "timing_sov.txt",  FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
		DEBUG_PRINT("f_open timing_sov.txt success!\n");
	}else{
		DEBUG_PRINT("f_open timing_sov.txt fails :-(\n");
		while (1);
	}




    // f_printf(&fid_sequence, "%s %s %s %d", "We", "are", "in", 2019);


    while (!f_eof(&fid_sequence)) {
    	f_gets(buf, MAX, &fid_sequence);
    	DEBUG_PRINT("While statement begin\n");

        if (strstr(buf, "CyclicBuffer_addProx") != NULL) {
        	DEBUG_PRINT("Enter CyclicBuffer_addProx\n");
            f_gets(buf2, MAX, &fid_cbaprox_ins);
//            tmpAgrIn1 = strtok((char *)buf2, sep);if (tmpAgrIn1 == NULL) {break;}
//            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
//            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
//            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
//            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
//            CyclicBuffer_addProx(proxBuff, atof(tmpAgrIn2), atof(tmpAgrIn3), atof(tmpAgrIn4), atof(tmpAgrIn5),
//                                 &OptSync);
        } else if (strstr(buf, "CyclicBuffer_addPos") != NULL) {
        	DEBUG_PRINT("Enter CyclicBuffer_addPos\n");
            f_gets(buf2, MAX, &fid_cbapos_ins);
//            tmpAgrIn1 = strtok((char *)buf2, sep);if (tmpAgrIn1 == NULL) {break;}
//            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
//            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
//            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
//            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}
//            tmpAgrIn6 = strtok(NULL, sep);if (tmpAgrIn6 == NULL) {break;}
//            xyIn[0]=atof(tmpAgrIn3);
//            xyIn[1]=atof(tmpAgrIn4);
////            boolean_T CyclicBuffer_addPos(xyBuff_type *buff, double elemNum, const double
//            Val[2], double numDim, double b_TT, const OptSync_type *OptSync)
//            CyclicBuffer_addPos(&xyBuff, atof(tmpAgrIn2), xyIn, atof(tmpAgrIn5), atof(tmpAgrIn6),
//                                 &OptSync);
        } else if (strstr(buf, "CyclicBuffer_addPsi") != NULL) {
        	DEBUG_PRINT("Enter CyclicBuffer_addPsi\n");
            f_gets(buf2, MAX, &fid_cbapsi_ins);
//            tmpAgrIn1 = strtok((char *)buf2, sep);if (tmpAgrIn1 == NULL) {break;}
//            tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
//            tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
//            tmpAgrIn4 = strtok(NULL, sep);if (tmpAgrIn4 == NULL) {break;}
//            tmpAgrIn5 = strtok(NULL, sep);if (tmpAgrIn5 == NULL) {break;}

//            CyclicBuffer_addPsi(&psiBuff, atof(tmpAgrIn2), atof(tmpAgrIn3), atof(tmpAgrIn4), atof(tmpAgrIn5),
//                                &OptSync);
        } else if (strstr(buf, "upSampAndFilt_AttPos") != NULL) {
        	DEBUG_PRINT("Enter upSampAndFilt_AttPos\n");
            //printf("Att Pos\n");
            //void upSampAndFilt_AttPos(const psiBuff_type *psiBuff, const xyBuff_type *xyBuff,
            //                          const OptSync_type *OptSync, boolean_T *shortTimeStep, double
            //                          highFreqTimeGrid[64], double *wantedSyncTime_T0, double *wantedSyncTime_T1,
            //                          double xUpSamp_f[64], double yUpSamp_f[64], double attUpSamp_f[64])
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
        } else if (strstr(buf, "stepBetweenSync") != NULL) {
        	DEBUG_PRINT("Enter stepBetweenSync\n");

            //printf("step between sync\n");
            //void stepBetweenSync(const proxBuff_type proxBuff[16], const double
            //highFreqTimeGrid[64], double wantedSyncTime_T0, double
            //                     wantedSyncTime_T1, const double xUpSamp_f[64], const double
            //                     yUpSamp_f[64], const double attUpSamp_f[64], const
            //                     OptSync_type *OptSync, boolean_T *done, double *RobTT,
            //                     double RobXYT[3], double RobCov[9], Prox_type *prox)

//            stepBetweenSync(proxBuff, highFreqTimeGrid, wantedSyncTime_T0, wantedSyncTime_T1,
//                    xUpSamp_f, yUpSamp_f, attUpSamp_f, &OptSync, &done, &RobTT, RobXYT, RobCov, &prox);

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


        } else if (strstr(buf, "slamOnVehicle") != NULL) {
        	DEBUG_PRINT("Enter slamOnVehicle\n");

            //printf("slamOnVehicle\n");
            //void slamOnVehicle(double RobTT, const double RobXYT[3], const double RobCov[9],
            //                   const Prox_type *prox, Map_type *Map, Rob_type *Rob, const
            //                   Sen_type Sen[17], Lmk_type Lmk[150], Obs_type Obs[150], const
            //                   Opt_type *Opt, Raw_type Raw[20], double xyRob[400], double
            //                   *pnum, boolean_T *LowFreqCycle, double RobPoseUpd[3], double
            //                   RobCovUpd[9], double LmkIds[20], double LmkEndps[80], double
            //                   LmkActions[20], struct0_T *Metry)
//            begin_time=usecTimestamp();
//            slamOnVehicle(RobTT, RobXYT, RobCov,
//                           &prox, &Map, &Rob, Sen, Lmk, Obs, &Opt, Raw, xyRob, &pnum, &LowFreqCycle,
//                           RobPoseUpd, RobCovUpd, LmkIds, LmkEndps, LmkActions, &Metry);
//            end_time=usecTimestamp();
//            if (LowFreqCycle){
//            	f_printf(&fid_timing, "dtime=%f", 1000*(double)(end_time-begin_time));
//            }else{
//            	f_printf(&fid_timing, "dtime=%f  ---", 1000*(double)(end_time-begin_time));
//            }
//
//            f_printf(&fid_sov_outs, "%d, ", stepNum);
//            f_printf(&fid_sov_outs, "%d, ", LowFreqCycle);
//            f_printf(&fid_sov_outs, "%.16f, %.16f, %.16f, ",RobPoseUpd[0],RobPoseUpd[1],RobPoseUpd[2]);
//            for (int i=0; i<9; i++) {
//                f_printf(&fid_sov_outs, "%.16f, ", RobCovUpd[i]);
//            }
//            f_printf(&fid_sov_outs, "\n");

        } else {  // Not supposed to  get here
        	DEBUG_PRINT("Reached unreachable place ?!?\n");
        	break;
        }
        stepNum++;
        // printf("string is: %s\n", buf);


    }
    f_close(&fid_sequence);
    f_close(&fid_cbaprox_ins);
    f_close(&fid_cbapos_ins);
    f_close(&fid_cbapsi_ins);

    f_close(&fid_cbaprox_outs);
    f_close(&fid_cbapos_outs);
    f_close(&fid_cbapsi_outs);
    f_close(&fid_stpbsync_outs);
    f_close(&fid_usafap_outs);
    f_close(&fid_sov_outs);
    f_close(&fid_timing);


    while (1){
    	vTaskDelay(M2T(100));
    	DEBUG_PRINT("Unit test finished...\n");
    }

    return;
}

