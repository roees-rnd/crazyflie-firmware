//
// Created by makeruser on 7/22/19.
//
#include "init.h"
#include "stdio.h"
#include "cf_feat_ext.h"
#include "cf_feat_ext_initialize.h"
#include <stdlib.h>
#include <assert.h>
#include <bits/time.h>
#include <time.h>

#define MAX 300

int main (int argc, char *argv[])
{

    FILE * fid_in;
    FILE * fid_out_new;

    int stepNum = 0;

    char buf[MAX];
    char buf2[MAX];
    char * tmpAgrIn1;
    char * tmpAgrIn2;
    char * tmpAgrIn3;
    char * tmpAgrIn4;
    const char sep[2] = ",";

    // initSlam inputs:
    Opt_type Opt;
    int navXY[2];
    float Psi;
    unsigned short ranges[16];
    // Outputs:
    float modelRANSAC[3];

    //void cf_feat_ext(const int navXY[2], float Psi, const unsigned short ranges[16],
    //                 const Opt_type *Opt, float modelRANSAC[3])


    clock_t begin_time[10000];
    clock_t end_time[10000];
    boolean_T isLowFreqCyc[10000];
    int num_clks=0;

    cf_feat_ext_initialize();
    // Call initialization functions:
    init(15, 200, &Opt);


    // Open input files:
    fid_in =      fopen ("/home/roees/CF/cf_feat_ext/02_Cmake/unitTestFiles/inputs.txt", "r");

    // Prepare output files:
    fid_out_new =  fopen("/home/roees/CF/cf_feat_ext/02_Cmake/unitTestFiles/outs_new.txt","w");



    // fprintf(fid_sequence, "%s %s %s %d", "We", "are", "in", 2019);


    while (!feof(fid_in)) {
        fgets(buf, MAX, fid_in);
        // nav x, y
        tmpAgrIn1 = strtok(buf, sep);if (tmpAgrIn1 == NULL) {break;}
        navXY[0]=atoi(tmpAgrIn1);
        tmpAgrIn2 = strtok(NULL, sep);if (tmpAgrIn2 == NULL) {break;}
        navXY[1]=atoi(tmpAgrIn2);
        // psi
        tmpAgrIn3 = strtok(NULL, sep);if (tmpAgrIn3 == NULL) {break;}
        Psi = atof(tmpAgrIn3);
        // ranges
        for (int i=0; i<16; i++) {
            tmpAgrIn4 = strtok(NULL, sep);
            if (tmpAgrIn4 == NULL) { break; }
            ranges[i] = atoi(tmpAgrIn4);
        }

        cf_feat_ext(navXY, Psi, ranges, &Opt, modelRANSAC);

        for (int i=0; i<3; i++) {
            fprintf(fid_out_new, "%.16f, ", modelRANSAC[i]);
        }
        fprintf(fid_out_new, "\n");

        stepNum++;
        // printf("string is: %s\n", buf);


    }
    fclose(fid_in);
    fclose(fid_out_new);

    for (int i=0; i<stepNum; i++){
        printf("dtime=%f", 1000*(double)(end_time[i]-begin_time[i])/CLOCKS_PER_SEC);
        if (isLowFreqCyc[i]) {
            printf(" ---\n");
        }else{
            printf("\n");
        }
    }

    return 0;
}
