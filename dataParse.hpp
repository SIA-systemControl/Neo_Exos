//
// Created by yc on 2021/9/3.
//

#ifndef CSVREADER_DATAPARSE_H
#define CSVREADER_DATAPARSE_H

#pragma once
struct init_cnt{
    int r_hip_m;
    int r_knee_m;
    int r_ankle_m;
    int l_hip_m;
    int l_knee_m;
    int l_ankle_m;
    int r_hip_s;
    int r_knee_s;
    int r_ankle_s;
    int l_hip_s;
    int l_knee_s;
    int l_ankle_s;
};

init_cnt set_init_cnt(int data[]){
    init_cnt initCnt{};
    initCnt.r_hip_m = data[0];
    initCnt.r_hip_s = data[1];
    initCnt.r_knee_m = data[2];
    initCnt.r_knee_s = data[3];
    initCnt.r_ankle_m = data[4];
    initCnt.r_ankle_s = data[5];
    initCnt.l_hip_m = data[6];
    initCnt.l_hip_s = data[7];
    initCnt.l_knee_m = data[8];
    initCnt.l_knee_s = data[9];
    initCnt.l_ankle_m = data[10];
    initCnt.l_ankle_s = data[11];
    return initCnt;
}

#endif //CSVREADER_DATAPARSE_H
