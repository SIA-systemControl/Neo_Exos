//
// Created by yc on 2021/12/1.
//
#include "calibration.h"
#include <cstdlib>
#include <cstdio>

int calibration_vel = 0;
extern bool FLG_pthread_online;

bool FLG_Calibration_Phase_End = false;
bool FLG_Calibration_ZeroPos = false;

// [0 0 0 0 0 1]

int Calibration_pos[6] = {1, 0, 0, 0, 0, 0};

void *CalibrationControl(void *arg) {
    Calibration_Key_Detect();
    exit(EXIT_SUCCESS);
}

void Calibration_Key_Detect() {
    while (FLG_pthread_online) {
        switch (getchar()) {
            case 'a':
                calibration_vel += 50;
                break;
            case 'd':
                calibration_vel -= 50;
                break;
            case 'w':
                ArrayShift(Calibration_pos, 6, 5);
                break;
            case 's':
                ArrayShift(Calibration_pos, 6, 1);
                break;
            case 'r':
                calibration_vel = 0;
                break;
            case 'q':
                FLG_Calibration_Phase_End = true;
                break;
            case 'z':
                FLG_Calibration_ZeroPos = true;
                break;

            default:
                break;
        }
    }
}

void ArrayShift(int a[], int n, int m) {
    // n -->  size of array
    // m -->  shift step

    int array_tmp[6];

    if (m >= n)
        m -= n;

    for (int i = 0; i < m; i++)
        array_tmp[i] = a[n - m + i];
    for (int i = 0; i < n - m; i++)
        array_tmp[i + m] = a[i];
    for (int i = 0; i < n; i++)
        a[i] = array_tmp[i];
}



