//
// Created by yc on 2021/6/24.
//
#include <cmath>
#include "traj_generate.h"

/**
 * For identify:
 *      Ts = 0.005
 * For Tracking:
 *      Ts = 0.001
 */
#define Ts 0.001

double base_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0) {
    double sum = 0;
    for (int i = 0; i < 8; i++)
        sum += a[i] * cos((i + 1) * GaitCycle * w) + b[i] * sin((i + 1) * GaitCycle * w);
    sum += q0;
    return sum;
}

double differential_1st_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P) {
    double sum = 0;
    for (int i = 0; i < 8; i++)
        sum += -((i + 1) * w / (P / 1000)) * a[i] * sin((i + 1) * GaitCycle * w) +
               ((i + 1) * w / (P / 1000)) * b[i] * cos((i + 1) * GaitCycle * w);
    return sum;
}

double differential_2ed_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P) {
    double sum = 0;
    for (int i = 0; i < 8; i++)
        sum += -((i + 1) * w / (P / 1000)) * ((i + 1) * w / (P / 1000)) * a[i] * cos((i + 1) * GaitCycle * w) +
               -((i + 1) * w / (P / 1000)) * ((i + 1) * w / (P / 1000)) * b[i] * sin((i + 1) * GaitCycle * w);
    return sum;
}

double differential_3rd_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P) {
    double sum = 0;
    for (int i = 0; i < 8; i++)
        sum += pow((i + 1) * w / (P / 1000), 3) * a[i] * sin((i + 1) * GaitCycle * w) +
               -pow((i + 1) * w / (P / 1000), 3) * b[i] * cos((i + 1) * GaitCycle * w);
    return sum;
}

double differential_4th_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P) {
    double sum = 0;
    for (int i = 0; i < 8; i++)
        sum += pow((i + 1) * w / (P / 1000), 4) * a[i] * cos((i + 1) * GaitCycle * w) +
               +pow((i + 1) * w / (P / 1000), 4) * b[i] * sin((i + 1) * GaitCycle * w);
    return sum;
}

double sineWave(int curve_count, double freq, double amp) {
    return amp * (sin(2 * 3.1415926 * freq * curve_count / 1000.0));
}

double cosineWave(int curve_count, double freq, double amp) {
    return amp * (cos(2 * 3.1415926 * freq * curve_count / 1000.0));
}

double identify_position(int curve_count, double a0, const double* a, const double* b ,double w){
    double sum = 0;
    for(int i=0; i<10;i++){
        sum += a[i]/(w*(i+1))*sin((i+1)*curve_count*Ts*w) - b[i]/(w*(i+1))*cos((i+1)*curve_count*Ts*w);
    }
    return sum + a0;
}

double identify_velocity(int curve_count, double a0, const double* a, const double* b ,double w){
    double sum = 0;
    for(int i=0; i<10;i++){
        sum += a[i]*cos((i+1)*curve_count*Ts*w) + b[i]*sin((i+1)*curve_count*Ts*w);
    }
    return sum;
}

double identify_acceleration(int curve_count, double a0, const double* a, const double* b ,double w){
    double sum = 0;
    for(int i=0; i<10;i++){
        sum += -a[i]*(w*(i+1))*sin((i+1)*curve_count*Ts*w) + b[i]*(w*(i+1))*cos((i+1)*curve_count*Ts*w);
    }
    return sum;
}

double identify_3rd(int curve_count, double a0, const double* a, const double* b ,double w){
    double sum = 0;
    for(int i=0; i<10;i++){
        sum += -a[i]*(w*(i+1)*w*(i+1))*cos((i+1)*curve_count*Ts*w) - b[i]*(w*(i+1)*w*(i+1))*sin((i+1)*curve_count*Ts*w);
    }
    return sum;
}

double identify_4th(int curve_count, double a0, const double* a, const double* b ,double w){
    double sum = 0;
    for(int i=0; i<10;i++){
        sum += a[i]*(w*(i+1)*w*(i+1)*w*(i+1))*sin((i+1)*curve_count*Ts*w) - b[i]*(w*(i+1)*w*(i+1)*w*(i+1))*cos((i+1)*curve_count*Ts*w);
    }
    return sum;
}

