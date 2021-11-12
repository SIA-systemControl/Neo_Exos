//
// Created by yc on 2021/6/24.
//

#ifndef MATRIX_DYNAMICS_TRAJ_GENERATE_H
#define MATRIX_DYNAMICS_TRAJ_GENERATE_H

/**
 * Generate position fitting of normal walking
 * @param GaitCycle  the percentage of Gait Cycle := mod(t,P)/P, where P is the time of whole cycle and t is the current time.
 * @param a Fourier params - coeff of cosine
 * @param b Fourier params - coeff of sine
 * @param w Fourier params - base frequency
 * @param q0 Fourier params - offset
 * @return
 */
double base_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0);

double differentia_1st_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P);

double differentia_2ed_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P);

double differentia_3rd_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P);

double differentia_4th_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0, double P);

double sineWave(int curve_count, double freq, double amp);

double cosineWave(int curve_count, double freq, double amp);

/**
 * For identification setting
 */

double identify_position(int curve_count, double a0, const double* a, const double* b ,double w);

double identify_velocity(int curve_count, double a0, const double* a, const double* b ,double w);

double identify_acceleration(int curve_count, double a0, const double* a, const double* b ,double w);

/**
 * for dynamics verification
 */

double identify_3rd(int curve_count, double a0, const double* a, const double* b ,double w);

double identify_4th(int curve_count, double a0, const double* a, const double* b ,double w);



#endif //MATRIX_DYNAMICS_TRAJ_GENERATE_H
