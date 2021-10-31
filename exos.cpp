//
// Created by yc on 2021/6/25.
// Last modified on 2021/8/26
// TODO:
//  (1) summary the polarity of each encoder ( motor_side [OK] spring_side [ ] ) ---> [OK]
//  (2) Added CSP tracking mode and realize on-line modified P  ----> [OK] ( now can reach P = 1800 with some vibration )
//  (3) adjust driver FOC configuration to obtain better position control performance in CSP_tracking case
//  (4) try Noload_case with variable Period P
//  (5) solve feedback velocity noise.
//

#include <iomanip>
#include "exos.h"
#include "dataDecode.h"

//#define SineWave
#define LEFT_Part
#define asynchronous_enable

std::vector<unsigned int> error_code_sequence;


//extern volatile bool FLG_BODY_BEND; //body bend indicator
extern int P_update;

bool FLG_pthread_online = true;
bool EtherCAT_ONLINE = true;
bool POST_RESET = false;
bool LEFT_GC_START = false;
bool FLG_LEFT_GC_1st = false;
bool FLG_RIGHT_GC_1st = false;
bool FLG_RIGHT_GC_end = false;
bool FLG_IDENTIFY_STARTUP = false;
bool FLG_TRACKING_STARTUP = false;

std::ofstream left_outFile;
std::ofstream left_velFile;
std::ofstream left_save_file;

std::ofstream left_filter_file;
std::ofstream right_filter_file;

#ifdef LEFT_Part
std::ofstream right_outFile;
std::ofstream right_velFile;
std::ofstream right_save_file;
#endif

std::ofstream gaitData;

/**
 * vel_des = Kp * (q_d - q_c)
 */
PID_position l_Hip_reset_pid(700, 0, 400);
PID_position l_Knee_reset_pid(600, 0, 250);
PID_position l_Ankle_reset_pid(600, 0, 400);

PID_position r_Hip_reset_pid(700, 0, 300);
PID_position r_Knee_reset_pid(700, 0, 200); // 1500,200
PID_position r_Ankle_reset_pid(700, 0, 200);

PID_position r_Hip_s2s_pid(1400, 0, 300);
PID_position r_Knee_s2s_pid(1400, 0, 200);
PID_position r_Ankle_s2s_pid(1400, 0, 200);


PID_position r_Hip_pd(0, 0, 0);
PID_position r_Knee_pd(0, 0, 0);
PID_position r_Ankle_pd(0, 0, 0);
double *input_4 = new double[100];
double *output_4 = new double[100];
double *input_5 = new double[100];
double *output_5 = new double[100];
double *input_6 = new double[100];
double *output_6 = new double[100];
Eigen::Vector3d d4q_r, d3q_r, ddq_r, dq_r, q_r;
Eigen::Vector3d dq_e_r, q_e_r;
dynamics right_SEA_dynamics(right);

#ifdef LEFT_Part

PID_position l_Hip_pd(0, 0, 0);
PID_position l_Knee_pd(0, 0, 0);
PID_position l_Ankle_pd(0, 0, 0);

dynamics left_SEA_dynamics(left);

double *input_1 = new double[100];
double *output_1 = new double[100];
double *input_2 = new double[100];
double *output_2 = new double[100];
double *input_3 = new double[100];
double *output_3 = new double[100];
Eigen::Vector3d d4q_l, d3q_l, ddq_l, dq_l, q_l;
Eigen::Vector3d dq_e_l, q_e_l;
#endif

Eigen::Matrix3d Select_Matrix_d;

Eigen::Vector3d tau;

Eigen::IOFormat PrettyPrint(4, 0, ",", "\n", "[", "]", "[", "]");

double freq = 1;

ButterworthLP bw_link = ButterworthLP(1000, 5, 2);

static int SIG_cnt = 0;

static int Gc_cnt = 0;

Eigen::Matrix2d GaitMatrix;

static double Gc_main = 0;
static double Gc_sub = 0;
static int curve_cnt_right = 0;
static int curve_cnt_left = 0;
static int time_cnt = 0;

int task_cmd = 0;


static void SIG_handle(int sig) {
    if (sig == SIGINT) {
        SIG_cnt++;
        if (gTaskFsm.m_gtaskFSM != TaskFSM(task_cmd)) {

            std::cout << "Current taskFSM = " << gTaskFsm.m_gtaskFSM << std::endl;
            std::cout << "passed taskFSM = " << task_cmd << std::endl;

            FLG_pthread_online = false;
            EtherCAT_ONLINE = false;
        }
        POST_RESET = true;
        if (SIG_cnt == 1) {
            std::cout << "[post-reset working... switching to RESET fsm]" << std::endl;
            FLG_pthread_online = false;
        }
    }
}

void *robotcontrol(void *arg) {

    task_cmd = *(int *) arg;

    std::cout << task_cmd << std::endl;

    gSysRunning.m_gWorkStatus = sys_working_POWER_ON;
    gTaskFsm.m_gtaskFSM = task_working_RESET;

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON) {
        ActivateMaster();
        ecstate = 0;
        gSysRunning.m_gWorkStatus = sys_working_SAFE_MODE;
        std::cout << "sys_working_SAFE_MODE." << std::endl;
    }

    ecrt_master_receive(master);

    left_outFile.open("left_FourierTest.csv", std::ios::out);
    left_outFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                 << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                 << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                 << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << ','
                 << "tau-dyn-1" << ',' << "tau-dyn-2" << ',' << "tau-dyn-3" << std::endl;

    left_velFile.open("left_FourierVel.csv", std::ios::out);
    left_velFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                 << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                 << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                 << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << std::endl;

    left_save_file.open("left_save.csv", std::ios::out);
    left_save_file << "Hip desired" << ',' << "Hip actual" << ',' << "Knee desired" << ',' << "Knee actual" << ','
                   << "Ankle desired" << ',' << "Ankle actual" << ',' << "deformation of ankle" << std::endl;

    left_filter_file.open("left_filter.csv", std::ios::out);
    left_filter_file << "original" << ',' << "cpp filter" << std::endl;

#ifdef LEFT_Part
    right_outFile.open("right_FourierTest.csv", std::ios::out);
    right_outFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                  << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                  << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                  << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << ','
                  << "tau-dyn-1" << ',' << "tau-dyn-2" << ',' << "tau-dyn-3" << std::endl;

    right_velFile.open("right_FourierVel.csv", std::ios::out);
    right_velFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                  << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                  << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                  << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << std::endl;

    right_save_file.open("right_save.csv", std::ios::out);
    right_save_file << "Hip desired" << ',' << "Hip actual" << ',' << "Knee desired" << ',' << "Knee actual" << ','
                    << "Ankle desired" << ',' << "Ankle actual" << ',' << "deformation of ankle" << std::endl;

    right_filter_file.open("right_filter.csv", std::ios::out);
    right_filter_file << "original" << ',' << "cpp filter" << std::endl;
#endif

    gaitData.open("gaitData.csv", std::ios::out);

    GaitMatrix << 1, 1, 1, 1;
    gMFSM.m_gaitMatrixFsm = Stance;

    while (EtherCAT_ONLINE) {
//        tic();

        if (gSysRunning.m_gWorkStatus == sys_woring_INIT_Failed) {
            EtherCAT_ONLINE = false;
            break;
            std::cout << "Slave(s) initialized failed." << std::endl;
        }
        signal(SIGINT, SIG_handle);
        usleep(1000000 / TASK_FREQUENCY);
        cyclic_task(task_cmd);

//        std::cout << "Time elapsed (ms): " << toc_double() * 10.0 * 1000.0<< std::endl;
    }
    releaseMaster();

    if (!FLG_pthread_online)
        puts("[Press Any Key to Release Period-thread].");

    left_outFile.close();
    left_velFile.close();
    left_save_file.close();
    left_filter_file.close();

#ifdef LEFT_Part
    right_outFile.close();
    right_velFile.close();
    right_save_file.close();
#endif

    gaitData.close();

    pthread_exit(nullptr);
}

void cyclic_task(int task_cmd) {
    static int current_pos = 0;
    int raw_init_motor_pos[joint_num] = {0, 0, 0, 0, 0, 0};
    int raw_init_spring_pos[joint_num] = {0, 0, 0, 0, 0, 0};
//    bw_link.stepInitialization(0);

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
        return;

    static int cycle_count = 0;
    cycle_count++;

    ecrt_master_receive(master);
    ecrt_domain_process(domainRx);
    ecrt_domain_process(domainTx);

    check_domain_state();

    if (!(cycle_count % 500)) {
        check_master_state();
        check_slave_config_states();
    }

    switch (gSysRunning.m_gWorkStatus) {
        case sys_working_SAFE_MODE: {
            check_master_state();
            check_slave_config_states();

            if ((master_state.al_states & ETHERCAT_STATUS_OP)) {
                bool tmp = true;

                for (int i = 0; i < active_num; i++) {
                    if (!(sc_state[i].al_state & ETHERCAT_STATUS_OP)) {
                        std::cout << "slave " << i << " al_state: " << sc_state[i].al_state << std::endl;
                        tmp = false;
                        gSysRunning.m_gWorkStatus = sys_woring_INIT_Failed;
                        break;
                    }
                }
                if (tmp) {
                    ecstate = 0;
                    gSysRunning.m_gWorkStatus = sys_working_OP_MODE;
                    std::cout << "sys_working_OP_MODE" << std::endl;
                }
            }
        }
            break;

        case sys_working_OP_MODE: {
            ecstate++;
            if (SERVE_OP < active_num) {
                if (ecstate <= 10) {
                    switch (ecstate) {
                        case 1:
                            for (int i = 0; i < active_num; i++) {
                                int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                                if (E_code != 0 || (EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x0008)) {
                                    std::cout << "[Error] occured at slave: " << i << std::endl;
                                    EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i],
                                                 (EC_READ_U16(domainTx_pd + offset.status_word[i]) |
                                                  0x0080));
                                } // bit7 set to 1 to reset fault
                                EC_WRITE_U8(domainRx_pd + offset.operation_mode[i], CSP);

                            }
                            break;
                        case 7:
                            for (int i = 0; i < active_num; i++) {
                                int cur_pos = EC_READ_S32(domainTx_pd + offset.actual_position[i]);
                                EC_WRITE_S32(domainRx_pd + offset.target_position[i], cur_pos);
                                std::cout << "Axis-" << i << " current position[cnt]: " << cur_pos << std::endl;
                            }
                            break;
                        default:
                            break;
                    }
                } else {
                    for (int i = 0; i < active_num; i++) {
                        unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                        if ((cur_status & 0x4f) == 0x40)
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x06);
                        else if ((cur_status & 0x6f) == 0x21)
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x07);
                        else if ((cur_status & 0x6f) == 0x23) {
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x0F);
                            SERVE_OP++;
                        }
                    }
                }
            } else {
                int tmp = true;
                for (int i = 0; i < active_num; i++) {
                    unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                    unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x6f) == 0x27)
                        std::cout << "Slave [" << i << "] Enable operation" << std::endl;
                    else {
                        std::cout << "Slave [" << i << "] not in Enable operation" << std::endl;

                        std::cout << "Slave [" << i << "] State: " << cur_status << std::endl;
                        std::cout << "Slave [" << i << "] E-code: " << E_code << std::endl;
                        switch (E_code) {
                            case 0x7500:
                                std::cout << " Communication Error." << std::endl;
                                break;
                            case 0x7300:
                                std::cout << " Sensor Error(Maybe CRC)." << std::endl;
                                break;
                            case 0x2220:
                                std::cout << " Continuous over current" << std::endl;
                                break;
                            case 0x3331:
                                std::cout << " Field circuit interrupted" << std::endl;
                                break;
                            default:
                                std::cout << " Other Error. (ref to datasheet)" << std::endl;
                                break;
                        }
                    }


                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0) {
                        tmp = false;
                        ecstate = 0;
                        break;
                    }
                }
                if (tmp) {
                    ecstate = 0;
                    gSysRunning.m_gWorkStatus = sys_working_WORK_STATUS;
                    std::cout << "sys_working_WORK_STATUS" << std::endl;
                }
            }
        }
            break;

        default: {
            static int reset_ready_cnt = 0;
            static int id_ready_cnt = 0;

            static int PD_cnt = 0;
            static int post_reset_cnt = 0;

            static int reset_timeout = TASK_FREQUENCY * 3;

            /** Preprocess of data */

            /**
             * Encoder cnt (Raw)
             */

            int cnt_motor_1 = EC_READ_S32(domainTx_pd + offset.actual_position[r_ankle]);
            int RPM_motor_1 = EC_READ_S32(domainTx_pd + offset.actual_velocity[r_ankle]);

            int cnt_spring_1 = EC_READ_S32(domainTx_pd + offset.second_position[r_ankle]);
            int RPM_spring_1 = EC_READ_S32(domainTx_pd + offset.second_velocity[r_ankle]);

            if (cnt_spring_1 < 0)
                cnt_spring_1 += 4096;

            int cnt_motor_2 = EC_READ_S32(domainTx_pd + offset.actual_position[r_knee]);
            int RPM_motor_2 = EC_READ_S32(domainTx_pd + offset.actual_velocity[r_knee]);

            int cnt_spring_2 = EC_READ_S32(domainTx_pd + offset.second_position[r_knee]);
            int RPM_spring_2 = EC_READ_S32(domainTx_pd + offset.second_velocity[r_knee]);

            if (cnt_spring_2 < 0)
                cnt_spring_2 += 4096;

            int cnt_motor_3 = EC_READ_S32(domainTx_pd + offset.actual_position[r_hip]);
            int RPM_motor_3 = EC_READ_S32(domainTx_pd + offset.actual_velocity[r_hip]);

            int cnt_spring_3 = EC_READ_S32(domainTx_pd + offset.second_position[r_hip]);

            if (cnt_spring_3 < 0)
                cnt_spring_3 += 4096;

            int RPM_spring_3 = EC_READ_S32(domainTx_pd + offset.second_velocity[r_hip]);

            int delta_cnt_m_1 = relative_encoder_cnt(cnt_motor_1, right_ankle_init_motor_cnt);
            int delta_cnt_s_1 = relative_encoder_cnt(cnt_spring_1, right_ankle_init_spring_cnt);

            int delta_cnt_m_2 = relative_encoder_cnt(cnt_motor_2, right_knee_init_motor_cnt);
            int delta_cnt_s_2 = relative_encoder_cnt(cnt_spring_2, right_knee_init_spring_cnt);

            int delta_cnt_m_3 = relative_encoder_cnt(cnt_motor_3, right_hip_init_motor_cnt);
            int delta_cnt_s_3 = relative_encoder_cnt(cnt_spring_3, right_hip_init_spring_cnt);

#ifdef LEFT_Part
            int cnt_motor_4 = EC_READ_S32(domainTx_pd + offset.actual_position[l_hip]);
            int RPM_motor_4 = EC_READ_S32(domainTx_pd + offset.actual_velocity[l_hip]);

            int cnt_spring_4 = EC_READ_S32(domainTx_pd + offset.second_position[l_hip]);
            int RPM_spring_4 = EC_READ_S32(domainTx_pd + offset.second_velocity[l_hip]);

            if (cnt_spring_4 < 0)
                cnt_spring_4 += 4096;

            int cnt_motor_5 = EC_READ_S32(domainTx_pd + offset.actual_position[l_knee]);
            int RPM_motor_5 = EC_READ_S32(domainTx_pd + offset.actual_velocity[l_knee]);

            int cnt_spring_5 = EC_READ_S32(domainTx_pd + offset.second_position[l_knee]);
            int RPM_spring_5 = EC_READ_S32(domainTx_pd + offset.second_velocity[l_knee]);

            if (cnt_spring_5 < 0)
                cnt_spring_5 += 4096;

            int cnt_motor_6 = EC_READ_S32(domainTx_pd + offset.actual_position[l_ankle]);
            int RPM_motor_6 = EC_READ_S32(domainTx_pd + offset.actual_velocity[l_ankle]);

            int cnt_spring_6 = EC_READ_S32(domainTx_pd + offset.second_position[l_ankle]);
            int RPM_spring_6 = EC_READ_S32(domainTx_pd + offset.second_velocity[l_ankle]);

            if (cnt_spring_6 < 0)
                cnt_spring_6 += 4096;

            int delta_cnt_m_4 = relative_encoder_cnt(cnt_motor_4, left_hip_init_motor_cnt);
            int delta_cnt_s_4 = relative_encoder_cnt(cnt_spring_4, left_hip_init_spring_cnt);

            int delta_cnt_m_5 = relative_encoder_cnt(cnt_motor_5, left_knee_init_motor_cnt);
            int delta_cnt_s_5 = relative_encoder_cnt(cnt_spring_5, left_knee_init_spring_cnt);

            int delta_cnt_m_6 = relative_encoder_cnt(cnt_motor_6, left_ankle_init_motor_cnt);
            int delta_cnt_s_6 = relative_encoder_cnt(cnt_spring_6, left_ankle_init_spring_cnt);
#endif

            /**
             * Derived data
             */

            /**
             * Motor radians after transmission
             */
            double lever_arm_1_rad = pos_inc2rad_17bits(delta_cnt_m_1 / transmission_ankle);
            double lever_arm_2_rad = pos_inc2rad_17bits(delta_cnt_m_2 / transmission_knee);
            double lever_arm_3_rad = pos_inc2rad_17bits(delta_cnt_m_3 / transmission_hip);
#ifdef LEFT_Part
            double lever_arm_4_rad = pos_inc2rad_17bits(delta_cnt_m_4 / transmission_hip);
            double lever_arm_5_rad = pos_inc2rad_17bits(delta_cnt_m_5 / transmission_knee);
            double lever_arm_6_rad = pos_inc2rad_17bits(delta_cnt_m_6 / transmission_ankle);
#endif

            /**
             * Link radians ( alpha = theta - q_l ---> q_l = theta - alpha)
             * The mapping of number(1,2,3...) to joint(hip, knee, ankle)
             * ==========================================================
             *                  1 --- right_ankle
             *                  2 --- right_knee
             *                  3 --- right_hip
             * ----------------------------------------------------------
             *                  4 --- left_hip
             *                  5 --- left_knee
             *                  6 --- left_ankle
             * ==========================================================
             */

            double link_1_rad = lever_arm_1_rad - pos_inc2rad_12bits(delta_cnt_s_1);
            double link_2_rad = lever_arm_2_rad - pos_inc2rad_12bits(delta_cnt_s_2);
            double link_3_rad = lever_arm_3_rad - pos_inc2rad_12bits(delta_cnt_s_3);
#ifdef LEFT_Part
            double link_4_rad = lever_arm_4_rad - pos_inc2rad_12bits(delta_cnt_s_4);
            double link_5_rad = lever_arm_5_rad - pos_inc2rad_12bits(delta_cnt_s_5);
            double link_6_rad = lever_arm_6_rad - pos_inc2rad_12bits(delta_cnt_s_6);
#endif

            /**
             * Velocity(RPM integer based)
             */
            double lever_arm_1_vel = RPM_motor_1 / transmission_ankle;
            double lever_arm_2_vel = RPM_motor_2 / transmission_knee;
            double lever_arm_3_vel = RPM_motor_3 / transmission_hip;

            double link_1_vel = lever_arm_1_vel - RPM_spring_1;
            double link_2_vel = lever_arm_2_vel - RPM_spring_2;
            double link_3_vel = lever_arm_3_vel - RPM_spring_3;

            double r_hip_rad = link_3_rad;
            double r_hip_vel = vel_RPM2rad(link_3_vel);
            double r_knee_rad = link_2_rad;
            double r_knee_vel = vel_RPM2rad(link_2_vel);
            double r_ankle_rad = link_1_rad;
            double r_ankle_vel = vel_RPM2rad(link_1_vel);

            double r_hip_m_vel = vel_RPM2rad(lever_arm_3_vel);
            double r_knee_m_vel = vel_RPM2rad(lever_arm_2_vel);
            double r_ankle_m_vel = vel_RPM2rad(lever_arm_1_vel);

            double r_hip_s_rad = pos_inc2rad_12bits(delta_cnt_s_3);
            double r_knee_s_rad = pos_inc2rad_12bits(delta_cnt_s_2);
            double r_ankle_s_rad = pos_inc2rad_12bits(delta_cnt_s_1);

            double r_hip_m_rad = lever_arm_3_rad;
            double r_knee_m_rad = lever_arm_2_rad;
            double r_ankle_m_rad = lever_arm_1_rad;

#ifdef LEFT_Part
            double lever_arm_4_vel = RPM_motor_4 / transmission_hip;
            double lever_arm_5_vel = RPM_motor_5 / transmission_knee;
            double lever_arm_6_vel = RPM_motor_6 / transmission_ankle;

            double link_4_vel = lever_arm_4_vel - RPM_spring_4;
            double link_5_vel = lever_arm_5_vel - RPM_spring_5;
            double link_6_vel = lever_arm_6_vel - RPM_spring_6;

            double l_hip_rad = link_4_rad;
            double l_hip_vel = vel_RPM2rad(link_4_vel);
            double l_knee_rad = link_5_rad;
            double l_knee_vel = vel_RPM2rad(link_5_vel);
            double l_ankle_rad = link_6_rad;
            double l_ankle_vel = vel_RPM2rad(link_6_vel);

            double l_hip_m_vel = vel_RPM2rad(lever_arm_4_vel);
            double l_knee_m_vel = vel_RPM2rad(lever_arm_5_vel);
            double l_ankle_m_vel = vel_RPM2rad(lever_arm_6_vel);

            double l_hip_s_rad = pos_inc2rad_12bits(delta_cnt_s_4);
            double l_knee_s_rad = pos_inc2rad_12bits(delta_cnt_s_5);
            double l_ankle_s_rad = pos_inc2rad_12bits(delta_cnt_s_6);

            double l_hip_m_rad = lever_arm_4_rad;
            double l_knee_m_rad = lever_arm_5_rad;
            double l_ankle_m_rad = lever_arm_6_rad;

#endif
            /**
             * thousand ratio of rated torque
             */
            double tau_mot_1 = tau_thousand2Nm(EC_READ_S16(domainTx_pd + offset.actual_torque[r_ankle]), 0.3);
            double tau_mot_2 = tau_thousand2Nm(EC_READ_S16(domainTx_pd + offset.actual_torque[r_knee]), 0.3);
            double tau_mot_3 = tau_thousand2Nm(EC_READ_S16(domainTx_pd + offset.actual_torque[r_hip]), 0.3);
            double tau_mot_4 = tau_thousand2Nm(EC_READ_S16(domainTx_pd + offset.actual_torque[l_hip]), 0.3);
            double tau_mot_5 = tau_thousand2Nm(EC_READ_S16(domainTx_pd + offset.actual_torque[l_knee]), 0.3);
            double tau_mot_6 = tau_thousand2Nm(EC_READ_S16(domainTx_pd + offset.actual_torque[l_ankle]), 0.3);

            switch (gTaskFsm.m_gtaskFSM) {
                case task_working_RESET: {

                    /** A. Reset to default motor position(vertical direction as reference link) */
                    if (!(cycle_count % 10)) { // show message per 10 ms. (base freq = 1Khz)
                        std::cout << "=====================" << std::endl;
                        std::cout << "task_working_RESET" << std::endl;
                        std::cout << "Time : " << PD_cnt / TASK_FREQUENCY << "[s]" << std::endl;
                        std::cout << "=====================" << std::endl;
                        std::cout << "vel of [l_ankle] motor = "
                                  << r_ankle_vel << " [rad/s]"
                                  << std::endl;

                        std::cout << "spr cnt: " << std::endl;
                        std::cout << cnt_spring_4 << std::endl;
                        std::cout << cnt_spring_5 << std::endl;
                        std::cout << cnt_spring_6 << std::endl;
                        std::cout << "motor cnt:" << std::endl;
                        std::cout << cnt_motor_4 << std::endl;
                        std::cout << cnt_motor_5 << std::endl;
                        std::cout << cnt_motor_6 << std::endl;

                        std::cout << "link_rad: " << std::endl;
                        std::cout << std::setprecision(3) << l_hip_rad << std::endl;
                        std::cout << std::setprecision(3) << l_knee_rad << std::endl;
                        std::cout << std::setprecision(3) << l_ankle_rad << std::endl;


                        for (int i = 0; i < active_num; i++) {
                            unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);
                            switch (E_code) {
                                case 0x7500:
                                    std::cout << " Communication Error." << std::endl;
                                    break;
                                case 0x7300:
                                    std::cout << " Sensor Error(CRC or AckBits)." << std::endl;
                                    break;
                                case 0x2220:
                                    std::cout << "  Continuous over current" << std::endl;
                                    break;
                                case 0x0000:
                                    std::cout << "  No Fault." << std::endl;
                                    break;
                                case 0x3331:
                                    std::cout << " Field circuit interrupted" << std::endl;
                                    break;
                                default:
                                    std::cout << " Other Error. (ref to DataSheet)" << std::endl;
                                    break;
                            }
                        }
                    }

                    if (reset_step == 0) {
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CSV);
#ifdef LEFT_Part
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CSV);
#endif
                        reset_step = 1;
                    }
                    /**
                     * Step1 reset l_hip and hold reset-point
                     */
                    if (reset_step == 1 &&
                        reset_ready_cnt != 0) { // reset_ready_cnt != 0 avoid assigning velocity in CSP mode.

                        double reset_r_hip_vel = r_Hip_reset_pid.pid_control(right_hip_init_rad,
                                                                             r_hip_rad, 3000);
                        double reset_r_knee_vel = r_Knee_reset_pid.pid_control(right_knee_init_rad,
                                                                               r_knee_rad, 3000);
                        double reset_r_ankle_vel = r_Ankle_reset_pid.pid_control(right_ankle_init_rad,
                                                                                 r_ankle_rad,
                                                                                 3000);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], reset_r_hip_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_knee], reset_r_knee_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_ankle], reset_r_ankle_vel);

                        double reset_l_hip_vel = l_Hip_reset_pid.pid_control(left_hip_init_rad,
                                                                             l_hip_rad, 3000);
                        double reset_l_knee_vel = l_Knee_reset_pid.pid_control(left_knee_init_rad,
                                                                               l_knee_rad, 3000);
                        double reset_l_ankle_vel = l_Ankle_reset_pid.pid_control(left_ankle_init_rad, l_ankle_rad,
                                                                                 3000);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_hip], reset_l_hip_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_knee], reset_l_knee_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_ankle], reset_l_ankle_vel);

                        if (PD_cnt++ > reset_timeout) {
                            // if timeout, force switch state machine.
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_knee], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_ankle], 0);

#ifdef LEFT_Part
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_hip], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_knee], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_ankle], 0);
#endif
                            /**
                             * Current OP mode = CSV
                             */
                            if (SIG_cnt == 0) {
                                gTaskFsm.m_gtaskFSM = TaskFSM(task_cmd);

                                std::cout << "switch to FMS: " << gTaskFsm.m_gtaskFSM << std::endl;

                                reset_step = 0;
                                reset_ready_cnt = 0;
                                PD_cnt = 0;
                                reset_timeout = 2 * TASK_FREQUENCY;  //TODO: normal = 2; sit2stand = 5
                            } else
                                EtherCAT_ONLINE = false;

                        }
                    }
                    reset_ready_cnt++;
                }
                    break;

                case task_working_Control: {

                    unsigned int An_Left_1 = EC_READ_U16(domainTx_pd + offset.analog_in1[l_hip]);
                    unsigned int An_Left_2 = EC_READ_U16(domainTx_pd + offset.analog_in2[l_hip]);
                    unsigned int An_Right_1 = EC_READ_U16(domainTx_pd + offset.analog_in1[r_hip]);
                    unsigned int An_Right_2 = EC_READ_U16(domainTx_pd + offset.analog_in2[r_hip]);

                    GaitMatrix = UpdateGaitMatrix(An_Left_1, An_Left_2, An_Right_1, An_Right_2);

                    /** B. Assign operation to motor(s) */

                    /** count for generated curve */
                    static double Gc_main = 0.8;;
                    static int curve_count_main = 0;   // unit(ms)
                    static int curve_count_sub = 0;   // unit(ms)
                    static int settle_cnt = 0;
                    static float max_err = 0;

                    static double tau_dyn_1_old = 0;
                    static double tau_dyn_2_old = 0;

#ifdef SineWave
                    //                    if(!(curve_count_main % 1000))
                    //                        freq += 0.5;

                    double hip_ref_rad = cosineWave(curve_count_main, freq, 0.1);
                    double knee_ref_rad = cosineWave(curve_count_main, freq, 0);
                    double ankle_ref_rad = cosineWave(curve_count_main, freq, 0.2);

                    double hip_ref_vel = 2 * Pi * freq * sineWave(curve_count_main, freq, -0.1);
                    double knee_ref_vel = 2 * Pi * freq * sineWave(curve_count_main, freq, 0);
                    double ankle_ref_vel = 2 * Pi * freq * sineWave(curve_count_main, freq, -0.2);

                    double hip_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count_main, freq, -0.1);
                    double knee_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count_main, freq, -0);
                    double ankle_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count_main, freq, 0.2);
#else

                    int GaitPhase = GaitMatrixParse(GaitMatrix);

                    double beta = 0.8;
                    double Gc_sub = 0;

                    if (!FLG_RIGHT_GC_1st) { // first forward step
                        Gc_main = 1.0 * (curve_count_main++ % P_main) / P_main + 0.4;
                        if (Gc_main >= 1.0)
                            Gc_main = 1.0;

                        if (Gc_main == 1.0) {
                            if (GaitPhase == HOHS) {
                                // means left start new GC
                                FLG_RIGHT_GC_1st = true;
                                LEFT_GC_START = true;
                                curve_count_main = 0;
                                gMFSM.m_gaitMatrixFsm = HO_HS;
                            } else if (GaitPhase != HOHS)
                                curve_count_main--;
                        }
                    } else { // normal cyclic walking
                        Gc_main = 1.0 * (curve_count_main++ % P_main) / P_main;
                        if (Gc_main == 0.0)
                            if (gMFSM.m_gaitMatrixFsm != HO_HS)  // waiting for <left>[Heel Strike] occur.
                                curve_count_main--;
                    }

                    /** Gait Phase FSM **/
                    switch (GaitPhase) {
                        case HOHS:
                            if (gMFSM.m_gaitMatrixFsm == ST_S) {
                                FLG_RIGHT_GC_end = true;
                                gMFSM.m_gaitMatrixFsm = HO_HS;
                            }
                            break;
                        case TOS: {
                            if ((gMFSM.m_gaitMatrixFsm == HO_HS) || (gMFSM.m_gaitMatrixFsm == Stance))
                                gMFSM.m_gaitMatrixFsm = T_ST;
                        }
                            break;
                        case SwS:
                            if (gMFSM.m_gaitMatrixFsm == T_ST)
                                gMFSM.m_gaitMatrixFsm = S_ST;
                            break;
                        case HSHO:
                            if (gMFSM.m_gaitMatrixFsm == S_ST)
                                gMFSM.m_gaitMatrixFsm = HS_HO;
                            break;
                        case STO:
                            if ((gMFSM.m_gaitMatrixFsm == HS_HO) || (gMFSM.m_gaitMatrixFsm == Stance))
                                gMFSM.m_gaitMatrixFsm = ST_T;
                            break;
                        case SSw:
                            if (gMFSM.m_gaitMatrixFsm == ST_T)
                                gMFSM.m_gaitMatrixFsm = ST_S;
                            break;
                        case doubleStance:
                            if ((gMFSM.m_gaitMatrixFsm == ST_S) || (gMFSM.m_gaitMatrixFsm == S_ST))
                                gMFSM.m_gaitMatrixFsm = Stance;
                            break;
                        default:
                            break;
                    }

                    if (!POST_RESET) {
                        double hip_ref_rad_r = 0.0 * base_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_r = 0.0 * base_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee);
                        double ankle_ref_rad_r = 1.0 * base_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_r =
                                0.0 * differentia_1st_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_knee, P_main);
                        double knee_ref_vel_r =
                                0.0 * differentia_1st_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_vel_r =
                                1.0 * differentia_1st_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        double hip_ref_acc_r =
                                0.0 * differentia_2ed_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P_main);
                        double knee_ref_acc_r =
                                0.0 * differentia_2ed_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_acc_r =
                                1.0 * differentia_2ed_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        double hip_ref_3rd_r =
                                0.0 * differentia_3rd_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P_main);
                        double knee_ref_3rd_r =
                                0.0 * differentia_3rd_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_3rd_r =
                                1.0 * differentia_3rd_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        double hip_ref_4th_r =
                                0.0 * differentia_4th_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P_main);
                        double knee_ref_4th_r =
                                0.0 * differentia_4th_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_4th_r =
                                1.0 * differentia_4th_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        if (FLG_RIGHT_GC_1st) { // right first GC is finished
                            if (!FLG_LEFT_GC_1st) { // left first GC is not finished
                                Gc_sub = 1.0 * (curve_count_sub++ % P_main) / P_main + 0.4; // offset = 40% Gc
                                if (Gc_sub == 1.0) {
                                    if (GaitPhase == HSHO) { // left heel strike occur
                                        FLG_LEFT_GC_1st = true; // FLAG of the end of right 1st GC.
                                        curve_count_sub = 0;
                                    } else if (GaitPhase != HSHO)
                                        curve_count_sub--; // holding
                                }
                            } else // left first GC is finished, enter normal walking
                            {
                                Gc_sub = 1.0 * (curve_count_sub++ % P_sub) / P_sub;
                                if (Gc_sub == 1.0 || Gc_sub == 0.0)
                                    if (gMFSM.m_gaitMatrixFsm != HS_HO)  // wait for Heel strike occur.
                                        curve_count_sub--;
                            }
                        } else { // right first GC is not finished --> hold stance initial pose
                            Gc_sub = 0.4; // correspond to offset
                        }


                        double hip_ref_rad_l = 0.8 * base_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_l = base_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee);
                        double Ankle_ref_rad_l = base_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_l =
                                0.8 * differentia_1st_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_knee, P_sub);
                        double knee_ref_vel_l =
                                differentia_1st_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_vel_l =
                                differentia_1st_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        double hip_ref_acc_l =
                                0.8 * differentia_2ed_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P_sub);
                        double knee_ref_acc_l = differentia_2ed_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee,
                                                                            P_sub);
                        double Ankle_ref_acc_l =
                                differentia_2ed_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        double hip_ref_3rd_l =
                                0.8 * differentia_3rd_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P_sub);
                        double knee_ref_3rd_l =
                                differentia_3rd_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_3rd_l =
                                differentia_3rd_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        double hip_ref_4th_l =
                                0.8 * differentia_4th_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P_sub);
                        double knee_ref_4th_l =
                                differentia_4th_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_4th_l =
                                differentia_4th_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);


#endif
                        /**
                         ** Filter Part (middle value filter)
                         **/

                        double l_hip_vel_f, l_knee_vel_f, l_ankle_vel_f;
                        double r_hip_vel_f, r_knee_vel_f, r_ankle_vel_f;
                        int filter_window = 30;
                        int avg_window = 10;

                        if (curve_count_main < filter_window) {
                            if (curve_count_main < avg_window) {

                                input_1[curve_count_main] = r_ankle_vel; // right part is assigned to [rad/s]
                                input_2[curve_count_main] = r_knee_vel;
                                input_3[curve_count_main] = r_hip_vel;
#ifdef LEFT_Part
                                input_4[curve_count_main] = l_hip_vel;
                                input_5[curve_count_main] = l_knee_vel;
                                input_6[curve_count_main] = l_ankle_vel;
#endif
                            } else {
                                double tmp1 =
                                        sum_of_array(input_1, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp2 =
                                        sum_of_array(input_2, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp3 =
                                        sum_of_array(input_3, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                input_1[curve_count_main] = tmp1;
                                input_2[curve_count_main] = tmp2;
                                input_3[curve_count_main] = tmp3;
#ifdef LEFT_Part
                                double tmp4 =
                                        sum_of_array(input_4, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp5 =
                                        sum_of_array(input_5, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp6 =
                                        sum_of_array(input_6, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                input_4[curve_count_main] = tmp4;
                                input_5[curve_count_main] = tmp5;
                                input_6[curve_count_main] = tmp6;
#endif
                            }
                            r_hip_vel_f = input_3[curve_count_main];
                            r_knee_vel_f = input_2[curve_count_main];
                            r_ankle_vel_f = input_1[curve_count_main];
#ifdef LEFT_Part
                            l_hip_vel_f = input_4[curve_count_main];
                            l_knee_vel_f = input_5[curve_count_main];
                            l_ankle_vel_f = input_6[curve_count_main];
#endif
                        } else {
                            left_shift_array(input_1, r_ankle_vel, filter_window);
                            left_shift_array(input_2, r_knee_vel, filter_window);
                            left_shift_array(input_3, r_hip_vel, filter_window);
                            bw_link.filter(input_1, output_1, filter_window, 0, true);
                            bw_link.filter(input_2, output_2, filter_window, 0, true);
                            bw_link.filter(input_3, output_3, filter_window, 0, true);
                            l_hip_vel_f = output_3[filter_window - 1];
                            l_knee_vel_f = output_2[filter_window - 1];
                            l_ankle_vel_f = output_1[filter_window - 1];
#ifdef LEFT_Part
                            left_shift_array(input_4, l_hip_vel, filter_window);
                            left_shift_array(input_5, l_knee_vel, filter_window);
                            left_shift_array(input_6, l_ankle_vel, filter_window);
                            bw_link.filter(input_4, output_4, filter_window, 0, true);
                            bw_link.filter(input_5, output_5, filter_window, 0, true);
                            bw_link.filter(input_6, output_6, filter_window, 0, true);
                            r_hip_vel_f = output_4[filter_window - 1];
                            r_knee_vel_f = output_5[filter_window - 1];
                            r_ankle_vel_f = output_6[filter_window - 1];
#endif
                        }

                        // RPM -> rad/s
                        //TODO: why x2?

                        l_hip_vel_f *= 2;
                        l_knee_vel_f *= 2;
                        l_ankle_vel_f *= 2;
#ifdef LEFT_Part
                        r_hip_vel_f *= 2;
                        r_knee_vel_f *= 2;
                        r_ankle_vel_f *= 2;

#endif

                        Eigen::Matrix3d Ks_l, Ks_r;
                        Ks_l << 43.93, 0, 0,
                                0, 90.84, 0,
                                0, 0, 33.13;

                        Ks_r << 60, 0, 0,
                                0, 60, 0,
                                0, 0, 60;

                        Eigen::Vector3d M, B_l, K_l, B_r, K_r;
#ifdef Tracking_Impendance
                        K_l << 40, 40, 20;
                        B_l << 0.6, 0.9, 0.5;// 1.8 1.2 0.6
                        K_r << 50, 40, 20;
                        B_r << 0.8, 0.8, 0.5; // 0.6 0.6 0.5

                        d4q_r << hip_ref_4th_r, knee_ref_4th_r, ankle_ref_4th_r;
                        d3q_r << hip_ref_3rd_r, knee_ref_3rd_r, ankle_ref_3rd_r;
                        ddq_r << hip_ref_acc_r, (knee_ref_acc_r), (ankle_ref_acc_r);
                        dq_r << hip_ref_vel_r, (knee_ref_vel_r), (ankle_ref_vel_r);
                        q_r << hip_ref_rad_r, (knee_ref_rad_r), (ankle_ref_rad_r);
                        dq_e_r << hip_ref_vel_r - r_hip_vel_f, (knee_ref_vel_r - r_knee_vel_f), (ankle_ref_vel_r -
                                                                                                 r_ankle_vel_f);
                        q_e_r << hip_ref_rad_r - r_hip_rad, (knee_ref_rad_r - r_knee_rad), (ankle_ref_rad_r -
                                                                                            r_ankle_rad);

#ifdef LEFT_Part
                        d4q_l << hip_ref_4th_l, knee_ref_4th_l, Ankle_ref_4th_l;
                        d3q_l << hip_ref_3rd_l, knee_ref_3rd_l, Ankle_ref_3rd_l;
                        ddq_l << hip_ref_acc_l, (knee_ref_acc_l), (Ankle_ref_acc_l);
                        dq_l << hip_ref_vel_l, (knee_ref_vel_l), (Ankle_ref_vel_l);
                        q_l << hip_ref_rad_l, (knee_ref_rad_l), (Ankle_ref_rad_l);
                        dq_e_l << hip_ref_vel_l - l_hip_vel_f, (knee_ref_vel_l - l_knee_vel_f), (Ankle_ref_vel_l -
                                                                                                 l_ankle_vel_f);
                        q_e_l << hip_ref_rad_l - l_hip_rad, (knee_ref_rad_l - l_knee_rad), (Ankle_ref_rad_l -
                                                                                            l_ankle_rad);
#endif

#endif

#ifdef Drag_Impendance
                        //                        K << 0,0,0;
                        //                        B << 0.8, 1.2, 0.4;
                        d4q_l << 0, 0, 0;
                        d3q_l << 0, 0, 0;
                        ddq_l << 0, (0), (0);
                        dq_l << 0, (0), (0);
                        q_l << link_1_rad, (link_2_rad), (link_3_rad);
                        dq_e_l << 0 - l_hip_vel_f, (0 - l_knee_vel_f), (0 - l_ankle_vel_f);
                        q_e_l << hip_ref_rad_l - link_1_rad, (knee_ref_rad_l - link_2_rad), (Ankle_ref_rad_l -
                                                                                             link_3_rad);
#endif

                        Select_Matrix_d << 1, 0, 0,
                                0, 1, 0,
                                0, 0, 1;

                        // all diagonal element == 1 meanings all-selected.
                        // Selecting states
                        dq_e_r = Select_Matrix_d * dq_e_r;
                        q_e_r = Select_Matrix_d * q_e_r;

                        ddq_r = Select_Matrix_d * ddq_r;
                        dq_r = Select_Matrix_d * dq_r;
                        q_r = Select_Matrix_d * q_r;

                        // calculate feedforward torque. (after transmission)
                        Eigen::Vector3d compensation_r;
//                        compensation_l = left_SEA_dynamics.feedforward_dynamics(d4q_l, d3q_l, ddq_l, dq_l, q_l,
//                                                                                Ks_l);
                        compensation_r = right_SEA_dynamics.Gravity_term(q_r);

                        compensation_r = 1.0 * compensation_r; //0.8

                        r_Hip_pd.pid_set_params(0.5, 0, 4); // 0.6,4 (0.85 boundary)
                        r_Knee_pd.pid_set_params(0.45, 0, 2.5); // 0.45,2.5
                        r_Ankle_pd.pid_set_params(0.5, 0, 3); // 0.45,3

                        double r_hip_Impedance =
                                0.8 * compensation_r(0) + (B_r(0) * dq_e_r(0) + K_r(0) * q_e_r(0));
                        double r_knee_Impedance =
                                compensation_r(1) + (B_r(1) * dq_e_r(1) + K_r(1) * q_e_r(1));
                        double r_ankle_Impedance =
                                compensation_r(2) + (B_r(2) * dq_e_r(2) + K_r(2) * q_e_r(2));

                        double r_hip_tau_spr = Ks_r(0, 0) * (r_hip_s_rad);
                        double r_knee_tau_spr = Ks_r(1, 1) * (r_knee_s_rad);
                        double r_ankle_tau_spr = Ks_r(2, 2) * (r_ankle_s_rad);

                        double tau_pd_hip_r = r_Hip_pd.pid_control(
                                r_hip_Impedance, r_hip_tau_spr, 2);

                        double tau_pd_knee_r = r_Knee_pd.pid_control(
                                r_knee_Impedance, r_knee_tau_spr, 2);

                        double tau_pd_ankle_r = l_Ankle_pd.pid_control(
                                r_ankle_Impedance, r_ankle_tau_spr, 1.5);

                        int tau_dyn_thousand_1 = tau_Nm2thousand(tau_pd_hip_r, 0.3);
                        int tau_dyn_thousand_2 = tau_Nm2thousand(tau_pd_knee_r, 0.3);
                        int tau_dyn_thousand_3 = tau_Nm2thousand(tau_pd_ankle_r, 0.3);

#ifdef LEFT_Part
                        l_Hip_pd.pid_set_params(0.6, 0, 6);
                        l_Knee_pd.pid_set_params(0.5, 0, 4.5);
                        l_Ankle_pd.pid_set_params(0.5, 0, 3);

                        Eigen::Vector3d compensation_l;
                        compensation_l = left_SEA_dynamics.feedforward_dynamics(d4q_l, d3q_l, ddq_l, dq_l, q_l,
                                                                                Ks_l);

                        compensation_l = 0.85 * compensation_l;

                        double l_hip_Impedance =
                                compensation_l(0) + (B_l(0) * dq_e_l(0) + K_l(0) * q_e_l(0));
                        double l_knee_Impedance =
                                compensation_l(1) + (B_l(1) * dq_e_l(1) + K_l(1) * q_e_l(1));
                        double l_ankle_Impedance =
                                compensation_l(2) + (B_l(2) * dq_e_l(2) + K_l(2) * q_e_l(2));

                        double l_hip_tau_spr = Ks_l(0, 0) * (l_hip_s_rad);
                        double l_knee_tau_spr = Ks_l(1, 1) * (l_knee_s_rad);
                        double l_ankle_tau_spr = Ks_l(2, 2) * (l_ankle_s_rad);


                        double tau_pd_hip_l = l_Hip_pd.pid_control(
                                l_hip_Impedance, l_hip_tau_spr, 2);

                        double tau_pd_knee_l = l_Knee_pd.pid_control(
                                l_knee_Impedance, l_knee_tau_spr, 2);

                        double tau_pd_ankle_l = l_Ankle_pd.pid_control(
                                l_ankle_Impedance, l_ankle_tau_spr, 1.5);

                        int tau_dyn_thousand_4 = tau_Nm2thousand(tau_pd_hip_l, 0.058);
                        int tau_dyn_thousand_5 = tau_Nm2thousand(tau_pd_knee_l, 0.058);
                        int tau_dyn_thousand_6 = tau_Nm2thousand(tau_pd_ankle_l, 0.058);
#endif
                        /**
                         **    Apply torque control
                         */


#ifndef asynchronous_enable
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);

                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_1);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_2);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_3);
#ifdef LEFT_Part
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);
//
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_4);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_5);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_6);
#endif

#endif

#ifdef asynchronous_enable
                        if (!FLG_RIGHT_GC_1st) {
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);
                        }
//
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_1);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_2);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_3);
//
                        if (LEFT_GC_START) {
                            if (!FLG_LEFT_GC_1st) {
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);
                            }
//                            EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_4);
//                            EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_5);
//                            EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_6);
                        }
#endif
                        /** ----------------------------------------------------------------------------------- */
                        if (!(cycle_count % 10)) { // show message per 1 ms.
                            std::cout << "===============================" << std::endl;
                            std::cout << "task_working_Control" << std::endl;
                            std::cout << "===============================" << std::endl;
                            std::cout << "time: " << 1.0 * curve_count_main / TASK_FREQUENCY << " [s] " << std::endl;
                            std::cout << "dq_e_l: " << std::endl;
                            std::cout << dq_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "q_e_l: " << std::endl;
                            std::cout << q_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "r_Hip motor [rad] = " << lever_arm_4_rad << std::endl;
                            std::cout << "r_Hip link [rad] = " << link_4_rad << std::endl;
                            std::cout << "r_Ankle motor [cnt] = "
                                      << EC_READ_S32(domainTx_pd + offset.actual_position[r_ankle]) << std::endl;
                            std::cout << "r_Ankle link [cnt] = "
                                      << EC_READ_S32(domainTx_pd + offset.second_position[r_ankle]) << std::endl;
                            int OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[l_hip]);
                            switch (OP_mode) {
                                case 8:
                                    std::cout << "OP mode : CSP" << std::endl;
                                    break;
                                case 9:
                                    std::cout << "OP mode : CSV" << std::endl;
                                    break;
                                case 10:
                                    std::cout << "OP mode : CST" << std::endl;
                                    break;
                                default:
                                    break;
                            }


                            for (int i = 0; i < active_num; i++)
                                error_code_sequence.emplace_back(EC_READ_U16(domainTx_pd + offset.Error_code[i]));


                            std::cout << " ---- Servo Error Status ---- " << std::endl;

                            for (int j = 0; j < active_num; j++) {
                                switch (error_code_sequence[j]) {
                                    case 0x3331:
                                        std::cout << "Joint [" << j << "] --> Field circuit interrupted" << std::endl;
                                        break;
                                    case 0x2220:
                                        std::cout << "Joint [" << j << "] --> Continuous over current (device internal)"
                                                  << std::endl;
                                        break;
                                    case 0x0000:
                                        std::cout << "Joint [" << j << "] --> NULL" << std::endl;
                                        break;
                                    default:
                                        std::cout << "Ref to data sheet." << std::endl;
                                        break;
                                }
                            }

                            std::vector<unsigned int>().swap(error_code_sequence);

                            std::cout << " ---- Gait Cycle ----" << std::endl;
                            std::cout << Gc_main << std::endl;

                            if (FLG_RIGHT_GC_end) {
                                Gc_cnt++;
                                FLG_RIGHT_GC_end = false;
                            } else {
//                                std::cout << "Current Gc is not completed." << std::endl;
                                std::cout << "Current Phase: " << gMFSM.m_gaitMatrixFsm << std::endl;
                            }
                            std::cout << "Current Gc is " << Gc_cnt << std::endl;


                            switch (gMFSM.m_gaitMatrixFsm) {
                                // 0 -> [1 -> 2 -> 3 -> 4 -> 5 -> 6]
                                case Stance: {
                                    std::cout << "2x stance" << std::endl;
                                    gaitData << "2x stance" << ',' << Stance << std::endl;
                                }
                                    break;
                                case ST_T: {
                                    std::cout << "Stance-Toe_off" << std::endl;
                                    gaitData << "Stance-Toe_off" << ',' << ST_T << std::endl;
                                }
                                    break;
                                case ST_S: {
                                    std::cout << "Stance-Swing" << std::endl;
                                    gaitData << "Stance-Swing" << ',' << ST_S << std::endl;
                                }
                                    break;
                                case HO_HS: {
                                    std::cout << "Heel_off - Heel_strike" << std::endl;
                                    gaitData << "Heel_off - Heel_strike" << ',' << HO_HS << std::endl;
                                }
                                    break;
                                case HS_HO: {
                                    std::cout << "Heel_strike - Heel_off" << std::endl;
                                    gaitData << "Heel_strike - Heel_off" << ',' << HS_HO << std::endl;
                                }
                                    break;
                                case S_ST: {
                                    std::cout << "Swing-Stance" << std::endl;
                                    gaitData << "Swing-Stance" << ',' << S_ST << std::endl;
                                }
                                    break;
                                case T_ST: {
                                    std::cout << "Toe_off-stance" << std::endl;
                                    gaitData << "Toe_off-stance" << ',' << T_ST << std::endl;
                                }
                                    break;
                                default:
                                    break;
                            }
//                            std::cout << "GaitPhase : " << GaitPhase << std::endl;
                            std::cout << "Left #1 :" << An_Left_1 << std::endl;
                            std::cout << "Left #2 :" << An_Left_2 << std::endl;
                            std::cout << "Right #1 :" << An_Right_1 << std::endl;
                            std::cout << "Right #2 :" << An_Right_2 << std::endl;
                        }

                        left_outFile << hip_ref_rad_l << ',' << knee_ref_rad_l << ',' << Ankle_ref_rad_l << ','
                                     << link_1_rad << ',' << link_2_rad << ',' << link_3_rad << ','
                                     << lever_arm_1_rad << ',' << lever_arm_2_rad << ',' << lever_arm_3_rad << ','
                                     << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l
                                     << std::endl;

                        left_velFile << hip_ref_vel_l << ',' << knee_ref_vel_l << ',' << Ankle_ref_vel_l << ','
                                     << l_hip_vel_f << ',' << l_knee_vel_f << ',' << l_ankle_vel_f << ','
                                     << l_hip_m_vel << ',' << l_knee_m_vel << ',' << l_ankle_m_vel << ','
                                     << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l << std::endl;

                        left_save_file << l_hip_Impedance << ',' << l_hip_tau_spr << ',' << l_knee_Impedance << ','
                                       << l_knee_tau_spr
                                       << ',' <<
                                       l_ankle_Impedance << ',' << l_ankle_tau_spr << ','
                                       << lever_arm_3_rad - link_3_rad
                                       << std::endl;

                        left_filter_file << compensation_l(1) << ',' << (B_l(1) * dq_e_l(1)) << ','
                                         << (K_l(1) * q_e_l(1))
                                         << std::endl;

#ifdef  LEFT_Part
                        right_outFile << hip_ref_rad_r << ',' << knee_ref_rad_r << ',' << ankle_ref_rad_r << ','
                                      << link_4_rad << ',' << link_5_rad << ',' << link_6_rad << ','
                                      << r_hip_m_vel << ',' << r_knee_m_vel << ',' << r_ankle_m_vel << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r
                                      << std::endl;

                        right_velFile << hip_ref_vel_r << ',' << knee_ref_vel_r << ',' << ankle_ref_vel_r << ','
                                      << r_hip_vel_f << ',' << r_knee_vel_f << ',' << r_ankle_vel_f << ','
                                      << lever_arm_4_vel << ',' << lever_arm_5_rad << ',' << lever_arm_6_rad << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r << std::endl;

                        right_save_file << r_hip_Impedance << ',' << r_hip_tau_spr << ',' << r_knee_Impedance << ','
                                        << r_knee_tau_spr
                                        << ',' <<
                                        r_ankle_Impedance << ',' << r_ankle_tau_spr << ','
                                        << lever_arm_3_rad - link_3_rad
                                        << std::endl;

                        right_filter_file << vel_RPM2rad(link_4_vel) << ',' << r_hip_vel_f << std::endl;
#endif
                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;

                    id_ready_cnt++;
                    if (curve_count_main >= P_main * (10 + 0.35)) {
                        EtherCAT_ONLINE = false;
                        std::cout << "[End of Program...]" << std::endl;
                        break;
                    }

                }
                    break;

                case task_working_Identification: {
                    static int curve_cnt = 0;
                    static int cnt_startup_hip = 0;
                    static int cnt_startup_knee = 0;
                    static int cnt_startup_ankle = 0;
                    curve_cnt++;
                    // this state is used to identify dynamics of exoskeleton's one-side limb
                    // the OP mode is running under CSP

                    if (!FLG_IDENTIFY_STARTUP) {
                        cnt_startup_hip = cnt_motor_3;
                        cnt_startup_knee = cnt_motor_2;
                        cnt_startup_ankle = cnt_motor_1;
                        FLG_IDENTIFY_STARTUP = true;
                    }

                    int OP_mode[3];
                    for (int i = 3; i < 6; i++) {
                        OP_mode[i] = EC_READ_U8(domainTx_pd + offset.modes_of_operation_display[i]);
                        if (OP_mode[i] != CSP)
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[i], CSP);
                    }

                    double Identify_hip_rad = 0.6 * identify_position(curve_cnt, i_a0_hip, i_a_hip, i_b_hip, i_w_hip);
                    double Identify_knee_rad =
                            0.6 * identify_position(curve_cnt, i_a0_knee, i_a_knee, i_b_knee, i_w_knee);
                    double Identify_ankle_rad = 0.6 * identify_position(curve_cnt, i_a0_ankle, i_a_ankle, i_b_ankle,
                                                                        i_w_ankle) - 0.57;

                    double Identify_hip_vel = 0.6 * identify_velocity(curve_cnt, i_a0_hip, i_a_hip, i_b_hip, i_w_hip);
                    double Identify_knee_vel =
                            0.6 * identify_velocity(curve_cnt, i_a0_knee, i_a_knee, i_b_knee, i_w_knee);
                    double Identify_ankle_vel =
                            0.6 * identify_velocity(curve_cnt, i_a0_ankle, i_a_ankle, i_b_ankle, i_w_ankle);


                    // Right Part
//                    int Identify_hip_cnt =
//                            cnt_startup_hip + 100 * pos_rad2inc_17bits(Identify_hip_rad - right_hip_init_rad);
//                    int Identify_knee_cnt =
//                            cnt_startup_knee + 80 * pos_rad2inc_17bits(Identify_knee_rad - right_knee_init_rad);
//                    int Identify_ankle_cnt =
//                            cnt_startup_ankle + 80 * pos_rad2inc_17bits(Identify_ankle_rad - right_ankle_init_rad);

                    // left Part
                    int Identify_hip_cnt =
                            cnt_startup_hip + 100 * pos_rad2inc_17bits(Identify_hip_rad - left_hip_init_rad);
                    int Identify_knee_cnt =
                            cnt_startup_knee + 80 * pos_rad2inc_17bits(Identify_knee_rad - left_knee_init_rad);
                    int Identify_ankle_cnt =
                            cnt_startup_ankle + 80 * pos_rad2inc_17bits(Identify_ankle_rad - left_ankle_init_rad);

                    EC_WRITE_S32(domainRx_pd + offset.target_position[l_hip], Identify_hip_cnt);
                    EC_WRITE_S32(domainRx_pd + offset.target_position[l_knee], Identify_knee_cnt);
                    EC_WRITE_S32(domainRx_pd + offset.target_position[l_ankle], Identify_ankle_cnt);


                    if (!(cycle_count % 2)) {
                        std::cout << "=============================" << std::endl;
                        std::cout << "task_working_Identification" << std::endl;
                        std::cout << "=============================" << std::endl;
                        std::cout << "Current Cycle: " << curve_cnt / 2000 << std::endl;
                        std::cout << "cnt_startup_hip : " << cnt_startup_hip << std::endl;
                        std::cout << "cnt_startup_knee : " << cnt_startup_knee << std::endl;
                        std::cout << "cnt_startup_ankle : " << cnt_startup_ankle << std::endl;
                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;

                    if (curve_cnt >= 2000 * 10) {
                        SIG_cnt++;
                        gTaskFsm.m_gtaskFSM = task_working_RESET;
                    }

                    right_outFile << Identify_hip_rad << ',' << Identify_knee_rad << ',' << Identify_ankle_rad << ','
                                  << r_hip_rad << ',' << r_knee_rad << ',' << r_ankle_rad << ',' << r_hip_m_rad << ','
                                  << r_knee_m_rad << ','
                                  << r_ankle_m_rad << ',' << tau_mot_3 << ',' << tau_mot_2 << ',' << tau_mot_1
                                  << std::endl;
                    right_velFile << Identify_hip_vel << ',' << Identify_knee_vel << ',' << Identify_ankle_vel << ','
                                  << r_hip_vel << ',' << r_knee_vel << ',' << r_ankle_vel << ',' << r_hip_m_vel << ','
                                  << r_knee_m_vel << ',' << r_ankle_m_vel << ',' << tau_mot_4 << ',' << tau_mot_5 << ','
                                  << tau_mot_6 << std::endl;

                    left_outFile << Identify_hip_rad << ',' << Identify_knee_rad << ',' << Identify_ankle_rad << ','
                                 << l_hip_rad << ',' << l_knee_rad << ',' << l_ankle_rad << ',' << l_hip_m_rad << ','
                                 << l_knee_m_rad << ',' << l_ankle_m_rad << ',' << tau_mot_4 << ',' << tau_mot_5 << ','
                                 << tau_mot_6 << std::endl;
                    left_velFile << Identify_hip_vel << ',' << Identify_knee_vel << ',' << Identify_ankle_vel << ','
                                 << l_hip_vel << ',' << l_knee_vel << ',' << l_ankle_vel << ',' << l_hip_m_vel << ','
                                 << l_knee_m_vel << ',' << l_ankle_m_vel << ',' << tau_mot_4 << ',' << tau_mot_5 << ','
                                 << tau_mot_6 << std::endl;

                }
                    break;

                case task_working_Impedance: {
                    if (!FLG_RIGHT_GC_1st) {
                        Gc_main = 1.0 * (curve_cnt_right % P_main) / P_main + 0.4;
                        if (Gc_main >= 1.0) {
                            Gc_main = 0.0;
                            FLG_RIGHT_GC_1st = true;
                            LEFT_GC_START = true;
                            curve_cnt_right = 0;
                        }
                    } else {
                        Gc_main = 1.0 * (curve_cnt_right % P_main) / P_main;
                        if (Gc_main == 0.0) // GC can not reach 1.0
                        {
                            curve_cnt_right = 0;
                            P_main = P_update;
                        }
                    }

                    curve_cnt_right++;

                    if (!POST_RESET) {
                        double hip_ref_rad_r = 0.6 * base_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_r = 0.6 * base_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee);
                        double ankle_ref_rad_r = 1.0 * base_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_r =
                                0.6 * differentia_1st_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_knee, P_main);
                        double knee_ref_vel_r =
                                0.6 * differentia_1st_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_vel_r =
                                1.0 * differentia_1st_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        double hip_ref_acc_r =
                                0.6 * differentia_2ed_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P_main);
                        double knee_ref_acc_r =
                                0.6 * differentia_2ed_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_acc_r =
                                1.0 * differentia_2ed_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        double hip_ref_3rd_r =
                                0.6 * differentia_3rd_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P_main);
                        double knee_ref_3rd_r =
                                0.6 * differentia_3rd_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_3rd_r =
                                1.0 * differentia_3rd_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        double hip_ref_4th_r =
                                0.6 * differentia_4th_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P_main);
                        double knee_ref_4th_r =
                                0.6 * differentia_4th_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_4th_r =
                                1.0 * differentia_4th_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        if (FLG_RIGHT_GC_1st) {
                            if (!FLG_LEFT_GC_1st) {
                                Gc_sub = 1.0 * (curve_cnt_left % P_sub) / P_sub + 0.4;
                                if (Gc_sub >= 1.0) {
                                    Gc_sub = 0.0;
                                    FLG_LEFT_GC_1st = true;
                                    curve_cnt_left = 0;
                                }
                            } else {
                                Gc_sub = 1.0 * (curve_cnt_left % P_sub) / P_sub;
                                if (Gc_sub == 0.0) {
                                    curve_cnt_left = 0;
                                    P_sub = P_update;
                                }
                            }
                        } else // stand-by state
                            Gc_sub = 0.4;

                        if (FLG_RIGHT_GC_1st)
                            curve_cnt_left++;


                        double hip_ref_rad_l = 0.6 * base_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_l = 0.6 * base_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee);
                        double Ankle_ref_rad_l = base_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_l =
                                0.6 * differentia_1st_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_knee, P_sub);
                        double knee_ref_vel_l =
                                0.6 * differentia_1st_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_vel_l =
                                differentia_1st_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        double hip_ref_acc_l =
                                0.6 * differentia_2ed_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P_sub);
                        double knee_ref_acc_l =
                                0.6 * differentia_2ed_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_acc_l =
                                differentia_2ed_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        double hip_ref_3rd_l =
                                0.6 * differentia_3rd_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P_sub);
                        double knee_ref_3rd_l =
                                0.6 * differentia_3rd_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_3rd_l =
                                differentia_3rd_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        double hip_ref_4th_l =
                                0.6 * differentia_4th_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P_sub);
                        double knee_ref_4th_l =
                                0.6 * differentia_4th_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_4th_l =
                                differentia_4th_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);



                        /**
                         ** Filter Part (middle value filter)
                         **/

                        double l_hip_vel_f, l_knee_vel_f, l_ankle_vel_f;
                        double r_hip_vel_f, r_knee_vel_f, r_ankle_vel_f;
                        int filter_window = 30;
                        int avg_window = 10;

                        if (curve_cnt_right < filter_window) {
                            if (curve_cnt_right < avg_window) {

                                input_1[curve_cnt_right] = r_ankle_vel; // right part is assigned to [rad/s]
                                input_2[curve_cnt_right] = r_knee_vel;
                                input_3[curve_cnt_right] = r_hip_vel;

                                input_4[curve_cnt_right] = l_hip_vel;
                                input_5[curve_cnt_right] = l_knee_vel;
                                input_6[curve_cnt_right] = l_ankle_vel;

                            } else {
                                double tmp1 =
                                        sum_of_array(input_1, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp2 =
                                        sum_of_array(input_2, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp3 =
                                        sum_of_array(input_3, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                input_1[curve_cnt_right] = tmp1;
                                input_2[curve_cnt_right] = tmp2;
                                input_3[curve_cnt_right] = tmp3;

                                double tmp4 =
                                        sum_of_array(input_4, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp5 =
                                        sum_of_array(input_5, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp6 =
                                        sum_of_array(input_6, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                input_4[curve_cnt_right] = tmp4;
                                input_5[curve_cnt_right] = tmp5;
                                input_6[curve_cnt_right] = tmp6;


                            }
                            r_hip_vel_f = input_3[curve_cnt_right];
                            r_knee_vel_f = input_2[curve_cnt_right];
                            r_ankle_vel_f = input_1[curve_cnt_right];

                            l_hip_vel_f = input_4[curve_cnt_right];
                            l_knee_vel_f = input_5[curve_cnt_right];
                            l_ankle_vel_f = input_6[curve_cnt_right];

                        } else {
                            left_shift_array(input_1, r_ankle_vel, filter_window);
                            left_shift_array(input_2, r_knee_vel, filter_window);
                            left_shift_array(input_3, r_hip_vel, filter_window);
                            bw_link.filter(input_1, output_1, filter_window, 0, true);
                            bw_link.filter(input_2, output_2, filter_window, 0, true);
                            bw_link.filter(input_3, output_3, filter_window, 0, true);
                            r_hip_vel_f = output_3[filter_window - 1];
                            r_knee_vel_f = output_2[filter_window - 1];
                            r_ankle_vel_f = output_1[filter_window - 1];

                            left_shift_array(input_4, l_hip_vel, filter_window);
                            left_shift_array(input_5, l_knee_vel, filter_window);
                            left_shift_array(input_6, l_ankle_vel, filter_window);
                            bw_link.filter(input_4, output_4, filter_window, 0, true);
                            bw_link.filter(input_5, output_5, filter_window, 0, true);
                            bw_link.filter(input_6, output_6, filter_window, 0, true);
                            l_hip_vel_f = output_4[filter_window - 1];
                            l_knee_vel_f = output_5[filter_window - 1];
                            l_ankle_vel_f = output_6[filter_window - 1];

                        }

                        // RPM -> rad/s
                        //TODO: why x2?

                        l_hip_vel_f *= 2;
                        l_knee_vel_f *= 2;
                        l_ankle_vel_f *= 2;

                        r_hip_vel_f *= 2;
                        r_knee_vel_f *= 2;
                        r_ankle_vel_f *= 2;


                        Eigen::Matrix3d Ks_r, Ks_l;

                        Ks_r << 60, 0, 0,
                                0, 60, 0,
                                0, 0, 60;

                        Ks_l << 60, 0, 0,
                                0, 60, 0,
                                0, 0, 60;

                        Eigen::Vector3d B_r, K_r, B_l, K_l;

#ifndef Drag_Impendance

                        K_r << 30, 20, 20;
                        B_r << 0.6, 0.1, 0.5; // 0.6 0.6 0.50

                        K_l << 30, 30, 20;
                        B_l << 0.1, 0.0, 0.2; // 0.6 0.6 0.5



                        d4q_r << hip_ref_4th_r, knee_ref_4th_r, ankle_ref_4th_r;
                        d3q_r << hip_ref_3rd_r, knee_ref_3rd_r, ankle_ref_3rd_r;
                        ddq_r << hip_ref_acc_r, (knee_ref_acc_r), (ankle_ref_acc_r);
                        dq_r << hip_ref_vel_r, (knee_ref_vel_r), (ankle_ref_vel_r);
                        q_r << hip_ref_rad_r, (knee_ref_rad_r), (ankle_ref_rad_r);
                        dq_e_r << hip_ref_vel_r - r_hip_vel_f, (knee_ref_vel_r - r_knee_vel_f), (ankle_ref_vel_r -
                                                                                                 r_ankle_vel_f);
                        q_e_r << hip_ref_rad_r - r_hip_rad, (knee_ref_rad_r - r_knee_rad), (ankle_ref_rad_r -
                                                                                            r_ankle_rad);

                        //left part =>

                        d4q_l << hip_ref_4th_l, knee_ref_4th_l, Ankle_ref_4th_l;
                        d3q_l << hip_ref_3rd_l, knee_ref_3rd_l, Ankle_ref_3rd_l;
                        ddq_l << hip_ref_acc_l, (knee_ref_acc_l), (Ankle_ref_acc_l);
                        dq_l << hip_ref_vel_l, (knee_ref_vel_l), (Ankle_ref_vel_l);
                        q_l << hip_ref_rad_l, (knee_ref_rad_l), (Ankle_ref_rad_l);
                        dq_e_l << hip_ref_vel_l - l_hip_vel_f, (knee_ref_vel_l - l_knee_vel_f), (Ankle_ref_vel_l -
                                                                                                 l_ankle_vel_f);
                        q_e_l << hip_ref_rad_l - l_hip_rad, (knee_ref_rad_l - l_knee_rad), (Ankle_ref_rad_l -
                                                                                            l_ankle_rad);

#endif
#ifdef Drag_Impendance
                        K_r << 0, 0, 0;
                        K_l << 0, 0, 0;
                        B_r << 0.2, 0.4, 0.4;
                        B_l << 0.1, 0.2, 0.2;

                        d4q_l << 0, 0, 0;
                        d3q_l << 0, 0, 0;
                        ddq_l << 0, (0), (0);
                        dq_l << 0, (0), (0);
                        q_l << l_hip_rad, l_knee_rad, l_ankle_rad;
                        dq_e_l << (0 - l_hip_vel_f), (0 - l_knee_vel_f), (0 - l_ankle_vel_f);
                        q_e_l << hip_ref_rad_l - l_hip_rad, (knee_ref_rad_l - l_knee_rad), (Ankle_ref_rad_l -
                                                                                            l_ankle_rad);

                        d4q_r << 0, 0, 0;
                        d3q_r << 0, 0, 0;
                        ddq_r << 0, (0), (0);
                        dq_r << r_hip_vel_f, r_knee_vel_f, r_ankle_vel_f;
                        q_r << r_hip_rad, r_knee_rad, r_ankle_rad;
                        dq_e_r << 0 - r_hip_vel_f, (0 - r_knee_vel_f), (0 - r_ankle_vel_f);
                        q_e_r << hip_ref_rad_r - r_hip_rad, (knee_ref_rad_r - r_knee_rad), (ankle_ref_rad_r -
                                                                                            r_ankle_rad);

#endif
                        Eigen::Vector3d compensation_r, compensation_l;
                        compensation_r = right_SEA_dynamics.Gravity_term(q_r);
                        compensation_l = left_SEA_dynamics.Gravity_term(q_l);

                        r_Hip_pd.pid_set_params(0.2, 0, 2);
                        r_Knee_pd.pid_set_params(0.5, 0, 2.5);
                        r_Ankle_pd.pid_set_params(0.5, 0, 0.3);

                        l_Hip_pd.pid_set_params(0.2, 0, 2);
                        l_Knee_pd.pid_set_params(0.3, 0, 3.5); // 0.35,3.5
                        l_Ankle_pd.pid_set_params(0.5, 0, 0.3);

                        // when running in DRAG mode, there will be ocillation if compensations are added.
#ifdef Drag_Impendance
                        double r_hip_Impedance =
                                (B_r(0) * dq_e_r(0) + K_r(0) * q_e_r(0));
                        double r_knee_Impedance =
                                (B_r(1) * dq_e_r(1) + K_r(1) * q_e_r(1));
                        double r_ankle_Impedance =
                                (B_r(2) * dq_e_r(2) + K_r(2) * q_e_r(2));

                        double l_hip_Impedance =
                                (B_l(0) * dq_e_l(0) + K_l(0) * q_e_l(0));
                        double l_knee_Impedance =
                                B_l(1) * dq_e_l(1) + K_l(1) * q_e_l(1);
                        double l_ankle_Impedance =
                                (B_l(2) * dq_e_l(2) + K_l(2) * q_e_l(2));
#endif

#ifdef Tracking_Impendance
                        double r_hip_Impedance =
                                compensation_r(0) + (B_r(0) * dq_e_r(0) + K_r(0) * q_e_r(0));
                        double r_knee_Impedance =
                                compensation_r(1) + (B_r(1) * dq_e_r(1) + K_r(1) * q_e_r(1));
                        double r_ankle_Impedance =
                                compensation_r(2) + (B_r(2) * dq_e_r(2) + K_r(2) * q_e_r(2));

                        double l_hip_Impedance =
                                compensation_l(0) + (B_l(0) * dq_e_l(0) + K_l(0) * q_e_l(0));
                        double l_knee_Impedance =
                                compensation_l(1) + (B_l(1) * dq_e_l(1) + K_l(1) * q_e_l(1));
                        double l_ankle_Impedance =
                                compensation_l(2) + (B_l(2) * dq_e_l(2) + K_l(2) * q_e_l(2));
#endif

                        double r_hip_tau_spr = Ks_r(0, 0) * (r_hip_s_rad);
                        double r_knee_tau_spr = Ks_r(1, 1) * (r_knee_s_rad);
                        double r_ankle_tau_spr = Ks_r(2, 2) * (r_ankle_s_rad);

                        double l_hip_tau_spr = Ks_l(0, 0) * (l_hip_s_rad);
                        double l_knee_tau_spr = Ks_l(1, 1) * (l_knee_s_rad);
                        double l_ankle_tau_spr = Ks_l(2, 2) * (l_ankle_s_rad);

                        double tau_pd_hip_r = r_Hip_pd.pid_control(
                                r_hip_Impedance, r_hip_tau_spr, 0.9);

                        double tau_pd_knee_r = r_Knee_pd.pid_control(
                                r_knee_Impedance, r_knee_tau_spr, 1.2);

                        double tau_pd_ankle_r = r_Ankle_pd.pid_control(
                                r_ankle_Impedance, r_ankle_tau_spr, 0.9);

                        double tau_pd_hip_l = l_Hip_pd.pid_control(
                                l_hip_Impedance, l_hip_tau_spr, 0.9);

                        double tau_pd_knee_l = l_Knee_pd.pid_control(
                                l_knee_Impedance, l_knee_tau_spr, 1.2);

                        double tau_pd_ankle_l = l_Ankle_pd.pid_control(
                                l_ankle_Impedance, l_ankle_tau_spr, 0.9);

                        int tau_dyn_thousand_1 = tau_Nm2thousand(tau_pd_hip_r, 0.3);
                        int tau_dyn_thousand_2 = tau_Nm2thousand(tau_pd_knee_r, 0.3);
                        int tau_dyn_thousand_3 = tau_Nm2thousand(tau_pd_ankle_r, 0.3);

                        int tau_dyn_thousand_4 = tau_Nm2thousand(tau_pd_hip_l, 0.3);
                        int tau_dyn_thousand_5 = tau_Nm2thousand(tau_pd_knee_l, 0.3);
                        int tau_dyn_thousand_6 = tau_Nm2thousand(tau_pd_ankle_l, 0.3);


#ifndef Drag_Impendance
                        if (!FLG_RIGHT_GC_1st) {
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);
                        }

//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_1);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_2);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_3);

                        if (LEFT_GC_START) {
                            if (!FLG_LEFT_GC_1st) {
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);
                            }
//                            EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_4);
//                            EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_5);
//                            EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_6);
                        }
#endif

#ifdef Drag_Impendance
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);

                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_1);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_2);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_3);

                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);

                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_4);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_5);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_6);
#endif

                        /** ----------------------------------------------------------------------------------- */
                        if (!(cycle_count % 10)) { // show message per 1 ms.
                            std::cout << "===============================" << std::endl;
                            std::cout << "task_working_Impedance" << std::endl;
                            std::cout << "===============================" << std::endl;
                            std::cout << "current P = [" << P_main << ',' << P_sub << ']' << std::endl;
                            std::cout << "next P = " << P_update << std::endl;
                            std::cout << "time: " << time_cnt++ / TASK_FREQUENCY << " [s] " << std::endl;
                            std::cout << "dq_e_l: " << std::endl;
                            std::cout << dq_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "q_e_l: " << std::endl;
                            std::cout << q_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "l_Hip motor [rad] = " << r_hip_m_rad << std::endl;
                            std::cout << "l_Hip link [rad] = " << r_hip_rad << std::endl;
                            std::cout << "l_Ankle motor [cnt] = "
                                      << EC_READ_S32(domainTx_pd + offset.actual_position[l_ankle]) << std::endl;
                            std::cout << "r_Ankle link [cnt] = "
                                      << EC_READ_S32(domainTx_pd + offset.second_position[l_ankle]) << std::endl;


                            for (int i = 0; i < active_num; i++)
                                error_code_sequence.emplace_back(EC_READ_U16(domainTx_pd + offset.Error_code[i]));
                            std::cout << " ---- Servo Error Status ---- " << std::endl;

                            for (int j = 0; j < active_num; j++) {
                                switch (error_code_sequence[j]) {
                                    case 0x3331:
                                        std::cout << "Driver [" << j << "] <-- Field circuit interrupted" << std::endl;
                                        break;
                                    case 0x2220:
                                        std::cout << "Driver [" << j
                                                  << "] <-- Continuous over current (device internal)"
                                                  << std::endl;
                                        break;
                                    case 0x0000:
                                        std::cout << "Driver [" << j << "] <-- NULL" << std::endl;
                                        break;
                                    default:
                                        std::cout << "Driver [" << j << "] <-- Ref to data sheet." << std::endl;
                                        break;
                                }
                            }

                            std::vector<unsigned int>().swap(error_code_sequence);

                        }
                        right_outFile << hip_ref_rad_r << ',' << knee_ref_rad_r << ',' << ankle_ref_rad_r << ','
                                      << r_hip_rad << ',' << r_knee_rad << ',' << r_ankle_rad << ','
                                      << r_hip_m_vel << ',' << r_knee_m_vel << ',' << r_ankle_m_vel << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r << ',' << Gc_main
                                      << std::endl;

                        right_velFile << hip_ref_vel_r << ',' << knee_ref_vel_r << ',' << ankle_ref_vel_r << ','
                                      << r_hip_vel_f << ',' << r_knee_vel_f << ',' << r_ankle_vel_f << ','
                                      << r_hip_m_vel << ',' << r_knee_m_vel << ',' << r_ankle_m_vel << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r << std::endl;

                        right_save_file << r_hip_Impedance << ',' << r_hip_tau_spr << ',' << r_knee_Impedance << ','
                                        << r_knee_tau_spr << ',' << r_ankle_Impedance << ',' << r_ankle_tau_spr << ','
                                        << lever_arm_3_rad - link_3_rad << std::endl;

                        right_filter_file << vel_RPM2rad(link_3_vel) << ',' << r_hip_vel_f << ',' << compensation_r(2)
                                          << std::endl;

                        left_outFile << hip_ref_rad_l << ',' << knee_ref_rad_l << ',' << Ankle_ref_rad_l << ','
                                     << l_hip_rad << ',' << l_knee_rad << ',' << l_ankle_rad << ','
                                     << l_hip_m_vel << ',' << l_knee_m_vel << ',' << l_ankle_m_vel << ','
                                     << tau_mot_4 << ',' << tau_mot_5 << ',' << tau_mot_6 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l
                                     << std::endl;

                        left_velFile << hip_ref_vel_l << ',' << knee_ref_vel_l << ',' << Ankle_ref_vel_l << ','
                                     << l_hip_vel_f << ',' << l_knee_vel_f << ',' << l_ankle_vel_f << ','
                                     << l_hip_m_vel << ',' << l_knee_m_vel << ',' << l_ankle_m_vel << ','
                                     << tau_mot_4 << ',' << tau_mot_5 << ',' << tau_mot_6 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l << std::endl;

                        left_save_file << l_hip_Impedance << ',' << l_hip_tau_spr << ',' << l_knee_Impedance << ','
                                       << l_knee_tau_spr << ',' << l_ankle_Impedance << ',' << l_ankle_tau_spr << ','
                                       << lever_arm_3_rad - link_3_rad << std::endl;

                        left_filter_file << vel_RPM2rad(link_5_vel) << ',' << l_knee_vel_f << std::endl;
                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;

                    if (curve_cnt_right >= P_main * (30 + 0.35)) {
                        EtherCAT_ONLINE = false;
                        std::cout << "[End of Program...]" << std::endl;
                        break;
                    }

                }
                    break;

                case task_working_Sit2Stand: {

                    if (!(cycle_count % 10)) { // show message per 10 ms. (base freq = 1Khz)
                        std::cout << "=====================" << std::endl;
                        std::cout << "task_working_Sit2Stand" << std::endl;
                        std::cout << "=====================" << std::endl;
                    }

                    double s2s_r_hip_vel = r_Hip_reset_pid.pid_control(1, r_hip_rad, 3000);
                    double s2s_r_knee_vel = r_Knee_reset_pid.pid_control(-0.7, r_knee_rad, 3000);
                    double s2s_r_ankle_vel = r_Ankle_reset_pid.pid_control(-0.3, r_ankle_rad, 3000);
                    EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], s2s_r_hip_vel);
                    EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_knee], s2s_r_knee_vel);
                    EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_ankle], s2s_r_ankle_vel);

                    double s2s_l_hip_vel = l_Hip_reset_pid.pid_control(1, l_hip_rad, 3000);
                    double s2s_l_knee_vel = l_Knee_reset_pid.pid_control(-0.7, l_knee_rad, 3000);
                    double s2s_l_ankle_vel = r_Ankle_reset_pid.pid_control(-0.3, l_ankle_rad, 3000);
                    EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_hip], s2s_l_hip_vel);
                    EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_knee], s2s_l_knee_vel);
                    EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_ankle], s2s_r_ankle_vel);

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;
                }
                    break;

                case task_working_CSP_tracking: {
                    static int cnt_startup_r_hip = 0;
                    static int cnt_startup_r_knee = 0;
                    static int cnt_startup_r_ankle = 0;
                    static int cnt_startup_l_hip = 0;
                    static int cnt_startup_l_knee = 0;
                    static int cnt_startup_l_ankle = 0;

                    if (!FLG_TRACKING_STARTUP) {
                        cnt_startup_r_hip = cnt_motor_3;
                        cnt_startup_r_knee = cnt_motor_2;
                        cnt_startup_r_ankle = cnt_motor_1;

                        cnt_startup_l_hip = cnt_motor_4;
                        cnt_startup_l_knee = cnt_motor_5;
                        cnt_startup_l_ankle = cnt_motor_6;

                        FLG_TRACKING_STARTUP = true;
                    }

                    if (!FLG_RIGHT_GC_1st) {
                        Gc_main = 1.0 * (curve_cnt_right % P_main) / P_main + 0.4;
                        if (Gc_main >= 1.0) {
                            Gc_main = 0.0;
                            FLG_RIGHT_GC_1st = true;
                            LEFT_GC_START = true;
                            curve_cnt_right = 0;
                        }
                    } else {
                        Gc_main = 1.0 * (curve_cnt_right % P_main) / P_main;
                        if (Gc_main == 0.0) // GC can not reach 1.0
                        {
                            curve_cnt_right = 0;
                            P_main = P_update;
                        }
                    }

                    curve_cnt_right++;

                    if (!POST_RESET) {
                        double hip_ref_rad_r = 0.6 * base_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_r = 0.6 * base_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee);
                        double ankle_ref_rad_r = 1.0 * base_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_r =
                                0.6 * differentia_1st_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_knee, P_main);
                        double knee_ref_vel_r =
                                0.6 * differentia_1st_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P_main);
                        double ankle_ref_vel_r =
                                1.0 * differentia_1st_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P_main);

                        int r_hip_tracking_cnt =
                                cnt_startup_r_hip + 100 * pos_rad2inc_17bits(hip_ref_rad_r - right_hip_init_rad);
                        int r_knee_tracking_cnt =
                                cnt_startup_r_knee + 80 * pos_rad2inc_17bits(knee_ref_rad_r - right_knee_init_rad);
                        int r_ankle_tracking_cnt =
                                cnt_startup_r_ankle + 80 * pos_rad2inc_17bits(ankle_ref_rad_r - right_ankle_init_rad);


                        if (FLG_RIGHT_GC_1st) {
                            if (!FLG_LEFT_GC_1st) {
                                Gc_sub = 1.0 * (curve_cnt_left % P_sub) / P_sub + 0.4;
                                if (Gc_sub >= 1.0) {
                                    Gc_sub = 0.0;
                                    FLG_LEFT_GC_1st = true;
                                    curve_cnt_left = 0;
                                }
                            } else {
                                Gc_sub = 1.0 * (curve_cnt_left % P_sub) / P_sub;
                                if (Gc_sub == 0.0) {
                                    curve_cnt_left = 0;
                                    P_sub = P_update;
                                }
                            }
                        } else // stand-by state
                            Gc_sub = 0.4;

                        if (FLG_RIGHT_GC_1st)
                            curve_cnt_left++;

                        double hip_ref_rad_l = 0.6 * base_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_l = 0.6 * base_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee);
                        double Ankle_ref_rad_l = base_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_l =
                                0.6 * differentia_1st_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_knee, P_sub);
                        double knee_ref_vel_l =
                                0.6 * differentia_1st_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P_sub);
                        double Ankle_ref_vel_l =
                                differentia_1st_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P_sub);

                        int l_hip_tracking_cnt =
                                cnt_startup_l_hip + 100 * pos_rad2inc_17bits(hip_ref_rad_l - left_hip_init_rad);
                        int l_knee_tracking_cnt =
                                cnt_startup_l_knee + 80 * pos_rad2inc_17bits(knee_ref_rad_l - left_knee_init_rad);
                        int l_ankle_tracking_cnt =
                                cnt_startup_l_ankle + 80 * pos_rad2inc_17bits(Ankle_ref_rad_l - left_ankle_init_rad);


                        /**
                         ** Filter Part (middle value filter)
                         **/

                        double l_hip_vel_f, l_knee_vel_f, l_ankle_vel_f;
                        double r_hip_vel_f, r_knee_vel_f, r_ankle_vel_f;
                        int filter_window = 30;
                        int avg_window = 10;

                        if (curve_cnt_right < filter_window) {
                            if (curve_cnt_right < avg_window) {

                                input_1[curve_cnt_right] = r_ankle_vel; // right part is assigned to [rad/s]
                                input_2[curve_cnt_right] = r_knee_vel;
                                input_3[curve_cnt_right] = r_hip_vel;

                                input_4[curve_cnt_right] = l_hip_vel;
                                input_5[curve_cnt_right] = l_knee_vel;
                                input_6[curve_cnt_right] = l_ankle_vel;

                            } else {
                                double tmp1 =
                                        sum_of_array(input_1, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp2 =
                                        sum_of_array(input_2, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp3 =
                                        sum_of_array(input_3, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                input_1[curve_cnt_right] = tmp1;
                                input_2[curve_cnt_right] = tmp2;
                                input_3[curve_cnt_right] = tmp3;

                                double tmp4 =
                                        sum_of_array(input_4, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp5 =
                                        sum_of_array(input_5, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                double tmp6 =
                                        sum_of_array(input_6, curve_cnt_right - avg_window, curve_cnt_right) /
                                        avg_window;
                                input_4[curve_cnt_right] = tmp4;
                                input_5[curve_cnt_right] = tmp5;
                                input_6[curve_cnt_right] = tmp6;


                            }
                            r_hip_vel_f = input_3[curve_cnt_right];
                            r_knee_vel_f = input_2[curve_cnt_right];
                            r_ankle_vel_f = input_1[curve_cnt_right];

                            l_hip_vel_f = input_4[curve_cnt_right];
                            l_knee_vel_f = input_5[curve_cnt_right];
                            l_ankle_vel_f = input_6[curve_cnt_right];

                        } else {
                            left_shift_array(input_1, r_ankle_vel, filter_window);
                            left_shift_array(input_2, r_knee_vel, filter_window);
                            left_shift_array(input_3, r_hip_vel, filter_window);
                            bw_link.filter(input_1, output_1, filter_window, 0, true);
                            bw_link.filter(input_2, output_2, filter_window, 0, true);
                            bw_link.filter(input_3, output_3, filter_window, 0, true);
                            r_hip_vel_f = output_3[filter_window - 1];
                            r_knee_vel_f = output_2[filter_window - 1];
                            r_ankle_vel_f = output_1[filter_window - 1];

                            left_shift_array(input_4, l_hip_vel, filter_window);
                            left_shift_array(input_5, l_knee_vel, filter_window);
                            left_shift_array(input_6, l_ankle_vel, filter_window);
                            bw_link.filter(input_4, output_4, filter_window, 0, true);
                            bw_link.filter(input_5, output_5, filter_window, 0, true);
                            bw_link.filter(input_6, output_6, filter_window, 0, true);
                            l_hip_vel_f = output_4[filter_window - 1];
                            l_knee_vel_f = output_5[filter_window - 1];
                            l_ankle_vel_f = output_6[filter_window - 1];

                        }

                        // RPM -> rad/s
                        //TODO: why x2?

                        l_hip_vel_f *= 2;
                        l_knee_vel_f *= 2;
                        l_ankle_vel_f *= 2;

                        r_hip_vel_f *= 2;
                        r_knee_vel_f *= 2;
                        r_ankle_vel_f *= 2;

                        dq_r << hip_ref_vel_r, (knee_ref_vel_r), (ankle_ref_vel_r);
                        q_r << hip_ref_rad_r, (knee_ref_rad_r), (ankle_ref_rad_r);
                        dq_e_r << hip_ref_vel_r - r_hip_vel_f, (knee_ref_vel_r - r_knee_vel_f), (ankle_ref_vel_r -
                                                                                                 r_ankle_vel_f);
                        q_e_r << hip_ref_rad_r - r_hip_rad, (knee_ref_rad_r - r_knee_rad), (ankle_ref_rad_r -
                                                                                            r_ankle_rad);

                        //left part =>
                        dq_l << hip_ref_vel_l, (knee_ref_vel_l), (Ankle_ref_vel_l);
                        q_l << hip_ref_rad_l, (knee_ref_rad_l), (Ankle_ref_rad_l);
                        dq_e_l << hip_ref_vel_l - l_hip_vel_f, (knee_ref_vel_l - l_knee_vel_f), (Ankle_ref_vel_l -
                                                                                                 l_ankle_vel_f);
                        q_e_l << hip_ref_rad_l - l_hip_rad, (knee_ref_rad_l - l_knee_rad), (Ankle_ref_rad_l -
                                                                                            l_ankle_rad);

                        if (!FLG_RIGHT_GC_1st) {
                            int r_hip_OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[r_hip]);
                            int r_knee_OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[r_knee]);
                            int r_ankle_OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[r_ankle]);

                            if (r_hip_OP_mode != CSP)
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CSP);
                            if (r_knee_OP_mode != CSP)
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CSP);
                            if (r_ankle_OP_mode != CSP)
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CSP);
                        }

                        EC_WRITE_S32(domainRx_pd + offset.target_position[r_hip], r_hip_tracking_cnt);
                        EC_WRITE_S32(domainRx_pd + offset.target_position[r_knee], r_knee_tracking_cnt);
                        EC_WRITE_S32(domainRx_pd + offset.target_position[r_ankle], r_ankle_tracking_cnt);

                        if (LEFT_GC_START) {
                            if (!FLG_LEFT_GC_1st) {
                                int l_hip_OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[l_hip]);
                                int l_knee_OP_mode = EC_READ_S8(
                                        domainTx_pd + offset.modes_of_operation_display[l_knee]);
                                int l_ankle_OP_mode = EC_READ_S8(
                                        domainTx_pd + offset.modes_of_operation_display[l_ankle]);

//                                if (l_hip_OP_mode != CSP)
//                                    EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CSP);
//                                if (l_knee_OP_mode != CSP)
//                                    EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CSP);
//                                if (l_ankle_OP_mode != CSP)
//                                    EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CSP);
                            }
//                            EC_WRITE_S16(domainRx_pd + offset.target_position[l_hip], l_hip_tracking_cnt);
//                            EC_WRITE_S16(domainRx_pd + offset.target_position[l_knee], l_knee_tracking_cnt);
//                            EC_WRITE_S16(domainRx_pd + offset.target_position[l_ankle], l_ankle_tracking_cnt);
                        }

                        /** ----------------------------------------------------------------------------------- */
                        if (!(cycle_count % 10)) { // show message per 1 ms.
                            std::cout << "===============================" << std::endl;
                            std::cout << "task_working_CSP_tracking" << std::endl;
                            std::cout << "time: " << time_cnt / TASK_FREQUENCY << " [s] " << std::endl;
                            std::cout << "Current P = " << P_main << std::endl;
                            std::cout << "===============================" << std::endl;

                            for (int i = 0; i < active_num; i++)
                                error_code_sequence.emplace_back(EC_READ_U16(domainTx_pd + offset.Error_code[i]));
                            std::cout << " ---- Servo Error Status ---- " << std::endl;

                            for (int j = 0; j < active_num; j++) {
                                switch (error_code_sequence[j]) {
                                    case 0x3331:
                                        std::cout << "Joint [" << j << "] --> Field circuit interrupted" << std::endl;
                                        break;
                                    case 0x2220:
                                        std::cout << "Joint [" << j << "] --> Continuous over current (device internal)"
                                                  << std::endl;
                                        break;
                                    case 0x0000:
                                        std::cout << "Joint [" << j << "] --> NULL" << std::endl;
                                        break;
                                    default:
                                        std::cout << "Ref to data sheet." << std::endl;
                                        break;
                                }
                            }

                            std::vector<unsigned int>().swap(error_code_sequence);

                        }
                        right_outFile << hip_ref_rad_r << ',' << knee_ref_rad_r << ',' << ankle_ref_rad_r << ','
                                      << r_hip_rad << ',' << r_knee_rad << ',' << r_ankle_rad << ','
                                      << r_hip_m_vel << ',' << r_knee_m_vel << ',' << r_ankle_m_vel << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << std::endl;

                        right_velFile << hip_ref_vel_r << ',' << knee_ref_vel_r << ',' << ankle_ref_vel_r << ','
                                      << r_hip_vel_f << ',' << r_knee_vel_f << ',' << r_ankle_vel_f << ','
                                      << r_hip_m_vel << ',' << r_knee_m_vel << ',' << r_ankle_m_vel << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ',' << std::endl;

                        right_filter_file << vel_RPM2rad(link_3_vel) << ',' << r_hip_vel_f << ',' << std::endl;

                        right_save_file << r_hip_tracking_cnt << ',' << r_knee_tracking_cnt << ','
                                        << r_ankle_tracking_cnt << std::endl;

                        left_outFile << hip_ref_rad_l << ',' << knee_ref_rad_l << ',' << Ankle_ref_rad_l << ','
                                     << l_hip_rad << ',' << l_knee_rad << ',' << l_ankle_rad << ','
                                     << l_hip_m_vel << ',' << l_knee_m_vel << ',' << l_ankle_m_vel << ','
                                     << tau_mot_4 << ',' << tau_mot_5 << ',' << tau_mot_6 << ','
                                     << std::endl;

                        left_velFile << hip_ref_vel_l << ',' << knee_ref_vel_l << ',' << Ankle_ref_vel_l << ','
                                     << l_hip_vel_f << ',' << l_knee_vel_f << ',' << l_ankle_vel_f << ','
                                     << l_hip_m_vel << ',' << l_knee_m_vel << ',' << l_ankle_m_vel << ','
                                     << tau_mot_4 << ',' << tau_mot_5 << ',' << tau_mot_6 << ',' << std::endl;

                        left_filter_file << vel_RPM2rad(link_5_vel) << ',' << l_knee_vel_f << std::endl;

                        left_save_file << l_hip_tracking_cnt << ',' << l_knee_tracking_cnt << ','
                                       << l_ankle_tracking_cnt << std::endl;

                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;

                    id_ready_cnt++;
                    if (curve_cnt_right >= P_main * (30 + 0.35)) {
                        EtherCAT_ONLINE = false;
                        std::cout << "[End of Program...]" << std::endl;
                        break;
                    }

                    time_cnt++;
                }
                    break;

                case task_working_Checking: {
                    /** A. Reset to default motor position(vertical direction as reference link) */
                    if (!(cycle_count % 10)) { // show message per 10 ms. (base freq = 1Khz)
                        std::cout << "=====================" << std::endl;
                        std::cout << "task_working_RESET" << std::endl;
                        std::cout << "Time : " << PD_cnt / TASK_FREQUENCY << "[s]" << std::endl;
                        std::cout << "=====================" << std::endl;
                        std::cout << "vel of [l_ankle] motor = "
                                  << r_ankle_vel << " [rad/s]"
                                  << std::endl;
                        std::cout << "---------------------" << std::endl;
                        std::cout << "       spr cnt: " << std::endl;
                        std::cout << "---------------------" << std::endl;
                        std::cout << "r_hip   =>" << cnt_spring_3 << std::endl;
                        std::cout << "r_knee  =>" << cnt_spring_2 << std::endl;
                        std::cout << "r_ankle =>" << cnt_spring_1 << std::endl;
                        std::cout << "l_hip   =>" << cnt_spring_4 << std::endl;
                        std::cout << "l_knee  =>" << cnt_spring_5 << std::endl;
                        std::cout << "l_ankle =>" << cnt_spring_6 << std::endl;
                        std::cout << "---------------------" << std::endl;
                        std::cout << "      motor cnt:" << std::endl;
                        std::cout << "---------------------" << std::endl;
                        std::cout << "r_hip   =>" << cnt_motor_4 << std::endl;
                        std::cout << "r_knee  =>" << cnt_motor_5 << std::endl;
                        std::cout << "r_ankle =>" << cnt_motor_6 << std::endl;
                        std::cout << "l_hip   =>" << cnt_motor_3 << std::endl;
                        std::cout << "l_knee  =>" << cnt_motor_2 << std::endl;
                        std::cout << "l_ankle =>" << cnt_motor_1 << std::endl;

                        std::cout << "---------------------" << std::endl;
                        std::cout << "calibrated link_rad: " << std::endl;
                        std::cout << "---------------------" << std::endl;
                        std::cout << std::setprecision(3) << "r_hip   = " << r_hip_rad << std::endl;
                        std::cout << std::setprecision(3) << "r_knee  = " << r_knee_rad << std::endl;
                        std::cout << std::setprecision(3) << "r_ankle = " << r_ankle_rad << std::endl;
                        std::cout << std::setprecision(3) << "l_hip   = " << l_hip_rad << std::endl;
                        std::cout << std::setprecision(3) << "l_knee  = " << l_knee_rad << std::endl;
                        std::cout << std::setprecision(3) << "l_ankle = " << l_ankle_rad << std::endl;


                        for (int i = 0; i < active_num; i++) {
                            unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);
                            switch (E_code) {
                                case 0x7500:
                                    std::cout << " Communication Error." << std::endl;
                                    break;
                                case 0x7300:
                                    std::cout << " Sensor Error(CRC or AckBits)." << std::endl;
                                    break;
                                case 0x2220:
                                    std::cout << "  Continuous over current" << std::endl;
                                    break;
                                case 0x0000:
                                    std::cout << "  No Fault." << std::endl;
                                    break;
                                case 0x3331:
                                    std::cout << " Field circuit interrupted" << std::endl;
                                    break;
                                default:
                                    std::cout << " Other Error. (ref to DataSheet)" << std::endl;
                                    break;
                            }
                        }

                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;
                }
                    break;

                default:
                    break;
            }
        }
            break;
    }

    // send process data objects
    ecrt_domain_queue(domainRx);
    ecrt_domain_queue(domainTx);
    ecrt_master_send(master);
}

