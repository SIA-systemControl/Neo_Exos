#include <iostream>
#include <fstream>
#include <sstream>
//#include "bodyBendDetection.h"
#include "Period.h"
#include "calibration.h"

bool FLG_TASK_SELECT = true;
bool FLG_Impedance_K_modified = true;

pthread_mutex_t mutex;
pthread_cond_t cond;

struct command_para {
    int task_cmd;
    double impedance_K[3];
    double impedance_B[3];
};

task_list g_task_list[] = {
        {task_working_RESET, "run task_working_RESET"},
        {task_working_CALIBRATION, "run task_working_CALIBRATION"},
        {task_working_Identification, "run task_working_Identification"},
        {task_working_Impedance, "run task_working_Impedance"},
        {task_working_CSP_tracking, "run task_working_CSP_tracking"},
        {task_working_Noload2Sit, "run task_working_Noload2Sit"},
        {task_working_Impedance_GRF, "run task_working_Impedance_GRF"},
        {task_working_Transparency, "run task_working_Transparency"},
        {task_working_Checking, "run task_working_Checking"},
};

void printf_task_str() {
    int size = sizeof(g_task_list) / sizeof(g_task_list[0]);
    printf("support task mode: \n");
    printf(" ----------------------------------------------- \n");
    for (int i = 0; i < size; i++) {
        printf("    [.%d] --> %s\n", i, g_task_list[i].info);
        printf(" ----------------------------------------------- \n");
    }
    printf("e.g. input \".0\" to run task_working_RESET.\n");
}


void *robotcontrol(void *arg);

int main(int argc, char *argv[]) {
    /* Thread-related operation */
    pthread_mutex_init(&mutex, nullptr);

    pthread_t robot;            // new pthread object
    pthread_attr_t attr_robot;  // new pthread object attribute

    pthread_t Period;
    pthread_attr_t attr_Period;

    pthread_t Calibration;
    pthread_attr_t attr_Calibration;

    struct command_para commandPara;

    int task_cmd = 0;
    int task_cmd_size = sizeof(g_task_list) / sizeof(g_task_list[0]);

    printf_task_str();
    char input_data[2] = {0};

    while (FLG_TASK_SELECT) {

        fgets(input_data, 20, stdin);

        if (input_data[0] == '.') {
            int id = atoi(&input_data[1]);

            if (id >= task_cmd_size || id < 0) {
                printf("invalid command, confirm and input again.\n");
                continue;
            } else {
                task_cmd = g_task_list[id].taskSelect;
                FLG_TASK_SELECT = false;
            }
        }
    }

    if (task_cmd != task_select(task_working_CALIBRATION)) {
        std::cout << " == Modified Impedance K ? (y/n) " << std::endl;
        char Confirm_Modified_K[2] = {0};

        while (FLG_Impedance_K_modified) {
            fgets(Confirm_Modified_K, 2, stdin);
            if (Confirm_Modified_K[0] == 'y') {
                std::cout << "Enter modified Impedance K_hip: " << std::endl;
                std::cin >> commandPara.impedance_K[0];

                std::cout << "Enter modified Impedance K_knee: " << std::endl;
                std::cin >> commandPara.impedance_K[1];

                std::cout << "Enter modified Impedance K_ankle: " << std::endl;
                std::cin >> commandPara.impedance_K[2];

                commandPara.impedance_B[0] = 0;
                FLG_Impedance_K_modified = false;
            } else if (Confirm_Modified_K[0] == 'n') {
                commandPara.impedance_K[0] = 0;

                commandPara.impedance_B[0] = 0;
                std::cout << "Use default parameters." << std::endl;
                FLG_Impedance_K_modified = false;
            }

        }
        std::cout << "===================================================" << std::endl;

        if (Confirm_Modified_K[0] == 'y') {
            std::cout << "Modified Impedance para: [" << commandPara.impedance_K[0] << ',' << commandPara.impedance_K[1]
                      << ',' << commandPara.impedance_K[2] << ']' << std::endl;
        } else
            std::cout << "Impedance para using default configure: [80,80,40]" << std::endl;
        std::cout << "===================================================" << std::endl;
    }

    std::cout << "Selected task_cmd = " << g_task_list[task_cmd].info << std::endl;
    commandPara.task_cmd = task_cmd;
    for (int i = 0; i < 3; i++) {
        std::cout << "Confirm input task_cmd in [" << 3 - i << "] second(s)." << std::endl;
        usleep(1000000);
    }

    if (pthread_attr_init(&attr_robot) != 0)
        perror("[ROBOT-THREAD INIT FAILURE!]");

    if (pthread_create(&robot, &attr_robot, robotcontrol, (void *) &commandPara) != 0) {
        perror("[ROBOT-THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }

    if (pthread_attr_init(&attr_Period) != 0)
        perror("[PERIOD-THREAD INIT FAILURE!]");

    if (pthread_create(&Period, &attr_Period, PeriodControl, nullptr) != 0) {
        perror("[PERIOD-THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }

    if (pthread_attr_init(&attr_Calibration) != 0)
        perror("[Calibration-THREAD INIT FAILURE!]");

    if (pthread_create(&Calibration, &attr_Calibration, CalibrationControl, nullptr) != 0) {
        perror("[Calibration-THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }



    pthread_join(robot, nullptr); // wait for releasing
    pthread_join(Period, nullptr);
    pthread_join(Calibration, nullptr);

    std::cout << "[ROBOT-THREAD DESTROYED!]" << std::endl;
    std::cout << "[PERIOD-THREAD DESTROYED!]" << std::endl;
    std::cout << "[Calibration-THREAD DESTROYED!]" << std::endl;

    pthread_attr_destroy(&attr_robot);
    pthread_attr_destroy(&attr_Period);
    pthread_attr_destroy(&attr_Calibration);

    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);

    return EXIT_SUCCESS;
}
