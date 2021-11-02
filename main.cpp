#include <iostream>
#include <fstream>
#include <sstream>
#include "dataDecode.h"
//#include "bodyBendDetection.h"
#include "Period.h"

bool FLG_TASK_SELECT = true;

pthread_mutex_t mutex;
pthread_cond_t cond;

task_list g_task_list[] = {
        {task_working_RESET,          "run task_working_RESET"},
        {task_working_Control,        "run task_working_Control"},
        {task_working_Identification, "run task_working_Identification"},
        {task_working_Impedance,      "run task_working_Impedance"},
        {task_working_Sit2Stand,      "run task_working_Sit2Stand"},
        {task_working_CSP_tracking,   "run task_working_CSP_tracking"},
        {task_working_Checking,       "run task_working_Checking"},
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

    int task_cmd = 0;
    int task_cmd_size = sizeof(g_task_list) / sizeof(g_task_list[0]);

    printf_task_str();
    char input_data[2] = {0};

    while (FLG_TASK_SELECT) {

        fgets(input_data, 20, stdin);

        if (input_data[0] == '.') {
            int id = atoi(&input_data[1]);

            if (id > task_cmd_size || id < 0) {
                printf("not valid input.\n");
                continue;
            }

            task_cmd = g_task_list[id].taskSelect;

            std::cout << g_task_list[id].info << std::endl;
            FLG_TASK_SELECT = false;
        }
    }

    if (pthread_attr_init(&attr_robot) != 0)
        perror("[ROBOT-THREAD INIT FAILURE!]");

    if (pthread_create(&robot, &attr_robot, robotcontrol, (void *) &task_cmd) != 0) {
        perror("[ROBOT-THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }

    if (pthread_attr_init(&attr_Period) != 0)
        perror("[PERIOD-THREAD INIT FAILURE!]");

    if (pthread_create(&Period, &attr_Period, PeriodControl, nullptr) != 0) {
        perror("[THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }

//    pthread_join(robot, nullptr); // wait for releasing
    pthread_join(Period, nullptr);

    std::cout << "[ROBOT-THREAD DESTROYED!]" << std::endl;
    std::cout << "[PERIOD-THREAD DESTROYED!]" << std::endl;

    pthread_attr_destroy(&attr_robot);
    pthread_attr_destroy(&attr_Period);

    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);

    return EXIT_SUCCESS;
}
