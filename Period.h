//
// Created by yc on 2021/10/25.
//

#ifndef NEO_TEST_PERIOD_H
#define NEO_TEST_PERIOD_H

#include <termio.h>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>

#define KEY_UP 0x41
#define KEY_DOWN 0x42

void *PeriodControl(void *arg);
void KeyDetect();

extern void SIG_handle(int sig);

typedef enum task_select {
    task_working_RESET,         // 0
    task_working_Identification,// 1
    task_working_Impedance,     // 2
    task_working_CSP_tracking,  // 3
    task_working_Checking       // 4
} task_select;

typedef struct task_list {
    task_select taskSelect;
    unsigned char info[50];
} task_list;



#endif //NEO_TEST_PERIOD_H
