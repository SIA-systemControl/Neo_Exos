//
// Created by yc on 2021/10/25.
//
#include "Period.h"

int kfd = 0;
struct termios cooked, raw;
int P_update = 3000;
char c;
extern bool FLG_pthread_online;

//void quit(int sig) {
//    (void) sig;
//    tcsetattr(kfd, TCSANOW, &cooked);
//    exit(0);
//}

void *PeriodControl(void *arg) {
//    signal(SIGINT, quit);
    KeyDetect();
    exit(EXIT_SUCCESS);
}


void KeyDetect() {
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("=========================");
    puts("  READING FROM KEYBOARD  ");
    puts("=========================");

    while (FLG_pthread_online) { // when FLG_pthread_online <- false, press any key to release this thread
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }


        switch (c) {
            case KEY_UP: {
                P_update += 200;
                printf("P = %d.\n", P_update);
            }
                break;
            case KEY_DOWN: {
                P_update -= 200;
                if (P_update < 0)
                    P_update = 0;
                printf("P = %d.\n", P_update);
            }
                break;
            default:
                break;
        }
    }
    puts("===================");
    puts("FLG_pthread_online <- false");
    puts("===================");
    tcsetattr(kfd, TCSANOW, &cooked);
}



