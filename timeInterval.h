//
// Created by sia on 21-9-6.
//

#ifndef SOFTEXOS_TIMEINTERVAL_H
#define SOFTEXOS_TIMEINTERVAL_H

#endif //SOFTEXOS_TIMEINTERVAL_H

#include <iostream>
#include <stack>
#include <ctime>

extern std::stack<clock_t> tictoc_stack;

void tic();
void toc();
double toc_double();