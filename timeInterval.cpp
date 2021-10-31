//
// Created by sia on 21-9-6.
//

#include "timeInterval.h"

std::stack<clock_t> tictoc_stack;

void tic() {
    if (tictoc_stack.size())
        tictoc_stack.pop();
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed (s): "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
}

double toc_double() {
    double val = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;

    return val;
}