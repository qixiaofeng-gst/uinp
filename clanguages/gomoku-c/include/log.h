//
// Created by qixiaofeng on 2020/7/29.
//

#ifndef GOMOKU_C_LOG_H
#define GOMOKU_C_LOG_H

#include <stdio.h>

#define M_debug_line() printf("\033[38;2;255;255;0mDebug line ====>>>\033[0m %s %s %d\n", \
__FILE__, __FUNCTION__, __LINE__);

void debug_print(char const *msgFormat, ...);

FILE *get_debug_log_file();

#endif //GOMOKU_C_LOG_H
