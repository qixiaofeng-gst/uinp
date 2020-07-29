//
// Created by qixiaofeng on 2020/7/29.
//

#include <stdio.h>
#include <stdarg.h>

#include "log.h"

FILE *
get_debug_log_file() {
    static FILE *debug_log = NULL;
    if (NULL == debug_log) {
        debug_log = fopen("debug.log", "w");
    }
    return debug_log;
}

void
debug_print(char const *msgFormat, ...) {
    va_list args;
    va_start(args, msgFormat);
    vfprintf(get_debug_log_file(), msgFormat, args);
    va_end(args);
}
