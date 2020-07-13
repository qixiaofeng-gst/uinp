#ifndef TESTOOL_H
#define TESTOOL_H

#include <stdio.h>

#include "configuration.h"

#define M_test_int(value, expected) _report_integer_test( \
    #value, \
    __FUNCTION__, \
    __FILE__, \
    __LINE__, \
    value, \
    expected, \
    m_is_test_verbose \
);

#define M_run_test_suite(__testSuite) \
    setup_test_suite(); \
    printf("Start test suite [%s]\n", #__testSuite); \
    __testSuite(); \
    report_test_suite();

void _report_integer_test(
    char const * const testedName,
    char const * const invokerName,
    char const * const fileName,
    int const lineNumber,
    int const value,
    int const expected,
    bool const isVerbose
);

void setup_test_suite();

void report_test_suite();

/* ======= Belows are utilities for debugging. ======= */
#define M_debug_line() printf("\033[38;2;255;255;0mDebug line ====>>>\033[0m %s %s %d\n", \
__FILE__, __FUNCTION__, __LINE__);

#endif // guard end for TESTOOL_H
