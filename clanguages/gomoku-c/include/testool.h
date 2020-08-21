#ifndef TESTOOL_H
#define TESTOOL_H

#include <stdio.h>

#include "configuration.h"

#define M_test_int(value, expected) report_integer_test( \
    #value, \
    __FUNCTION__, \
    __FILE__, \
    __LINE__, \
    value, \
    expected, \
    m_is_test_verbose \
)

#define M_run_test_suite(__testSuite) \
    setup_test_suite(); \
    printf("Start test suite [%s]\n", #__testSuite); \
    __testSuite(); \
    report_test_suite()

void report_integer_test(
    char const * testedName,
    char const * invokerName,
    char const * fileName,
    int lineNumber,
    int value,
    int expected,
    bool isVerbose
);

void setup_test_suite();

void report_test_suite();

#endif // guard end for TESTOOL_H
