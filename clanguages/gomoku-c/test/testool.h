#ifndef TESTOOL_H
#define TESTOOL_H

#include <stdio.h>

#include "configuration.h"

#define M_test_int(value, expected) _reportIntegerTest( \
    #value, \
    __FUNCTION__, \
    __FILE__, \
    __LINE__, \
    value, \
    expected, \
    M_is_test_verbose \
);

#define M_run_test_suite(__testSuite) \
    setupTestSuite(); \
    printf("Start test suite [%s]\n", #__testSuite); \
    __testSuite(); \
    reportTestSuite();

void _reportIntegerTest(
    char const * const testedName,
    char const * const invokerName,
    char const * const fileName,
    int const lineNumber,
    int const value,
    int const expected,
    bool const isVerbose
);

void setupTestSuite();

void reportTestSuite();

#endif // guard end for TESTOOL_H
