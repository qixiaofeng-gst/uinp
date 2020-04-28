#ifndef TESTOOL_H
#define TESTOOL_H

#include <stdio.h>

#define M_test_int(value, expected) reportIntegerTest( \
    #value, \
    value, \
    expected \
);

#define M_run_test_suite(__testSuite) \
    setupTestSuite(); \
    printf("Start test suite [%s]\n", #__testSuite); \
    __testSuite(); \
    reportTestSuite();

void reportIntegerTest(char * const testedName, int const value, int const expected);

void setupTestSuite();

void reportTestSuite();

#endif // guard end for TESTOOL_H
