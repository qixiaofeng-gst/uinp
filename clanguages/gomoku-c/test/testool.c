#include <stdio.h>
#include <stdbool.h>

char const * const trueFlag = "\033[32mpassed\033[0m";
char const * const falseFlag = "\033[31mfailed\033[0m";

int totalTestCount = 0;
int passedTestCount = 0;
int failedTestCount = 0;

void
_reportIntegerTest(
    char const * const testedName,
    char const * const invokerName,
    char const * const fileName,
    int const lineNumber,
    int const value,
    int const expected,
    bool const isVerbose
) {
    bool isPassed = (value == expected);
    printf(
        "Test [%s], testing value[%s] = %d, expected: %d\n",
        isPassed ? trueFlag : falseFlag,
        testedName,
        value,
        expected
    );
    if (isVerbose) {
        printf(
            "    Invoked at %s (%d) (%s)\n",
            fileName,
            lineNumber,
            invokerName
        );
    }
    ++totalTestCount;
    if (isPassed) {
        ++passedTestCount;
    } else {
        ++failedTestCount;
    }
}

void
setupTestSuite()
{
    totalTestCount = 0;
    passedTestCount = 0;
    failedTestCount = 0;
    printf("\033[42m====>>> Ready for new test cases.\033[0m\n");
}

void
reportTestSuite()
{
    printf(
        "Total count: %d; passed test count: \033[32m%d\033[0m; \
failed test count: \033[31m%d\033[0m.\n",
        totalTestCount,
        passedTestCount,
        failedTestCount
    );
}
