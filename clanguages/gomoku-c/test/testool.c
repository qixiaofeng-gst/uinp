#include <stdio.h>
#include <stdbool.h>

char const * const trueFlag = "\033[32mpassed\033[0m";
char const * const falseFlag = "\033[31mfailed\033[0m";
char const * const darkColorPrefix = "\033[38;2;128;128;128m";

int totalTestCount = 0;
int passedTestCount = 0;
int failedTestCount = 0;

void
report_integer_test(
    char const * const testedName,
    char const * const invokerName,
    char const * const fileName,
    int const lineNumber,
    int const value,
    int const expected,
    bool const isVerbose
) {
    bool isFailed = (false == (value == expected));
    if (isVerbose || isFailed) {
        printf(
                "Test [%s], testing value[ %s ]: %d, expected: %d\n",
                isFailed ? falseFlag : trueFlag,
                testedName,
                value,
                expected
        );
        if (isFailed) {
            printf(
                    "%s    Invoked at %s (%d) (%s)\033[0m\n",
                    darkColorPrefix,
                    fileName,
                    lineNumber,
                    invokerName
            );
        }
    }
    ++totalTestCount;
    if (isFailed) {
        ++failedTestCount;
    } else {
        ++passedTestCount;
    }
}

void
setup_test_suite()
{
    totalTestCount = 0;
    passedTestCount = 0;
    failedTestCount = 0;
    printf("\033[42m====>>> Ready for new test cases.\033[0m\n");
}

void
report_test_suite()
{
    printf(
        "Total count: %d; passed test count: \033[32m%d\033[0m; \
failed test count: \033[31m%d\033[0m.\n",
        totalTestCount,
        passedTestCount,
        failedTestCount
    );
}
