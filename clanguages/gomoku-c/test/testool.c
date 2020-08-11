#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

char const *const trueFlag = "\033[32mpassed\033[0m";
char const *const falseFlag = "\033[31mfailed\033[0m";
char const *const darkColorPrefix = "\033[38;2;128;128;128m";

int const G_report_number_width = 5;
bool g_summary_hooked = false;
size_t g_current_test_count = 0;
size_t g_current_passed_count = 0;
size_t g_current_failed_count = 0;
size_t g_total_test_count = 0;
size_t g_total_passed_count = 0;
size_t g_total_failed_count = 0;
clock_t g_suite_start_time = 0;
clock_t g_test_total_time = 0;

void
report_integer_test(
        char const *const testedName,
        char const *const invokerName,
        char const *const fileName,
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
    ++g_current_test_count;
    if (isFailed) {
        ++g_current_failed_count;
    } else {
        ++g_current_passed_count;
    }
}

void
setup_test_suite() {
    g_current_test_count = 0;
    g_current_passed_count = 0;
    g_current_failed_count = 0;
    g_suite_start_time = clock();
    printf("[>>> \033[42m\033[30m[Test Suite Start]\033[0m <<<]\n");
}

void
p_print_clock(char const *name, clock_t timeCost) {
    double const timeInSeconds = (double) timeCost / CLOCKS_PER_SEC;
    printf("%s cost %6.6f second(s).\n", name, timeInSeconds);
}

void
p_report_test_summary() {
    int color = g_total_failed_count > 0 ? 41 : 42;
    printf("[====>>> \033[%dm\033[30m[Test summary]\033[0m <<<====]:\n", color);
    printf(
            "Total count: %*lu; "
            "passed test count: \033[32m%*lu\033[0m; "
            "failed test count: \033[31m%*lu\033[0m.\n",
            G_report_number_width,
            g_total_test_count,
            G_report_number_width,
            g_total_passed_count,
            G_report_number_width,
            g_total_failed_count
    );
    p_print_clock("Test totally", g_test_total_time);
}

void
report_test_suite() {
    if (false == g_summary_hooked) {
        atexit(p_report_test_summary);
        g_summary_hooked = true;
    }
    printf(
            "Total count: %*lu; "
            "passed test count: \033[32m%*lu\033[0m; "
            "failed test count: \033[31m%*lu\033[0m.\n",
            G_report_number_width,
            g_current_test_count,
            G_report_number_width,
            g_current_passed_count,
            G_report_number_width,
            g_current_failed_count
    );
    clock_t const timeCost = clock() - g_suite_start_time;
    g_test_total_time += timeCost;
    p_print_clock("Test suite", timeCost);
    g_total_test_count += g_current_test_count;
    g_total_passed_count += g_current_passed_count;
    g_total_failed_count += g_current_failed_count;
}
