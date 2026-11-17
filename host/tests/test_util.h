#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <math.h>
#include <stdio.h>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        g_checks++;                                                          \
        if (!(cond))                                                         \
        {                                                                    \
            g_failures++;                                                    \
            fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(actual, expected, eps)                                      \
    do                                                                         \
    {                                                                          \
        double _a = (double)(actual);                                          \
        double _e = (double)(expected);                                        \
        double _t = (double)(eps);                                             \
        g_checks++;                                                            \
        if (!(fabs(_a - _e) <= _t))                                            \
        {                                                                      \
            g_failures++;                                                      \
            fprintf(stderr, "  FAIL %s:%d: %s = %.6f, expected %.6f (+-%.6f)\n", \
                    __FILE__, __LINE__, #actual, _a, _e, _t);                  \
        }                                                                      \
    } while (0)

#define RUN(test)        \
    do                   \
    {                    \
        printf("  %s\n", #test); \
        test();          \
    } while (0)

#define TEST_REPORT()                                            \
    do                                                           \
    {                                                            \
        printf("  %d checks, %d failures\n", g_checks, g_failures); \
        return g_failures == 0 ? 0 : 1;                          \
    } while (0)

#endif /* TEST_UTIL_H */
