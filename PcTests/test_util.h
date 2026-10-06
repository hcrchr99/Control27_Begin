/**
 * @file    test_util.h
 * @brief   PC 单测极简断言框架（仅 host 编译，不进固件）
 *
 * 无第三方依赖：CHECK 布尔断言 + CHECK_NEAR 浮点近等断言，
 * 计数器在 main.c 定义，进程退出码 = 失败数是否为零。
 */
#ifndef PC_TEST_UTIL_H
#define PC_TEST_UTIL_H

#include <math.h>
#include <stdio.h>

extern int s_total;
extern int s_fail;

#define CHECK(cond)                                                         \
    do                                                                      \
    {                                                                       \
        s_total++;                                                          \
        if (!(cond))                                                        \
        {                                                                   \
            s_fail++;                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                   \
    } while (0)

#define CHECK_NEAR(val, ref, eps)                                           \
    do                                                                      \
    {                                                                       \
        s_total++;                                                          \
        double _v = (double)(val), _r = (double)(ref);                      \
        if (!(fabs(_v - _r) <= (double)(eps)))                              \
        {                                                                   \
            s_fail++;                                                       \
            printf("  FAIL %s:%d: %s = %.9g, expected %.9g +- %g\n",        \
                   __FILE__, __LINE__, #val, _v, _r, (double)(eps));        \
        }                                                                   \
    } while (0)

#endif /* PC_TEST_UTIL_H */
