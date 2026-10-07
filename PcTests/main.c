/**
 * @file    main.c
 * @brief   PC 单测入口：跑全部用例，失败数非零则退出码 1
 */
#include <stdio.h>
#include "test_util.h"

int s_total;
int s_fail;

void TestPid_Run(void);
void TestUserLib_Run(void);
void TestLpfEase_Run(void);
void TestRcCmd_Run(void);

int main(void)
{
    printf("== RM_Begin algorithm PC tests ==\n");
    TestPid_Run();
    TestUserLib_Run();
    TestLpfEase_Run();
    TestRcCmd_Run();
    printf("\n%d checks, %d failed\n", s_total, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
