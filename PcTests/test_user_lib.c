/**
 * @file    test_user_lib.c
 * @brief   user_lib 四函数单测（W2.1，host 编译）——含边界值：
 *          钳位双侧、死区开区间语义（边界不清零）、循环限幅回绕/边界/非法区间。
 */
#include "test_util.h"
#include "user_lib.h"
#include <math.h>

static void TestAbsLimit(void)
{
    CHECK_NEAR(abs_limit(0.5f, 1.0f), 0.5, 1e-9);  /* 区间内原样 */
    CHECK_NEAR(abs_limit(2.0f, 1.0f), 1.0, 1e-9);  /* 上限收口 */
    CHECK_NEAR(abs_limit(-2.0f, 1.0f), -1.0, 1e-9);/* 下限收口（对称） */
    CHECK_NEAR(abs_limit(1.0f, 1.0f), 1.0, 1e-9);  /* 恰在边界 */
    CHECK_NEAR(abs_limit(-1.0f, 1.0f), -1.0, 1e-9);
}

static void TestFloatConstrain(void)
{
    CHECK_NEAR(float_constrain(5.0f, 0.0f, 10.0f), 5.0, 1e-9);
    CHECK_NEAR(float_constrain(-1.0f, 0.0f, 10.0f), 0.0, 1e-9);
    CHECK_NEAR(float_constrain(11.0f, 0.0f, 10.0f), 10.0, 1e-9);
}

static void TestFloatDeadband(void)
{
    CHECK_NEAR(float_deadband(5.0f, 0.0f, 10.0f), 0.0, 1e-9);  /* 开区间内置 0 */
    CHECK_NEAR(float_deadband(-1.0f, -1.0f, 10.0f), -1.0, 1e-9);/* 下边界不清零 */
    CHECK_NEAR(float_deadband(10.0f, 0.0f, 10.0f), 10.0, 1e-9); /* 上边界不清零 */
    CHECK_NEAR(float_deadband(-5.0f, 0.0f, 10.0f), -5.0, 1e-9); /* 区间外原样 */
}

static void TestLoopFloatConstrain(void)
{
    /* [0, 360) 风格角度域 */
    CHECK_NEAR(loop_float_constrain(90.0f, 0.0f, 360.0f), 90.0, 1e-9);
    CHECK_NEAR(loop_float_constrain(370.0f, 0.0f, 360.0f), 10.0, 1e-9);
    CHECK_NEAR(loop_float_constrain(-20.0f, 0.0f, 360.0f), 340.0, 1e-9);
    CHECK_NEAR(loop_float_constrain(720.0f, 0.0f, 360.0f), 360.0, 1e-9); /* 整圈落边界 */
    CHECK_NEAR(loop_float_constrain(0.0f, 0.0f, 360.0f), 0.0, 1e-9);     /* 边界原样 */
    CHECK_NEAR(loop_float_constrain(360.0f, 0.0f, 360.0f), 360.0, 1e-9);

    /* 多圈回绕 */
    CHECK_NEAR(loop_float_constrain(1000.0f, -180.0f, 180.0f), -80.0, 1e-6);

    /* 非法区间（max<min）原样返回 */
    CHECK_NEAR(loop_float_constrain(5.0f, 10.0f, 0.0f), 5.0, 1e-9);
}

void TestUserLib_Run(void)
{
    printf("[user_lib]\n");
    TestAbsLimit();
    TestFloatConstrain();
    TestFloatDeadband();
    TestLoopFloatConstrain();
}
