/**
 * @file    user_lib.h
 * @brief   浮点小工具（W2.1，精选移植自 control-2026 Modules/algorithm/user_lib）
 *
 * 只移植规划 §3.7 点名的四个函数，原版其余条目（矩阵/向量/动态内存分配）
 * 一并不要——本工程静态分配、禁 malloc。loop_float_constrain 处理"转圈"
 * 的量（角度）：转过一圈就回到起点，W2 麦轮解算和以后机械臂运动学的
 * 角度归一化都会用到。
 */
#ifndef USER_LIB_H
#define USER_LIB_H

#include <stdint.h>

/* 对称限幅：把 num 限制在 [-Limit, +Limit] 之内，超出就贴在边界上 */
float abs_limit(float num, float Limit);

/* 区间限幅：把 Value 限制在 [minValue, maxValue] 之内
 *（minValue 大于 maxValue 时结果没有意义，不要这样用） */
float float_constrain(float Value, float minValue, float maxValue);

/* 死区：数值落在 (minValue, maxValue) 之间就当成 0；
 * 正好踩在边界上不算（原样返回）——比如 minValue=-1 时，-1 本身原样返回 */
float float_deadband(float Value, float minValue, float maxValue);

/* 循环限幅：数值超出上限就从下限那边接着数，低于下限就从上限那边接着数，
 * 像时钟 12 点过后是 1 点一样——角度 370° 其实就是 10°。
 *（minValue 大于 maxValue 时原样返回不做处理） */
float loop_float_constrain(float Input, float minValue, float maxValue);

#endif /* USER_LIB_H */
