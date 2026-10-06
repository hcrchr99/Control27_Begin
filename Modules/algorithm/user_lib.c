/**
 * @file    user_lib.c
 * @brief   浮点小工具实现（移植自 control-2026 user_lib.c，逐函数原样保留）
 */
#include "user_lib.h"

float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    /* 输入可能一次超出好几圈（比如 1000° 超出 [0,360] 两圈多），
     * 所以减/加一圈往往不够，要用循环反复加减、直到回到范围里为止。
     * 角度场景最多差几圈，循环两三次就结束。
     * 不用现成的取余函数（fmodf）是因为它对负数的返回值方向容易搞错
     * （fmodf(-20,360) 返回 -20 而不是想要的 340），反复加减不会错。 */
    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}
