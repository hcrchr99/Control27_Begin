/**
 * @file    ease.c
 * @brief   限速斜坡实现（纯算法，见 ease.h）
 */
#include "ease.h"
#include <math.h>

float Ease_Step(float cur, float target, float max_step)
{
    float delta = target - cur;

    if (!(max_step > 0.0f))
    {
        return cur; /* 步长不是正数（含 NaN）：留在原地，绝不允许一步跳到目标 */
    }
    if (fabsf(delta) <= max_step)
    {
        return target; /* 剩的距离不足一步：正好到达，不冲过头 */
    }

    return cur + ((delta > 0.0f) ? max_step : -max_step);
}
