/**
 * @file    lpf.c
 * @brief   一阶低通滤波实现（纯算法，见 lpf.h）
 */
#include "lpf.h"

void Lpf_Init(Lpf_t *f, float alpha)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }
    f->alpha = alpha;
    f->out = 0.0f;
    f->primed = false;
}

float Lpf_Apply(Lpf_t *f, float in)
{
    if (!f->primed)
    {
        f->primed = true;
        f->out = in;    /* 第一个数直接通过，不从 0 慢慢爬向它 */
        return f->out;
    }
    f->out += (in - f->out) * f->alpha; /* 向新数据靠一小步，靠多少由 alpha 定 */
    return f->out;
}
