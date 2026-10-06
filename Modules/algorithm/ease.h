/**
 * @file    ease.h
 * @brief   限速斜坡（W2.1 自研轻量件）：把"目标突变"变成"每步最多挪一小段"
 *
 * 纯算法（Modules 层开发规划 §3.7）：零硬件依赖。执行器保护层（W2.4）
 * 每个周期调一次：角度指令突变时，机构跟着平滑挪过去，而不是一步跳到位。
 */
#ifndef EASE_H
#define EASE_H

/**
 * @brief  从 cur 朝 target 挪一步，这一步不超过 max_step
 * @param  cur      当前位置
 * @param  target   目标位置
 * @param  max_step 一步最多挪多少（必须为正；≤0 时留在原地不动）
 * @retval 挪完这一步后的新位置；距离足够近时正好停在 target 上，不会冲过头
 */
float Ease_Step(float cur, float target, float max_step);

#endif /* EASE_H */
