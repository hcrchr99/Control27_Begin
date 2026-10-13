/**
 * @file    alarm.h
 * @brief   声光告警（W2.4）：非阻塞记录当前故障码 + 节奏发生器出蜂鸣音型
 *
 * 定位（Modules 层开发规划 §3.6 / 架构 §5.1）：
 *  - 只做"码 → 声光"的执行端：谁在什么时候 Set 什么码是业务的事
 *    （daemon 100ms 收集各模块心跳/故障码后 Set；chassis/grab 消费的
 *    rc_cmd 两级故障 estop/失联走指令归零路径，不经过本模块）；
 *  - 全程非阻塞：Set 仅记录，Poll 按绝对时间推进音型相位——调用频率
 *    无关（daemon 100ms / 测试台 10ms 皆正确，Bsp_GetMs 无符号差分
 *    天然抗回绕），禁止在 ISR 里调用（坑#9 同族）；
 *  - 底层走 bsp_gpio（LED1）+ bsp_pwm（蜂鸣器）：蜂鸣器是无源件，
 *    靠 TIM14 CH1（PA7）方波激励——响=50% 占空比、停=0，音调频率由
 *    TIM14 PSC/ARR（.ioc 时基）决定，本模块不管音调只管响停节奏。
 *    LED1 与测试台/daemon 心跳共用：非 OK 码播放期间本模块每个相位
 *    都重写 LED1（接管），回 OK 后不再碰（心跳自行恢复）——同码同灯，
 *    声光同相（蜂鸣器响 = LED1 亮）。
 *
 * 音型/闪码表（同码同灯，追加新码必须在此补一行）：
 *   码               蜂鸣音型（循环，ms）                LED1
 *   OK               静音                                不占用（让心跳）
 *   LINK_LOST        响200-停200-响200-停1400（双短）    随音型闪
 *   POWER_LOW        响800-停800（长鸣）                 随音型闪
 *   TASK_OVERRUN     响100-停100 ×3-停1500（三短）       随音型闪
 *   ACT_STALL        响600-停200-响100-停100-响100-停2000（一长两短）
 *                                                        随音型闪
 */
#ifndef ALARM_H
#define ALARM_H

#include <stdint.h>

typedef enum
{
    ALARM_OK = 0,           /* 无故障：静音，LED1 让给心跳 */
    ALARM_LINK_LOST,        /* 遥控失联（rc_cmd watchdog 判定） */
    ALARM_POWER_LOW,        /* 母线低压 */
    ALARM_TASK_OVERRUN,     /* 业务任务超期（daemon 心跳检查） */
    ALARM_ACT_STALL,        /* 关节堵转（actuator 电流判据坐实） */
    /* 追加只增不改（§3.6 冻结纪律）；音型表（本头文件注释 + alarm.c
     * s_pat[]）与 Alarm_Set 越界上界三处同步补。ALARM_ACT_TIMEOUT 留给
     * W2.5 grab 抓取超时按需追加 */
} AlarmCode_t;

/**
 * @brief  记录当前故障码（非阻塞，仅赋值；后来者覆盖，单码模型）
 * @param  code 合法码；超界忽略（防调用方错传噪声）
 * @note   同码重复 Set 无副作用；可在任意任务上下文调用（枚举赋值原子）
 */
void Alarm_Set(AlarmCode_t code);

/**
 * @brief  当前故障码（只读窥视，测试台/daemon 状态打印用）
 */
AlarmCode_t Alarm_Get(void);

/**
 * @brief  节奏发生器：按音型表驱动蜂鸣器 + LED1（非阻塞单次迭代）
 * @note   任务侧周期调用（daemon 100ms / 测试台 Poll 10ms 皆可）；
 *         码切换在下一个 Poll 拍生效并从相位 0 重放
 */
void Alarm_Poll(void);

#endif /* ALARM_H */
