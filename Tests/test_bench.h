/**
 * @file    test_bench.h
 * @brief   板级测试台：各 S 阶段验收程序常驻主干，新板 bring-up 逐模块重跑
 *
 * 设计（替代"验收完即删"与"分支归档"两种旧方案）：
 *  - 测试代码永远参与编译（防 API 演进后烂掉），正常运行只多几 KB flash；
 *  - 跑哪项由 robot_config.h 的 ROBOT_TEST_BENCH 宏选择（默认 0=关闭，
 *    测试任务退化为心跳 + 栈高水位观测，不干扰业务）；
 *  - 测试台激活时占用 SPI 等外设：业务任务启动时调 TestBench_Yield 让位。
 *
 * 新板 bring-up 顺序（对齐 BSP层开发规划 六，依赖序）：
 *   上电心跳（NONE 项）→ GPIO/EXTI → 编码器 → SPI/无线 → 后续 PWM/ADC/I2C
 */
#ifndef TEST_BENCH_H
#define TEST_BENCH_H

#include <stdbool.h>
#include "robot_config.h"

/* 测试项编号：ROBOT_TEST_BENCH 取下列值之一（新增测试在 test_bench.c 登记表追加） */
typedef enum
{
    TEST_BENCH_NONE = 0,    /* 关闭：仅心跳 + 栈高水位（程序存活证据） */
    TEST_BENCH_GPIO,        /* S1：按键 EXTI → LED1/蜂鸣器 */
    TEST_BENCH_ENCODER,     /* S3：四路编码器手转（增量/累计） */
    TEST_BENCH_SPI,         /* S2：SPI 通路（nRF24 STATUS/RF_CH 回写回读） */
    TEST_BENCH_PWM,         /* S4：TIM4 20kHz duty 阶梯 / TIM5 50Hz 脉宽阶梯 */
    TEST_BENCH_REMOTE,      /* S2：无线链路收发体检 + 收包统计 */
    TEST_BENCH_MOTOR,       /* S5：四路开环正反转阶梯 + 编码器符号交叉验证 */
    TEST_BENCH_SERVO,       /* S5：中位/扫描/五路卸力手掰（PM10S+SG90） */
    TEST_BENCH_POWER,       /* S6：V/I/P/J 打印对表 + 双 rank 同步性 */
    TEST_BENCH_OLED,        /* S7：四项显示对表 + 拔插排线自愈 + 地址参数化探测 */
    TEST_BENCH_ACTUATOR = 10, /* W2.4：actuator 缓动/堵转/热保护 + alarm 声光联验。
                               * 显式 =10：编号只增不改（W2.2 时即预留本项） */
    TEST_BENCH_RC_CMD = 11, /* W2.2：rc_cmd 全链路（对照 PTX 信号发生器 12s 周期）。
                             * 显式 =11：与 10 同理防占位漂移 */
    TEST_BENCH_TUNE = 13,   /* W2.3：速度环独立整定台（串口命令调参+1kHz JustFloat
                             * 回传+启动自检防正反馈；12 曾为 DWT 终审台，结案清理） */
} TestBenchId_t;

/**
 * @brief  测试台是否激活（= 将占用 SPI 等共享外设）
 * @note   业务任务初始化前调用；true 则调 TestBench_Yield 让位。
 */
bool TestBench_Active(void);

/**
 * @brief  业务任务让位钩子：测试台激活时打印提示并永久挂起调用任务。
 * @param  task_name 任务名（仅日志用）
 * @note   测试台未激活时立即返回，无副作用。
 */
void TestBench_Yield(const char *task_name);

#endif /* TEST_BENCH_H */
