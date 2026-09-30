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
#ifndef F103RC_TEST_BENCH_H
#define F103RC_TEST_BENCH_H

#include <stdbool.h>
#include "robot_config.h"

/* 测试项编号：ROBOT_TEST_BENCH 取下列值之一（新增测试在 test_bench.c 登记表追加） */
typedef enum
{
    TEST_BENCH_NONE = 0,    /* 关闭：仅心跳 + 栈高水位（程序存活证据） */
    TEST_BENCH_GPIO,        /* S1：按键 EXTI → LED1/蜂鸣器 */
    TEST_BENCH_ENCODER,     /* S3：四路编码器手转（增量/累计） */
    TEST_BENCH_SPI,         /* S2：SPI 通路（nRF24 STATUS/RF_CH 回写回读） */
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

#endif /* F103RC_TEST_BENCH_H */
