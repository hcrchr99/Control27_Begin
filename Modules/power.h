/**
 * @file    power.h
 * @brief   功率采样换算：原始码 → 电压/电流/功率/关节电流（窗口均值）
 *
 * 定位（架构 v1.1 §3.3：自 sense_task 上浮为本模块，2026-09-30 裁决）：
 *  - 换算可脱离整车测试、标定点全系统唯一——系数只从 robot_config.h 读；
 *  - 本模块无任务，由调用方按 20~50ms 节拍驱动（sense_task / 测试台）；
 *  - 换算链：bsp_adc 全窗均值（高频噪声滤除）→ 本模块线性换算（负载
 *    变化滤除交给调用方节拍）→ UserApp 排版上 OLED（检录项 ±10%）。
 *  - 失联语义：未初始化 / 读取失败一律返回 NaN（与 servo/motor 的
 *    NaN 失安全同风格，调用方以 != 自比判别）。
 */
#ifndef F103RC_POWER_H
#define F103RC_POWER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  初始化（幂等）：确保 bsp_adc 已启动（校准 + 双同步 DMA + 丢前 2 窗）。
 * @retval true = 就绪可读；false = bsp_adc 启动失败（看 bsp 日志定位）
 */
bool Power_Init(void);

/**
 * @brief  母线电压（V）。全窗均值换算：raw_v/4095 × 3.3 × ROBOT_POWER_V_K。
 * @note   ROBOT_POWER_V_K 含分压比与基准偏差，万用表对表后定值（唯一标定入口）。
 */
float Power_GetVoltage(void);

/**
 * @brief  母线电流（A）。全窗均值换算：raw_i/4095 × 3.3 × ROBOT_POWER_I_K。
 */
float Power_GetCurrent(void);

/**
 * @brief  功率（W）。单次快照同窗 U×I——不拆两次调用拼乘，保证同窗一致
 *         （双同步硬件保证 U/I 无相位差）。
 */
float Power_GetPower(void);

/**
 * @brief  关节电流（A）：k=0 大臂（ADC3_CH1/IN12/PC2）、k=1 小臂（ADC3_CH2/
 *         IN13/PC3），数字舵机供电回路采样，消费方 actuator（堵转判定注入），
 *         sense/OLED 亦可复用。
 * @param  k 关节序号，0..ROBOT_POWER_JOINT_COUNT-1
 * @note   量程 0~3A 级，换算系数 ROBOT_POWER_JOINT_I_K 待硬件定案后标定。
 */
float Power_GetJointCurrent(uint8_t k);

#endif /* F103RC_POWER_H */
