/**
 * @file    robot_config.h
 * @brief   全局可调参数集中地（BSP层开发规划 一.5 / 九）
 *
 * 规则：
 *  - 不确定数值先进这里占位（注释"待硬件实测"），主逻辑零改动。
 *  - Modules/UserApp 只允许 include bsp_xxx.h 与本文件，禁止直接碰 HAL。
 *  - 车端/遥控器端共用 Modules/remote，用 CONFIG_REMOTE_UNIT 区分两端工程。
 */
#ifndef F103RC_ROBOT_CONFIG_H
#define F103RC_ROBOT_CONFIG_H

/* ================================ 编译开关 ================================ */

/* 遥控器端（PTX）工程打开此开关；车端（PRX）不定义 */
/* #define CONFIG_REMOTE_UNIT 1 */

/* ================================ Motor ================================== */

#define ROBOT_MOTOR_COUNT           4
/* 编码器：线数 x 4(四倍频) x 减速比 —— 待硬件实测 */
#define ROBOT_ENC_PPR               (1024 * 4 * 1)
/* 输出满占空比对应 -1000..+1000 的符号约定与轮向修正 —— 待整车联调 */
#define ROBOT_MOTOR_SIGN            1

/* ================================ Servo ================================== */

#define ROBOT_SERVO_COUNT           3
/* 50Hz / 1000 step 下 500~2500us = 0~180 度，软限位留边 —— 待硬件实测 */
#define ROBOT_SERVO_PULSE_MIN_US    500
#define ROBOT_SERVO_PULSE_MAX_US    2500

/* ================================== SPI =================================== */

/* 单次字节级传输超时（HAL_SPI_TransmitReceive 参数）。SPI2 实际 4.5MHz，
 * 15B 一帧总线时间 <100us，10ms 超时余量充足 —— 待实测 */
#define ROBOT_SPI_TIMEOUT_MS        10

/* ================================ Remote ================================= */

/* 无线链路单包载荷（字节），W1 验收目标 15B。收发两端必须一致 */
#define ROBOT_REMOTE_PAYLOAD        15
/* 链路看门狗超时：300~500ms，收不到包判定失联 —— 待联调定型 */
#define ROBOT_REMOTE_WATCHDOG_MS    400
/* PRX 帧队列深度：IRQ 搬运后待上层消费的帧数，满则丢最旧 */
#define ROBOT_REMOTE_RX_QUEUE       4
/* PTX 单包发送等待 IRQ 的兜底超时：自动重传最坏 10次×3.75ms=37.5ms，上取 50 */
#define ROBOT_REMOTE_TX_TIMEOUT_MS  50

/* ---- nRF24 RF 参数（经 Nrf24_Config_t 下发，全部待双板联调定型） ---- */
#define ROBOT_RF_CHANNEL            40      /* 2.440GHz，联调遇干扰可改 */
#define ROBOT_RF_DATA_RATE          1       /* 1=1Mbps（距离优先），0=2Mbps */
#define ROBOT_RF_TX_POWER           3       /* RF_PWR 3 = 0dBm */
#define ROBOT_RF_ADDR_WIDTH         5       /* 字节，3~5 */
#define ROBOT_RF_ADDR_BYTES         {0x32, 0x4E, 0x61, 0x6E, 0x6F}  /* 占位地址，两端一致 */
#define ROBOT_RF_RETR_DELAY         15      /* ×250us = 3.75ms（>15B@1Mbps 包时长） */
#define ROBOT_RF_RETR_COUNT         10      /* 自动重传上限 */

/* ================================ Power ================================== */

/* 功率采样窗口均值样本数（bsp_adc 双同步循环缓冲内取窗）—— 待实测 */
#define ROBOT_POWER_WINDOW          16
/* ADC 原始值 -> 物理量换算系数（分压比 / 采样电阻）—— 待硬件实测 */
#define ROBOT_POWER_V_K             1.0f
#define ROBOT_POWER_I_K             1.0f

/* ================================ Link =================================== */

/* 车端各任务间通信的统一命令包结构尺寸占位，随 UserApp 二期细化 */
#define ROBOT_CMD_MAX_SIZE          16

#endif /* F103RC_ROBOT_CONFIG_H */
