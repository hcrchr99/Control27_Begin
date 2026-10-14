/**
 * @file    robot_config.h
 * @brief   全局可调参数集中地（BSP层开发规划 一.5 / 九）—— Remoter 遥控器版
 *
 * 规则：
 *  - 不确定数值先进这里占位（注释"待硬件实测"），主逻辑零改动。
 *  - Modules/UserApp 只允许 include bsp_xxx.h 与本文件，禁止直接碰 HAL。
 *  - 车端（PRX）与遥控器端（PTX）共用 Modules/remote，两端 robot_config 的
 *    Remote/RF/RC_Cmd 三段数值必须逐字一致（协议两端同步演进）。
 */
#ifndef REMOTER_ROBOT_CONFIG_H
#define REMOTER_ROBOT_CONFIG_H

/* ================================ 编译开关 ================================ */

/* 遥控器端（PTX）工程打开此开关；车端（PRX）不定义 */
#define CONFIG_REMOTE_UNIT 1

/* ================================== SPI =================================== */

/* 单次字节级传输超时（HAL_SPI_TransmitReceive 参数）。SPI2 实际 4.5MHz，
 * 15B 一帧总线时间 <100us，10ms 超时余量充足 */
#define ROBOT_SPI_TIMEOUT_MS        10

/* ================================== ADC =================================== */

/* 摇杆单通道转换轮询超时（HAL_ADC_PollForConversion 参数）。
 * 单转换 71.5+12.5 周期 @12MHz ≈ 7us，2ms 超时千倍余量 */
#define ROBOT_ADC_TIMEOUT_MS        2

/* ================================ Remote ================================= */

/* 无线链路单包载荷（字节），W1 验收 15B。收发两端必须一致 */
#define ROBOT_REMOTE_PAYLOAD        15
/* 链路看门狗超时：300~500ms，收不到包判定失联（与车端一致） */
#define ROBOT_REMOTE_WATCHDOG_MS    400
/* 【临时诊断】收发角色互换：1=本板收(PRX)/对端发，0=正常角色（与车端同款诊断口） */
#define ROBOT_DIAG_ROLE_SWAP        0
/* 自动应答（Enhanced ShockBurst ACK）：1=开启（发帧等 ACK，失败自动重传） */
#define ROBOT_RF_AUTO_ACK           1
/* PRX 帧队列深度（车端语义；发送端仅随协议文件引用） */
#define ROBOT_REMOTE_RX_QUEUE       4
/* PTX 单包发送等待 IRQ 的兜底超时：自动重传最坏 10次×3.75ms=37.5ms，上取 50 */
#define ROBOT_REMOTE_TX_TIMEOUT_MS  50

/* ---- nRF24 RF 参数（经 Nrf24_Config_t 下发，⚠ 与车端逐字一致） ---- */
#define ROBOT_RF_CHANNEL            90      /* 2490MHz：WiFi13 掩码上沿之上，实验室近距离 */
#define ROBOT_RF_DATA_RATE          1       /* 1=1Mbps（距离优先），0=2Mbps */
#define ROBOT_RF_TX_POWER           3       /* RF_PWR 3 = 0dBm */
#define ROBOT_RF_ADDR_WIDTH         5       /* 字节，3~5 */
#define ROBOT_RF_ADDR_BYTES         {0x32, 0x4E, 0x61, 0x6E, 0x6F}  /* 两端一致 */
#define ROBOT_RF_RETR_DELAY         15      /* ×250us = 3.75ms（>15B@1Mbps 包时长） */
#define ROBOT_RF_RETR_COUNT         10      /* 自动重传上限 */

/* ---- RC_Cmd 遥控协议层（W2.2 §3.8 帧布局冻结；⚠ 与车端逐字一致） ---- */
/* 逻辑量满幅：vx/vy/omega 取 -1000..+1000（发送端钳位，接收端 ÷满幅归一化） */
#define ROBOT_CMD_RANGE             1000
/* 关节速率指令满幅：joint[] 取 -100..+100 */
#define ROBOT_CMD_JOINT_RANGE       100
/* 关节指令路数（= RCPayload_t.joint[] 元素数，与车端五舵机对齐；
 * 改动会打破 15B 帧长静态断言，属协议变更须两端同时改） */
#define ROBOT_CMD_JOINT_COUNT       5
/* 失联渐停时长（接收端语义；发送端仅随协议文件引用） */
#define ROBOT_CMD_DECAY_MS          300

/* ================================ Input（遥控器输入） ===================== */

/* 发送周期（ms）：100Hz 与 W1/W2.2 联调基准一致 */
#define ROBOT_SEND_PERIOD_MS        10
/* 信号灯心跳半周期（ms）：LED2 每 500ms 翻转 = 1Hz 心跳 */
#define ROBOT_HEARTBEAT_MS          500

/* 摇杆标定（原始值 0..4095）—— 中点/死区待真件实测后回写：
 * 中点=摇杆松手时的 raw 读数，死区=松手抖动包络再留余量 */
#define ROBOT_JOY_CENTER_COUNTS     2048u   /* 待硬件实测 */
#define ROBOT_JOY_DEADBAND_COUNTS   120u    /* 待硬件实测 */
/* 摇杆按压键（K_SW/PA0）暂不消费——语义待拍板（候选：omega 临时轴/慢速模式） */

/* 船型开关键→帧字段映射的速率值：按住=该关节 +满速率（正向点动）。
 * ⚠ 方向语义待用户拍板：当前只有"按住正向"，反向候选方案=
 * ①加第二排键 ②"拨上=去端点/拨下=回中" ③短按步进。定案前先发 +rate */
#define ROBOT_KEY_JOINT_RATE        ROBOT_CMD_JOINT_RANGE

#endif /* REMOTER_ROBOT_CONFIG_H */
