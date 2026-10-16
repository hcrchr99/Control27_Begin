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

/* 输入源选择（编译期二选一，不做运行时仲裁——循 ROBOT_DIAG_ROLE_SWAP 先例，
 * "当前输入是谁"在编译后必须只有一种答案）：
 *   1 = WEB ：网页遥控台经 USART1 喂 RcInput_Raw_t 快照（输入硬件空窗期的
 *             出厂值）。网页帧看门狗 ROBOT_WEB_WATCHDOG_MS 超时 → 停发 RF，
 *             车端看门狗接管走已验的渐停+断链声光路径；
 *   0 = PHYS：板载 12 路方向刀 + 摇杆 ADC（真件焊上后切回，remoter_app 的
 *             物理采集分支恢复生效，web_input 模块被 --gc-sections 回收） */
#define ROBOT_RMT_INPUT_WEB         1
/* 网页帧新鲜窗长（ms）：网页 20Hz 心跳（50ms/帧）留 10 帧余量；
 * 超窗 = 浏览器崩了/关标签页/拔线，停发 RF 让车端接管 */
#define ROBOT_WEB_WATCHDOG_MS       500u

/* 发送周期（ms）：100Hz 与 W1/W2.2 联调基准一致 */
#define ROBOT_SEND_PERIOD_MS        10
/* 信号灯心跳半周期（ms）：LED2 每 500ms 翻转 = 1Hz 心跳 */
#define ROBOT_HEARTBEAT_MS          500

/* 摇杆标定（原始值 0..4095，**仅 PHYS 物理电位器路径**；WEB 源直传逻辑量不过此）：
 * 中点=摇杆松手时的 raw 读数，死区=松手抖动包络再留余量 */
#define ROBOT_JOY_CENTER_COUNTS     2048u   /* 待硬件实测 */
#define ROBOT_JOY_DEADBAND_COUNTS   120u    /* 待硬件实测 */
/* 摇杆按压键（K_SW/PA0）暂不消费——语义待拍板（候选：慢速模式） */

/* 船型开关（自复位、中位双断）→ 帧字段：K_n 的 A/B 刀 = joint[n-1] 正/反向，
 * 幅度 = ROBOT_KEY_JOINT_RATE；同开关 A/B 同 1 按矛盾输入归零（失安全）。
 * K_6（bit10/11）= 整车旋转 omega（2026-10-16 拍板）：A=+/B=−，
 * 幅度 = ROBOT_KEY_OMEGA_RATE。
 * ⚠ 物理分组与舵机 ID 的错位（K_3 在爪子区=舵机4、K_5 在手腕区=舵机3）
 * 与 A/B 互斥假设均待上手核对，改 remoter_app.c 的 s_switch_pins 表即可 */
#define ROBOT_KEY_JOINT_RATE        ROBOT_CMD_JOINT_RANGE
/* 整车旋转速率（±ROBOT_CMD_RANGE=±1000）：开关是二值输入，与关节开关同
 * 哲学=满速率 1000。半量程 500 会被车端手感整形（死区 0.46+平方曲线）
 * 压到 ~0.5% 满幅=肉眼不可见的爬行（2026-10-16 实车实证）；嫌快往下调 */
#define ROBOT_KEY_OMEGA_RATE        1000

/* ---- 双击识别（flags 事件位，两端契约见 rc_input.h）----
 * 窗长按自复位开关的"拨上-回中-再拨"手感估的，待实测调；保持拍数 =
 * HOLD_MS ÷ 发送周期（100ms=10 帧），抗单帧丢失；车端按 0→1 沿切模式。
 * 绑定表在 remoter_app.c（默认空表=功能休眠），绑定行示例：
 *   { RCINPUT_KEY_K1_A, 1 }  → K_1A 双击 → flags bit1（如 快/慢 切换）
 *   { RCINPUT_KEY_K5_A, 2 }  → K_5A 双击 → flags bit2（如 关节/坐标 切换） */
#define ROBOT_DCLICK_WINDOW_MS      600u    /* 待手感标定 */
#define ROBOT_DCLICK_HOLD_MS        100u

#endif /* REMOTER_ROBOT_CONFIG_H */
