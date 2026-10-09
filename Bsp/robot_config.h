/**
 * @file    robot_config.h
 * @brief   全局可调参数集中地（BSP层开发规划 一.5 / 九）
 *
 * 规则：
 *  - 不确定数值先进这里占位（注释"待硬件实测"），主逻辑零改动。
 *  - Modules/UserApp 只允许 include bsp_xxx.h 与本文件，禁止直接碰 HAL。
 *  - 车端/遥控器端共用 Modules/remote，用 CONFIG_REMOTE_UNIT 区分两端工程。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ================================ 编译开关 ================================ */

/* 遥控器端（PTX）工程打开此开关；车端（PRX）不定义 */
/* #define CONFIG_REMOTE_UNIT 1 */

/* ================================ Motor ================================== */

#define ROBOT_MOTOR_COUNT           4
/* 编码器三段语义宏（W2.3 自原 ROBOT_ENC_PPR 拆分，合成结果同名保留，
 * 消费者仅 motor.c 测速换算）—— 待硬件实测：
 * LINES = 编码器线数（铭牌/手册）；QUAD = 四倍频（TIM TI12 模式固定 ×4）；
 * GEAR = 减速比（测的是输出轴转速，减速比乘在计数里） */
#define ROBOT_ENC_LINES             13
#define ROBOT_ENC_QUAD              4
#define ROBOT_MOTOR_GEAR            30.0
/* 每输出轴转一圈的计数总数 = 线数 × 四倍频 × 减速比 */
#define ROBOT_ENC_PPR               (ROBOT_ENC_LINES * ROBOT_ENC_QUAD * ROBOT_MOTOR_GEAR)
/* 输出满占空比对应 -1000..+1000 的符号约定与轮向修正 —— 待整车联调 */
#define ROBOT_MOTOR_SIGN            1
/* 死区：|duty| 低于此值按 0 处理（占空比小数量纲 -1.0..+1.0；占位，开环实测后填） */
#define ROBOT_MOTOR_DEADBAND        0.0f
/* 通道映射表（2026-10-02 布线未定，按序号直连占位）：
 * MOTOR_CHx → { PWM 通道号, DIR 引脚对序号 }。
 * PWM 通道号 = bsp_pwm.h 的 PWM_20K_CH1..4（1..4）；DIR 序号 1..4 =
 * bsp_pin.h 的 PIN_DIRxA/PIN_DIRxB 引脚对。布线定案后只改此处，代码零改动 */
#define ROBOT_MOTOR_CH1_PWM         1
#define ROBOT_MOTOR_CH1_DIR         1
#define ROBOT_MOTOR_CH2_PWM         2
#define ROBOT_MOTOR_CH2_DIR         2
#define ROBOT_MOTOR_CH3_PWM         3
#define ROBOT_MOTOR_CH3_DIR         3
#define ROBOT_MOTOR_CH4_PWM         4
#define ROBOT_MOTOR_CH4_DIR         4

/* ---- W2.3 速度环（M 法测速 + 位置式 PID，MaxOut=1.0f 直喂 Motor_SetDuty）----
 * 整定纪律（Modules 规划）：先 P 后 I 再 D；每改一次参数在下面补一行：
 *   日期 | 工况 | Kp/Ki/Kd | 现象（超调/振荡/稳态误差）
 * Kd 初值 0：速度反馈是测速窗的阶梯值，微分项会放大这种量化噪声，出现
 * 振荡后再引入并配微分滤波 */
/* M 法测速窗（拍）：每拍读一次编码器增量累计，满 CALC_MS 拍折算一次 rpm。
 * ⚠ 折算按 Bsp_GetUs 实测窗长，拍数≠毫秒（任务节拍≠1kHz）——曾按固定
 * 系数折算致读数整体×2 且随调度漂移锯齿抖动（2026-10-09 表显 190 ↔
 * A相 625Hz 定案）。本值只是窗长下限：越大越平滑越钝，嫌钝可缩只改此宏 */
#define ROBOT_MOTOR_SPEED_CALC_MS       20
/* 设定转速钳位（rpm）—— 待按电机铭牌实测收边 */
#define ROBOT_MOTOR_SPEED_MAX_RPM       300.0f
/* PID 参数（输出量纲 = duty -1..+1）—— 整定记录：
 *   2026-10-07 | Kp=0.02 Ki=0 | CH4 ref±30/60 | bang-bang 极限环：系统增益
 *   实测 ≈200rpm/duty（旧电源 20%→40rpm，50%→113rpm），Kp 过大使 duty 顶满
 *   ±1.0 来回猛撞，实测 ±150rpm 振荡
 *   2026-10-07 | Kp=0.005 Ki=0.05 | 稳但稳态差大（60rpm 稳在 38）——根因
 *   不是参数：DWT CYCCNT 被 LAR 锁（坑#15）dt 钳 1µs 积分等效死亡；
 *   解锁后复测：±30/±60 阶跃零稳态误差（60 稳 57.6~60.8），+60 首拍 ~17%
 *   超调 200ms 内落定、无振荡
 *   2026-10-09 | Kp=0.0058 Ki=0.09 Kd=0 | 换电源模块重标（增益≈305rpm/duty）+
 *   修测速窗×2/CYCCNT 回绕两坑后，单电机空载粗标（CH4，ref 110~150）；
 *   ⚠ 空载值，实车负载下须细调
 * ⚠ LINES/GEAR 已重标：13 线 ×4 ×减速比 30 = 1560 counts/输出圈 */
#define ROBOT_MOTOR_SPEED_KP            0.0058f
#define ROBOT_MOTOR_SPEED_KI            0.09f
#define ROBOT_MOTOR_SPEED_KD            0.0f
/* 积分累计上限（duty 量纲）：抗启动冲击/堵转甩积分 */
#define ROBOT_MOTOR_SPEED_INTEGRAL_LIMIT 0.65f
/* 微分低通时间常数（秒），Kd≠0 时生效 */
#define ROBOT_MOTOR_SPEED_D_LPF_RC      0.01f
/* 电机↔编码器通道映射（值 1..4 = bsp_encoder.h 的 ENC_CH1..4；同 PWM/DIR
 * 纪律：实机布线核对后只改此处，代码零改动） */
#define ROBOT_MOTOR_CH1_ENC         1
#define ROBOT_MOTOR_CH2_ENC         2
#define ROBOT_MOTOR_CH3_ENC         3
#define ROBOT_MOTOR_CH4_ENC         4
/* 测试台 6 速度阶跃序列（rpm，四路同步阶跃）：验收判据 = 阶跃无超调振荡 */
#define ROBOT_MOTOR_SPEED_TEST_LIST { 30.0f, 60.0f, -30.0f, 0.0f }

/* ================================ Servo ================================== */

/* 构型定案（2026-10-01 §3.2；2026-10-05 增补第 5 路爪旋转，F407VG 迁移）：
 * ID1=大臂 ID2=小臂 = PM10S 数字舵机（250Hz 帧率驱动）；ID3=手腕
 * ID4=爪开合 ID5=爪旋转 = SG90（50Hz）。卸力：停脉冲=卸力对两类一致
 * （2026-10-05 实测修正，PM10S 非 fail-hold）—— id↔关节↔通道 以实机布线为准 */
#define ROBOT_SERVO_COUNT           5
/* 器件分型位表（1=数字 PM10S / 0=SG90），顺序 ID1..ID5——保留给 W2.4
 * actuator 的判据选择（卸力两类一致，不再关联分型） */
#define ROBOT_SERVO_DIGITAL_LIST    { 1, 1, 0, 0, 0 }
/* 通道映射：SERVO_IDx → bsp_pwm 脉宽通道号（PWM_250HZ_CH1..2 = 5..6 走
 * TIM9@250Hz，PWM_50HZ_CH1..3 = 7..9 走 TIM5@50Hz，见 bsp_pwm.h——不同
 * 帧率不能共 TIM，故按组连号）。布线未定按序号直连占位；定案后只改此处 */
#define ROBOT_SERVO_PWM_CH_LIST     { 5, 6, 7, 8, 9 }
/* 每 id 脉宽标定（µs，0..180° 线性映射两端）：标称 500~2500 占位，实机标定
 * 后填（PM10S 标称 0.5~2.5ms；SG90 实际行程普遍不足 180°）—— 不留"待实测"过夜。
 * 注意 PM10S 组帧长 4ms，2500µs 脉宽合法（bsp_pwm 按组钳帧长上限） */
#define ROBOT_SERVO_PULSE_MIN_US_LIST   { 500, 500, 500, 500, 500 }
#define ROBOT_SERVO_PULSE_MAX_US_LIST   { 2500, 2500, 2500, 2500, 2500 }
/* 每 id 软限位（0..180° 坐标系，占位 ±5° 收边）—— 待结构定机械限位后收紧 */
#define ROBOT_SERVO_LIMIT_MIN_DEG_LIST  { 5.0f, 5.0f, 5.0f, 5.0f, 5.0f }
#define ROBOT_SERVO_LIMIT_MAX_DEG_LIST  { 175.0f, 175.0f, 175.0f, 175.0f, 175.0f }
/* Init 中位稳定窗：发中位脉冲后经此时长 SetAngle 方生效（上电猛冲防护，
 * 非阻塞计时，与 bsp_pwm"上电 0 脉宽"构成两级安全链） */
#define ROBOT_SERVO_CENTER_SETTLE_MS    300

/* ================================== SPI =================================== */

/* 单次字节级传输超时（HAL_SPI_TransmitReceive 参数）。SPI2 实际 5.25MHz
 * （APB1 42MHz/8，F407VG 迁移后；F103 时代为 4.5MHz），
 * 15B 一帧总线时间 <100us，10ms 超时余量充足 —— 待实测 */
#define ROBOT_SPI_TIMEOUT_MS        10

/* ================================ Remote ================================= */

/* 无线链路单包载荷（字节），W1 验收目标 15B。收发两端必须一致 */
#define ROBOT_REMOTE_PAYLOAD        15
/* 链路看门狗超时：300~500ms，收不到包判定失联 —— 待联调定型 */
#define ROBOT_REMOTE_WATCHDOG_MS    400
/* 【临时诊断】收发角色互换：1=车端发(PTX)/测试板收(PRX)，0=正常角色。
 * 用于切开"测试板模块发射坏"与"车板模块接收坏"，诊断完改回 0 */
#define ROBOT_DIAG_ROLE_SWAP        0
/* 自动应答（Enhanced ShockBurst ACK）：1=开启（PTX 收到 ACK 才算成功，失败自动重传，
 * PRX 按 PID 自动去重副本）；0=单向广播。链路修复后已重新启用验证，
 * 若模块 ACK 路径异常（PTX fail 增长）改回 0 即回退广播形态 */
#define ROBOT_RF_AUTO_ACK           1
/* PRX 帧队列深度：IRQ 搬运后待上层消费的帧数，满则丢最旧 */
#define ROBOT_REMOTE_RX_QUEUE       4
/* PTX 单包发送等待 IRQ 的兜底超时：自动重传最坏 10次×3.75ms=37.5ms，上取 50 */
#define ROBOT_REMOTE_TX_TIMEOUT_MS  50

/* ---- nRF24 RF 参数（经 Nrf24_Config_t 下发，全部待双板联调定型） ---- */
#define ROBOT_RF_CHANNEL            90      /* 2490MHz：WiFi13 掩码上沿(2484)与蓝牙(2480)之上，
                                             * 实测避开 2.4G 干扰；超出 ISM 上沿(2483.5)，仅限实验室近距离测试 */
#define ROBOT_RF_DATA_RATE          1       /* 1=1Mbps（距离优先），0=2Mbps */
#define ROBOT_RF_TX_POWER           3       /* RF_PWR 3 = 0dBm */
#define ROBOT_RF_ADDR_WIDTH         5       /* 字节，3~5 */
#define ROBOT_RF_ADDR_BYTES         {0x32, 0x4E, 0x61, 0x6E, 0x6F}  /* 占位地址，两端一致 */
#define ROBOT_RF_RETR_DELAY         15      /* ×250us = 3.75ms（>15B@1Mbps 包时长） */
#define ROBOT_RF_RETR_COUNT         10      /* 自动重传上限 */

/* ================================ Power ================================== */

/* 功率采样窗口均值样本数（bsp_adc 双同步循环缓冲内取窗）—— 待实测 */
#define ROBOT_POWER_WINDOW          16
/* ADC 原始值 -> 物理量换算系数（检录硬项唯一标定入口，Modules 规划 §3.4）。
 * V_K 当前值 = 引脚级链路标定（2026-10-06，电源模块供电）：灌 PA5 多点线性
 * 拟合残差 ±2mV 级，K 吸收链路增益。⚠ ADC 基准 = VDDA，随供电形态漂移——
 * USB 供电时实测读数整体偏高且随时间增大（换电源模块后 0.9745→0.993 复准，
 * BSP 坑 #13）：供电形态变更（含最终上电池）必须复标。分压电路就绪后 T3
 * 对表更新：最终 V_K = 分压比 × 0.993，并更新标定日期 */
#define ROBOT_POWER_V_K             0.993f
/* 电流链路系数（PA4 电流采样运放）—— 待采样电路就绪后同法对表 */
#define ROBOT_POWER_I_K             0.989f
/* 关节电流路数（双 rank2 的 IN12/IN13 两路，k=0 大臂 / k=1 小臂——器件归属仅注释，
 * BSP 侧见 bsp_pin.h）。消费方 Modules/power → actuator（堵转判定注入） */
#define ROBOT_POWER_JOINT_COUNT     2
/* 关节电流换算系数（两路共用；若实测两路增益不同再拆为 _LIST）——
 * 量程 0~3A 级采样电阻+运放方案待硬件定，先占位 */
#define ROBOT_POWER_JOINT_I_K       0.993f

/* ================================== IIC =================================== */

/* I²C 事务超时（ms）：SSD1306 整页 129B @100kHz ≈ 12ms，取约 2 倍余量 */
#define ROBOT_IIC_TIMEOUT_MS        20
/* I²C 连续失败该次数触发总线恢复（HAL_I2C_DeInit→Init，BSP 坑#10 规划决议）——
 * 拔插排线自愈路径，恢复次数可由 Iic_GetRecoverCount 观测 */
#define ROBOT_IIC_RECOVER_N         3

/* ================================== Oled ================================== */

/* SSD1306 7 位从机地址：SA0=0 → 0x3C（0.96 寸模块默认）；换 0x3D 屏只改此处 */
#define ROBOT_OLED_I2C_ADDR         0x3C
/* Oled_Refresh 连续失败该次数后自动重跑器件初始化——拔插排线 = OLED 模块
 * 掉电，寄存器态全丢，须重发 init 序列（含充电泵 0x8D）才能恢复显示（坑#5） */
#define ROBOT_OLED_REINIT_AFTER_FAILS  5

/* ================================ Link =================================== */

/* 车端各任务间通信的统一命令包结构尺寸占位，随 UserApp 二期细化 */
#define ROBOT_CMD_MAX_SIZE          16

/* ---- RC_Cmd 遥控协议层（W2.2 §3.8 帧布局冻结；两端 robot_config 必须同步）---- */
/* 逻辑量满幅：vx/vy/omega 取 -1000..+1000（发送端钳位，接收端 ÷满幅归一化） */
#define ROBOT_CMD_RANGE             1000
/* 关节速率指令满幅：joint[] 取 -100..+100 */
#define ROBOT_CMD_JOINT_RANGE       100
/* 关节指令路数（= RCPayload_t.joint[] 元素数，与五舵机 ROBOT_SERVO_COUNT 对齐；
 * 改动会打破 15B 帧长静态断言，属协议变更须两端同时改） */
#define ROBOT_CMD_JOINT_COUNT       5
/* 失联渐停时长：指令从失联检测时刻起线性滑落到零（防甩矿渐停，非猛刹车） */
#define ROBOT_CMD_DECAY_MS          300

/* ================================ Vofa =================================== */

/* VOFA+ JustFloat 整定数据流（UART4/PC10-PC11，与日志 USART2 分口）：
 * 波特率——1kHz×4float+帧尾=20B/帧，921600 下 IT 发送占空比 ~22%；
 * 适配器（CH343）与 VOFA+ 端口设置须一致。通道数在 VOFA+ 协议设置里填 4 */
#define ROBOT_VOFA_BAUD             921600
#define ROBOT_VOFA_MAX_CH           8
/* 命令行缓冲（整定命令最长如 "P0.0050"/"R-300.0"） */
#define ROBOT_VOFA_RX_LINE          24

/* ================================ TestBench =============================== */

/* 板级测试台选择（取值见 Tests/test_bench.h 枚举）：0=关闭（业务固件常态，
 * 测试任务只打心跳+栈高水位）；新板 bring-up 时按依赖序逐项改选：
 * 1=GPIO(S1) 2=ENCODER(S3) 3=SPI(S2) 4=PWM(S4) 5=REMOTE(S2 链路收发)
 * 6=MOTOR(S5 开环+闭环阶梯) 7=SERVO(S5) 8=POWER(S6) 9=OLED(S7)
 * 10=ACTUATOR(预留 W2.4) 11=RC_CMD(W2.2) 13=TUNE(W2.3 速度环整定台，
 * 串口命令调参+1kHz 回传+启动自检；12 曾为 DWT 终审台已清理不用)。
 * 激活时业务任务自动让位。 */
#define ROBOT_TEST_BENCH            13

#endif /* ROBOT_CONFIG_H */
