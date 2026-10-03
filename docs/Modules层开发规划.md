# 《机甲夺矿》Modules 层开发规划 v1.0

> 上游：《RM校内赛代码分层架构》v1.1（2026-09-30 修订，L3 清单与接口定稿的裁决记录在案）。  
> 姊妹篇：《BSP层开发规划》v1.2（S0~S4 已验收；测试台规范见其 §十，**本文所有验收均按该规范登记测试台**）。  
> 依据：BSP S0~S4 落地实况与经验（remote 双板联调 W1 验收、nRF24 三处手册级修复、测试台常驻机制）。

## 一、Module 层定位与边界

四层中的 L3：**封装器件知识与设备级算法**，对 UserApp 只暴露"设备能力"（设转速/设角度/读功率），对下只消费 BSP 外设接口。两条判断标准与 BSP 互为镜像——

- 封装的是**器件知识**（TB6612 真值表、SSD1306 初始化序列、脉宽↔角度映射、V/I 换算系数）→ 本层；
- 封装的是**业务决策**（机构状态机、麦轮正逆解、按键→动作映射）→ UserApp，禁止下沉。

**规则（强制，继承架构文档 §四并细化）：**

1. include 白名单：`bsp_xxx.h`、`robot_config.h`、`cmsis_os.h`（仅限 remote 先例的 ISR 通知模式，新用法须先在本文登记）、`<stdint/stdbool/string>`。**禁止**触碰 HAL / 寄存器 / CubeMX 句柄。
2. 模块间不互相 include；**唯一登记例外：actuator → servo**（保护策略包装器件层，单向——策略无器件知识、器件无策略，合并不了，拆开才能各自单测；架构文档 §四.2 已同步）。需要跨模块协作时一律上浮 UserApp 编排。
3. 静态分配，禁止 malloc；一切阻塞必须有界（超时值进 `robot_config.h`）。
4. ISR 禁则继承 BSP 规划 §五：回调只 osSignalSet 级搬运，解析/状态机在任务侧。
5. 命名：`模块_动作`（`Motor_SetDuty`、`Servo_Release`），实例 `g_` 前缀；可调数值进 `robot_config.h` 对应分组（Motor/Servo/Power/Link 已有分组骨架）。
6. **每个模块验收即两件事**：① API 对 UserApp 冻结（之后只修 bug 不改签名）；② 验收代码按"一拆二"迁 `Tests/test_xxx.c` 登记测试台，编号只增不改。
7. 器件驱动动工前先列 **《spec 对照清单》**（逐节读器件手册，位域/时序/真值表逐条记录再写码）——nRF24 三处修复的直接推论，见 §二.3。

与 Tests/ 的边界重申：测试台是人用的诊断（手动选项目、详细打印），本层模块是机器人用的业务（静默、1Hz 统计级输出）；同一模块两边各留一份消费者是刻意的。

## 二、起点：remote 落地范式（S2 沉淀，后续模块照此办理）

### 2.1 已交付与接口现状

`Modules/remote/` 双文件结构已冻结：`nrf24_defs.h`（器件常量）+ `nrf24.c/.h`（寄存器级驱动）+ `remote.c/.h`（链路层：PRX/PTX 双模式、IRQ 服务、400ms 看门狗、收发诊断计数）。接口为**字节流帧**形态（`Remote_ReadPacket/SendPacket`），帧格式留给上层；W1 双板联调验收通过（100pkts/s、收包率 100%、看门狗 400ms、ch90=2490MHz 避 WiFi）。

### 2.2 六条范式（本层后续模块的开发模板）

1. **器件与链路分文件**：器件常量/寄存器操作独立成对（nrf24_defs/nrf24），链路逻辑独立（remote）——换器件只动前者，换协议只动后者。
2. **IRQ 极简 + 行为注入**：`Exti_Attach` 注册回调仅 `osSignalSet`，搬运与解析全在 `Remote_Service`（"谁 Init 谁服务"模式，Init 记录调用者线程）。
3. **判据写"总线存活"而非"出厂状态"**：模块不随 MCU 复位掉电，复位后读到残留寄存器（如 RX_DR=0x40）不是故障，全 0x00/0xFF 才是总线死（bf64d02 教训）。
4. **诊断计数内建**：`GetRxCount/GetTxFailCount` 随模块交付，联调期 1s 统计行直接可用，不另搭 instrumentation。
5. **一源两板**：`CONFIG_REMOTE_UNIT` 编译开关区分车端/发送端，同一份源码进多个 CubeMX 工程（F103C8_PTX_T 测试板已验证此路线；遥控器整机 Remoter 生成后同模式接入）。
6. **测试台登记 + 验收一拆二**：A1/A2/A3 体检、寄存器 dump、逐包打印进 `test_remote.c`（TEST_BENCH_REMOTE）；业务循环进 `remote_task.c`。

### 2.3 手册级教训（推广为硬性流程）

S2 三处修复全部是"凭直觉写位域/命令"所致，LA 抓 MOSI 波形才定案：

- 命令拼装**严禁**一律 `reg & 0x1F`（0xA0 被截成写 CONFIG，SPI 读回却正常——读正常≠写生效）；
- 位域以手册为准不凭记忆（码率 [DR_LOW,DR_HIGH] 双位编码、FIFO_STATUS 的 TX_EMPTY=bit4）;
- 硬件疑难的终审手段是 **LA 抓全部引脚逐字节解码**，不猜。

→ 推广：**任何新器件驱动（TB6612/SSD1306/舵机）动工前，先在本文档对应小节补《spec 对照清单》**（数据手册章节 → 本工程用到的条目），写码时逐条对照，验收时逐条打勾。

### 2.4 W2 待补：RC_Cmd 遥控协议层

链路层之上缺"指令语义"层——这是车端闭环控制开跑的**硬前置**（并发语义三行字见架构文档 §5.1）。随 §3.8 落地。

## 三、模块规格与 API 定稿

### 3.1 motor —— TB6612 ×4（S5 开环 / W2 速度环）

> 原架构文档名 dcmotor，v1.1 收敛为 motor（对齐 BSP 规划 v1.1 与目录现状）。

```c
typedef enum { MOTOR_CH1 = 0, MOTOR_CH2, MOTOR_CH3, MOTOR_CH4, MOTOR_CH_COUNT } MotorCh_t;
bool Motor_Init(void);                           /* STBY 保持低（上电安全），装配方 PID（W2） */
void Motor_SetDuty(MotorCh_t ch, float duty);    /* -1.0..+1.0，符号即方向；先写 DIR 后写 PWM；NaN 滑行
                                                  * 【2026-10-03 变更】int16±1000→float：与 Pwm_SetDuty 同量纲，
                                                  * 速度环 MaxOut(1.0f) 直喂零换算 */
void Motor_Enable(void);                         /* STBY 拉高 */
void Motor_Disable(void);                        /* duty=0 + STBY 拉低，双保险 */
/* —— W2 速度环（algorithm 就绪后）—— */
void  Motor_SetSpeedRpm(MotorCh_t ch, float rpm);
float Motor_GetSpeedRpm(MotorCh_t ch);           /* Encoder_Read × ROBOT_ENC_PPR 换算 */
```

**实现要点**：① 符号→方向映射在本层（BSP 的 PWM 不知方向），真值表按手册：IN1=1/IN2=0 正转、0/1 反转、0/0 滑行、1/1 短刹——写进《spec 对照清单》；② **同边沿切换**：改向时先置新方向再给 PWM，杜绝共导通；③ 死区 `ROBOT_MOTOR_DEADBAND`（占位，实测补）；④ 轮向修正 `ROBOT_MOTOR_SIGN`——**电机序号硬件未定**，通道→(PWM ch, DIR 引脚对) 映射表收进 robot_config.h 一处改；⑤ Disable 必须 duty=0 + STBY 低同时做。

**验收（S5）**：四路开环正反转（duty 阶梯 200/500/1000；【2026-10-03】float 化后为 0.2/0.5/1.0），方向与编码器计数符号交叉验证；Disable 后手转轮无阻力矩（STBY 断）。测试台 **6=MOTOR**。

### 3.2 servo —— 舵机 ×4（S5，器件层）

> **与 actuator 的分层关系**：本模块只回答"怎么动"（器件知识：脉宽↔角度、软限位、卸力），不知道"何时动、动多久"；actuator（§3.3）是保护策略层，回答"何时动/动多久"，包装本模块而不重复其知识。数据流：grab_task → Act_SetTarget →（Act_Update 状态机）→ Servo_SetAngle / Servo_Release → bsp_pwm TIM5。

**器件构型（2026-10-01 确认，两类舵机能力不同，模块内建分型能力表）：**

| 关节 | 器件 | 类型 | 保持/卸力行为 |
| --- | --- | --- | --- |
| 大臂 / 小臂 | 数字舵机 ×2（PM10S 级：金属齿 25T、9~13kg·cm、0.13~0.18s/60°、脉宽 0.5~2.5ms、空载 100mA） | 数字，**fail-hold** | **发一次脉冲即锁存保持，此后停发脉冲≠卸力**（内部闭环照常出力）；软件无卸力手段，真卸力需供电断开（硬件 MOS 开关，见硬件建议） |
| 手腕 | SG90（~1.8kg·cm，塑料齿） | 模拟 | 需持续 50Hz 脉冲保持；**停发脉冲 = 真卸力**（Pwm_Release 即可）；业务上**全程保持不松** |
| 爪子 | SG90（定案 2026-10-01，360° 方案作废），齿轮组传动，矿物轻 | 模拟位置模式 | 同手腕：需持续 50Hz 脉冲、停发脉冲=真卸力；夹持出力间歇短时 |

```c
typedef enum { SERVO_ID1 = 0, SERVO_ID2, SERVO_ID3, SERVO_ID4, SERVO_COUNT } ServoId_t;
/* ID 与关节映射、器件型号进 robot_config.h（SERVO 组每 id 独立配置脉宽/限位/型号）；
 * 按构型：ID1=大臂(数字) ID2=小臂(数字) ID3=手腕(SG90) ID4=爪子(SG90)——以实机布线为准 */
bool Servo_InitAll(void);                        /* 数字舵机发一次目标脉冲；SG90 发中位脉冲 300ms 再可控 */
void Servo_SetAngle(ServoId_t id, float deg);    /* 0..180；软限位钳位后 deg→us 换算（每 id 独立标定） */
bool Servo_Release(ServoId_t id);                /* 返回=本次是否真卸力【2026-10-03 变更 void→bool】；
                                                  * SG90：停脉冲=真卸力；数字舵机：停脉冲=锁存不变（见上表） */
bool Servo_CanUnload(ServoId_t id);              /* 能力查询：该 id 软件能否卸力（actuator 装配用，无副作用）。
                                                  * 【2026-10-03 改名】ReleaseCutsPower→CanUnload：语义从"切断供电"
                                                  * 实现细节上移为"软件卸力能力"；不合并进 Release（装配期查询必须
                                                  * 无副作用，且器件解释权收在 servo 一处） */
```

**实现要点**：① 脉宽↔角度线性映射按每 id 独立标定（`ROBOT_SERVO_PULSE_MIN/MAX_US[id]`）——PM10S 标称 0.5~2.5ms、SG90 标称 500~2500µs 但实际行程普遍不足 180°，**全部待实机标定**；② 软限位每关节独立（`ROBOT_SERVO_LIMIT[SERVO_COUNT][2]`，占位 ±5° 待结构定）；③ 四通道持续 50Hz 发脉冲（SG90 必需，数字舵机无害且行为统一），`Pwm_Release` 仅用于 SG90 卸力与全关断；④ Init 的"中位 300ms"与 bsp_pwm 的"上电 0 脉宽"构成两级安全链；⑤ S5 开工时同步 `robot_config.h` 的 `ROBOT_SERVO_COUNT` 3→4 并填器件分型表。

**验收（S5）**：上电缓至中位无猛冲；0→90→180° 扫描平滑；SG90 Release 后手掰无保持力矩；**数字舵机停脉冲后仍保持（验证 fail-hold 特性并记录）**。测试台 **7=SERVO**。

**硬件建议（转硬件组，影响保护闭环）**：① 数字舵机供电回路加 GPIO 控制的 MOS 开关——没有它，大臂/小臂堵转时软件只能报警不能动作（fail-hold 锁住出力）；② 电流采样（ADC3 IN12/13）量程按大舵机堵转设计（0~3A 级，采样电阻+运放，阈值待实测）；③ SG90 与大舵机分路供电，避免小舵机噪声污染大舵机电流采样。

### 3.3 actuator —— 执行器保护策略（W2.4，规划 5.1/5.2 的载体）

> **现实约束（2026-10-01 三次修正，按器件分型）**：①PWM 舵机无位置反馈——"到位"只能开环假定（`Act_IsSettled`），MOVING 时长=指令斜坡 T_move=|Δ|/ease_dps 是确定性事件，"运动超时"判据不存在；②**器件分型决定保护能力**（servo §3.2 能力表）——SG90（模拟）：停脉冲=真卸力，供电时长=发脉冲时长，热保护有效；数字舵机（fail-hold）：停脉冲≠卸力、锁存后恒供电，**热保护无意义，电流判据是唯一真判据**，且触发后软件只能报警（除非硬件加了供电开关）。保护语义 = **能测就测（电流），测得了就断（SG90 卸力/断电开关），测不了也断不了的交给报警+操作手**。

《机甲夺矿》保护策略的落点（规划 5.1/5.2）：缓动防甩矿、堵转工程防线。接口在架构文档 §六草案上修正：

```c
typedef struct {
    uint8_t  servo_id;
    float    min_deg, max_deg;   /* 软限位 < 机械限位 5~10° */
    float    ease_dps;           /* 缓动限速 100~150 °/s（PM10S 速度 330+°/s，斜坡始终由我们限速） */
    /* —— 电流堵转判据（数字大舵机实例必配；SG90 实例 get_current_a=NULL）—— */
    float (*get_current_a)(void);/* 绑 Power_GetJointCurrent(k)；NULL=无采样通道 */
    float    stall_current_a;    /* 堵转电流阈值（量程按 0~3A 设计），占位待实测 */
    uint32_t stall_confirm_ms;   /* 超阈值确认时长（防抓矿/抬臂正常冲击误判） */
    /* —— 累计供电热保护（仅爪子实例配置；数字实例恒供电、手腕全程保持均无统计意义，置 0 禁用）—— */
    uint32_t energize_window_ms, energize_max_ms, cooldown_ms;  /* 典型 30s / 20s / 5s */
} ActConfig_t;
bool Act_Init(Actuator_t *a, const ActConfig_t *cfg);
void Act_SetTarget(Actuator_t *a, float deg);      /* 重算斜坡，缓动转进 */
void Act_Update(Actuator_t *a, uint32_t now_ms);   /* grab_task 10~20ms 周期调：斜坡/判据/累计器 */
void Act_Release(Actuator_t *a);                   /* SG90：停脉冲卸力；数字：停脉冲+置故障（真卸力待断电） */
bool Act_IsSettled(const Actuator_t *a);           /* 开环：指令斜坡走完（非实测到位） */
bool Act_IsStalled(const Actuator_t *a);           /* 电流判据已触发 */
bool Act_NeedsCooldown(const Actuator_t *a);       /* 热保护超限（仅 SG90 实例，软模式） */
```

**状态机（Update 内推进；MOVING→HOLDING 是纯指令侧确定性转移，无"是否到位"判断）：**

```text
IDLE --SetTarget--> MOVING(按 ease_dps 斜坡发脉冲；SG90 实例供电累计器累加)
MOVING ── T_move 到（纯时间比较，必然发生）──> HOLDING(恒发 target 脉冲；累计器继续)
HOLDING --SetTarget--> MOVING(重算斜坡)
任意态 --Act_Release--> RELEASED（SG90：真卸力；数字：停脉冲+故障态，实际仍锁存出力）
MOVING/HOLDING --电流超阈值持续 stall_confirm_ms--> 立即 Act_Release + stalled（数字实例同时报警 ALARM_ACT_STALL）
HOLDING --供电累计超限（仅 SG90）--> 置 NeedsCooldown（软模式：grab 择机卸力）
```

**保护矩阵（四关节各自策略，2026-10-01 按业务约束修订）：**

**分工铁律：actuator 只有器件知识，只负责报告（`Act_IsStalled`/`Act_NeedsCooldown`）与执行（`Act_Release`）；"什么时候动作是业务安全的"只有 grab_task 知道——触发器在 actuator，拍板时机在 grab。** 手腕就是极限例证：actuator 永远不可能知道"手腕全程不能松"这个业务约束。

| 关节 | 器件 | 堵转判据 | 热保护 | 触发后响应（grab 按关节拍板） |
| --- | --- | --- | --- | --- |
| 大臂 / 小臂 | 数字 fail-hold，**有电流采样** | **电流判据**（唯一真判据，运动+保持全程生效） | 禁用（锁存后恒供电，无统计意义） | 电流触发=立即停脉冲+`Act_IsStalled`+**报警**；fail-hold 下软件卸不了力，真卸力靠硬件供电开关（无则操作手断电） |
| 手腕 | SG90，无采样 | **无判据**——且**全程业务性保持**=正常态就是持续供电，时间代理彻底失效 | **禁用**（必然误报） | 防线=负载轻（1.8kg·cm 对手腕绰绰有余，结构留裕量）+报警兜底（`Act_NeedsCooldown` 若启用仅作告警，**不自动停脉冲**——机构会失控）+操作手急停；停脉冲仅业务明确安全时（回库/赛毕） |
| 爪子 | SG90（定案），齿轮组传动，矿物轻 | **无判据** | **有效且推荐**：夹持出力是间歇短时（窗口累计低），堵转/异常长夹顶得破上限；触发停脉冲=真卸力=掉矿——矿轻+可重抓，可接受 | grab 拍板：`Act_NeedsCooldown` → 立即 `Act_Release`（掉矿代价 < 烧机） |

**爪子器件定案（2026-10-01，与结构组确认）：SG90，360° 速度模式方案作废**——四路舵机全部位置模式，`Servo_SetAngle`/actuator 角度斜坡全链适用，servo.h **S5 验收即冻结**（无悬案）。

**实现要点**：① 状态机、判据、累计器全在 `Act_Update` 节拍内闭环，不经 daemon；② 电流获取走**依赖注入**（config 函数指针），actuator 不知道 power 存在——不新增第二个模块间 include 例外，PC 单测可注入模拟电流源；③ 实例的器件能力经 `Servo_CanUnload(id)` 查询（actuator→servo 唯一登记例外内的合法调用；【2026-10-03 改名】原 ReleaseCutsPower），Release 分支按能力表走；④ 实例由 grab_task 持有（静态 4 份），斜坡/确认计时/累计器纯逻辑可 PC 单测；⑤ 滑动累计器：环形桶数组（30×1s 桶），Release/IDLE 期间不累加；用滑动窗口而非连续计时——堵转可"卡一下松一下"绕过连续计时器，滑动窗口里休息只能靠老桶出窗"还款"，歇不够就攒不回来；参数整定三步：数任务（比赛全序列最长连续出力秒数）→ 上限=任务极限×~1.3 → 实车重载全程验证不误触发（太小=运输途中被强制冷却掉矿，太大=堵转烧机前保护未到限）。

**验收（W2.4）**：SetTarget 后斜坡走完转 HOLDING（示波器：SG90 通道脉冲从步进变恒定），SG90 正常保持永不被自动卸力；人为阻滞大臂（数字+电流）→ 超阈值持续 `stall_confirm_ms` 触发 `Act_IsStalled`+报警，瞬时冲击不误触发；人为制造爪子长供电 → `Act_NeedsCooldown` 置位且夹矿时不强制卸力（手腕实例热保护已禁用、全程保持不误报）；数字舵机 Release 后实测仍锁存（记录入验收日志）。测试台 **10=ACTUATOR**。
### 3.4 power —— 功率采样换算（S6，与 bsp_adc 同日接力）

> 自架构文档 sense_task 上浮为本模块（v1.1 裁决）：换算可脱离整车测试、标定点全系统唯一。

```c
bool  Power_Init(void);                    /* bsp_adc 启动 + 丢前 2 窗口样本 */
float Power_GetVoltage(void);              /* 窗口均值（ROBOT_POWER_WINDOW=16）后换算 */
float Power_GetCurrent(void);
float Power_GetPower(void);                /* 同窗 U×I，双 ADC 同步采样无相位误差 */
float Power_GetJointCurrent(uint8_t k);    /* ADC3 大臂/小臂电流 k=0..1（IN12/IN13，数字舵机
                                            * 供电回路采样，量程 0~3A 待硬件定）；消费方为
                                            * actuator（注入做堵转判定），sense/OLED 亦可复用 */
```

**实现要点**：① DMA 32 位字拆分（低 16=ADC1 电流/高 16=ADC2 电压）——取错位功率就错，先已知分压验证；② 换算系数 `ROBOT_POWER_V_K/I_K` 是**检录硬项**（OLED ±10%）的唯一标定入口，万用表对表后定值并记录标定日期；③ 本模块无任务，被 sense_task 20~50ms 调用。

**验收（S6）**：串口打印 V/I/P，与万用表误差 <5%；双 ADC 同步性（负载突变时 U/I 同拍变化）。测试台 **8=POWER**。

> **S6 结算（2026-10-06）**：软件链路+测试台 8 过验——T0 循环存活 / T1 字序定案（低半字=ADC1、高半字=ADC2，CMSIS 位段+双通道引脚实验实证）/ T2 丢前 2 窗 / T5 ADC3 双通道（PC2→J0、PC3→J1 不串扰）。V_K=0.993（引脚级对表，**电源模块供电**——USB 供电下 VDDA 下漂致读数整体偏高且随时间增大，供电形态变更必须复标，BSP 坑 #13）。从机触发链两坑（EXTTRIG / CONT）详见 BSP 坑 #11/#12。**挂起待硬件**：T3 电池分压对表（最终 V_K=分压比×0.993）、T4 双 ADC 同步性（KEY1 阶跃源已就绪）、T6 关节电流标定（JOINT_I_K 占位中）。

### 3.5 oled —— SSD1306（S7，与 bsp_iic 同日接力）

```c
bool Oled_Init(void);                                        /* 初始化序列 + 上电稳定等待 */
void Oled_Printf(uint8_t x, uint8_t y, const char *fmt, ...); /* 写帧缓冲（1KB 静态） */
void Oled_Refresh(void);                                     /* 帧缓冲 → I²C 整帧刷 */
```

**实现要点**：① I²C 阻塞发送可接受（调用方 sense_task 为低优先级），但**必须带超时**——排线接触不良会拉死 I²C2，bsp_iic 的总线恢复是最后防线；② 初始化序列按手册逐节进《spec 对照清单》（上电稳定期、页/列寻址模式）；③ 显示内容四项：电压/电流/功率/链路状态（检录项优先），排版归 sense_task，本层只管像素。

**验收（S7）**：四项内容刷新无花屏/拖影；拔插排线后 Oled_Refresh 超时不挂死、恢复插回自愈。测试台 **9=OLED**。

> **S7 结算（2026-10-03）**：测试台 9=OLED 板上过验——T0 探测+初始化（0x3C ACK、屏亮无花屏）/ T1 四项对表（V/I/P 与串口同拍一致，整型 mV/mA/mW；LINK=DOWN 属预期——本测试不初始化 remote，W2 接线后转真）/ T2 500ms 周期刷新稳定 / T3 拔插自愈（bsp_iic 连败 3 次总线恢复 + **Oled_Refresh 连败 5 次自动重发 init**——拔插排线=模块掉电寄存器态全丢，仅恢复总线不重发 init 屏不亮，比规划实现要点①多一层）/ T4 地址参数化（0x3C ACK、未插器件 0x68 无 ACK，坑#11 验收点落实）。API 冻结三件套与规划一致；`Oled_Printf` 定稿像素坐标 + **行尾自动擦除**（数值变短旧字符不清自除）；SSD1306 手册对照清单落 `Modules/oled.h` 头部（§8.1.5 控制字节 / §8.7 GDDRAM 位序 / §10 init 序列，充电泵 0x8D 0x14 漏发=全黑）。器件库=afiskon/stm32-ssd1306 移植改造（MIT：传输层换 bsp_iic、失败闩锁、删浮点弧线、字体裁剪 6x8/7x10）；KEY1 切图形演示页（10ms 节拍采样）。

### 3.6 alarm —— 故障码声光（W2，daemon 的执行端）

```c
typedef enum {
    ALARM_OK = 0, ALARM_LINK_LOST, ALARM_POWER_LOW,
    ALARM_TASK_OVERRUN, ALARM_ACT_TIMEOUT, /* …追加只增不改 */
} AlarmCode_t;
void Alarm_Set(AlarmCode_t code);   /* 非阻塞，仅记录当前码 */
void Alarm_Poll(void);              /* 非阻塞节奏发生器：蜂鸣音型 + LED 闪烁码，daemon 100ms 调 */
```

**实现要点**：① 底层全走 bsp_gpio（PIN_LED1/LED2/ALARM 宏已有）；② 音型/闪码表进本头文件注释，同码同灯；③ 与测试台心跳共用 LED1 时注意——测试台激活时本模块随业务任务 Yield。

**验收（W2.4）**：注入失联（停发射频）→ ALARM_LINK_LOST 音型出现；恢复自动回 OK。不单独占测试台编号，随 10=ACTUATOR 项联验。

### 3.7 algorithm —— 纯算法库（W2 第一项，速度环与 actuator 的前置）

**PID 移植自 control-2026**（`Modules/algorithm/controller`，Wang Hongxi 风格，gitee.com/shu-robomaster/control-2026），保留其接口与优化环节位掩码，两处适配：

```c
/* 适配 1：剔除 arm_math 依赖（F103 无 DSP，不引入 CMSIS-DSP 编译链）
   适配 2：bsp_dwt → 本工程 bsp_sys；dt 仍由实例内时间戳自算（Bsp_GetUs 差分），
           调用方不传 dt，控制环节拍精确且调用点干净 */
typedef struct {                      /* PID_Init_Config_s：配置块 */
    float Kp, Ki, Kd;
    float MaxOut;                     /* 输出限幅（速度环内 = duty 满量程 1.0f 对应量纲） */
    float DeadBand;                   /* 死区 */
    PID_Improve_e Improve;            /* 优化环节位掩码，按需启用 */
    float IntegralLimit;              /* 积分限幅 */
    float CoefA, CoefB;               /* 变速积分（积分分离） */
    float Output_LPF_RC, Derivative_LPF_RC;  /* 输出/微分低通 */
} PID_Init_Config_s;

void  PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);
float PIDCalculate(PIDInstance *pid, float measure, float ref);  /* 双输入：反馈 + 设定 */
```

优化环节按需启用（位掩码 `PID_Improvement_e` 原样保留）：速度环建议 **积分限幅 + 梯形积分 + 微分先行（D on Measurement）+ 微分滤波**；**堵转检测（PID_ErrorHandle）对电机有实际价值**（|ref-measure|/|ref|>0.95 持续计数 → 上报故障码给 alarm），与 actuator 的舵机堵转防线（电流判据+供电热保护）形成电机/舵机两侧互补。

其余自研轻量件（学长仓库无可直接复用项）：

```c
float Lpf_Apply(Lpf_t *f, float in);               /* 一阶低通：摇杆/采样平滑 */
float Ease_Step(float cur, float target, float max_step); /* 限速斜坡：actuator 恢复用 */
/* user_lib 精选移植（control-2026 Modules/algorithm/user_lib）：
   abs_limit / float_constrain / float_deadband / loop_float_constrain（角度环形差值，
   编码器累计回绕与麦轮解算会用到） */

```

**运动学解算的归属（2026-10-01 与用户确认）**：

- **麦轮逆解（vx/vy/omega → 4×轮速）归 UserApp/chassis**，不进本模块——它与遥控指令映射、速度限幅、死区、旋转中心手感**一体调参**，是底盘应用的有机部分（对齐 control-2026 chassis 传统，架构 3.3"麦轮正逆解不在此层"即此意）；数据流：`RC_Cmd_GetCopy` 快照 → 麦轮逆解纯公式 → 4×目标轮速 → `Motor_SetSpeedRpm`。公式本身只是线性组合，真正的工作量在参数整定——这正是它长在应用层的证据。
- **机械臂 IK 暂不启用**（2026-10-01 用户拍板）：架构 §八预埋决策不变——若启用放本模块（纯几何函数 `ArmIk_Solve2R`，臂长/限位参数化进 robot_config.h，PC 单测后 grab 调用），届时 grab 的目标源可从示教姿态表无痛切换，`Act_SetTarget` 接口不变。启用前本模块不含任何 IK 代码。

**实现要点**：① 纯 C 无硬件依赖（除 PID 内部时间戳走 bsp_sys 外不 include 任何 bsp/cmsis），可 host 编译 PC 单测（出错边界：dt=0、err 跳变、积分饱和、堵转计数溢出）；② 不占测试台编号（PC 单测），上板行为随 6=MOTOR（速度环）与 10=ACTUATOR（ease）间接验证；③ F103 无 FPU，float 为软浮点——控制环 1kHz 内开销可忽略（S4 已验证同类用法），但禁止在 ISR 用。

### 3.8 remote 扩展 —— RC_Cmd 遥控协议层（W2，新增文件 rc_cmd.c/.h）

链路层 remote.h **不动**（已冻结），协议语义作为独立文件叠加：

```c
#pragma pack(1)
typedef struct {                 /* 13B，载荷 15B 余 2B 备用；两端工程必须同步改 */
    uint8_t  seq;                /* 发送序号（应用层连续性检查；无线层副本由 nRF24 PID 去重） */
    int16_t  vx, vy, omega;      /* 操作意图（逻辑量 -1000..1000，物理映射在车端） */
    int8_t   joint[3];           /* -100..100 速率指令 */
    uint16_t keys;               /* 按键位图 */
    uint8_t  flags;              /* bit0 = estop，连发 3 帧 */
} RCPayload_t;
#pragma pack()

typedef struct {
    float vx, vy, omega;         /* 已映射的定量指令 */
    float joint_rate[3];
    uint16_t keys;
    bool  valid, estop;
    uint32_t last_frame_ms;
} RC_Cmd_t;

void RC_Cmd_Update(const RCPayload_t *f);  /* remote_task 独占调用（唯一写者） */
bool RC_Cmd_GetCopy(RC_Cmd_t *out);        /* 唯一读法：关调度内 memcpy 快照 */
```

**实现要点**：① 并发三行字照架构文档 §5.1 落地——唯一写者、快照读、两级故障语义（link_timeout 指令向零衰减 300ms / estop 立即零）；② 看门狗复用 `Remote_IsLinkUp`，失联时 Update 由 remote_task 以空帧驱动（写 valid=false，保证"第二写者"语义成立）；③ 发送端（F103C8_PTX_T 测试板及未来遥控器整机）同文件复用，急停键触发连发；④ 帧布局在 **W2.2 冻结**，此后两端同步演进。

**验收（W2.2）**：双板走通"摇杆→RC_Cmd 快照"全链路；estop 帧 ≤50ms 生效零输出；停发 400ms 内 valid=false 且指令向零衰减。测试台 **11=RC_CMD**。

## 四、开发顺序（对齐 S5~S7 与 W2，实际进度超前可提前启动）

| 阶段 | 窗口 | 内容 | 验收标准 | 测试台 |
| --- | --- | --- | --- | --- |
| S5 | 10/5（可提前） | motor 开环 + servo | 四路正反转/符号交叉验证；舵机中位/扫描/卸力 | 6=MOTOR 7=SERVO |
| S6 | 10/6 | ✅ bsp_adc → power（2026-10-06 软件侧过验） | V/I 链路级对表过（V_K=0.993）；T3/T4/T6 待采样电路上板 | 8=POWER |
| S7 | 10/7 | ✅ bsp_iic → oled（2026-10-03 板上过验） | 四项显示对表/排线拔插自愈/地址参数化探测全过（§3.5 结算） | 9=OLED |
| W2.1 | 10/8 | algorithm 冻结 + PC 单测 | 边界用例全过（host 编译） | —（PC） |
| W2.2 | 10/8 | rc_cmd 协议层 + 帧布局冻结 | estop ≤50ms；失联向零衰减 | 11=RC_CMD |
| W2.3 | 10/8 | motor 速度环 | 阶跃响应无超调振荡（整定记录进 robot_config.h 注释） | 随 6=MOTOR |
| W2.4 | 10/9 | actuator + alarm | 限时/缓动/卸力时序正确；故障音型正确 | 10=ACTUATOR |
| W2.5 | 10/9 | **全测试台回归**（编号 0~11 逐项重跑）+ 交接 UserApp | 全绿；chassis/grab 开跑 | 全部 |

依赖链：algorithm →（速度环、actuator）；bsp_adc → power；bsp_iic → oled；S5 两模块仅依赖已验收 BSP，**可立即开工**。提交规范继承 BSP 规划 §六（每阶段 ≥1 次功能小提交、作者真名、message 注明验收结果）。

## 五、与 UserApp 的交接约定

- **冻结时点表**：remote.h ✅已冻结 / motor·servo S5 验收即冻结 / power S6 / **oled ✅S7 冻结（2026-10-03，`Oled_Init` / `Oled_Printf(x,y,...)` / `Oled_Refresh` 只修 bug 不改签名）** / algorithm·rc_cmd W2.1~2.2 / actuator·alarm W2.4。冻结后只修 bug 不改签名，新需求走版本演进。
- robot_config.h 填充责任：Motor 组（SIGN/DEADBAND/PPR 实测值）、Servo 组（**器件分型表 id↔关节↔型号** + 每 id 脉宽/限位实机值 + 数字舵机**堵转阈值 STALL_CURRENT_A / 确认时长 STALL_CONFIRM_MS** + SG90 **热保护 ENERGIZE_WINDOW_MS·ENERGIZE_MAX_MS·COOLDOWN_MS**）、Power 组（V_K/I_K 标定值+日期）、Link 组（帧布局）——各模块验收时**顺手填掉占位**，不留"待实测"过夜。
- 麦轮正逆解、机构状态机、按键映射归 UserApp（chassis/grab），本层不预置任何业务概念；IK 启用时放 algorithm，grab 接口不变。
- 遥控器整机 = `Hardware/Remoter` 独立 CubeMX 工程（**现仅 .ioc 未生成**，W2 生成后开工），按 F103C8_PTX_T 已验证的模式接入：独立工程 + `CONFIG_REMOTE_UNIT`，复用本层 remote/rc_cmd。**F103C8_PTX_T 是 nRF24 链路测试板（S2 遗产：双板联调/载波/频偏/角色互换诊断），不承担遥控器职能。**

## 六、坑清单（Modules 视角，按踩中概率排序）

1. **TB6612 真值表想当然**：IN1/IN2 组合 00=滑行、11=**短路刹车**（不是停）；PWM 必须接 PWMA/B 脚而非 IN 脚——写码前逐条进 spec 对照清单。
2. **电机序号/轮向未定**（README 明示由布线定）：映射表 robot_config.h 一处改，代码零改动；先开环+编码器符号交叉验证再闭环，**编码器符号 × 电机方向不一致 = 正反馈自激**。
3. **PID 整定乱序**：先 P 后 I 再 D，抗积分饱和与输出限幅（对应 duty 满量程）必须内建而非外挂；整定参数注释进 robot_config.h（含日期与工况）。
4. **舵机堵转（按器件分型，2026-10-01）**：两类舵机能力完全不同——数字舵机（大臂/小臂）fail-hold：发一次脉冲锁存保持，**停脉冲≠卸力、锁存后恒供电**（热保护对它无意义），唯一真判据是电流，触发后软件只能报警（真卸力需硬件供电开关）；SG90（手腕/爪子）模拟：停脉冲=真卸力；**但热保护只对"间歇出力"的关节有效**——手腕全程业务性保持（正常态=持续供电），时间代理无判别力，必须禁用，防线靠轻载+报警+操作手；无位置反馈：MOVING 时长=指令斜坡 T_move（确定性），勿幻想"运动超时"判据；正常保持与堵转在指令侧同貌。500~2500µs 全部待实机标定（PM10S 标称 0.5~2.5ms、SG90 实际行程普遍不足 180°）。
5. **SSD1306 初始化时序**：上电需稳定期，初始化序列缺一步就花屏；寻址模式（页/列）搞错=乱码但不报错——对照清单逐项打勾。【2026-10-03 已闭环】对照清单落 `Modules/oled.h` 头部（§8.1.5/§8.7/§10 逐节，含充电泵 0x8D 0x14 漏发=全黑）；S7 板上过验无花屏。
6. **I²C 挂死**：OLED 排线接触不良会拉死 I²C2，超时+总线恢复（bsp_iic）是硬要求，禁止无限等（继承 BSP 坑 #10）。【2026-10-03 已闭环】bsp_iic 超时 20ms + 连败 3 次恢复落地；T3 拔插自愈过验（OLED 层连败 5 次自动重发 init 补齐"模块掉电"场景）。
7. **ADC 取错位/首批样本**：32 位字拆分（低 ADC1/高 ADC2）与丢前 2 窗口，S6 第一件事是已知分压验证（继承 BSP 坑 #3/#4）。【2026-10-06 已闭环】字序经 CMSIS 位段（低半字=主机 ADC1/高半字=从机 ADC2）+ 板上双通道引脚实验定案；实际破的坑是双同步从机触发链（BSP 坑 #11/#12）与 VDDA 供电漂移（#13）。
8. **SRAM 预算**：OLED 帧缓冲 1KB 为本层最大单项（§七），新增 ≥256B 缓冲先登记。
9. **ISR 误用**：algorithm/actuator 一切函数禁止进 ISR（软浮点+状态机）；ISR 只 osSignalSet。
10. **共地！！！**：舵机/电机大电流路径与信号地必须共点，驱动异常先查地再查码（继承 README 铁律）。
11. **I²C2 预留插针（2026-10-01 硬件决策）**：PB10/PB11 总线上多留一组 4P 插针（3.3V/GND/SCL/SDA）作车端扩展槽——地址不冲突（OLED 0x3C vs IMU 0x68/0x6A/0x6B），插针上**不额外放上拉**（OLED 与 IMU 模块板载上拉并联即可，勿叠加第三个），电源引 3.3V 与 OLED 同域防混电平；**软件前提：S7 的 bsp_iic 必须地址参数化**——现有草案 `Iic_Write(buf,len)` 未带地址（隐含写死 OLED），须统一为 `Iic_Write/Read(addr,...)` 对称接口，作为 S7 验收点之一；杜邦线外接器件务必共地。【2026-10-03 软件前提已落实】`Iic_Write/Read/WriteReg/ReadReg/IsDeviceReady` 全部 7 位地址入参定稿，T4 探测过验（0x3C ACK/0x68 无 ACK）；硬件侧决议不变。

## 七、资源预算增量（基线：BSP 规划 §八，剩余 ≥22 KB）

| 项 | 占用 | 说明 |
| --- | --- | --- |
| OLED 帧缓冲 | 1 KB | 128×64/8，本层最大单项 |
| motor 实例 + PID×4 | ~0.2 KB | 通道映射表 + PID 状态 |
| actuator 实例×4 | ~0.2 KB | 静态于 grab_task（4 舵机各一份） |
| RC_Cmd + 快照双缓冲 | ~0.1 KB | 13B 载荷 + float 命令结构 |
| algorithm/Power/alarm | <0.1 KB | 纯状态，无大缓冲 |
| **合计** | **<1.6 KB** | 剩余仍 ≥20 KB；新增大缓冲先登记 |

---
编制：2026-09-30 | 依据：BSP S0~S4 验收实况（81f732c）+ remote 双板联调经验 | 上游：架构 v1.1 / BSP 规划 v1.2
