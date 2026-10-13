# 《机甲夺矿》UserApp 层开发规划 v1.0

> 上游：《RM校内赛代码分层架构》v1.2（四层单向依赖 UserApp → Modules → Bsp → Hardware）。  
> 姊妹篇：《Modules层开发规划》v1.1（模块 API 与"业务决策禁止下沉"边界裁决）。  
> 依据：2026-10-10 主控烧毁待换片的硬件空窗期决策——借数字孪生（web_sim，协议同构）提前开发 UserApp 业务链，孪生可验的部分先行，上板必验的挂账。

## 一、层定位与边界

四层中的 L4：**业务决策**——机构状态机、麦轮正逆解、按键→动作映射、手感/量纲换算、保护触发后的拍板时机。Modules 给"设备能力"（设转速/设角度/读功率），本层决定"什么时候给多少"。

**规则（继承架构文档 §四并细化）：**

1. include 白名单：`Modules` 各头文件、`robot_config.h`、`<stdint/stdbool/string/math>`。禁止触碰 `bsp_*.h`（那是 Modules 的下游）、禁止 HAL/寄存器。
2. **应用平行**：chassis/grab/sense/daemon 互不 include，只通过 RC_Cmd 快照与 robot_config 共享类型；跨任务协作走 freertos.c 编排。
3. 任务入口强覆盖 CubeMX `__weak StartApp_Xxx_Task`（remote_task/chassis_task 既有模式，CubeMX 文件零改动）。
4. 纯函数与任务骨架分离：业务算术放 `*_ctrl.c`（纯函数、调用方持有数据、PC 单测与孪生桥共用），任务文件 `*_task.c` 只做节拍/使能权/让位。**这是本层的"一拆二"**——对应 Modules 层"模块逻辑/测试台"分法。
5. 诊断打印禁止进本层（归 Tests/ 与 bsp_log），本层静默跑业务，1Hz 统计级输出除外。
6. 验收口径两栏制，每个业务点验收时登记在哪一栏：
   - **孪生可验**：逻辑正确性、手感曲线、按键合成、状态机、急停/失联语义可视化；
   - **上板必验**：电机 PID 带载整定、舵机 PULSE/LIMIT 标定、编码器/ADC/RF 实链路、把手感参数在实车上复标。

## 二、起点：数字孪生桥路线（PcTwin）

### 2.1 协议同构（2026-10-11 勘察定案）

孪生上行 15B 帧与 rc_cmd v2 逐字节一致（`seq u8 | vx/vy/omega i16 LE | joint[5] i8 | keys u16 | flags u8`），同为小端；"无应用层校验"语义等价（nRF24 硬件 CRC16 + ACK / WebSocket 帧边界 + TCP 校验）。孪生关节序、estop 位、速率语义（接收端自行积分+限位）全部对齐——**孪生遥控端就是协议 v2 的一个合法发送端**。

### 2.2 桥接结构

```
网页推杆 → 15B帧 → ws_bridge.py(HTTP 8000 + WS 8765，替代 start.py 参考后端位)
                  → twin_harness.exe（C 子进程，管道行协议 F/S）
                      跑固件同款源文件：rc_cmd.c + chassis_ctrl.c + grab_ctrl.c
                  → 下行 JSON（整形后底盘逻辑量 + 合成关节速率 + 急停闩锁态）
                  ← 3D 模型按真链路输出动作；telemetry JSON 回传 pose/轮速/关节角供对表
```

- **跑的是同一份 C 源码，不是复刻逻辑**——孪生上看到的手感/急停/渐停即为固件行为。FreeRTOS 快照锁由 `PcTests/freertos_stub` 空实现顶替（同 PcTests 先例）。
- 对照验证：孪生积分关节角（rad，±90° 满幅）vs C 链 Grab 目标角（0..180° 舵机坐标），换算 `deg = rad×180/π + 90`，桥每 50 帧打印最大偏差；爪开合（joint[3]，孪生为 0..12mm 行程）量纲不同，对表跳过。
- 运行：`PcTwin/启动孪生桥.bat`（与 `web_sim/server/start.py` 二选一，都占 8765）。
- 孪生能力圈（运动学级：麦轮正逆解 + 关节积分 + 电机一阶滞后，无动力学/无碰撞）恰好覆盖本层业务；执行器件行为（PID/堵转/脉宽）不在圈内，仍归上板。

### 2.3 现有任务骨架

- `remote_task.c`：RC_Cmd 唯一写者，IRQ 驱动 + 20ms 兜底，PRX 全链已过验。
- `chassis_task.c`：1kHz `Motor_SpeedLoopUpdate` 节拍已挂，缺的正是逆解段（§三.2 补上后接线）。

## 三、任务规格与 API

### 3.1 五任务总表

| 任务 | 周期 | 优先级 | 输入 → 输出 | 现状 |
|---|---|---|---|---|
| remote_task | IRQ + 20ms 兜底 | 高 | nRF24 帧 → RC_Cmd 唯一写者 | ✅ W2.2 过验 |
| chassis_task | 1ms（速度环节拍） | 高 | RC_Cmd 快照 → §3.2 四步链 → 4×Motor_SetSpeedRpm | 骨架在，逆解段待接 |
| grab_task | 10~20ms | 中 | RC_Cmd 快照 → §3.3 积分链 → Act_SetTarget×5 | 待建（actuator/alarm 已并主线 caf12af） |
| sense_task | 20~50ms | 低 | Power 换算 → OLED 排版 | 未建 |
| daemon_task | 100ms | 低 | 心跳 → Alarm 故障码；喂看门狗 | 未建 |

### 3.2 chassis_ctrl —— 底盘业务链（本轮交付，已冻结）

三步流水（每步独立可测，孪生桥逐级透出中间量）：

```c
typedef struct { float vx, vy, omega; } Chassis_Vec_t;   /* 归一化或物理量，看语境 */
void Chassis_Shape(const Chassis_Vec_t *in, Chassis_Vec_t *out);  /* 死区+平方手感 */
void Chassis_ToPhysical(const Chassis_Vec_t *norm, Chassis_Vec_t *phys); /* → m/s, rad/s */
void Chassis_MecanumIK(const Chassis_Vec_t *phys, float rpm_out[4]);     /* → 四轮 rpm */
void Chassis_MecanumFK(const float wheel_omega_radps[4], Chassis_Vec_t *phys); /* 正解对照 */
```

业务规则（本轮定案）：

- **输入适配归发送端（2026-10-11 用户裁决）**：遥控器/网页端有什么物理输入（摇杆、点动键），由发送端折算成帧里 vx/vy/omega/joint[] 归一化量——车载只认字段，**不读 keys 位图**（keys 语义归发送端定义与消费）。初版曾把孪生遥控端的"底盘按键兜底"搬进车载 Chassis_Mix，已废除——那是让孪生的键盘形态污染业务层，数字孪生只是调试工具，不为它做设计；孪生网页端键盘→摇杆折算（main.js）即发送端映射的参照。
- **手感**：死区 `ROBOT_APP_CHASSIS_DEADBAND`（吃回中漂移）→ 死区外重铺满量程 → 平方曲线（轻推细、推满猛）。急停/失联指令已被 rc_cmd 归零/渐停，照常整形即得安全输出，本层无需再挡。
- **逆解**：PROTOCOL.md §3 公式（γ=FL−1/FR+1/RL+1/RR−1 已含在式内）；输出正 rpm = 轮子向前滚，电机电气方向修正归接线层（ROBOT_MOTOR_SIGN/通道映射表）。任一轮超 `ROBOT_MOTOR_SPEED_MAX_RPM` 四轮等比缩（保方向降车速，不削单轮）。
- 孪生可验：三步链逐级数值、手感曲线形状、饱和行为。上板必验：满量程 0.45m/s 是否与实车匹配、电机方向映射、带载手感参数。

### 3.3 grab_ctrl —— 抓取业务链（本轮交付，已冻结）

```c
typedef struct { float deg[5]; float rate[5]; uint32_t last_ms; bool started; } Grab_State_t;
void Grab_Init(Grab_State_t *g, const float deg0[5]);
void Grab_Update(Grab_State_t *g, const float rate_cmd[5], uint32_t now_ms);
/* 调用方从 RC_Cmd 快照摘出 joint[5] 传入（2026-10-11 收窄，同 chassis 吃
 * Chassis_Vec_t 的口径：纯函数不认识协议类型，速率指令已内含两级故障语义） */
```

业务规则（本轮定案）：

- **速率积分**：快照 joint[]（归一化速率）× `ROBOT_APP_JOINT_RATE_DPS`（180°/s）× dt，直接积分——rc_cmd 快照已内含急停归零/失联渐停语义，本层不再另造，**急停=速率零=角度保持（不回零，矿不松爪）**。关节点动键若遥控器将来配备，由发送端折算成 joint[] 速率字段（同 §3.2 输入适配裁决）。
- **软限位**：借 `ROBOT_SERVO_LIMIT_MIN/MAX_DEG_LIST`（同一 0..180° 舵机坐标系），顶到头钳停不累积；实机机械限位定案后随 Servo 表一并收紧。
- 孪生可验：积分收敛、限位钳停、estop 保持、失联渐停位移曲线、两侧速率标定一致性（对表偏差）。上板必验：180°/s 标定值实机核对、舵机零点/脉宽表、堵转/热保护（actuator 联调）。

### 3.4 sense_task / daemon_task（未建，规划占位）

sense：Power_Get* 换算 → OLED 四行排版（电压/电流/功率/链路状态）。daemon：各模块心跳收集 → Alarm_Set（码表追加只增不改）→ 看门狗喂狗；保护触发后的卸力时机（NeedsCooldown 后何时 Act_Release）在此拍板。两者依赖的 actuator/alarm 已随 W2.4 板验收官并入主线（caf12af），随时可建。

## 四、协议口径表（UserApp 段，2026-10-11 冻结草案）

> 帧（15B/字段/estop）已在 W2.2 冻结不重复；本表是"位的含义与物理标定"，数值源 = 孪生默认（CAD 同源），**实车复标后回写**。

| 项 | 值 | 备注 |
|---|---|---|
| keys 位图 | 车载不消费——语义归发送端定义并在发送端折算成指令字段 | 2026-10-11 用户裁决：输入设备适配归发送端；若将来定义模式键等非指令位再立表 |
| 满量程 vx/vy | 0.45 m/s | 孪生默认，实车按电机能力复标 |
| 满量程 omega | 1.5 rad/s | 同上 |
| 关节速率满幅 | 180 °/s | 孪生按舵机估值（协议未冻结此项），实机核对 |
| 麦轮几何 | L=0.3075m / R=0.029m | CAD 实测，装配复核 |
| 关节序 | 大臂/小臂/手腕/爪开合/爪旋转 | = 舵机 ID1..5（rc_cmd v2 冻结）；爪开合孪生侧为 0..12mm 行程，对表跳过 |
| 坐标系 | x 前 / y 左 / z 上（REP-103） | 孪生一致 |

## 五、冻结表

| 日期 | 项 | 状态 |
|---|---|---|
| 2026-10-11 | chassis_ctrl 四函数签名（Mix 初版即废：输入适配归发送端） | 冻结（只修 bug 不改签名） |
| 2026-10-11 | grab_ctrl 两函数签名 + Grab_State_t（Update 形参当日收窄为 rate_cmd[5]，keys 兜底废除的收尾） | 冻结 |
| 2026-10-11 | 输入适配纪律：车载不消费 keys，发送端折算指令字段 | 裁决记档 |
| 2026-10-11 | 满量程组 / 麦轮几何 | 冻结草案（实车复标回写数值，不改语义） |
| 2026-10-11 | PcTwin 管道协议（F/S 行 + C 前缀 JSON） | 冻结（孪生模型仍在优化，桥两端只依赖 15B 帧，模型变动零返工） |

## 六、回归与欠账

**本轮已验（2026-10-11）**：PcTests 237 checks 全绿（新增 chassis_ctrl 20 + grab_ctrl 17）；twin_harness 管道烟测（手感整形/关节积分/急停闩锁/解除恢复逐项对账）；ws_bridge WS 握手+三帧往返烟测。固件构建：见提交说明（UserApp 新文件为纯逻辑，进固件 GLOB 需零告警）。【2026-10-13 更新】合并 w2.4-pending 后 PcTests **291 checks 全绿**（+test_actuator），业务纯函数在固件内暂无调用者、被 --gc-sections 回收属预期（消费者=task 层与 PC 侧）。

**上板欠账（换片与合并已完成 caf12af；剩余=task 接线+实机复标）**：

1. chassis_task 接线：`RC_Cmd_GetCopy → 四步链 → Motor_SetSpeedRpm` + 使能权拍板；
2. grab_task 建任务：Grab_Update → Act_SetTarget（actuator 5 实例）；
3. 实机复标：满量程组 / 180°/s / 手感参数 / ROBOT_MOTOR_SIGN 方向映射 / Servo PULSE+LIMIT 表（含五舵机死区数与端点扫描——W 模式直发脉宽，"调整阶段"清）；
4. 电机 PID 带载细调（现值 Kp=0.0058/Ki=0.09 空载）、编码器/ADC/RF 实链路回归（测试台 0→11+13 常规流程，12 作废）；
5. kinematics（已实装封存 kinematics-pending 支线，重启先猎直线 x 漂移）：重启后孪生 FK 遥测可做交叉验证——注意孪生已有坐标/直线/姿态/法向五模式（T 行），开环精度预算按实测死区 2.2°/关节重定。
