# Hardware（CubeMX 生成代码）对照《RM校内赛代码分层架构 v1.0》审查报告

- 审查对象：`Hardware/F103RC/`（STM32CubeMX 6.18.1 / FW_F1 V1.8.7 / CMake 工程链）
- 对照文档：资料库《RM校内赛代码分层架构 v1.0》（含 3.2 Bsp 表、第五章数据流、W1 落地顺序）
- 审查日期：2026-09-26

---

## 一、结论总览

时钟树、串口对调（UART4=HC-05 / UART5=调试）、编码器四路选型与重映射、GPIO 基本盘、FreeRTOS 五任务框架等**主体方向与规划一致**；但存在 **4 个 P0 级功能缺陷**（其中 1 个是 ST HAL 宏的隐蔽坑）和若干参数偏差，**当前代码尚不能支撑规划中的 remote / bsp_usart / bsp_adc 功能**。按 P0 → P2 列出如下。

---

## 二、P0 —— 功能性错误（不改则规划功能失效）

### P0-1 UART4 / UART5 均未启用 NVIC 中断，串口方案整条链路不通

- **位置**：`F103RC.ioc`（NVIC 表只有 DMA2_Ch3、TIM6、TIM7）；`Core/Src/stm32f1xx_it.c`（只有 TIM6/TIM7/DMA2_Channel3 三个 handler，无 `UART4_IRQHandler` / `UART5_IRQHandler`）。
- **原因分析**：
  - 规划要求 UART5 调试口用 **IT 环形缓冲**——没有 `UART5_IRQn` 就永远不会进 `HAL_UART_RxCpltCallback`，缓冲收不到任何字节；
  - 规划要求 UART4 **DMA 循环接收 + IDLE 判帧**——HAL 的 `HAL_UARTEx_ReceiveToIdle_DMA()` 依赖 UART4 自身中断来检测 IDLE 标志，中断未使能则只会在 DMA 半满/全满时回调，判帧机制失效，`remote_task` 的"IDLE 事件驱动"模型整个落空。
- **修改建议**：CubeMX → NVIC → 勾选 UART4 全局中断、UART5 全局中断，优先级设 **5**（FreeRTOS 下调用 FromISR API 的硬下限，见 FreeRTOSConfig.h `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`），重新生成。

### P0-2 ADC1/ADC2 未配置双 ADC 同步模式，规划的"同步采样"完全没有落地

- **位置**：`Core/Src/adc.c`：ADC1、ADC2 均为 `ExternalTrigConv = ADC_SOFTWARE_START`、`ContinuousConvMode = DISABLE`、无 DMA；`F103RC.ioc` 中 ADC2 无 Dual Mode 字段。
- **原因分析**：规划 3.2 bsp_adc 要求"ADC1_IN4 + ADC2_IN5 **双同步规则组一次读 32 位 DR** + 定时触发"。当前两个 ADC 各自独立、软件触发，① 拿不到 32 位联合 DR，② 功率的电压/电流不同步，乘积会有相位误差，③ 没有固定采样节拍。
- **修改建议（2026-09-26 定稿：采用方案 B，见 P0-3 与《ADC方案B配置指南.md》）**：
  1. CubeMX 中把 ADC1 的 Mode 设为 **Dual Regular Simultaneous Only**（ADC1 为主、ADC2 为从）；
  2. 触发改**连续转换 + DMA1_Ch1 循环**（方案 B 已定稿），不使用任何硬件触发源——原因见 P0-3：F103 的 ADC1/2 触发选择器（EXTSEL）不含 TIM7 TRGO，原规划"TIM7 TRGO 1kHz 触发"硬件上无法成立；
  3. DMA 用 **DMA1_Channel1**（F1 的 ADC1/2 共用 DMA1_Ch1，双同步模式下 DR 为 32 位，DMA 配成 Word + Circular），ADC/DMA 中断均不使能。

### P0-3 TIM7 触发方案在 F103 上不成立（规划缺陷），需改定位为"1 kHz 软件触发节拍"

> **勘误说明**：最初版报告建议"TIM7 TRGO=UPDATE 作为 ADC 触发源"，经复核不成立。TIM7 作为基本定时器**本身可以输出 TRGO**（CR2 的 MMS 位存在，CubeMX 也为其生成了 MasterConfig 代码），但 **ADC1/2 的 EXTSEL 触发选择器中根本没有 TIM7 TRGO 这一档**——HAL 头文件 `stm32f1xx_hal_adc_ex.h` 第 168~199 行给出 ADC1/2 规则组全部触发源：T1_CC1/T1_CC2/T1_CC3/T2_CC2/T3_TRGO/T4_CC4/EXTI11(重映射后=TIM8_TRGO)/软件启动，无 TIM7。而本工程 TIM1/2/3/8 全部用作编码器、TIM4_CC4 是电机 PWM 第 4 路、EXTI11 是按键——**F103RCT6 上不存在空闲的 ADC 硬件触发源**，这是规划文档 3.2 的固有缺陷，不是 CubeMX 配置错误。

- **位置**：`Core/Src/tim.c` `MX_TIM7_Init()`：`Prescaler = 0`、`Period = 65535`（≈1.099 kHz，非 1 kHz）；TIM7 中断已使能（优先级 5）——在修正方案里这个中断反而要用起来。
- **修改建议（2026-09-26 定稿：采用方案 B）**：
  - **方案 B（已选定）**：ADC1/2 双同步规则组 + `ContinuousConvMode = ENABLE` + `DMAContinuousRequests = ENABLE` + DMA1_Ch1 循环搬运（Word 宽度），不使能 ADC/DMA 中断；sense_task（20~50 ms）随时读最新值，功率按固定样本数窗口平均。放弃固定 1 kHz 节拍，但连续模式实际采样率更高且零中断开销。**详细 CubeMX 配置步骤、代码骨架与联调验证清单见工作区《ADC方案B配置指南.md》**。
  - ~~方案 A（TIM7 1kHz 中断软件启动）~~：不再采用，原因见 P0-3——F103 的 ADC1/2 触发选择器（EXTSEL）不含 TIM7 TRGO 档，原规划"TIM7 TRGO 1kHz 触发"硬件上无法成立。
  - TIM7 的 MasterConfig/TRGO 保留为默认（RESET），它不再承担触发职责；**不要**为此占用 TIM3_TRGO/T4_CC4 等已被规划占用的触发源。

### P0-4 ⚠️ F1 HAL 宏陷阱：TIM2 重映射会把 SWJ_CFG 写成非法值 0b111，导致 SWD 调试口失效

- **位置**：`stm32f1xx_hal_msp.c:78` 先执行 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`（写 SWJ_CFG=010，JTAG 禁用、SWD 保留，正确）；随后 `tim.c:419` 的 `__HAL_AFIO_REMAP_TIM2_PARTIAL_1()` 执行。
- **原因分析**：本工程 HAL 库 `stm32f1xx_hal_gpio_ex.h` 中 `AFIO_REMAP_PARTIAL` 宏的实现是：
  ```c
  tmpreg = AFIO->MAPR;          // SWJ_CFG 是写只读位，读回恒为 0
  tmpreg &= ~REMAP_PIN_MASK;
  tmpreg |= AFIO_MAPR_SWJ_CFG;  // = 0x07000000，把 SWJ_CFG 三位全写 1！
  ...
  ```
  SWJ_CFG 写回 **0b111 是 RM0008 定义的禁止值**（实测多为 JTAG-DP 与 SW-DP 全部禁用）。后果：程序跑到 `MX_TIM2_Init()` 之后 **SWD 调试口直接掉线**，只能"Connect under reset"救回。
- **修改建议**（任选其一，推荐 ①）：
  1. 在 `tim.c` 的 `/* USER CODE BEGIN TIM2_Init 2 */`（`MX_TIM2_Init` 末尾、重映射之后）补一行：`__HAL_AFIO_REMAP_SWJ_NOJTAG();` 重新把 SWJ_CFG 写回 010；
  2. 或在用户代码中手工写 MAPR：`AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_TIM2_REMAP) | AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;`
  3. 该行建议同步记入团队 wiki，凡 F1 工程 + 任何 `__HAL_AFIO_REMAP_*` 组合都适用。

---

## 三、P1 —— 与规划参数不一致 / 遗漏

### P1-1 TIM4 电机 PWM 实际是 10 kHz，规划要求 20 kHz

- **位置**：`tim.c` `MX_TIM4_Init()`：`PSC = 2-1`、`ARR = 3600-1` → 72 MHz / 2 / 3600 = **10 kHz**。
- **修改建议**：`PSC = 1-1`（即分频 1），ARR 不变 → 20 kHz，同时保留 3600 级占空比分辨率。

### P1-2 FreeRTOS 堆仅 5120 B，明显不够；chassis/grab 任务栈 512 B 偏小

- **位置**：`FreeRTOSConfig.h:67` `configTOTAL_HEAP_SIZE = 5120`；`freertos.c`：Chassis/Grab/Daemon 栈均为 128 words（512 B）。
- **原因分析**：5 任务动态栈合计 896 words = 3584 B，加 5 个 TCB 后堆余量不足 1 KB，后续 remote 队列、信号量一创建就会 `xTaskCreate` 失败/崩溃。且 F103 **无 FPU**，麦轮逆解 + 4 路速度环全部是软件浮点，512 B 栈大概率溢出。
- **修改建议**：堆提到 **≥10 KB**（48 KB SRAM 完全装得下）；Chassis 栈 ≥256 words；Grab ≥192 words；按规划 W1 用 `uxTaskGetStackHighWaterMark` 做栈普查后再收敛。

### P1-3 ADC 采样时间全是 1.5 周期，威胁 OLED ±10% 检录项；ADC3 只有单通道

- **位置**：`adc.c` 三个 ADC 的 `SamplingTime = ADC_SAMPLETIME_1CYCLE_5`；ADC3 只配置了 IN12（PC3/IN13 已设为模拟脚但未进转换序列）。
- **原因分析**：1.5 周期采样要求信号源阻抗 ≲200 Ω，而功率/电压来自运放/分压网络（规划已注明采样电阻/分压比 TBD），采样会明显失真。规划明确 ADC3_IN12/13 两路慢速采集。
- **修改建议**：功率/电压通道采样时间改为 **55.5~239.5 周期**；ADC3 补第二个 rank（IN13）。

### P1-4 蜂鸣器引脚缺失

- **位置**：`gpio.c` 全部输出：PB0/PB1/PB4/PB5/PC0/PC1/PC8/PC9（方向×8）+ PA10(STBY) + PB12(LED1) + PC13(LED2)。
- **原因分析**：规划 bsp_gpio 明确含"蜂鸣器"（Modules/alarm 的声光报警依赖它），当前引脚分配中没有任何蜂鸣器。
- **修改建议**：CubeMX 补一个 GPIO_Output；LQFP64 上 PC4 尚空闲可优先使用。

### P1-5 SPI2 不在规划中，属多余外设

- **位置**：`spi.c` SPI2 主模式全双工，PB13/14/15，默认 18 Mbit/s。
- **原因分析**：规划全篇 OLED 走 I2C2，无任何 SPI 设备。多一个外设既违反"按规划生成"的边界，也把 PB13/14/15 从 GPIO 池里锁死。
- **修改建议**：与结构/电控组确认是否有预留器件（如高速传感器）；无则 CubeMX 里删掉 SPI2，释放三个引脚（PC4 之外又多 3 个备选，P1-4 的蜂鸣器可从这批里挑）。

---

## 四、P2 —— 小问题与建议

| # | 位置 | 问题 | 建议 |
|---|------|------|------|
| 1 | `main.h:62`、`gpio.c:73`、.ioc `PC5.GPIO_Label` | HC-05 STATE 引脚标签拼错：`HC5_ATATE` | 改为 `HC5_STATE` |
| 2 | `gpio.c:96` PA11/PA12 | 按键 EXTI 上升沿 + **NOPULL（悬浮）**，且 `EXTI15_10_IRQn` 未使能（配了 EXTI 模式却不进中断） | 若按键对地：改下降沿 + 内部上拉；不用中断则改回普通输入模式 |
| 3 | `tim.c` TIM1/TIM8 | `IC1Filter=5` 但 `IC2Filter=0`（.ioc 只勾了 IC1），与 TIM3（两路都 5）不一致 | .ioc 中把 IC2Filter 统一设为 5 |
| 4 | `tim.c` TIM5 | 50 Hz 正确，但 ARR=1000 → 20 µs/步，舵机行程分辨率约 1.8°/步，偏粗 | 建议 `PSC=72-1, ARR=20000-1`（1 µs/步），不改变 50 Hz |
| 5 | `usart.c:43` | UART4 波特率 115200 | 确认 HC-05 模块 AT 设置后的实际波特率（出厂常为 9600） |
| 6 | `freertos.c:117` | 任务名 `App_Chassis_Tas` 被 CubeMX 15 字符限制截断 | 无功能影响，接受或日后自行重命名 |
| 7 | `main.h:60` PC13=LED2 | PC13 驱动能力弱（~3 mA sink） | LED 限流电阻 ≥1 kΩ，建议接成低电平点亮 |
| 8 | 工程结构 | 目前仅有 CubeMX 的 `Hardware/F103RC`（对应规划 Hardware 层 + FreeRTOS 内核），Bsp/Modules/UserApp 尚未创建——符合 W1 第 1 步的预期状态 | 后续新建 Bsp/Modules/UserApp 时记得在 `CMakeLists.txt` 中追加这些源目录 |

---

## 五、确认无误的项（与规划逐条对上）

1. **时钟树** ✅：HSE 8 MHz × PLL9 = 72 MHz；APB1=36 MHz（定时器自动 ×2 = 72 MHz）；APB2=72 MHz；ADC=PCLK2/6 = 12 MHz（<14 MHz 上限）；FLASH_LATENCY_2。
2. **串口对调方案** ✅：HC-05 → UART4（PC10/PC11），调试 → UART5（PC12/PD2），DMA2_Channel3 RX=循环+字节+内存递增（RM0008 规划依据正确落实），初始化顺序 MX_DMA_Init 在 MX_UART4_Init 之前 ✅。
3. **编码器** ✅：TIM1(PA8/PA9)、TIM2(PA15/PB3 + `__HAL_AFIO_REMAP_TIM2_PARTIAL_1`)、TIM3(PA6/PA7)、TIM8(PC6/PC7)，全部 TI12 四倍频、ARR=65535、16 位计数（32 位扩展靠软件）。
4. **TIM5 舵机 50 Hz** ✅（分辨率见 P2-4）、TIM4 四通道 PB6~PB9 ✅、TIM5 四通道 PA0~PA3（CH4 备用已使能，与规划一致）✅。
5. **I2C2** ✅：PB10/PB11、100 kHz、AF 开漏。
6. **GPIO 基本盘** ✅：方向×8、STBY(PA10)、LED×2(PB12/PC13)、按键×2(PA11/PA12)、HC-05 STATE(PC5)（除 P1-4 蜂鸣器缺失）。
7. **FreeRTOS 框架** ✅：5 任务优先级 Remote=High / Chassis=AboveNormal / Grab=Normal / Sense=BelowNormal / Daemon=Low，与规划"高/高/中/低/低"梯度一致；HAL 时基换 TIM6、SysTick 让给内核 ✅；PendSV=15 ✅；`INCLUDE_vTaskDelayUntil=1`（周期任务需要）✅；DMA2_Ch3 与 TIM7 中断优先级 5 = `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` ✅。
8. **JTAG 禁用已配置** ✅（但执行后会被 P0-4 破坏，须按 P0-4 修复）。

---

## 六、修改优先级路线图

1. **立刻改（阻断调试与串口联调）**：P0-4 SWJ 回写补丁 → P0-1 两个串口中断 → P1-1 TIM4 20 kHz。
2. **W1 内改（bsp_adc / remote 联调前）**：P0-2 双 ADC 同步（触发按 P0-3 勘误后的方案 A/B 执行）→ P0-3 TIM7 精确 1 kHz → P1-2 堆与栈 → P1-3 采样时间。另建议把规划文档 3.2 中"TIM7 TRGO 1 kHz 触发"的表述同步修订为"TIM7 1 kHz 中断软件触发（或连续转换 + DMA）"。
3. **与结构组确认后改**：P1-4 蜂鸣器引脚、P1-5 SPI2 去留、P2-5 HC-05 波特率。
4. 其余 P2 项随下次 CubeMX 重新生成一并处理（注意：所有手写补丁必须放在 `USER CODE BEGIN/END` 区内，否则会被覆盖——尤其 P0-4 的补丁只能放 USER CODE 段）。

---

## 七、复查记录（2026-09-27，按 nRF24L01 新规划复查）

> 背景：遥控链路已由 HC-05/UART 更改为 nRF24L01/SPI2（CSN=PC12 / CE=PD2 / IRQ=PC5），UART5 释放、UART4 转调试口。原报告 P0-2/P0-3/P1-5/P2-1/P2-5 的语境随此变更部分过时，本节按新规划逐项复核重新生成的代码。

### 已修复 / 已落实 ✅

| 原编号 | 结论 |
|---|---|
| P0-1 | ✅ UART4_IRQn、DMA2_Ch3、DMA1_Ch1 均已使能（优先级 5）；UART5 已整体移除，与新规划一致 |
| P0-2 | ✅ ADC1/2 双同步（REGSIMULT）+ 连续转换 + DMA1_Ch1（Word/Word/循环/MemInc）全部落地；CubeMX 强制的 DMA1_Ch1 中断已由 main.c USER CODE 2 的 `HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn)` 屏蔽（补丁重新生成后保留） |
| P0-3 | ✅ TIM7 已整体移除（方案 B 下属可选项）；ADC 触发不再依赖任何定时器 |
| P0-4 | ✅（口径修正后关闭）宏写 0b111 属实但"必然掉线"未定论；防御补丁 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 在 tim.c USER CODE TIM2_MspInit 1 中保留 |
| P1-1 | ✅ TIM4：PSC=0、ARR=3600-1 → 72 MHz/3600 = 20 kHz，3600 级分辨率保留 |
| P1-2 | ✅ 基本修复：堆 20480 B；Chassis 1024 / Grab 768 / Remote 256 / Sense 256 words（遗留：Daemon 仍 128 words，见下） |
| P1-3 | ✅ 采样时间全部 55.5 周期；ADC3 IN12/IN13 两通道（Scan 使能、NbrOfConversion=2） |
| P1-4 | ✅ 蜂鸣器 PC4 = Alarm 输出 |
| P2-1 | ✅（过时）HC-05 移除，PC5 已改名 nRF24_IRQ |
| P2-3 | ✅ TIM1/2/3/8 的 IC1Filter/IC2Filter 统一为 5 |
| P2-5 | ✅（过时）UART4 现为调试口，115200 合理 |

### 新问题（本次复查发现）

- **N1（P1）SPI2 时钟超 nRF24L01 上限**：`spi.c` `BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2` → APB1 36 MHz/2 = **18 Mbit/s > 10 MHz**。改为 `_4`（9 MHz）或 `_8`（4.5 MHz）。
- **N2（P1）nRF24_IRQ（PC5）配了 EXTI 上升沿 + 上拉，但 EXTI9_5_IRQn 未在 NVIC 使能**，`stm32f1xx_it.c` 无对应 handler → 中断模式形同虚设，当前只能靠 remote_task 轮询。二选一：NVIC 勾 EXTI9_5（优先级 5）；或把 PC5 改回普通输入，明确走 1 ms 轮询降级方案。
- **N3（P2）按键仍是 EXTI 下降沿 + NOPULL 悬浮，且 EXTI15_10_IRQn 未使能**（原 P2-2 只修了一半）：要么 NVIC 勾 EXTI15_10，要么改普通输入 + 内部上拉。
- **N4（P2）nRF24_CSN（PC12）初始电平为 RESET（低）**：SPI 片选空闲态应为高。CubeMX 中把 PC12 的 GPIO output level 改为 High（或初始化代码里对 CSN 单独 `HAL_GPIO_WritePin(..., GPIO_PIN_SET)`）。PD2（CE）初始低正确。
- **N5（P2）Daemon 任务栈仍 128 words（512 B）**：含心跳检查/喂狗/故障码路径，建议 ≥192 words。

### 过时项说明（不再跟踪）

P1-5（SPI2 去留）：SPI2 已按新规划成为 nRF24L01 遥控的正选外设，本条作废。原报告第一~六章中所有以"HC-05 走 UART4 / UART5 调试"为前提的表述，均以 2026-09-27 资料库两份文档的勘误为准。

### 复查二（2026-09-27 14:56，针对 N1~N5）

| 编号 | 状态 |
|---|---|
| N1 SPI2 超速 | ✅ 已修：BaudRatePrescaler = `_8` → 36/8 = 4.5 Mbit/s（nRF24 上限 10 MHz 内） |
| N2 nRF24_IRQ EXTI | ✅ 已修：EXTI9_5_IRQn 使能（优先级 5），`EXTI9_5_IRQHandler` 已生成并调用 `HAL_GPIO_EXTI_IRQHandler(nRF24_IRQ_Pin)` |
| N3 按键 EXTI | ✅ 已修（复查三确认）：EXTI15_10_IRQn 已使能，KEY1/KEY2 handler 已生成，内部上拉已加 |
| N4 CSN 初始电平 | ✅ 已修：PC12 设了 PinState=SET，`HAL_GPIO_WritePin(nRF24_CSN, GPIO_PIN_SET)`，且已从 RESET 组移出 |
| N5 Daemon 栈 | ✅ 已修：128 → 768 words |

### 复查三（2026-09-27 15:36，UART4 去DMA + N3 收尾）

- **N3 关闭** ✅：`EXTI15_10_IRQHandler` 已生成（KEY1/KEY2），NVIC 已使能。
- **设计变更**：调试口 UART4 不再使用 DMA（用户决策）。核实结果：ioc 中 DMA 请求仅剩 ADC1（DMA1_Ch1），`usart.c` 无 `hdma_uart4` 残留，DMA2_Ch3 中断已移除，`MX_DMA_Init` 仍在 `MX_ADC1_Init` 之前 ✅。变更合理——调试口无协议帧，IDLE 判帧属 HC-05 遗留需求；附带收益是 DMA2 控制器整体闲置、少一个中断源。
- **对 BSP 的约束**：`bsp_log` 不得使用阻塞的 `HAL_UART_Transmit`，日志走 `HAL_UART_Transmit_IT` + 环形缓冲（115200 波特率下 CPU 开销可接受）；接收同样 IT 单字节入环形缓冲。
- 文档已同步：README、资料库《RM校内赛开发规划》《RM校内赛代码分层架构》均已加入勘误说明。

两处 USER CODE 补丁（`tim.c` NOJTAG、`main.c` DisableIRQ(DMA1_Ch1)）重新生成后仍在 ✅。**至此硬件层（CubeMX 生成部分）已无阻断项，可进入 Bsp 层开发。**
