# 《机甲夺矿》BSP 层开发规划 v1.2

> 上游文档：《RM校内赛开发规划》v1.0（2026-09-27 遥控链路已变更为 nRF24L01/SPI2）。
> 本文档基线：Hardware/F103RC CubeMX 生成代码（CMake 工具链，2026-09-27 复核通过）。
> **v1.1 修订（2026-09-27）**：BSP 只封装**片上外设**（PWM/编码器/ADC/UART/SPI/I²C/GPIO/EXTI/DWT）；
> Motor/Servo/Power/OLED/Remote 等含"器件知识"的封装全部移入 Modules（对齐 control-2026 分层）。
> **v1.2 修订（2026-09-30）**：新增第十节《Tests/ 板级测试台规范》——验收即测试项、常驻主干；
> SPI2 频率勘误（4.5MHz）；bsp_pwm API 更新（duty 0.0~1.0f、TIM5×4）。

## 一、BSP 层定位与边界

四层架构：**UserApp → Modules → Bsp → Hardware(CubeMX/HAL)**。

**BSP 的职责**：把 CubeMX 生成的片上外设句柄（`htim4`、`hadc1`、`hspi2`……）封装成与"具体器件"无关的外设级接口。判断标准只有一条——**封装的是片上外设，还是器件知识**：

- `Pwm_SetDuty(ch, val)` ✅ BSP（片上 PWM，不知道那是电机）
- `Motor_SetDuty(ch, val)` ❌ Modules（TB6612 = PWM + 方向 GPIO×8 + STBY 的组合，含器件时序知识）

**规则（必须遵守）：**

1. BSP 允许 `extern` CubeMX 句柄、直接调 HAL；**禁止** include Modules/UserApp 的任何头文件。
2. **BSP 的 API 命名与头文件不得出现器件概念**（Motor/Servo/Oled/Nrf24），只允许外设概念（Pwm/Encoder/Adc/Spi/Iic/Gpio/Log）。
3. Modules（含器件驱动 Motor/Servo/Power/Oled/Remote）只允许 include `bsp_xxx.h` 与 `robot_config.h`，**禁止**直接碰 HAL 或寄存器。
4. BSP 内**禁止** malloc / 任何动态分配 / 无超时的阻塞等待；所有缓冲静态定义。
5. 不确定数值全部进 `robot_config.h`，引脚宏全部进 `bsp_pin.h`，主逻辑零改动。
6. 命名：BSP 层 `外设_动作`（`Pwm_SetDuty`、`Encoder_Read`），Modules 层 `模块_动作`（`Motor_SetDuty`、`Servo_SetAngle`），实例 `g_` 前缀。

## 二、CubeMX 基线核对表（2026-09-27 已复核）

此前《Hardware代码审查报告》的 P0/P1/P2 问题在重新生成的代码中**已全部修复**，BSP 开发可直接开工：

| 外设 | 生成代码实测 | 结论 |
| --- | --- | --- |
| TIM4 电机 PWM | PSC=0，ARR=3599 → 72MHz/3600 = **20 kHz**，4 通道 | ✅（此前 10kHz 已修正） |
| TIM5 舵机 PWM | PSC=1439，ARR=999 → **50 Hz**，1000 步分辨率（20µs/步） | ✅ |
| 编码器 TIM1/2/3/8 | TI12 双边沿计数，IC Filter 统一 = 5 | ✅（filter 不一致已修正） |
| TIM2 重映射 | Partial Remap1（PA15/PB3），后接 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 防御 | ✅ |
| ADC1/ADC2 | 双同步 REGSIMULT + ADC1 连续转换 + DMA1_Ch1 循环（Word），DMA 中断在 main.c USER CODE 2 屏蔽 | ✅ 方案B 落地 |
| ADC3 | 扫描 IN12(PC2)/IN13(PC3)，55.5 周期采样 | ✅（IN13 缺失已修正） |
| UART4 | 115200，纯中断收发，NVIC 已使能（优先级 5），无 DMA | ✅ 符合 09-27 勘误 |
| UART5 | 未初始化（引脚让给 nRF24 CSN/CE） | ✅ |
| SPI2 | 主模式 8bit，CPOL=0/CPHA=1EDGE，软件 NSS，分频 8 → **4.5 MHz**（APB1 36MHz/8；v1.1 勘误 2026-09-29：原"9 MHz"误按 72MHz 主频计算，SPI2 挂 APB1） | ✅ |
| I²C2 | 100 kHz 标准模式（OLED） | ✅ |
| nRF24 引脚 | CSN=PC12 / CE=PD2 输出（CSN 默认拉高），IRQ=PC5 下降沿+上拉 EXTI | ✅ |
| 按键 | PA11/PA12 下降沿+上拉 EXTI，EXTI15_10 NVIC 已使能 | ✅（此前中断未使能已修正） |
| LED / 蜂鸣器 | LED1=PB12、LED2=PC13，蜂鸣器 Alarm=**PC4** | ✅（蜂鸣器缺失已补） |
| TB6612 | STBY=PA10（默认拉低），方向 GPIO×8 = PB0/PB1/PB4/PB5/PC0/PC1/PC8/PC9 | ✅ 上电安全 |
| FreeRTOS | CMSIS_V1，堆 **20480 B**，tick 1kHz，5 任务栈：Remote 256 / Chassis 1024 / Grab 768 / Sense 256 / Daemon 768（word） | ✅（堆 5120 已扩容） |
| DMA 资源 | 仅 DMA1_Ch1 服务 ADC1；DMA2 全空闲 | ✅ 与勘误一致 |

**遗留决策项（已关闭，2026-09-29 勘误）**：SPI2 实际时钟 = APB1 36MHz / 分频 8 = **4.5 MHz**（原记"9 MHz"系误按 72MHz 主频计算），远低于 nRF24L01+ 上限 10 MHz，无需调整 CubeMX 分频。另：bsp_spi 的 CSN/CE 电平封装已落地（`Spi_Csn/Spi_Ce`，内部委托 bsp_gpio），CSN/CE 时序知识仍归 Modules/remote。

## 三、目录结构与工程接入

分层目录放在**仓库根**、与 `Hardware/` 平级（对齐 control-2026 结构）：

```
RM_Begin/                     ← 仓库根（git 仓库根目录）
├── Bsp/                      # 片上外设封装层（bsp_xxx.c/.h 平铺）
│   ├── bsp_pin.h             # 全部引脚宏（与 main.h 对齐，唯一硬件地图）
│   ├── bsp_sys.c/.h          # DWT 微秒延时、毫秒时间戳、时钟自检
│   ├── bsp_log.c/.h          # UART4 IT 环形缓冲日志 + printf 重定向
│   ├── bsp_gpio.c/.h         # GPIO 读写 + EXTI 注册（LED/蜂鸣器/按键/nRF24 CSN·CE 底层）
│   ├── bsp_pwm.c/.h          # TIM4×4 20kHz duty / TIM5×4 50Hz 脉宽，统一接口
│   ├── bsp_encoder.c/.h      # TIM1/2/3/8 编码器
│   ├── bsp_adc.c/.h          # ADC1/2 DMA 循环缓冲 + ADC3 扫描轮询
│   ├── bsp_spi.c/.h          # SPI2 全双工字节收发
│   └── bsp_iic.c/.h          # I²C2 阻塞读写（带超时）
├── Modules/                  # 器件驱动 + 功能模块层（与 Bsp 并行开发）
│   ├── motor.c/.h            # TB6612：PWM×4 + DIR×8 + STBY 组合
│   ├── servo.c/.h            # 脉宽→角度映射、软限位、卸力
│   ├── power.c/.h            # ADC 原始值→电压/电流/功率换算（窗口均值）
│   ├── oled.c/.h             # SSD1306 器件驱动 + 排版刷新
│   └── remote/               # nRF24L01 寄存器驱动 + 链路协议（W1 关键路径）
├── UserApp/                  # 应用任务层（remote_task.c 已随 W1 落地，其余二期）
├── Tests/                    # 板级测试台（验收即测试项，常驻主干；规范见第十节）
└── Hardware/
    └── F103RC/               # CubeMX 工程 = CMake 工程根（Core/Drivers/Middlewares 不动）
```

与 control-2026 的差异说明：学长仓库 Bsp 内部按外设分子目录（`can/ log/ spi/`…）；本规划模块少、先用平铺，CMake 已用 `GLOB_RECURSE` 收集，日后模块变多可平滑升级为子目录，不用再改构建脚本。

**CMake 接入（已落地并验证）**：仓库根放总入口 `CMakeLists.txt`（对齐 control-2026，CLion 直接打开 RM_Begin 即可加载全部 target）；CubeMX 工程作为子目录引入。已做的三处路径适配：

```cmake
# —— RM_Begin/CMakeLists.txt（总入口，新建，CubeMX 永不覆盖）——
cmake_minimum_required(VERSION 3.22)
project(RM_Begin C ASM)
add_subdirectory(Hardware/F103RC)

# —— Hardware/F103RC/CMakeLists.txt 用户区：用 CMAKE_CURRENT_LIST_DIR 定位分层目录
#    （原 CMAKE_SOURCE_DIR 在根入口下会指向仓库根，../../Bsp 会指到仓库外）——
file(GLOB_RECURSE BSP_SOURCES    CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../Bsp/*.c")
file(GLOB_RECURSE MODULE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../Modules/*.c")
file(GLOB_RECURSE USERAPP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../UserApp/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${BSP_SOURCES} ${MODULE_SOURCES} ${USERAPP_SOURCES}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../Bsp"
    "${CMAKE_CURRENT_LIST_DIR}/../../Modules"
    "${CMAKE_CURRENT_LIST_DIR}/../../UserApp"
)

# —— Hardware/F103RC/cmake/gcc-arm-none-eabi.cmake：链接脚本同样改用文件自身定位 ——
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_CURRENT_LIST_DIR}/../STM32F103xx_FLASH.ld\"")
```

`CMakePresets.json` 已移到仓库根（`toolchainFile` 指向 `Hardware/F103RC/cmake/gcc-arm-none-eabi.cmake`，构建目录 `build/Debug`）。**验证结果（2026-09-27）**：`cmake --preset Debug` + 全量编译通过，`F103RC.elf` 链接成功（RAM 48.8%，FLASH 10%），Bsp 源文件被正确编入 target。

## 四、模块规格（API 草案）

**BSP 层（片上外设，9 个）：**

| 模块 | 封装资源 | 主要 API | 服务对象 |
| --- | --- | --- | --- |
| bsp_sys | DWT、时钟 | `Bsp_Init` / `Bsp_GetUs` / `Bsp_GetMs` / `Bsp_DelayUs` | 全局 |
| bsp_log | UART4 IT + 环形缓冲 | `Log_Init` / `Log_Printf(fmt,...)` / `Log_Poll` | 调试全局 |
| bsp_gpio | GPIO 读写 + EXTI 注册 | `Gpio_Write(pin,on)` / `Gpio_Read(pin)` / `Exti_Attach(line,cb)` | Modules 各器件 |
| bsp_pwm | TIM4 CH1-4（20kHz）、TIM5 CH1-4（50Hz） | `Pwm_SetDuty(ch, 0.0f..1.0f)`（<0 钳 0，按各通道所属 TIM 的 ARR 查表换算）/ `Pwm_SetPulseUs(ch,us)`（仅 50Hz 组，BSP 钳 0..20000us）/ `Pwm_Release(ch)` | Modules: motor/servo |
| bsp_encoder | TIM1/2/3/8 | `Encoder_InitAll` / `Encoder_Read(ch)→int32 增量`（内部累计 16bit 溢出） | Modules: chassis |
| bsp_adc | ADC1/2 DMA 循环 + ADC3 扫描 | `Adc_GetLatest(&vi)`（V/I 成对）/ `Adc3_Read(ch)` | Modules: power |
| bsp_spi | SPI2 全双工 | `Spi_Transfer(tx,rx,len)` / `Spi_Csn(on)` / `Spi_Ce(on)` | Modules: remote |
| bsp_iic | I²C2 | `Iic_Write(buf,len)` / `Iic_Read(addr,buf,len)`（均带超时） | Modules: oled |

**Modules 首批器件驱动（5 个，与 BSP 并行开发）：**

| 模块 | 器件/功能 | 主要 API | 底层依赖 |
| --- | --- | --- | --- |
| Modules/motor | TB6612 ×4 | `Motor_SetDuty(ch,-1000..+1000)`（符号即方向；内部写方向 GPIO 后以 |duty|/1000.0f 调 `Pwm_SetDuty`）/ `Motor_Enable/Disable`（STBY） | bsp_pwm + bsp_gpio |
| Modules/servo | 舵机 ×3 | `Servo_SetAngle(id,deg)`（软限位）/ `Servo_Release(id)`（停脉冲=卸力） | bsp_pwm |
| Modules/power | 功率采样 | `Power_GetVoltage/Current/Power`（窗口均值）/ `Joint_GetCurrent(k)` | bsp_adc |
| Modules/oled | SSD1306 | `Oled_Init` / `Oled_Printf(x,y,...)` / `Oled_Refresh` | bsp_iic |
| Modules/remote | nRF24L01 + 链路 | `Nrf24_Init(mode)`（PTX/PRX）/ `Nrf24_ReadRx(buf15)` + 300~500ms 看门狗 | bsp_spi + bsp_gpio + EXTI |

**实现要点（验收关键）：**

1. **bsp_pwm**：纯片上接口，不知道"电机/舵机"为何物；TIM5 每 step=20µs；Init 后默认 duty=0 / 0 脉宽（不发脉冲=卸力；上电安全链的片上部分，不依赖上层调用顺序；舵机中位脉冲由 Modules/servo 的 Init 发，500~2500µs 量程是舵机知识不进 BSP）。v1.2 勘误（2026-09-30）：SetDuty 契约由 -1000..+1000 改为 float 0.0..1.0（负值钳 0 失安全；原整数档在 ARR=3599 下每档 3.6 CCR 有量化损耗）；TIM5 CH4 已确认启用，由 ×3 改 ×4。
2. **bsp_gpio / EXTI**：LED、蜂鸣器、按键、CSN/CE 都是普通 GPIO；`Exti_Attach(line, cb)` 注册的回调运行在 ISR 上下文，只允许置标志 / `osThreadNotify`。
3. **bsp_encoder**：16 位计数器读数差分处理回绕（`(int16_t)(now - last)`）；换算系数（线数×4、减速比）放 `robot_config.h`，BSP 只出原始增量。
4. **bsp_adc**：双同步模式 DMA 一次写 32 位（低 16 位=ADC1 电流，高 16 位=ADC2 电压）；DMA 启动后**丢弃前 2 个窗口**的首批样本；窗口均值样本数放 `robot_config.h`。
5. **bsp_log**：TX 用 `HAL_UART_Transmit_IT` 排队；RX 每字节中断搬进 256B 环形缓冲，`Log_Poll` 由低优先级任务每 ≥10ms 取；`fputc` 重定向使 `printf` 可用。
6. **bsp_spi / bsp_iic**：全部带超时；CSN/CE 建立时间 ≥5µs 的时序由 Modules/remote 用 `Bsp_DelayUs` 控制；I²C 连续失败触发总线恢复（DeInit→Init）。
7. **Modules/motor**：`SetDuty` 内先写方向 GPIO 再写 PWM（同边沿切换防共导通）；`Motor_Disable` 必须同时 duty=0 + STBY 拉低；上电默认 Disable（CubeMX 已保证 STBY 复位电平低）。
8. **Modules/servo**：50Hz/1000 步下 500~2500µs = 0~180°；`Init` 先发中位脉冲 300ms 再使能（防上电猛冲）；`Release` 停发 PWM 卸力（规划五.2 机制的硬件基础）。
9. **Modules/remote**：nRF24 寄存器级完整实现（R/W_REGISTER、FLUSH_TX/RX、RF_SETUP、重传参数）；IRQ 经 `Exti_Attach` 回调里只清标志 + `osThreadNotify`，取包/解析在 remote_task；看门狗 300~500ms 在本层；**PTX/PRX 双模式**，遥控器端直接复用。
10. **Modules/oled**：I²C 阻塞发送即可（调用方是 BelowNormal 的 sense_task），但必须带超时防 I²C 挂死。

## 五、中断与优先级分配

| 中断源 | 优先级 | 做什么 | 约束 |
| --- | --- | --- | --- |
| TIM6（HAL 时基） | 15（最低） | HAL_IncTick | 无 RTOS 调用 |
| EXTI9_5（nRF24 IRQ，PC5） | 5 | 清标志 + `osSemaphoreRelease`/`osThreadNotify` | 必须 ≥5（需进 RTOS API）；≤3µs 返回 |
| EXTI15_10（KEY1/2，PA11/12） | 5 | 置按键事件标志 | 不做消抖（消抖在任务侧） |
| UART4 | 5 | TX 队列推进 / RX 单字节入环形缓冲 | 严禁 printf；每字节 86.8µs 到达，ISR 必须 <10µs |
| DMA1_Ch1（ADC） | 已屏蔽 | — | 循环模式由硬件搬运，无中断 |

规则：任何需要调用 RTOS API 的中断，优先级数值必须 ≥5（F1 只有 4 位抢占优先级，CubeMX 已按此配置）；中断里禁止 printf / malloc / 阻塞 SPI 等待。

## 六、开发顺序（对齐里程碑 W1/W2）

| 阶段 | 时间 | 内容 | 验收标准 |
| --- | --- | --- | --- |
| S0 | 9/27~9/28 | Bsp/ 目录 + CMake 接入 + bsp_pin.h + bsp_sys + bsp_log | 交叉编译通过，printf 走 UART4 输出正常（已开工） |
| S1 | 9/29 | bsp_gpio（含 EXTI 注册） | 按键点亮 LED、蜂鸣器提示音 |
| S2 | 9/29~10/2 | **bsp_spi + Modules/remote（W1 关键路径）** | 车端 PRX 稳定收 15B 包、收包率 >95%、PTX 端遥控器整机联调 |
| S3 | 10/3 | bsp_encoder ×4 | 手转四轮：计数连续、换向符号正确、无跳变 |
| S4 | 10/4 | bsp_pwm（TIM4/TIM5） | 20kHz duty / 50Hz 脉宽输出经示波器验证 |
| S5 | 10/5 | Modules motor + servo | 四路开环正反转；舵机上电中位、0~180° 扫描、卸力/恢复 |
| S6 | 10/6 | bsp_adc + Modules/power | 串口打印 V/I，与万用表误差 <5%，双 ADC 同步性验证 |
| S7 | 10/7 | bsp_iic + Modules/oled | OLED 显示电压/电流/任务心跳 |

W2 剩余时间（10/8~10/9）做 BSP+Modules 回归，交给 Modules 功能层（速度环、运动学）。

**提交规范**（规划四.6）：每个 S 阶段 ≥1 次功能小提交，作者真名，commit message 注明模块与验收结果——这是自主设计的唯一证明。

## 七、坑清单（BSP 实施视角，按踩中概率排序）

1. **SPI2 时钟**：实际 4.5 MHz（APB1 36MHz/分频 8；v1.1 勘误 2026-09-29，原"9 MHz"算错总线），nRF24L01+ 手册上限 10 MHz，余量充足；若读寄存器仍偶发错误，CubeMX 改分频 16 → 2.25 MHz（不手改生成代码）。
2. **UART4 无 DMA**：115200 下每 86.8µs 一次 RX 中断，ISR 只搬一个字节；`Log_Poll` 必须及时取，否则环形缓冲溢出丢日志（调试口可容忍）。
3. **双 ADC 数据拆分**：DMA 缓冲是 32 位（ADC1|ADC2<<16），取错位功率就错；先验证已知分压读数。
4. **ADC 首批样本无效**：DMA 循环启动后前几拍是旧值，`bsp_adc` 内部丢弃前 2 个窗口。
5. **上电安全链**：STBY 复位电平低（CubeMX 已保证）+ `bsp_pwm` Init 后输出 duty=0 / 0 脉宽（不发脉冲=舵机卸力）+ Modules 使能时序（motor/servo Init 里强制）——不依赖上层调用顺序。
6. **PC13 LED 驱动仅 3mA**：限流电阻已由硬件保证，软件不得把 PC13 用作他用。
7. **新加 remap 必补 SWJ 防御**：任何 F1 remap 组合之后补 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`（现有 TIM2 已带）。
8. **编码器电平**：引脚配置为 NOPULL，若编码器是 5V 开漏输出，确认板上外部上拉存在（规划坑 #14）。
9. **EXTI 共享中断线**：PC5 与 PA11/12 分属 EXTI9_5 / EXTI15_10 两条线，`HAL_GPIO_EXTI_IRQHandler` 自动判引脚，勿在回调里混判断逻辑。
10. **I²C 挂死**：OLED 排线接触不良会把 I²C2 拉死，`bsp_iic` 必须带超时与总线恢复，不许无限等。

## 八、资源预算（48KB SRAM）

| 项 | 占用 | 说明 |
| --- | --- | --- |
| FreeRTOS 堆 | 20 KB | 已含 5 任务栈约 9.5 KB（word 计） |
| ADC DMA 缓冲 | 1 KB | 256 对样本 ×4B（uint32） |
| 日志环形缓冲 | 0.5 KB | 256B RX + TX 队列 |
| OLED 帧缓冲 | 1 KB | 128×64/8 |
| 全局结构 + 栈余量 | ~2 KB | RC_Cmd、Actuator 实例等 |
| **剩余** | **≥22 KB** | 充足，但新增大缓冲仍需先登记 |

Flash：HAL + FreeRTOS 约 30KB 起，RCT6 256KB 充足，无需关注。

## 九、与上层的交接约定

- BSP 的外设 API 完成即冻结：Modules 开发期间 BSP 只修 bug 不改签名；Modules 器件驱动（motor/servo/oled/remote/power）同样对 UserApp 冻结。
- `robot_config.h` 由 BSP 骨架先建好分组（Motor/Servo/Remote/Power/Link），占位默认值注释"待硬件实测"。
- 遥控器端（PTX）与车端共用 `Modules/remote` + `bsp_spi/gpio/log`，工程上通过 `robot_config.h` 的编译开关区分两端（如 `#define CONFIG_REMOTE_UNIT`）。

## 十、Tests/ 板级测试台规范（2026-09-30 落地）

### 10.1 定位与动机

`Tests/` 是**常驻主干的板级测试台**，取代两种旧做法：验收代码"验完即删"（资产消失，S1/S3 的验收程序因此失传）和"分支归档"（代码烂在分支里，API 演进后编译不过）。核心原则：**验收即测试项，永远参与编译**（正常运行仅多几 KB flash），新板 bring-up 或硬件排障时逐项重跑。

⚠ 与业务任务的边界：`Tests/` 是**给人用的诊断**（手动选项目、带详细打印）；`UserApp/` 是**给机器人用的业务**（静默常驻、喂控制逻辑）。同一外设允许两边各有一个消费者（如 `test_remote.c` 与 `remote_task.c`），但业务逻辑禁止写进 Tests/，诊断打印禁止带进 UserApp。测试台激活时业务任务必须调 `TestBench_Yield("任务名")` 让位（共享外设互斥）。

### 10.2 架构三件套

| 件 | 位置 | 职责 |
| --- | --- | --- |
| 选择宏 | `Bsp/robot_config.h` → `ROBOT_TEST_BENCH` | 跑哪一项（0=关闭，业务固件常态，测试任务退化为心跳+栈高水位） |
| 调度器 | `Tests/test_bench.c`（强覆盖 CubeMX 的 `StartApp_TestBench_Task`，Normal/512w） | Init 一次 → 10ms 轮询 Poll → 2s 心跳（LED1 1Hz + `uxTaskGetStackHighWaterMark` 自证栈余量） |
| 登记表 | `Tests/test_bench.c` 的 `s_table[]` | `{枚举, 名称, Init, Poll}` 四元组；枚举在 `test_bench.h` |

`TestBench_Yield(name)`：测试台激活时打印提示并永久挂起调用任务（业务任务让位的唯一通道）；未激活时立即返回无副作用。CubeMX 侧依赖：`App_TestBench_T` 任务（512 word）+ `INCLUDE_uxTaskGetStackHighWaterMark=1`。

### 10.3 新增测试项三步流程

1. 写 `Tests/test_xxx.c`：实现 `void Test_xxx_Init(void)`（一次性，失败置内部 `s_ok=false`）与 `void Test_xxx_Poll(void)`（**非阻塞单次迭代**，慢节奏统计自管节拍，禁止长循环阻塞调度）；
2. `test_bench.c`：声明 + `s_table[]` 追加一行；
3. `robot_config.h`：`ROBOT_TEST_BENCH` 改成对应编号，注释里的选择表同步补一行。

**编号规则（硬性）：只增不改不插**。编号是外部配置值（robot_config.h 里手填的），重排/插入会使既有配置悄悄改变语义——新项一律追加到枚举尾部。当前表：0=NONE 1=GPIO(S1) 2=ENCODER(S3) 3=SPI(S2) 4=PWM(S4) 5=REMOTE(S2 链路)。

### 10.4 验收代码迁移模式（以 remote 为范例）

每个 S 阶段的临时验收代码，验收通过后不是删除，而是**一拆二**：

- **业务路径** → `UserApp/`（正式任务：静默、1Hz 统计、给二期留数据接缝，如 `remote_task.c`）；
- **诊断路径** → `Tests/`（A1/A2/A3 体检、逐包打印、寄存器 dump，如 `test_remote.c`）。

既有范例：`Tests/test_gpio.c`（S1）/`test_encoder.c`（S3，按 commit message 的验收语义重写）/`test_spi.c`/`test_remote.c`（S2，从 remote_acceptance.c 迁入）。业务与诊断各留一份相似骨架是刻意的：两者会朝不同方向演化（诊断加深度、业务加逻辑），抽公共骨架反而耦合。

### 10.5 编写要点（踩坑沉淀）

- **判据写"总线存活"而非"出厂状态"**：模块 VCC 不随 MCU 复位掉电，复位后读到残留状态（如 nRF24 RX_DR=0x40）不是故障；只有全 0x00/0xFF 才是总线死。参考 `test_remote.c` 的 A1 三分支打日志；
- 共享外设的测试项与业务项互斥靠 `TestBench_Yield`，勿自行加锁；
- 测试打印走 `Log_Printf`（UART4 队列满丢弃不阻塞），高频逐包打印仅限排障期，常态用 1s 统计行；
- 新板 bring-up 顺序即枚举顺序：NONE 心跳 → GPIO → ENCODER → SPI → REMOTE → 后续 PWM/ADC/I2C。

编制日期：2026-09-27 | 基线：Hardware/F103RC 已复核代码 | 上游：《RM校内赛开发规划》v1.0
第十节：2026-09-30 | 依据：S2 双板联调实践（remote 验收转正为首个迁移范例）
