# ADC 方案 B 配置指南（双同步 + 连续转换 + DMA1_Ch1 循环）

- 适用工程：`Hardware/F103RC`（STM32F103RCT6 / FW_F1 V1.8.7 / CubeMX 6.18.1）
- 定稿日期：2026-09-26（替代原规划"TIM7 TRGO 1 kHz 硬件触发"方案，原因见 §1）
- 对应模块：Bsp/bsp_adc（待建）→ UserApp/sense_task

> **⚠ 2026-10-06 勘误（S6 板上实测，以下三处以本勘误为准，正文保留作历史）**：
> ① §2.1 "DMA Continuous Requests=Enabled" 与 §3 对应自检行是 **F4/L4 系概念——F103 的 ADC CR2 无 DDS 位**，.ioc 无此项不是漏配，循环 DMA 在 F1 持续搬运；
> ② §4.1 启动函数应为 **`HAL_ADCEx_MultiModeStart_DMA`**（`HAL_ADC_Start_DMA` 在多模式下直接返回 HAL_ERROR，stm32f1xx_hal_adc.c 多模式分支）；
> ③ **从机 ADC2 须补 `CR2.EXTTRIG`（HAL 缺口，CubeMX 填不了）+ `ContinuousConvMode=ENABLE`（CubeMX 可配，原工程漏配，已改 .ioc 源头）**——实测症状两阶段："永不触发（高半字恒 2000）"→"触发一拍即停（冻在单次采样值）"，详见 BSP 坑 #11/#12；另：ADC 基准=VDDA，供电形态变更必须复标（BSP 坑 #13）。

---

## 1. 方案定位与勘误结论

| 项 | 结论 |
|---|---|
| 原规划 | ADC1_IN4 + ADC2_IN5 双同步，TIM7 TRGO 1 kHz 触发 |
| 勘误原因 | F103 的 ADC1/2 规则组触发选择器（EXTSEL）**没有 TIM7 TRGO 档**，可选触发源仅 T1_CC1/CC2/CC3、T2_CC2、T3_TRGO、T4_CC4、EXTI11（重映射=TIM8_TRGO）、软件启动——且已被编码器/电机 PWM/按键全部占用 |
| **定稿方案 B** | **ADC1/2 双同步规则组 + 连续转换（Continuous）+ DMA1_Ch1 循环搬运，无硬件触发、无 ADC/DMA 中断；sense_task 随时读最新值，按固定样本数窗口平均** |
| TIM7 新定位 | 保留为 1 kHz 通用节拍定时器（可选，供周期统计/低速任务用），不再承担 ADC 触发；其 1 kHz 中断可按需保留或关闭 |
| 代价与补偿 | 放弃固定 1 kHz 采样节拍 → 用"持续转换 + 窗口平均"达到同等功率计量精度；实际采样率（数十 kHz）远高于 1 kHz，均值统计量更平滑 |

---

## 2. CubeMX 配置步骤（在现有 F103RC.ioc 上修改）

### 2.1 ADC1（主机）
| 配置项 | 当前值 | 改为 |
|---|---|---|
| Mode | Independent mode | **Dual Regular Simultaneous Only**（选后 ADC2 自动变为从机） |
| Scan Conversion Mode | Disabled | Disabled（保持，每 ADC 单通道） |
| Continuous Conversion Mode | Disabled | **Enabled**（方案 B 核心） |
| Discontinuous Conversion Mode | Disabled | Disabled |
| External Trigger Conversion Source | Software Start | Software Start（保持） |
| DMA Continuous Requests | Disabled | **Enabled**（关键！否则 DMA 循环一轮后自动停止） |
| End Of Conversion Selection | EOC at the end of ... | 保持默认即可 |
| Rank1 Channel | ADC1_IN4（PA4） | 保持 |
| Rank1 Sampling Time | 1.5 Cycles | **55.5 Cycles**（见 §5 源阻抗说明） |

### 2.2 ADC2（从机）
| 配置项 | 改为 |
|---|---|
| Rank1 Channel | ADC2_IN5（PA5），保持 |
| Rank1 Sampling Time | **必须同为 55.5 Cycles**（双同步要求主从同 rank 同采样周期，否则同步错位） |

### 2.3 DMA（在 ADC1 的 DMA Settings 页添加）
| 配置项 | 值 |
|---|---|
| Stream/Channel | **DMA1 Channel 1**（F1 上 ADC1 的 DMA 固定挂 DMA1_Ch1，没有别的选择） |
| Direction | Peripheral To Memory |
| Mode | **Circular** |
| Peripheral Data Width / Increment | Word / 不递增 |
| Memory Data Width | **Word**（双同步下 ADC1 的 DR 是 32 位联合寄存器，低 16 位=ADC1、高 16 位=ADC2，一次 Word 搬运 = 一对同步样本） |
| Memory Increment | **Enable**（配合循环模式在缓冲区里环形覆盖） |
| NVIC（DMA1_Channel1 中断） | **不勾**（方案 B 零中断；数据由 DMA 持续刷新到内存，任务直接读。若以后想用半满/全满做"新样本就绪"提示再开，优先级必须 ≥5） |

### 2.4 ADC3（独立慢速，不在双同步组内）
- IN12（PC2）/ IN13（PC3）：**把 IN13 加为 Rank2**（当前只配了 IN12，规划要求两路）；
- 采样时间同样改 ≥55.5 Cycles；
- Continuous：Disabled，软件触发轮询即可（sense_task 里 20~50 ms 一次）。

### 2.5 重新生成后检查 main.c
生成顺序应为 `MX_DMA_Init()` → `MX_ADC1_Init()` → `MX_ADC2_Init()` → `MX_ADC3_Init()`（DMA 时钟先于 ADC 使能，CubeMX 已保证）。

---

## 3. 生成代码的关键差异（对照当前 adc.c）

重新生成后 `adc.c` 应出现/确认以下内容（这是自检清单）：

```c
/* ADC1 —— 主机，方案 B 的三个标志位缺一不可 */
hadc1.Init.ContinuousConvMode      = ENABLE;    /* 连续转换 */
hadc1.Init.DMAContinuousRequests   = ENABLE;    /* DMA 循环请求 */
hadc1.Init.NbrOfConversion         = 1;
/* sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;  主从两侧一致 */

/* 双同步模式由 HAL_ADCEx_MultiModeConfigChannel 配置（CubeMX 自动生成）： */
if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) { ... }
/* multimode.Mode = ADC_DUALMODE_REGSIMULT; */

/* DMA：DMA1_Channel1，Word 宽度，Circular */
hdma_adc1.Init.Mode = DMA_CIRCULAR;
hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
```

ADC2 段不应再出现 DMA/NVIC 配置（从机数据走联合 DR，不独立申请 DMA）。

---

## 4. bsp_adc 模块接口与 sense_task 集成

### 4.1 缓冲与启动（bsp_adc.c）

```c
#include "adc.h"

/* 环形样本缓冲：N 对同步样本，DMA 循环覆盖，最新值永远在 buffer 里 */
#define ADC_PAIR_BUF_N   16u                 /* 16 对 × 5.7us ≈ 91us 的样本窗，够均值滤波 */
static uint32_t adc_pair_buf[ADC_PAIR_BUF_N];

void Bsp_Adc_Start(void)                     /* BSPInit 里、所有 MX_xxx_Init 之后调用 */
{
    /* F1 上电后必须校准（每个 ADC 一次），否则精度明显偏差 */
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    /* Length 语义：DMA 传输"项数"。Word 宽度下 1 项 = 1 对同步样本（32 位联合 DR）。
       烧录后用调试器确认 DMA CNDTR 在 1..N 间循环、缓冲持续刷新（见 §6 验证清单） */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_pair_buf, ADC_PAIR_BUF_N);
}
```

### 4.2 对上接口（bsp_adc.h，Bsp 层禁词自查：不出现"功率/矿"等业务概念）

```c
typedef struct { uint16_t raw_i, raw_v; } AdcPairRaw;   /* 双同步原始对 */

bool AdcPair_GetLatest(AdcPairRaw *out);     /* 取最新一对：关中断内 memcpy 两个 half-word */
bool AdcPair_GetAvg(AdcPairRaw *out, uint8_t n); /* 取最近 n 对的均值（n ≤ 缓冲深度） */
```

> 读最新值的并发说明：`uint32_t` 在 Cortex-M3 上天然原子，直接读一对样本无撕裂风险；
> 若取"最近 n 对均值"，期间 DMA 会持续覆盖环形缓冲，严格做法是临界区内拷出快照再算均值，
> 简化做法是容忍个别样本新旧混合（对均值结果影响可忽略，注明即可）。

### 4.3 sense_task 集成（UserApp/sense，业务层）

```c
/* robot_config.h —— 功率标定系数全部集中于此（硬件 TBD 一改即生效） */
#define PWR_V_DIV_RATIO      11.0f   /* 分压比 TBD */
#define PWR_I_GAIN_V_PER_A   0.066f  /* 运放增益/采样电阻 合成灵敏度 TBD */
#define PWR_VREF             3.3f

/* sense_task 周期 20~50ms（vTaskDelayUntil） */
void Sense_Task(void const *arg)
{
    AdcPairRaw avg;
    for (;;)
    {
        osDelayUntil(&xLastWakeTime, SENSE_PERIOD_MS);   /* 20~50ms */

        if (AdcPair_GetAvg(&avg, 8))                     /* 8 对样本窗口平均 */
        {
            float v_bat = (avg.raw_v * PWR_VREF / 4095.0f) * PWR_V_DIV_RATIO;
            float i_bus = (avg.raw_i * PWR_VREF / 4095.0f) / PWR_I_GAIN_V_PER_A;
            float pwr_w = v_bat * i_bus;

            OLED_Printf(...);   /* 电压/电流/功率/电量四项 → OLED（检录项） */
        }
        Adc3_PollSlow();      /* ADC3 IN12/13 软件触发轮询，带超时 */
    }
}
```

---

## 5. 采样时间与负载预算

| 项 | 数值 |
|---|---|
| ADC 时钟 | PCLK2/6 = 12 MHz（当前配置，上限 14 MHz，合规） |
| 单对转换时间 | 55.5 + 12.5 = 68 周期 ≈ **5.7 µs/对** |
| 连续模式样本率 | ≈ 176 k对/s → DMA 带宽 ≈ 700 KB/s（32 位 Word，可接受） |
| 若改 239.5 Cycles | ≈ 65 k对/s → ≈ 260 KB/s，更省 DMA 带宽；源阻抗容限也更大 |
| **为什么必须加长采样** | 功率/电压来自运放输出/分压网络（规划 TBD），1.5 周期要求源阻抗 ≲200 Ω，分压网络通常 kΩ 级，**1.5 周期采样会明显失真，直接威胁 OLED ±10% 检录项**。55.5 周期对应源阻抗约 2.4 kΩ，239.5 周期约 10 kΩ |
| CPU 开销 | **零中断**，sense_task 每 20~50 ms 读一次内存，开销可忽略 |

---

## 6. 验证清单（bsp_adc 联调时逐项打勾）

1. **DMA 循环确认**：调试器 Watch `adc_pair_buf` + DMA1 CNDTR，确认 CNDTR 在 1..N 循环递减、缓冲数值持续变化（连续转换已启动的标志）；
2. **双同步确认**：给 IN4/IN5 接同一已知直流源，读出的两路 raw 应同步变化；快速扰动一路时另一路不受拖累；
3. **校准确认**：确认 `HAL_ADCEx_Calibration_Start` 在 `HAL_ADC_Start_DMA` 之前执行（顺序错了白校准）；
4. **长度语义确认**：若发现缓冲相邻两 Word 分别只含 ADC1/ADC2 数据（交错），说明 Length 按 half-word 计数了——把 Length 加倍或改用 `uint16_t` 缓冲+2N 长度即可，联调时记录实际行为；
5. **窗口均值收敛**：示波器看 OLED 功率读数，接固定负载，波动应 <±2%（均值窗口可调 n）；
6. **标定流程**：用万用表实测电池电压/母线电流，回填 `robot_config.h` 三个宏，OLED 与实测偏差 ≤±10%（检录线）；
7. **栈普查**：把 DMA 缓冲（静态分配，不占 FreeRTOS 堆）与 sense_task 高水位一起记入 W1 调试日志。

---

## 7. 与规划文档的一致性

- 规划文档 3.2 bsp_adc 行已勘误（"TIM7 TRGO 1 kHz 触发"已加删除线，改为方案 B 表述）；
- 5.2 数据流中"sense_task: bsp_adc 双同步采样 → 功率换算 → OLED"表述与方案 B 兼容，未改动；
- TIM7 相关条目（目录结构注释、TIM7 1 kHz）保留：TIM7 作为可选节拍定时器仍然成立，只是不再触发 ADC。
