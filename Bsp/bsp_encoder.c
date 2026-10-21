/**
 * @file    bsp_encoder.c
 * @brief   四路正交编码器实现：静态句柄表 + 16bit 回绕差分
 *          + 边沿时间戳采样环（M/T 测速，mt-speed-pending S-B）
 */
#include "bsp_encoder.h"
#include "bsp_pin.h"
#include "bsp_gpio.h"
#include "bsp_sys.h"
#include "tim.h"

/* HAL 句柄表（extern 自 tim.h）。
 * ⚠ S0 教训：TIM1/TIM2/... 实例宏是寄存器块指针（USART_TypeDef* 同理），
 * 严禁当句柄用；这里一律 &htimx。 */
static TIM_HandleTypeDef *const s_enc_tim[4] = { &htim1, &htim2, &htim3, &htim8 };

/* 各通道上次计数快照，仅 Encoder_Read（单一读者）访问 */
static uint16_t s_enc_last[4] = { 0u, 0u, 0u, 0u };

/* GetCount 的独立快照与累计：与 Encoder_Read 互不干扰（各消费各的 CNT 差分） */
static uint16_t s_pos_last[4] = { 0u, 0u, 0u, 0u };
static int32_t  s_pos_acc[4]  = { 0, 0, 0, 0 };

/* ==================== 边沿时间戳采样环（M/T 测速） ==================== */

/* 每通道一组：中断只写 ring+head，TakeNew 只读 ring、读写 take_idx。
 * 并发论证（免临界区）：
 *  - head 是单调递增的写序号，读端先拍快照再拷贝，只会看到"已落盘"的样本；
 *  - 正常路径（没掉队）中断只会写 head 快照之后的槽位，与拷贝区间不重叠；
 *  - 掉队批（dropped=true）理论上有与新边沿撕裂的可能，但该批已被声明
 *    不连续、消费方复位即弃——见 bsp_encoder.h 的 TakeNew 契约；
 *  - 全程 32bit 对齐读写，Cortex-M 单核无重排可见性问题。 */
static Encoder_Sample_t s_ring[4][ENC_RING_DEPTH];
static volatile uint16_t s_ring_head[4] = { 0u, 0u, 0u, 0u };   /* 单调写序号 */
static uint16_t s_take_idx[4] = { 0u, 0u, 0u, 0u };             /* 消费游标（仅任务侧） */

/* 边沿侧的回绕展开账本（仅中断上下文访问） */
static uint16_t s_edge_last[4] = { 0u, 0u, 0u, 0u };
static int32_t  s_edge_acc[4]  = { 0, 0, 0, 0 };

/* 边沿捕获：读 CNT → 回绕展开 → 连同时间戳落环。只有整数运算、无浮点
 * 无日志（ExtiCallback 契约：ISR 上下文，禁阻塞禁 printf） */
static void EdgeCapture(uint32_t idx)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_tim[idx]);
    int16_t diff = (int16_t)(now - s_edge_last[idx]);
    s_edge_last[idx] = now;
    s_edge_acc[idx] += diff;

    uint16_t h = s_ring_head[idx];
    Encoder_Sample_t *slot = &s_ring[idx][h & (ENC_RING_DEPTH - 1u)];
    slot->cnt = s_edge_acc[idx];
    slot->us = Bsp_GetUs();
    s_ring_head[idx] = (uint16_t)(h + 1u);  /* 最后才推进写序号：head 之前的必已落盘 */
}

/* 一线一回调（bsp_gpio 契约）：四个薄壳各自绑死通道号 */
static void EdgeCaptureCh1(void) { EdgeCapture(0u); }
static void EdgeCaptureCh2(void) { EdgeCapture(1u); }
static void EdgeCaptureCh3(void) { EdgeCapture(2u); }
static void EdgeCaptureCh4(void) { EdgeCapture(3u); }

/* 把 EXTI 线映射到端口：等价 HAL_GPIO_Init 里的 EXTICR 一手，但绝不碰
 * MODER——HAL 的 EXTI 模式会把引脚改回输入态，编码器复用计数当场断 */
static void EdgeExtiMap(uint16_t pin, GPIO_TypeDef *port)
{
    uint32_t line = POSITION_VAL(pin);
    uint32_t code = (port == GPIOA) ? 0u
                  : (port == GPIOB) ? 1u
                  : (port == GPIOC) ? 2u
                  : (port == GPIOD) ? 3u : 4u;
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    uint32_t shift = (line & 3u) * 4u;
    uint32_t tmp = SYSCFG->EXTICR[line >> 2u];
    tmp &= ~(0xFuL << shift);
    tmp |= (code << shift);
    SYSCFG->EXTICR[line >> 2u] = tmp;
}

/* 开一条线的双边沿触发（正反向边沿都要盖时间戳），并给引脚配内部上拉：
 * 编码器未插/插头接触不良时 AF 脚是悬空天线，电机高占空比的换向噪声会在
 * 上面打出假边沿 interrupt 风暴（优先级 5 永远抢占任务 → 1kHz 环饿死）。
 * ⚠ 只动 PUPDR（AF 态下上下拉独立生效），HAL_GPIO_Init 会碰 MODER 断计数 */
static void EdgeExtiEnable(uint16_t pin, GPIO_TypeDef *port)
{
    EdgeExtiMap(pin, port);
    EXTI->IMR  |= pin;
    EXTI->RTSR |= pin;
    EXTI->FTSR |= pin;

    uint32_t pos = POSITION_VAL(pin) * 2u;
    uint32_t tmp = port->PUPDR;
    tmp &= ~(3uL << pos);
    tmp |= (1uL << pos);            /* 01 = 内部上拉 */
    port->PUPDR = tmp;
}

void Encoder_InitAll(void)
{
    for (uint32_t i = 0; i < 4u; i++)
    {
        /* 编码器模式两通道都要 Start 计数才走起来 */
        (void)HAL_TIM_Encoder_Start(s_enc_tim[i], TIM_CHANNEL_1);
        (void)HAL_TIM_Encoder_Start(s_enc_tim[i], TIM_CHANNEL_2);

        /* 采样环基线：以当前 CNT 为零点，首个边沿的差分从这里起算 */
        s_edge_last[i] = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_tim[i]);
        s_edge_acc[i] = 0;
        s_ring_head[i] = 0u;
        s_take_idx[i] = 0u;
    }

    /* 先挂回调再开中断（Exti_Attach 契约：仅初始化期注册）。
     * ⚠ CH3 挂 PA7（B 相）：PA6 与 PC6 同为 EXTI 线 6 不可共存，
     * B 相边沿率与 A 相相同，对测速等价（详见 bsp_encoder.h 脚位一节） */
    (void)Exti_Attach(PIN_ENC1A_GPIO_PIN, EdgeCaptureCh1);
    (void)Exti_Attach(PIN_ENC2A_GPIO_PIN, EdgeCaptureCh2);
    (void)Exti_Attach(PIN_ENC3B_GPIO_PIN, EdgeCaptureCh3);
    (void)Exti_Attach(PIN_ENC4A_GPIO_PIN, EdgeCaptureCh4);

    EdgeExtiEnable(PIN_ENC1A_GPIO_PIN, PIN_ENC1A_GPIO_PORT);
    EdgeExtiEnable(PIN_ENC2A_GPIO_PIN, PIN_ENC2A_GPIO_PORT);
    EdgeExtiEnable(PIN_ENC3B_GPIO_PIN, PIN_ENC3B_GPIO_PORT);
    EdgeExtiEnable(PIN_ENC4A_GPIO_PIN, PIN_ENC4A_GPIO_PORT);

    /* EXTI9_5 收编线 6/7/8（线 15 随既有 EXTI15_10 向量，gpio.c 已使能）。
     * 优先级与 EXTI15_10 一致取 5：回调无 RTOS 调用，同级不打乱既有次序 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

int32_t Encoder_Read(uint8_t ch)
{
    if (ch < ENC_CH1 || ch > ENC_CH4)
    {
        return 0;
    }
    uint32_t idx = ch - ENC_CH1;

    uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_tim[idx]);
    uint16_t last = s_enc_last[idx];
    s_enc_last[idx] = now;

    /* uint16 差值经 int16 解释，计数回绕（0→65535 / 65535→0）天然正确 */
    int16_t diff = (int16_t)(now - last);
    return (int32_t)diff;
}

int32_t Encoder_GetCount(uint8_t ch)
{
    if (ch < ENC_CH1 || ch > ENC_CH4)
    {
        return 0;
    }
    uint32_t idx = ch - ENC_CH1;

    uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_tim[idx]);
    int16_t diff = (int16_t)(now - s_pos_last[idx]);
    s_pos_last[idx] = now;
    s_pos_acc[idx] += diff;     /* int32 累计，±21.4 亿计数内不回绕 */
    return s_pos_acc[idx];
}

uint16_t Encoder_Head(uint8_t ch)
{
    if (ch < ENC_CH1 || ch > ENC_CH4)
    {
        return 0;
    }
    return s_ring_head[ch - ENC_CH1];
}

uint8_t Encoder_TakeNew(uint8_t ch, Encoder_Sample_t *out, uint8_t max, bool *dropped)
{
    if (dropped != NULL)
    {
        *dropped = false;
    }
    if (ch < ENC_CH1 || ch > ENC_CH4 || out == NULL || max == 0u)
    {
        return 0;
    }
    uint32_t idx = ch - ENC_CH1;

    uint16_t head = s_ring_head[idx];       /* 快照：只认此刻之前落盘的样本 */
    uint16_t take = s_take_idx[idx];
    uint16_t n_new = head - take;           /* uint16 相减，回绕天然正确 */
    if (n_new > ENC_RING_DEPTH)             /* 掉队超一圈：最老样本已被覆盖 */
    {
        if (dropped != NULL)
        {
            *dropped = true;
        }
        take = (uint16_t)(head - ENC_RING_DEPTH);
        n_new = ENC_RING_DEPTH;
    }
    if (n_new > max)                        /* 容量截断：余下的下次接着给 */
    {
        n_new = max;
    }
    for (uint16_t k = 0u; k < n_new; k++)
    {
        out[k] = s_ring[idx][(take + k) & (ENC_RING_DEPTH - 1u)];
    }
    s_take_idx[idx] = (uint16_t)(take + n_new);
    return (uint8_t)n_new;
}
