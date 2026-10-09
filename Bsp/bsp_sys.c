/**
 * @file    bsp_sys.c
 * @brief   DWT 微秒计时/延时 + 时钟自检
 */
#include "bsp_sys.h"
#include "bsp_pin.h"
#include "stm32f4xx_hal.h"

#define BSP_SYS_EXPECT_HZ   168000000u  /* HSE 8MHz，PLLM=4/PLLN=168/VCO=336→PLL÷2，
                                         * APB1=HCLK/4(42M)/定时器84M、APB2=HCLK/2(84M)/定时器168M，
                                         * 见 F407VG.ioc / main.c SystemClock_Config */

/* ---- DWT 微秒时基（坑#15 修正版）----
 * 本板为 F407 修订 5/6（REV_ID=0x101F，RM0090 1677 页），新修订 die 的
 * CoreSight 宏单元带 LAR 锁（0xE0001FB0），出厂默认上锁：软件对 DWT 的
 * 读写全部落空（CTRL/CYCCNT/LSR 读出同值 0x40000001 总线常数、写 CYCCNT
 * 无效），现象为"使能位全对但计数器不走"。先写解锁密钥 0xC5ACCE55 再
 * 使能即恢复正常（2026-10-07 板上实测，测试台 12 快窗比值≈1.0000）。
 * ⚠ 换用老修订 die（无 LAR）时此写无害；J-Link 在场/离场不再影响。 */

/* ---- CYCCNT 软件回绕扩展 ----
 * CYCCNT 是 32 位 @168MHz，每 25.57 秒转满归零——Bsp_GetUs 若直接换算
 * 会是 25.6s 周期的锯齿（注释曾称"71.6 分钟回绕"是漏算 CYCCNT 本体回绕，
 * 坑#15 家族第二案）。手法同 bsp_encoder 16bit 扩展：每次调用做无符号
 * 差分累进（前提：相邻两次调用间隔 < 25.57s——现全部调用方均为控制环/
 * 日志级频率，远满足）。
 * ⚠ 累计器必须 64 位：u32 累计器自身 25.57s 就溢出，等于没修
 * （2026-10-09 双口对审实测：JF 流每 25.60s 一谷，即 u32 累计器溢出）。
 * 对外 u32 µs 返回值 71.6 分钟才回绕，跨此点的差分靠调用方无符号减法
 * 自动正确，与扩展前契约一致 */
static uint32_t s_cyc_last;
static uint64_t s_cyc_ext;

void Bsp_Init(void)
{
    /* 时钟自检：SystemCoreClock 与 RCC 实际时钟源不符则停机闪烁 LED2（PC13，低电平亮）。
     * 调用点在 MX_GPIO_Init 之后，GPIO 已可用。 */
    SystemCoreClockUpdate();
    if (SystemCoreClock != BSP_SYS_EXPECT_HZ ||
        (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            for (volatile uint32_t i = 0u; i < 1680000u; i++) { __NOP(); }
        }
    }

    /* DWT 微秒计数器：LAR 解锁（新修订 die 必须）→ 清零 → 使能，次序固定。
     * ⚠ J-Link 断开（qc）会把调试域打回默认（TRCENA 清/LAR 回锁），烧录后
     * 不按复位直接跑的业务路径须再调 Bsp_DwtReArm()（任务级补一枪） */
    Bsp_DwtReArm();
}

/* DWT 重整使能链（幂等）：LAR 解锁 → TRCENA → 清零 → CYCCNTENA。
 * 任何"调试器刚拔/刚断开"的场景调用一次即恢复计数 */
void Bsp_DwtReArm(void)
{
    (*(volatile uint32_t *)0xE0001FB0u) = 0xC5ACCE55u;   /* DWT LAR 解锁密钥 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cyc_last = 0u;        /* 计数器清零须同步扩展快照，否则首次 GetUs 差分爆表 */
}

uint32_t Bsp_GetUs(void)
{
    uint32_t now = DWT->CYCCNT;
    s_cyc_ext += (uint32_t)(now - s_cyc_last);  /* 无符号差分：25.57s 回绕自动正确 */
    s_cyc_last = now;
    /* 64 位累计 → 除法降为 64/32（__aeabi_uldivmod，控制环频率可承受）；
     * 返回值 71.6 分钟才回绕 */
    return (uint32_t)(s_cyc_ext * 1000000u / SystemCoreClock);
}

uint32_t Bsp_GetMs(void)
{
    return HAL_GetTick();
}

/* 芯片身份只读窥视：DEV_ID 低 16 位（正品 F407 家族 = 0x0413）+ REV_ID 高 16 位。
 * 硅片级身份，丝印可仿、这个仿不了——新板 bring-up 验芯片第一步（坑#15 教训） */
uint32_t Bsp_GetDevId(void)
{
    return DBGMCU->IDCODE;
}

void Bsp_DelayUs(uint32_t us)
{
    /* 忙等 CYCCNT 回绕差分；u32 满量程（71.6 分钟）内任意时长安全 */
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks)
    {
        __NOP();
    }
}
