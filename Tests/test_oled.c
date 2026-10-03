/**
 * @file    test_oled.c
 * @brief   S7 测试项 9=OLED：四项显示对表 + 拔插排线自愈 + 地址参数化探测
 *
 * 验收标准（Modules 规划 §3.5 / BSP 坑#10 / Modules 坑#11）：
 *  - T0 探测与初始化：ROBOT_OLED_I2C_ADDR ACK、Oled_Init 无错、屏亮无花屏；
 *  - T1 四项内容（电压/电流/功率/链路状态）与串口同拍数值一致、无花屏/拖影；
 *  - T2 周期刷新稳定（500ms 门控；2s 心跳/栈高水位正常，~92ms 刷新阻塞不顶调度）；
 *  - T3 拔插自愈：刷新中拔 OLED 4P 排线 → Oled_Refresh 超时返回 false 不挂死
 *    （连败 ROBOT_IIC_RECOVER_N 触发 bsp_iic 总线恢复，recover 计数增长）；
 *    插回 → 模块掉电寄存器态全丢，连败 ROBOT_OLED_REINIT_AFTER_FAILS 次
 *    自动重发 init（无需复位）自动恢复显示；
 *  - T4 地址参数化（坑#11 S7 验收点）：0x3C 探测 ACK、未插器件 0x68（插针
 *    预留 IMU 地址）探测无 ACK——同一 API 靠地址参数访问不同器件。
 *
 * 操作（板上）：
 *  - 常态 500ms 刷新四项 + 串口同拍打印（整型小数化 mV/mA/mW，坑#14）；
 *  - KEY1 = 数据页 ↔ 图形演示页（线/框/圆，验证帧缓冲像素路径）；
 *    按键采样固定在 10ms Poll 节拍（S6 教训：放进打印门消抖会倍化周期）；
 *  - T3 操作：拔排线 → 看串口 FAIL 与 recover 计数 → 插回 → 看自动恢复。
 *  - 链路状态说明：本测试不初始化 Modules/remote（SPI 归 5=REMOTE 项），
 *    Remote_IsLinkUp 恒 false → 屏显 LINK=DOWN 为正确表现；W2 接 RC_Cmd
 *    后该项自然转真。
 */
#include "test_bench.h"
#include "oled.h"
#include "ssd1306.h"        /* 图形演示页原语（线/框/圆，本层冻结 API 之外的自检路径） */
#include "ssd1306_fonts.h"
#include "power.h"
#include "remote.h"
#include "bsp_iic.h"
#include "bsp_pin.h"
#include "bsp_gpio.h"
#include "bsp_sys.h"
#include "bsp_log.h"

/* 按键消抖：连续 N 拍(10ms)电平一致才认稳定（与 test_power 同参） */
#define TEST_OLED_KEY_STABLE    3u

/* 探测地址（T4）：插针预留 IMU 7 位地址，本测试不插器件，预期无 ACK */
#define TEST_OLED_PROBE_IMU     0x68u

/* float(物理量) → 整型打印值（×1000，即 mV/mA/mW）；NaN → -1（test_power 同法） */
static int32_t ToMilli(float x)
{
    return (x != x) ? -1 : (int32_t)(x * 1000.0f);
}

/* KEY1 稳定下降沿检测（消抖后翻转，返回 true 一次；上拉空闲 = 高） */
static bool Key1_PressedEdge(void)
{
    static uint8_t s_cnt;
    static bool    s_stable = true;

    bool level = Gpio_Read(PIN_KEY1);
    if (level == s_stable)
    {
        s_cnt = 0u;
        return false;
    }
    if (++s_cnt >= TEST_OLED_KEY_STABLE)
    {
        s_cnt = 0u;
        s_stable = level;
        return !level;                  /* 稳定到低 = 按下沿 */
    }
    return false;
}

/* 数据页：四项标签框架（数值由 500ms 刷新逐行覆盖） */
static void Draw_DataPage(void)
{
    ssd1306_Fill(Black);
    Oled_Printf(0, 0,  "S7 OLED TEST");
    Oled_Printf(0, 10, "V=----mV");
    Oled_Printf(0, 20, "I=----mA");
    Oled_Printf(0, 30, "P=----mW");
    Oled_Printf(0, 40, "LINK=----");
    Oled_Printf(0, 54, "KEY1=GFX demo");
    (void)Oled_Refresh();
}

/* 图形演示页：线/框/圆验证帧缓冲像素路径（DrawPixel 位序 = 手册 §8.7 对照项） */
static void Draw_GfxPage(void)
{
    ssd1306_Fill(Black);
    ssd1306_DrawRectangle(0, 0, 127, 63, White);
    ssd1306_Line(0, 0, 127, 63, White);
    ssd1306_Line(127, 0, 0, 63, White);
    ssd1306_DrawCircle(64, 32, 18, White);
    ssd1306_SetCursor(46, 28);
    (void)ssd1306_WriteString("GFX OK", Font_7x10, White);
    (void)Oled_Refresh();
}

void Test_Oled_Init(void)
{
    Log_Printf("[T-OLED] 9=OLED 就绪：500ms 刷新 V/I/P+LINK，KEY1=切图形演示页\r\n");

    /* 四项数值源就绪：power 未 Init 时 GetVoltage 等返 NaN（屏显 -1），
     * 内部 Adc_Init 幂等——main 已启过则此调用无副作用 */
    bool pw = Power_Init();
    Log_Printf("[T-OLED] Power_Init %s，标定系数 VK=%dm IK=%dm（×0.001）\r\n",
               pw ? "就绪" : "失败（看 bsp 日志）",
               (int)(ROBOT_POWER_V_K * 1000.0f), (int)(ROBOT_POWER_I_K * 1000.0f));

    /* T4 地址参数化：同一 API 按地址访问不同器件（0x68 无 ACK 属预期，
     * 探测失败计入 bsp_iic 连败计数但远不及恢复阈值，无副作用） */
    bool oled_ack = Iic_IsDeviceReady(ROBOT_OLED_I2C_ADDR);
    bool imu_ack = Iic_IsDeviceReady(TEST_OLED_PROBE_IMU);
    Log_Printf("[T-OLED] T4 探测 0x%02X=%s（预期 ACK）/ 0x%02X=%s（预期无ACK，插针预留IMU）\r\n",
               (unsigned)ROBOT_OLED_I2C_ADDR, oled_ack ? "ACK" : "无应答",
               (unsigned)TEST_OLED_PROBE_IMU, imu_ack ? "ACK!" : "无ACK");

    bool ok = Oled_Init();
    Log_Printf("[T-OLED] T0 Oled_Init %s（%s）\r\n",
               ok ? "成功" : "失败",
               ok ? "无花屏即过" : "查接线/共地/上拉/地址，见 S7 计划接线清单");
    if (ok)
    {
        Draw_DataPage();
    }
}

void Test_Oled_Poll(void)
{
    static uint32_t s_last;
    static bool     s_gfx_page = false;
    static bool     s_was_refresh_ok = true;
    uint32_t now = Bsp_GetMs();

    /* KEY1 页面切换：采样必须在 10ms Poll 节拍（打印门之前，S6 板上实测教训） */
    if (Key1_PressedEdge())
    {
        s_gfx_page = !s_gfx_page;
        if (s_gfx_page)
        {
            Draw_GfxPage();
        }
        else
        {
            Draw_DataPage();
        }
        Log_Printf("[T-OLED] 切换 → %s\r\n", s_gfx_page ? "图形演示页" : "数据页");
    }

    if ((now - s_last) < 500u)
    {
        return;
    }
    s_last = now;

    /* 数据页：四项内容刷新（与串口同拍对表 T1）；演示页只做链路占位不刷 */
    bool refresh_ok = true;
    if (!s_gfx_page)
    {
        int32_t v_mV = ToMilli(Power_GetVoltage());
        int32_t i_mA = ToMilli(Power_GetCurrent());
        int32_t p_mW = ToMilli(Power_GetPower());
        bool link = Remote_IsLinkUp();

        Oled_Printf(0, 10, "V=%dmV", v_mV);
        Oled_Printf(0, 20, "I=%dmA", i_mA);
        Oled_Printf(0, 30, "P=%dmW", p_mW);
        Oled_Printf(0, 40, "LINK=%s", link ? "UP" : "DOWN");
        refresh_ok = Oled_Refresh();
    }

    /* T2/T3 观察：刷新态迁移与恢复计数（拔插排线时 recover 应增长，
     * 插回后 refresh 恢复 OK 且屏内容自动重现） */
    if (refresh_ok != s_was_refresh_ok)
    {
        Log_Printf("[T-OLED] T3 刷新态 → %s（recover=%u）\r\n",
                   refresh_ok ? "OK" : "FAIL",
                   (unsigned)Iic_GetRecoverCount());
        s_was_refresh_ok = refresh_ok;
    }

    Log_Printf("[T-OLED] V=%dmV I=%dmA P=%dmW LINK=%s refresh=%s recover=%u\r\n",
               ToMilli(Power_GetVoltage()), ToMilli(Power_GetCurrent()),
               ToMilli(Power_GetPower()), Remote_IsLinkUp() ? "UP" : "DOWN",
               refresh_ok ? "OK" : "FAIL",
               (unsigned)Iic_GetRecoverCount());
}
