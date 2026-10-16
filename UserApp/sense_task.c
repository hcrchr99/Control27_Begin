/**
 * @file    sense_task.c
 * @brief   感知任务：Power 采样换算 → OLED 四行检录显示——强覆盖 freertos.c
 *          的 __weak StartApp_Sense_Task（同 chassis/daemon 模式，CubeMX 文件
 *          零改动）
 *
 * 职责边界（架构 §3.3 / UserApp 规划 §3.4）：本任务只做"取数 → 排版 → 刷新"
 * 的搬运工——电压/电流/功率换算全在 Modules/power（线性换算+窗口均值），
 * 像素全在 Modules/oled，本文件不碰 ADC/I²C 细节。显示内容四项（电压/电流/
 * 功率/链路状态）是比赛检录定案口径（误差 ≤±10% 硬项，系数标定入口在
 * robot_config.h 的 ROBOT_POWER_*_K）。
 *
 * 刷新策略（2026-10-17 拍板，评估文档 T10 裁决）：I2C2 提 400kHz（CubeMX
 * Fast Mode，整屏刷新 ~92ms→~25ms），本任务 50ms 延时 + 25ms 刷新 ≈ 75ms
 * 实际显示周期——检录人眼足够，且单页 I²C 事务对 20ms 超时的余量从 1.7×
 * 提到 6.7×（排线稍差不再撞超时触发总线自愈）。若实车边沿差要退回 100kHz：
 * .ioc 改回 + ROBOT_SENSE_PERIOD_MS 改 200，本文件零改动。
 *
 * 失败语义：Power_Get* 未就绪/读失败返 NaN（power.h 失安全约定），排版层
 * 显示 "--"；采样链零点偏移的小负数按零显示（功率路径电流物理上不为负）。
 * Oled_Init 失败不挡进循环——Oled_Refresh 连败 5 次自动重发器件 init
 * （拔插排线自愈路径，测试台 9 已过验），本任务只管按拍喂。
 *
 * 边界裁决：本层排版无状态、无业务决策（纯展示），不拆 *_ctrl.c 纯函数
 * 文件——UserApp 规划 §一.4 的"纯函数分离"针对业务算术，排版够不上。
 * 循环内不打印（UserApp 静默纪律）：数值对表走测试台 8/9，链路异常有声光。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "power.h"
#include "remote.h"
#include "oled.h"
#include "robot_config.h"
#include "test_bench.h"

void StartApp_Sense_Task(void const * argument);

/* float(物理量) → mV/mA/mW 整型；NaN（未就绪/读失败）→ INT32_MIN 哨兵，
 * 排版层显示 "--"——与零点偏移的小负数区分开（test_oled 的 -1 哨兵在此
 * 不够用：-1 会和真实读数 -0.001A 混淆） */
static int32_t ToMilli(float x)
{
    return (x != x) ? INT32_MIN : (int32_t)(x * 1000.0f);
}

void StartApp_Sense_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);              /* 等 Bsp/Log 初始化（Bsp_Init 在 main 已完成） */
    TestBench_Yield("sense");   /* 测试台激活时让位（I2C2/ADC 归测试台 8/9） */

    Power_Init();               /* 幂等：main 已启过 ADC 则无副作用（test_oled 同法） */
    bool oled_ok = Oled_Init(); /* 幂等，内含 100ms 上电稳定；失败靠 Refresh 自愈 */
    Log_Printf("[SENSE] Power 就绪，Oled_Init %s（四行检录显示，%ums 一拍）\r\n",
               oled_ok ? "成功" : "失败（连败自愈兜底）",
               (unsigned)ROBOT_SENSE_PERIOD_MS);

    for (;;)
    {
        int32_t u_mV = ToMilli(Power_GetVoltage());
        int32_t i_mA = ToMilli(Power_GetCurrent());
        int32_t p_mW = ToMilli(Power_GetPower());
        bool    link = Remote_IsLinkUp();

        /* 整型小数化（坑#14：未链 _printf_float，%f 打出来是空的）——
         * mV/mA/mW 拆成整数段+小数段拼；小负数（零点偏移）钳到零 */
        if (u_mV == INT32_MIN) { Oled_Printf(0, 0,  "U=--.-- V"); }
        else {
            if (u_mV < 0) { u_mV = 0; }
            Oled_Printf(0, 0,  "U=%d.%02d V", u_mV / 1000, u_mV % 1000 / 10);
        }
        if (i_mA == INT32_MIN) { Oled_Printf(0, 10, "I=--.--- A"); }
        else {
            if (i_mA < 0) { i_mA = 0; }
            Oled_Printf(0, 10, "I=%d.%03d A", i_mA / 1000, i_mA % 1000);
        }
        if (p_mW == INT32_MIN) { Oled_Printf(0, 20, "P=--.-- W"); }
        else {
            if (p_mW < 0) { p_mW = 0; }
            Oled_Printf(0, 20, "P=%d.%02d W", p_mW / 1000, p_mW % 1000 / 10);
        }
        Oled_Printf(0, 30, "LINK %s", link ? "UP" : "DOWN");

        (void)Oled_Refresh();   /* 失败自愈在驱动内（连败 5 次重 init），任务层不处理 */
        osDelay(ROBOT_SENSE_PERIOD_MS);
    }
}
