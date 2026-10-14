/**
 * @file    remoter_app.c
 * @brief   遥控器主任务：采输入 → 建帧 → 100Hz 发包（发送链路与 PTX 测试板同款）
 *
 * 数据流：6 船型开关（10ms 电平读，低有效）+ 摇杆 ADC 对 → RcInput_Raw_t
 * → RcInput_Build 纯函数建帧 → Remote_SendPacket（阻塞等 ACK/重传）。
 * 节奏：ROBOT_SEND_PERIOD_MS 周期，发包耗时从周期里扣除，超期只歇 1ms。
 *
 * 板上观察点：
 *  - LED2（PB7，高亮）：1Hz 心跳；初始化失败改为"每秒三短闪"故障码；
 *  - 串口（USART1 PA9 @115200）：上电自检结果 + 每秒发包/丢包统计。
 *
 * 与车端的接续：K_n→joint[n] 的方向语义、omega 输入轴、摇杆中点/死区标定
 * 都是待拍板/待实测项，集中在 robot_config.h Input 段与 rc_input.h 注释。
 */
#include "cmsis_os.h"
#include <string.h>
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_gpio.h"
#include "bsp_pin.h"
#include "bsp_adc.h"
#include "bsp_spi.h"
#include "rc_cmd.h"
#include "rc_input.h"
#include "remote.h"
#include "robot_config.h"
#include "remoter_app.h"

/* 输入标定默认值（摇杆中点/死区待真件实测后回写 robot_config.h） */
static const RcInput_Config_t s_input_cfg = {
    .joy_center     = ROBOT_JOY_CENTER_COUNTS,
    .joy_deadband   = ROBOT_JOY_DEADBAND_COUNTS,
    .key_joint_rate = (int16_t)ROBOT_KEY_JOINT_RATE,
};

/* 开关脚表：下标 = 帧内 joint[]/keys 位序（K_1..K_5；急停与预留键单独读） */
static const GpioPin_t s_key_pins[ROBOT_CMD_JOINT_COUNT] = {
    PIN_KEY_1, PIN_KEY_2, PIN_KEY_3, PIN_KEY_4, PIN_KEY_5,
};

/* 初始化失败就地显示：每秒三短闪（约 300ms 亮 3 次），日志每秒重发一条 */
static void Fail_Blink(const char *reason)
{
    for (;;)
    {
        Log_Printf("[RMT] init FAIL: %s\r\n", reason);
        for (int i = 0; i < 3; i++)
        {
            Gpio_Set(PIN_LED1);
            osDelay(60u);
            Gpio_Reset(PIN_LED1);
            osDelay(140u);
        }
        osDelay(800u);
    }
}

void Remoter_Main(void const *argument)
{
    (void)argument;
    Bsp_Init();
    Log_Init();
    osDelay(100u);      /* 等无线模块电源稳定（PTX 同款纪律） */

    if (!Spi_Init())
    {
        Fail_Blink("SPI2 未就绪（查 CubeMX 句柄/使能）");
    }
    if (!Remote_Init(REMOTE_MODE_PTX))
    {
        Fail_Blink("Remote_Init 失败（SPI/接线/共地/模块）");
    }

    Log_Printf("[RMT] init OK: 遥控器 CH%d @%dms 周期\r\n",
               (int)ROBOT_RF_CHANNEL, (int)ROBOT_SEND_PERIOD_MS);

    RcInput_Raw_t raw;
    RCPayload_t   frame;
    uint8_t       seq = 0u;
    uint32_t      attempts = 0u;
    uint32_t      period_start = Bsp_GetMs();
    uint32_t      last_stat_ms = period_start;
    uint32_t      last_hb_ms = period_start;
    bool          led_on = false;

    for (;;)
    {
        /* ---- 采集：开关电平（低有效）+ 摇杆 ADC 对 ---- */
        raw.keys = 0u;
        for (int k = 0; k < ROBOT_CMD_JOINT_COUNT; k++)
        {
            if (!Gpio_Read(s_key_pins[k]))      /* 按下 = 拉到地 */
            {
                raw.keys |= (uint8_t)(1u << k);
            }
        }
        raw.estop = (uint8_t)(!Gpio_Read(PIN_KEY_EB) ? 1u : 0u);
        if (!Adc_JoyRead(&raw.joy_x, &raw.joy_y))
        {
            /* ADC 异常按中点处理 = 底盘指令归零（失安全），不阻塞发送链路 */
            raw.joy_x = ROBOT_JOY_CENTER_COUNTS;
            raw.joy_y = ROBOT_JOY_CENTER_COUNTS;
        }

        /* ---- 建帧 + 发送 ---- */
        RcInput_Build(&s_input_cfg, &raw, (uint8_t)(++seq), &frame);
        attempts++;
        (void)Remote_SendPacket((const uint8_t *)&frame,
                                (uint8_t)sizeof frame);

        /* ---- 心跳 + 统计 ---- */
        uint32_t now = Bsp_GetMs();
        if ((now - last_hb_ms) >= ROBOT_HEARTBEAT_MS)
        {
            last_hb_ms = now;
            led_on = !led_on;
            if (led_on) { Gpio_Set(PIN_LED1); } else { Gpio_Reset(PIN_LED1); }
        }
        if ((now - last_stat_ms) >= 1000u)
        {
            Log_Printf("[RMT] tx=%u fail=%u seq=%u v=(%d,%d) k=0x%02X e=%d\r\n",
                       (unsigned)attempts,
                       (unsigned)Remote_GetTxFailCount(),
                       (unsigned)frame.seq,
                       (int)frame.vx, (int)frame.vy,
                       (unsigned)frame.keys,
                       (int)((frame.flags & RC_FLAG_ESTOP) != 0u));
            last_stat_ms = now;
        }

        /* ---- 周期对齐：发包耗时从周期内扣除，超期只歇 1ms ---- */
        uint32_t elapsed = Bsp_GetMs() - period_start;
        osDelay((elapsed >= ROBOT_SEND_PERIOD_MS) ? 1u
                 : (ROBOT_SEND_PERIOD_MS - elapsed));
        period_start = Bsp_GetMs();
    }
}
