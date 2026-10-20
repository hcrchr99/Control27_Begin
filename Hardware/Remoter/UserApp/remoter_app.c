/**
 * @file    remoter_app.c
 * @brief   遥控器主任务：采输入 → 建帧 → 100Hz 发包（发送链路与 PTX 测试板同款）
 *
 * 数据流（输入源二选一，robot_config.h ROBOT_RMT_INPUT_WEB 编译期切换）：
 *  WEB ：网页遥控台经 USART1 喂逻辑量快照（vx/vy/omega ±1000 网页直传，20Hz
 *        心跳 +变化即发），网页帧看门狗超时 → 停发 RF——车端 400ms 看门狗接管走
 *        已验的渐停+断链声光路径，"输入设备掉线"不伪装成"链路活着"；
 *  PHYS：6 只自复位船型开关 ×2 方向刀（10ms 电平读，低有效，两拍一致消抖
 *        滤弹跳）+ 摇杆 ADC 对经 RcInput_NormalizeAxis 归一化 → RcInput_Raw_t。
 * 之后同路：RcInput_Build 纯函数建帧（轴钳位直通+方向刀→joint；omega 已在
 * 采集侧备好——WEB=网页 omega 轴直传，PHYS=K_6 刀经 RcInput_FoldOmega 折算
 * 定速率）→ Remote_SendPacket（阻塞等 ACK/重传）。节奏：ROBOT_SEND_PERIOD_MS
 * 周期，发包耗时从周期里扣除，超期只歇 1ms。
 *
 * 板上观察点：
 *  - LED2（PB7，高亮）：1Hz 心跳；初始化失败改为"每秒三短闪"故障码；
 *  - 串口（USART1 PA9 @115200）：上电自检结果 + 每秒发包/丢包统计。
 *
 * 与车端的接续：K_n→joint[n] 的物理分组错位、A/B 互斥假设、摇杆中点/死区
 * 标定都是待上手项，集中在 robot_config.h Input 段与 rc_input.h 注释。
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
#include "web_input.h"
#include "remote.h"
#include "robot_config.h"
#include "remoter_app.h"

/* 输入标定默认值（中点/死区仅 PHYS 摇杆路径消费，待真件实测后回写） */
static const RcInput_Config_t s_input_cfg = {
    .joy_center     = ROBOT_JOY_CENTER_COUNTS,
    .joy_deadband   = ROBOT_JOY_DEADBAND_COUNTS,
    .key_joint_rate = (int16_t)ROBOT_KEY_JOINT_RATE,
    .key_omega_rate = (int16_t)ROBOT_KEY_OMEGA_RATE,
};

#if !ROBOT_RMT_INPUT_WEB
/* ---- PHYS 物理采集专用（WEB 模式下整段不编译，免未用告警） ---- */

/* 开关脚表：[开关号][0=A 正向刀/1=B 反向刀]，开关号 n → 帧内字段，
 * K_1..K_5 → joint[0..4]，第 6 只 K_6 → 整车旋转 omega（2026-10-16 拍板） */
static const GpioPin_t s_switch_pins[6][2] = {
    { PIN_KEY_1A, PIN_KEY_1B },
    { PIN_KEY_2A, PIN_KEY_2B },
    { PIN_KEY_3A, PIN_KEY_3B },
    { PIN_KEY_4A, PIN_KEY_4B },
    { PIN_KEY_5A, PIN_KEY_5B },
    { PIN_KEY_6A, PIN_KEY_6B },
};

/* 开关消抖状态（两拍一致才翻转，见 RcInput_Debounce） */
static uint16_t s_keys_db;        /* 消抖输出态：位布局见 rc_input.h 宏组 */
static uint16_t s_keys_raw_prev;  /* 上拍原始读数 */

/* 双击绑定表：{消抖字位掩码, flags 位序号 1..6}——mask=0 为表尾哨兵。
 * 默认空表=功能休眠；启用示例（语义归车端，flags 契约见 rc_input.h）：
 *   { RCINPUT_KEY_K1_A, 1 },   K_1A 双击 → flags bit1（如 快/慢 切换）
 *   { RCINPUT_KEY_K5_A, 2 },   K_5A 双击 → flags bit2（如 关节/坐标 切换） */
static const RcInput_DclickBind_t s_dclick_binds[] = {
    { 0u, 0u },     /* 哨兵：绑定行加在本行之前 */
};
static RcInput_DclickState_t
    s_dclick_state[sizeof s_dclick_binds / sizeof s_dclick_binds[0]];

static const RcInput_DclickCfg_t s_dclick_cfg = {
    .window_ms  = (uint16_t)ROBOT_DCLICK_WINDOW_MS,
    .hold_ticks = (uint8_t)(ROBOT_DCLICK_HOLD_MS / ROBOT_SEND_PERIOD_MS),
};

#endif /* !ROBOT_RMT_INPUT_WEB */

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

    Log_Printf("[RMT] init OK: 遥控器 CH%d @%dms 周期 输入=%s\r\n",
               (int)ROBOT_RF_CHANNEL, (int)ROBOT_SEND_PERIOD_MS,
               ROBOT_RMT_INPUT_WEB ? "WEB(等网页帧)" : "PHYS(板载开关+摇杆)");

    RcInput_Raw_t raw;
    RCPayload_t   frame;
    uint8_t       seq = 0u;
    uint32_t      attempts = 0u;
    uint32_t      period_start = Bsp_GetMs();
    uint32_t      last_stat_ms = period_start;
    uint32_t      last_hb_ms = period_start;
    bool          led_on = false;
    memset(&frame, 0, sizeof frame);    /* 断发期统计行引用旧帧，先给全零 */
#if ROBOT_RMT_INPUT_WEB
    WebInput_State_t web;
    WebInput_Init(&web);
    bool web_up_prev = false;           /* 上电未连网页 = 断流态，不发包 */
#endif

    for (;;)
    {
        bool do_send = true;
        uint32_t now_ms = Bsp_GetMs();

#if ROBOT_RMT_INPUT_WEB
        /* ---- 网页输入：串口字节 → 解析 → 看门狗判新鲜 ---- */
        uint8_t  rxb[32];
        uint16_t rxn = Log_Poll(rxb, (uint16_t)sizeof rxb);
        if (rxn > 0u)
        {
            WebInput_Feed(&web, rxb, rxn, now_ms);
        }
        do_send = WebInput_Get(&web, &raw, now_ms, ROBOT_WEB_WATCHDOG_MS);
        if (do_send != web_up_prev)
        {
            Log_Printf(do_send
                ? "[RMT] web up：帧流恢复，RF 发送开启\r\n"
                : "[RMT] web lost：网页看门狗超时，停发 RF（车端将渐停+断链告警）\r\n");
            web_up_prev = do_send;
        }
#else
        /* ---- 采集：12 路方向刀（低有效）→ 两拍一致消抖 → 摇杆 ADC 对 ---- */
        uint16_t raw_now = 0u;
        for (int sw = 0; sw < 6; sw++)
        {
            if (!Gpio_Read(s_switch_pins[sw][0]))
            {
                raw_now |= (uint16_t)(1u << (2u * sw));
            }
            if (!Gpio_Read(s_switch_pins[sw][1]))
            {
                raw_now |= (uint16_t)(1u << (2u * sw + 1u));
            }
        }
        if (!Gpio_Read(PIN_KEY_EB))
        {
            raw_now |= RCINPUT_KEY_ESTOP_BIT;
        }
        s_keys_db = RcInput_Debounce(s_keys_db, s_keys_raw_prev, raw_now);
        s_keys_raw_prev = raw_now;

        raw.keys = (uint16_t)(s_keys_db & RCINPUT_KEY_DIR_MASK);
        raw.estop = (uint8_t)((s_keys_db & RCINPUT_KEY_ESTOP_BIT) != 0u);
        raw.omega = RcInput_FoldOmega(raw.keys, s_input_cfg.key_omega_rate);
        {
            /* 摇杆 ADC：原始值 → 轴逻辑量（设备清噪死区+归一化在此收口）；
             * ADC 异常按中点处理 = 轴指令归零（失安全），不阻塞发送链路 */
            uint16_t adc_x, adc_y;
            if (!Adc_JoyRead(&adc_x, &adc_y))
            {
                adc_x = ROBOT_JOY_CENTER_COUNTS;
                adc_y = ROBOT_JOY_CENTER_COUNTS;
            }
            raw.vx = RcInput_NormalizeAxis(adc_x, s_input_cfg.joy_center,
                                           s_input_cfg.joy_deadband);
            raw.vy = RcInput_NormalizeAxis(adc_y, s_input_cfg.joy_center,
                                           s_input_cfg.joy_deadband);
        }

        /* ---- 双击检测：逐绑定路步进，事件按 flags 位收集 ---- */
        uint8_t  dclick_flags = 0u;
        for (uint8_t i = 0u;
             i < (uint8_t)(sizeof s_dclick_binds / sizeof s_dclick_binds[0]);
             i++)
        {
            if (s_dclick_binds[i].level_mask == 0u)
            {
                break;                          /* 表尾哨兵 */
            }
            if (RcInput_DclickStep(&s_dclick_state[i],
                                   (s_keys_db & s_dclick_binds[i].level_mask) != 0u,
                                   now_ms, &s_dclick_cfg) != 0u)
            {
                dclick_flags |= (uint8_t)(1u << s_dclick_binds[i].flag_index);
            }
        }
#endif

        /* ---- 建帧 + 发送（WEB 断流期整段跳过 = 停发 RF，不伪装链路活着） ---- */
        if (do_send)
        {
            RcInput_Build(&s_input_cfg, &raw, (uint8_t)(++seq), &frame);
#if !ROBOT_RMT_INPUT_WEB
            frame.flags |= dclick_flags;    /* 双击事件位（Build 只管 estop 位） */
#endif
            attempts++;
            (void)Remote_SendPacket((const uint8_t *)&frame,
                                    (uint8_t)sizeof frame);
        }

        /* ---- 心跳 + 统计 ---- */
        uint32_t now = now_ms;
        if ((now - last_hb_ms) >= ROBOT_HEARTBEAT_MS)
        {
            last_hb_ms = now;
            led_on = !led_on;
            if (led_on) { Gpio_Set(PIN_LED1); } else { Gpio_Reset(PIN_LED1); }
        }
        if ((now - last_stat_ms) >= 1000u)
        {
#if ROBOT_RMT_INPUT_WEB
            /* w=u/d 网页链路态；o=板端解码出的 omega（WEB=网页 omega 轴直传，
             * 推杆应看到连续值；恒 0 = 旧固件，重烧）；ok/err/dr=累计合法帧/
             * CRC 错帧/帧外丢字节（err、dr 齐涨 = 网页侧没在发/波特率不对，
             * 只有 dr 涨 = 帧头错位/新旧版本帧长错配） */
            Log_Printf("[RMT] tx=%u fail=%u seq=%u v=(%d,%d) o=%d k=0x%04X e=%d "
                       "w=%c ok=%u err=%u dr=%u\r\n",
                       (unsigned)attempts,
                       (unsigned)Remote_GetTxFailCount(),
                       (unsigned)frame.seq,
                       (int)frame.vx, (int)frame.vy, (int)frame.omega,
                       (unsigned)frame.keys,
                       (int)((frame.flags & RC_FLAG_ESTOP) != 0u),
                       web_up_prev ? 'u' : 'd',
                       (unsigned)web.frames_ok,
                       (unsigned)web.frames_bad,
                       (unsigned)web.bytes_dropped);
#else
            Log_Printf("[RMT] tx=%u fail=%u seq=%u v=(%d,%d) o=%d k=0x%04X e=%d\r\n",
                       (unsigned)attempts,
                       (unsigned)Remote_GetTxFailCount(),
                       (unsigned)frame.seq,
                       (int)frame.vx, (int)frame.vy, (int)frame.omega,
                       (unsigned)frame.keys,
                       (int)((frame.flags & RC_FLAG_ESTOP) != 0u));
#endif
            last_stat_ms = now;
        }

        /* ---- 周期对齐：发包耗时从周期内扣除，超期只歇 1ms ---- */
        uint32_t elapsed = Bsp_GetMs() - period_start;
        osDelay((elapsed >= ROBOT_SEND_PERIOD_MS) ? 1u
                 : (ROBOT_SEND_PERIOD_MS - elapsed));
        period_start = Bsp_GetMs();
    }
}
