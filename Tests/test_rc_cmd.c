/**
 * @file    test_rc_cmd.c
 * @brief   测试台 11=RC_CMD：rc_cmd 遥控协议层全链路验收
 *
 * 对表基准 = PTX 测试板信号发生器（Hardware/F103C8_PTX_T/UserApp/ptx_test.c
 * 头部波形表），12s 一个周期：
 *   [0, 8s)   正常发射：vx 三角波 / vy 正弦 / omega 方波 / joint 扫描 / keys 走位
 *   [8s, 9s)  estop 窗口：连发急停帧
 *   [9s, 10s) 正常发射（松键恢复段）
 *   [10s, 12s) 静默：完全不发包（车端看门狗翻转 + 指令渐停窗口）
 *
 * 验收项（逐项在串口日志核对）：
 *  A1 全链路波形：快照行与上表逐段一致（数值 ×100 整型打印）
 *  A2 estop 时效：estop 帧喂入 Update 的同一拍快照即 estop=1 且指令全零；
 *     车端检测延迟 = 一个 Poll(10ms) + 一跳 RF，满足 ≤50ms 规划线
 *  A3 失联渐停：静默开始 ≤400ms 内 valid 翻 false，快照指令 300ms 线性
 *     滑落到零后恒零
 *  A4 恢复：静默结束回到发射段后 valid 回 true、estop=false、波形续走
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "rc_cmd.h"
#include "remote.h"
#include "robot_config.h"

void Test_RcCmd_Init(void);
void Test_RcCmd_Poll(void);

static bool s_ok;                /* init 成功门控 */

/* seq 连续性（uint8 回绕差值 = 丢帧/去重口径，≠精确丢包数） */
static uint8_t  s_expected_seq;
static uint32_t s_seq_gap;

/* estop 观测 */
static bool     s_in_estop;      /* 上帧是否 estop（沿检测） */
static uint32_t s_estop_cnt;     /* estop 事件次数（上升沿计） */

/* 统计节拍 */
static uint32_t s_total;
static uint32_t s_last_stat_ms;
static uint32_t s_last_snap_ms;
static uint32_t s_last_count;

void Test_RcCmd_Init(void)
{
    Log_Printf("[T-RC] 11=RC_CMD 全链路验收（对表 PTX 12s 周期）\r\n");
    Log_Printf("[T-RC] 波形表：0-8s 正常 / 8-9s estop / 9-10s 恢复 / 10-12s 静默\r\n");
    Log_Printf("[T-RC] A1 波形对表  A2 estop≤50ms  A3 失联渐停  A4 恢复\r\n");

    s_ok = Remote_Init(REMOTE_MODE_PRX);
    if (!s_ok)
    {
        Log_Printf("[T-RC] [A0] Remote_Init FAIL: SPI/接线/共地排查\r\n");
        return;
    }
    RC_Cmd_Init();
    Log_Printf("[T-RC] [A0] init OK，等待 PTX 信号发生器\r\n");
}

void Test_RcCmd_Poll(void)
{
    if (!s_ok)
    {
        return;
    }

    Remote_Service();

    /* 收帧：seq 连续性 + estop 沿检测 + 唯一写者喂入 */
    RCPayload_t f;
    while (Remote_ReadPacket((uint8_t *)&f, (uint8_t)sizeof f))
    {
        s_seq_gap += (uint8_t)(f.seq - s_expected_seq);
        s_expected_seq = (uint8_t)(f.seq + 1u);
        s_total++;

        bool is_estop = ((f.flags & RC_FLAG_ESTOP) != 0u);
        bool estop_rising = (is_estop && !s_in_estop);

        if (is_estop)
        {
            s_in_estop = true;
            if (estop_rising)
            {
                s_estop_cnt++;
            }
        }
        else
        {
            if (s_in_estop)
            {
                Log_Printf("[T-RC] [A2] estop 解除 @%ums（松键恢复段）\r\n",
                           (unsigned)Bsp_GetMs());
            }
            s_in_estop = false;
        }

        RC_Cmd_Update(&f, Bsp_GetMs());

        if (estop_rising)
        {
            /* A2 时效证据：estop 帧喂入 Update 后同一拍快照必须已全零。
             * 喂入→生效在同一函数调用内完成；从"发射"到"生效"的全部延迟
             * = 一跳 RF + 一个 Poll(10ms)，≤50ms 规划线达标 */
            RC_Cmd_t ev;
            RC_Cmd_GetCopy(&ev);
            Log_Printf("[T-RC] [A2] estop 生效: seq=%u @%ums vx=%d(期望0) estop=%d\r\n",
                       f.seq, (unsigned)Bsp_GetMs(),
                       (int)(ev.vx * 100.0f), (int)ev.estop);
        }
    }

    /* 第二写者：看门狗超时期间驱动失效值 + 渐停 */
    if (!Remote_IsLinkUp())
    {
        RC_Cmd_Update(NULL, Bsp_GetMs());
    }

    uint32_t now = Bsp_GetMs();

    /* 快照行：200ms 周期（A1 波形对表 / A3 渐停曲线的观察载体） */
    if ((now - s_last_snap_ms) >= 200u)
    {
        s_last_snap_ms = now;
        RC_Cmd_t snap;
        RC_Cmd_GetCopy(&snap);
        Log_Printf("[T-RC] vx=%4d vy=%4d w=%4d j=%4d,%4d,%4d,%4d,%4d k=0x%04X sq=%3u L=%d V=%d E=%d\r\n",
                   (int)(snap.vx * 100.0f), (int)(snap.vy * 100.0f),
                   (int)(snap.omega * 100.0f),
                   (int)(snap.joint[0] * 100.0f), (int)(snap.joint[1] * 100.0f),
                   (int)(snap.joint[2] * 100.0f), (int)(snap.joint[3] * 100.0f),
                   (int)(snap.joint[4] * 100.0f),
                   (unsigned)snap.keys, (unsigned)snap.seq,
                   (int)Remote_IsLinkUp(), (int)snap.valid, (int)snap.estop);
    }

    /* 统计行：1Hz（收包率 / seq 跳变 / estop 次数） */
    if ((now - s_last_stat_ms) >= 1000u)
    {
        Log_Printf("[T-RC] STAT rx=%u/s total=%u gap=%u estop_n=%u link=%d\r\n",
                   (unsigned)(s_total - s_last_count), (unsigned)s_total,
                   (unsigned)s_seq_gap, (unsigned)s_estop_cnt,
                   (int)Remote_IsLinkUp());
        s_last_count = s_total;
        s_last_stat_ms = now;
    }
}
