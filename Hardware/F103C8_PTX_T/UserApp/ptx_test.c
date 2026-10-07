/**
 * @file    ptx_test.c
 * @brief   F103C8 测试发送端：W2.2 信号发生器（车端测试台 11=RC_CMD 的对表基准）
 *
 * 入口经 freertos.c 的 StartDefaultTask USER CODE 区转发到 Ptx_Test_Main
 * （defaultTask 是 CubeMX 生成的强符号，不能像车端任务那样用弱符号覆盖）。
 * 代码路径与未来遥控器一致：完整 Bsp 移植版 + Modules/remote（PTX 模式，
 * rc_cmd.h 的 RCPayload_t 即协议帧），IRQ 经 Exti_Attach 注册、osSignalSet
 * 唤醒 SendPacket——车端收包率统计的发送基准即 seq 字段（uint8 回绕）。
 *
 * ============================ 波形表（车端对表用） ============================
 * 12s 一个周期，相位 = Bsp_GetMs() 相对上电起点取模：
 *   [0, 8s)    正常发射，100Hz：
 *                vx     三角波 ±1000，10s 周期（4s 时过零、8s 到 +1000 附近）
 *                vy     正弦 ±800，4s 周期（32 点查表）
 *                omega  方波 ±500，4s 周期（2s 正 2s 负）
 *                joint[k] 三角波 ±100，周期 (3+k)s（k=0..4 → 3/4/5/6/7s）
 *                keys   位走灯 0x0001 << (秒数 % 16)，每秒走一位
 *   [8s, 9s)   estop 窗口：flags bit0=1 连发（其余字段照填，车端强制归零）
 *   [9s, 10s)  正常发射（松键恢复段，车端 estop 应回 false、波形续走）
 *   [10s, 12s) 静默：完全不发包（车端看门狗 400ms 翻转 + 指令 300ms 渐停窗口）
 * 发送端纪律：每帧整帧清零再填。测试板无按键无 ADC——急停触发即程序化窗口，
 * 真按键归 Remoter 整机（未排期）。
 *
 * 遗留能力保留：载波测试（对端看 rpd）、角色互换频偏扫描（ROBOT_DIAG_ROLE_SWAP）。
 */
#include "cmsis_os.h"
#include <string.h>
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_spi.h"    /* Spi_Ce——S2 起靠隐式声明碰巧工作的历史隐患，显式化 */
#include "rc_cmd.h"
#include "remote.h"
#include "nrf24.h"
#include "robot_config.h"

void Ptx_Test_Main(void const * argument);

/* 三角波：period_ms 周期在 [-amp, +amp] 间线性往返（phase_ms 为全局相位） */
static int32_t Tri_(uint32_t period_ms, uint32_t phase_ms, int32_t amp)
{
    uint32_t p = phase_ms % period_ms;
    uint32_t half = period_ms / 2u;
    if (p < half)
    {
        return -amp + (int32_t)((2u * (uint32_t)amp * p) / half);
    }
    return amp - (int32_t)((2u * (uint32_t)amp * (p - half)) / half);
}

/* RF 物理层体检：连续载波 5 秒（附录 C：CONT_WAVE + PLL_LOCK + CE 常高）。
 * 对端 PRX 在这 5 秒内看 rpd：rpd=1 ⇒ 天线/距离/频段物理层全通；
 * rpd=0 ⇒ 硬件问题（天线/供电/模块/频率），与收发包逻辑无关。 */
static void CarrierTest(void)
{
    uint8_t rf = Nrf24_ReadReg(NRF_REG_RF_SETUP);
    Nrf24_WriteReg(NRF_REG_RF_SETUP,
                   (uint8_t)(rf | NRF_RF_SETUP_CONT_WAVE | NRF_RF_SETUP_PLL_LOCK));
    Spi_Ce(true);
    for (uint32_t i = 5u; i > 0u; i--)
    {
        Log_Printf("[PTX] 载波测试 %us...（看对端 rpd）\r\n", (unsigned)i);
        osDelay(1000u);
    }
    Spi_Ce(false);
    Nrf24_WriteReg(NRF_REG_RF_SETUP, rf);   /* 复原，正常发包模式 */
}

void Ptx_Test_Main(void const * argument)
{
    (void)argument;
    Bsp_Init();
    Log_Init();
    osDelay(100u);      /* 等模块电源稳定 */

    if (!Remote_Init(ROBOT_DIAG_ROLE_SWAP ? REMOTE_MODE_PRX : REMOTE_MODE_PTX))
    {
        Log_Printf("[PTX] Remote_Init FAIL: SPI/接线/共地排查\r\n");
        for (;;) { osDelay(1000u); }
    }
    Log_Printf("[PTX] init OK: %s CH%d\r\n",
               ROBOT_DIAG_ROLE_SWAP ? "诊断互换:本板收(PRX)" : "信号发生器 12s 周期 @100Hz",
               (int)ROBOT_RF_CHANNEL);

#if ROBOT_DIAG_ROLE_SWAP
    /* 频偏定位扫描：对端持续发射中，逐频道采样 RPD 找能量真实落点。
     * 若对端实际中心偏离标称频道（晶振偏差/兼容片频偏），rpd=1 的频道
     * 就是它的真实落点；锁到命中频道后进入接收。 */
    {
        Nrf24_FlushRx();
        uint8_t hits[13] = {0};
        for (int round = 0; round < 5; round++)
        {
            for (uint8_t ch = 70u; ch <= 82u; ch++)
            {
                Nrf24_WriteReg(NRF_REG_RF_CH, ch);
                osDelay(2u);    /* PLL 重锁（130us）+ RPD 稳定（40us）+ 余量 */
                hits[ch - 70u] += (uint8_t)(Nrf24_ReadReg(NRF_REG_RPD) & 0x01u);
            }
        }
        uint8_t best = ROBOT_RF_CHANNEL;
        int best_hits = -1;
        for (int i = 0; i < 13; i++)
        {
            Log_Printf("[C8-RX] sweep ch=%d rpd_hits=%d/5\r\n", 70 + i, hits[i]);
            if (hits[i] > best_hits) { best_hits = hits[i]; best = (uint8_t)(70 + i); }
        }
        Nrf24_WriteReg(NRF_REG_RF_CH, best);
        Nrf24_FlushRx();
        Log_Printf("[C8-RX] LOCK ch=%d (标称 %d, 偏差 %+d MHz)%s\r\n",
                   best, (int)ROBOT_RF_CHANNEL, (int)best - (int)ROBOT_RF_CHANNEL,
                   (best == ROBOT_RF_CHANNEL) ? "" : " ← 频偏实锤");
    }

    /* 诊断互换：测试板临时当接收端，验证测试板模块的接收能力 */
    {
        uint8_t  frame[ROBOT_REMOTE_PAYLOAD];
        uint32_t total = 0u, last_stat_ms = Bsp_GetMs(), last_count = 0u;

        for (;;)
        {
            Remote_Service();
            while (Remote_ReadPacket(frame, (uint8_t)sizeof frame))
            {
                Log_Printf("RX seq=%u total=%u\r\n", frame[0],
                           (unsigned)Remote_GetRxCount());
            }
            uint32_t now = Bsp_GetMs();
            if ((now - last_stat_ms) >= 1000u)
            {
                uint32_t cnt = Remote_GetRxCount();
                Log_Printf("[C8-RX] %upkts/s total=%u rpd=%d st=0x%02X\r\n",
                           (unsigned)(cnt - last_count), (unsigned)cnt,
                           (int)(Nrf24_ReadReg(NRF_REG_RPD) & 0x01u),
                           Nrf24_GetStatus());
                last_count = cnt;
                last_stat_ms = now;
            }
        }
    }
#endif

    /* TX FIFO 写入校验：写一包后 TX_EMPTY(bit6) 必须变 0，
     * 仍为 1 = 载荷根本没进 FIFO（SPI 写路径/模块问题） */
    {
        uint8_t probe[ROBOT_REMOTE_PAYLOAD] = {0};
        Nrf24_WriteRegBuf(NRF_CMD_W_TX_PAYLOAD, probe, (uint8_t)sizeof probe);
        uint8_t fifo = Nrf24_ReadReg(NRF_REG_FIFO_STATUS);
        Log_Printf("[PTX] fifo_chk=0x%02X (TX_EMPTY=%d, 期望 0)\r\n",
                   fifo, (int)((fifo & NRF_FIFO_TX_EMPTY) != 0u));
        Nrf24_FlushTx();
    }

    CarrierTest();

    /* ==================== 信号发生器主循环（波形表见文件头） ==================== */
    static const int16_t SIN32[32] = {
        0, 156, 306, 444, 566, 665, 739, 785, 800, 785, 739, 665, 566, 444, 306, 156,
        0, -156, -306, -444, -566, -665, -739, -785, -800, -785, -739, -665, -566, -444, -306, -156,
    };

    uint32_t seq = 0u;
    uint32_t attempts = 0u;
    uint32_t ack_ok = 0u;
    uint32_t last_stat_ms = Bsp_GetMs();
    uint32_t t0 = Bsp_GetMs();

    for (;;)
    {
        uint32_t phase = (Bsp_GetMs() - t0) % 12000u;

        if (phase < 10000u)     /* [10s,12s) 静默：完全不发包 */
        {
            RCPayload_t f;
            memset(&f, 0, sizeof f);        /* 发送端纪律：整帧清零再填 */

            f.vx = (int16_t)Tri_(10000u, phase, 1000);
            f.vy = SIN32[(phase % 4000u) * 32u / 4000u];
            f.omega = (int16_t)(((phase % 4000u) < 2000u) ? 500 : -500);
            for (int k = 0; k < ROBOT_CMD_JOINT_COUNT; k++)
            {
                f.joint[k] = (int8_t)Tri_((uint32_t)(3000 + 1000 * k), phase, 100);
            }
            f.keys = (uint16_t)(0x0001u << ((phase / 1000u) % 16u));
            if (phase >= 8000u && phase < 9000u)    /* [8s,9s) estop 窗口连发；
                                                     * [9s,10s) 松键恢复段（车端验闩锁解除） */
            {
                f.flags |= RC_FLAG_ESTOP;
            }
            f.seq = (uint8_t)(++seq);       /* seq 只在实发帧上递增 */

            attempts++;
            if (Remote_SendPacket((uint8_t *)&f, (uint8_t)sizeof f))
            {
                ack_ok++;
            }
        }

        uint32_t now = Bsp_GetMs();
        if ((now - last_stat_ms) >= 1000u)
        {
            /* st=STATUS arc=ARC_CNT(本包重传次数) los=PLOS_CNT(累计丢包)：
             * arc/los 恒 0 = 从未发射（查 CE 接线/PWR_UP）；
             * los 增长 = 发射了但收不到 ACK（查天线/供电/共地/距离/频道） */
            uint8_t st = Nrf24_GetStatus();
            uint8_t obs = Nrf24_ReadReg(NRF_REG_OBSERVE_TX);
            Log_Printf("[PTX] att=%u ok=%u fail=%u seq=%u st=0x%02X arc=%u los=%u\r\n",
                       (unsigned)attempts, (unsigned)ack_ok,
                       (unsigned)Remote_GetTxFailCount(), (unsigned)seq,
                       st, (unsigned)(obs & 0x0Fu), (unsigned)(obs >> 4));
            last_stat_ms = now;
        }
        osDelay(10u);       /* ≈100Hz；实际节奏 = 10ms + 发送阻塞时长，两端以计数对账 */
    }
}
