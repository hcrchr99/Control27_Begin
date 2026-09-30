/**
 * @file    test_encoder.c
 * @brief   S3 测试项：四路编码器手转（bsp_encoder 验收语义复原）
 *
 * 验收标准（commit bb35b3b）：手转四轮计数连续、换向符号正确、无跳变；
 * 方向符号与直觉相反属正常，由上层 ROBOT_MOTOR_SIGN 标定。
 * ⚠ 新板首测先查共地（S3 血泪教训：编码器与 MCU 不共地 → 四路症状各异，
 *   恒单方向/跳动/串扰，换什么配置都没用）。
 */
#include "test_bench.h"
#include "bsp_encoder.h"
#include "bsp_sys.h"
#include "bsp_log.h"

void Test_Encoder_Init(void)
{
    Encoder_InitAll();
    Log_Printf("[T-ENC] 手转四轮，每 200ms 打印 inc(增量)/tot(累计)\r\n");
}

void Test_Encoder_Poll(void)
{
    static uint32_t s_last;
    uint32_t now = Bsp_GetMs();
    if ((now - s_last) < 200u)
    {
        return;
    }
    s_last = now;

    Log_Printf("[T-ENC] inc:%d %d %d %d tot:%d %d %d %d\r\n",
               (int)Encoder_Read(ENC_CH1), (int)Encoder_Read(ENC_CH2),
               (int)Encoder_Read(ENC_CH3), (int)Encoder_Read(ENC_CH4),
               (int)Encoder_GetCount(ENC_CH1), (int)Encoder_GetCount(ENC_CH2),
               (int)Encoder_GetCount(ENC_CH3), (int)Encoder_GetCount(ENC_CH4));
}
