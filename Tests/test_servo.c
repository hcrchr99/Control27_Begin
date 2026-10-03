/**
 * @file    test_servo.c
 * @brief   S5 测试项 7=SERVO：中位稳定 → 角度扫描 → SG90 卸力 / 数字 fail-hold
 *
 * 验收标准（Modules 层开发规划 §3.2）：上电缓至中位无猛冲；0→90→180° 扫描
 * 平滑；SG90 Release 后手掰无保持力矩；数字舵机停脉冲后仍保持（fail-hold
 * 特性验证并记录）。
 *
 * 序列（自动循环，2s/步）：
 *   Init 发中位（300ms 稳定窗由 servo 模块自管，不占用本序列）→
 *   四路同步扫描 0/90/180/90/0（软限位钳位生效：0→5°、180→175°，
 *   占位 ±5°）→ SG90（ID3 手腕/ID4 爪子）Release：手掰应无保持力矩 →
 *   数字（ID1 大臂/ID2 小臂）停脉冲：应仍锁存原位 = fail-hold（记录现象）→
 *   循环回扫描（SetAngle 自动恢复发脉冲，验证 Release 后可重上力）。
 */
#include "test_bench.h"
#include "servo.h"
#include "bsp_sys.h"
#include "bsp_log.h"

void Test_Servo_Init(void)
{
    Servo_InitAll();
    Log_Printf("[T-SERVO] 7=SERVO：中位→0/90/180 扫描→SG90 手掰→数字 fail-hold，"
               "2s/步（软限位占位 ±5°）\r\n");
}

void Test_Servo_Poll(void)
{
    static uint32_t s_last;
    static uint8_t  s_step;     /* 0..4 = 扫描，5 = SG90 Release，6 = 数字停脉冲 */
    uint32_t now = Bsp_GetMs();
    if ((now - s_last) < 2000u)
    {
        return;
    }
    s_last = now;

    static const float s_sweep[] = { 0.0f, 90.0f, 180.0f, 90.0f, 0.0f };
    const uint8_t n = (uint8_t)(sizeof(s_sweep) / sizeof(s_sweep[0]));

    if (s_step < n)
    {
        float deg = s_sweep[s_step];
        for (uint8_t id = 0u; id < SERVO_COUNT; id++)
        {
            Servo_SetAngle((ServoId_t)id, deg);
        }
        Log_Printf("[T-SERVO] 四路同步 %u°（限位钳位后 5..175）\r\n", (unsigned)deg);
    }
    else if (s_step == n)
    {
        bool wrist = Servo_Release(SERVO_ID3);
        bool claw  = Servo_Release(SERVO_ID4);
        Log_Printf("[T-SERVO] SG90 已 Release（真卸力 手腕=%d 爪子=%d，应=1）："
                   "手掰应无保持力矩\r\n", wrist, claw);
    }
    else
    {
        bool arm  = Servo_Release(SERVO_ID1);
        bool fore = Servo_Release(SERVO_ID2);
        Log_Printf("[T-SERVO] 数字舵机已停脉冲（真卸力 大臂=%d 小臂=%d，应=0）："
                   "应仍锁存保持（fail-hold 实测记录）\r\n", arm, fore);
    }
    s_step = (uint8_t)((s_step + 1u) % (n + 2u));
}
