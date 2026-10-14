/**
 * @file    rc_cmd.c
 * @brief   RC_Cmd 遥控协议层实现（帧解析/归一化/两级故障语义/快照读）
 *
 * 语义与布局冻结说明见 rc_cmd.h 头部。本文件是纯逻辑 + 一个关调度拷贝：
 * 除快照锁（FreeRTOS）外无任何硬件依赖，PC 单测喂假时间即可全覆盖。
 */
#include "rc_cmd.h"
#include <string.h>

/* 关调度快照锁（架构 §5.1 处方；白名单登记见 rc_cmd.h 头部）。
 * PC 单测由 PcTests/freertos_stub 提供两个空函数替身 */
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------ 模块内状态 ------------------------------ */
/* s_cmd 是快照源：唯一写者 = remote_task（Update），读全走 GetCopy */
static RC_Cmd_t s_cmd;
static bool     s_decaying;        /* 失联渐停进行中 */
static uint32_t s_decay_start_ms;  /* 渐停起点 = 失联检测时刻 */
static float    s_decay_src[3 + ROBOT_CMD_JOINT_COUNT]; /* 渐停起点指令快照（vx,vy,omega,joint…） */

/* ------------------------------ 内部小工具 ------------------------------ */

/* 全部指令归零（不动 keys/seq/last_frame_ms/valid/estop——那些是链路信息不是指令） */
static void ZeroCommands_(void)
{
    s_cmd.vx = 0.0f;
    s_cmd.vy = 0.0f;
    s_cmd.omega = 0.0f;
    for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
    {
        s_cmd.joint[i] = 0.0f;
    }
}

/* 逻辑量 → 归一化比例值：÷满幅；接收端防御性钳位（发送端也钳，双保险） */
static float Normalize_(int32_t raw, int32_t range)
{
    if (raw > range)
    {
        raw = range;
    }
    if (raw < -range)
    {
        raw = -range;
    }
    return (float)raw / (float)range;
}

/* ------------------------------ 外部接口 ------------------------------ */

void RC_Cmd_Init(void)
{
    memset(&s_cmd, 0, sizeof s_cmd);   /* 全零态：valid=false 恰为 0，语义正确 */
    s_decaying = false;
}

void RC_Cmd_Update(const RCPayload_t *f, uint32_t now_ms)
{
    if (f != NULL)
    {
        /* ---- 正常帧路径：链路活着 ---- */
        s_cmd.seq = f->seq;
        s_cmd.keys = f->keys;
        s_cmd.last_frame_ms = now_ms;

        if ((f->flags & RC_FLAG_ESTOP) != 0u)
        {
            /* 急停：指令立即归零、estop 闩住（每一帧都重写标志，
             * 松键后的 estop=0 帧自然解除——不需要计时器） */
            s_cmd.estop = true;
            ZeroCommands_();
        }
        else
        {
            s_cmd.estop = false;
            s_cmd.vx = Normalize_(f->vx, ROBOT_CMD_RANGE);
            s_cmd.vy = Normalize_(f->vy, ROBOT_CMD_RANGE);
            s_cmd.omega = Normalize_(f->omega, ROBOT_CMD_RANGE);
            for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
            {
                s_cmd.joint[i] = Normalize_(f->joint[i], ROBOT_CMD_JOINT_RANGE);
            }
        }

        s_cmd.valid = true;
        s_decaying = false;   /* 新帧打断进行中的渐停 */
    }
    else
    {
        /* ---- 失联路径（第二写者，仅应在 Remote_IsLinkUp()==false 时驱动）----
         * valid 立即 false；指令从本时刻起 ROBOT_CMD_DECAY_MS 内线性滑落到零。
         * estop 标志保持不动：急停闩锁不因链路死而解除 */
        if (!s_decaying)
        {
            /* 渐停起点：把最后的指令值快照下来，之后按 elapsed 比例缩放 */
            s_decay_src[0] = s_cmd.vx;
            s_decay_src[1] = s_cmd.vy;
            s_decay_src[2] = s_cmd.omega;
            for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
            {
                s_decay_src[3 + i] = s_cmd.joint[i];
            }
            s_decay_start_ms = now_ms;
            s_decaying = true;
        }
        s_cmd.valid = false;

        /* uint32 减法：毫秒计数器回绕时 elapsed 依然正确 */
        uint32_t elapsed = now_ms - s_decay_start_ms;
        if (elapsed >= ROBOT_CMD_DECAY_MS)
        {
            ZeroCommands_();      /* 滑停完成，恒零 */
            s_decaying = false;
        }
        else
        {
            float scale = 1.0f - (float)elapsed / (float)ROBOT_CMD_DECAY_MS;
            s_cmd.vx = s_decay_src[0] * scale;
            s_cmd.vy = s_decay_src[1] * scale;
            s_cmd.omega = s_decay_src[2] * scale;
            for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
            {
                s_cmd.joint[i] = s_decay_src[3 + i] * scale;
            }
        }
    }
}

bool RC_Cmd_GetCopy(RC_Cmd_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    /* 关调度：任务切换被暂停，写者无法插进来，整结构体拷贝必然完整一致 */
    vTaskSuspendAll();
    *out = s_cmd;
    xTaskResumeAll();
    return true;
}
