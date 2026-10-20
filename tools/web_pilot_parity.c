/* 双端对账测试（不入构建）：网页打包器（PROTOCOL.md 同款逻辑）产出的帧
 * 必须被固件 web_input 逐字段还原。帧字节由 Python 按线协议 v2 打包生成
 * （vx/vy/omega 为 i16 小端有符号逻辑量）。改线协议时两处同步：
 * 本文件帧表 + 生成脚本（tools 里同款 pack/crc8 逻辑，见 PROTOCOL.md §2）。 */
#include <stdio.h>
#include "web_input.h"

int main(void)
{
    /* Python 打包：pack(wseq, vx, vy, omega, keys, estop) */
    static const uint8_t f0[12] = {0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; /* 零帧 */
    static const uint8_t f1[12] = {0x5A,0x01,0xE8,0x03,0x18,0xFC,0xF4,0x01,0x05,0x00,0x00,0x02}; /* 满±+omega500+K1A/K3A */
    static const uint8_t f2[12] = {0x5A,0x02,0xFB,0xFF,0x7B,0x00,0x18,0xFC,0x08,0x02,0x01,0xE0}; /* 负小值+omega满负+K2B+急停 */
    static const uint8_t f3[12] = {0x5A,0x03,0x18,0xFC,0xE8,0x03,0x09,0x03,0xFF,0x0F,0x01,0x43}; /* 反向满±+omega777+全刀位 */

    static const struct { int16_t vx, vy, o; uint16_t k; uint8_t e; } exp[4] = {
        {0, 0, 0, 0x0000, 0}, {1000, -1000, 500, 0x0005, 0},
        {-5, 123, -1000, 0x0208, 1}, {-1000, 1000, 777, 0x0FFF, 1},
    };
    static const uint8_t *frames[4] = { f0, f1, f2, f3 };

    WebInput_State_t st;
    RcInput_Raw_t    raw;
    int fails = 0;

    WebInput_Init(&st);
    for (int i = 0; i < 4; i++)
    {
        WebInput_Feed(&st, frames[i], WEBINPUT_FRAME_LEN, (uint32_t)(50u * i));
        if (!WebInput_Get(&st, &raw, (uint32_t)(50u * i), 500u))
        {
            printf("FAIL f%d: 帧被判非新鲜\n", i);
            fails++;
            continue;
        }
        if (raw.vx != exp[i].vx || raw.vy != exp[i].vy || raw.omega != exp[i].o ||
            raw.keys != exp[i].k || raw.estop != exp[i].e)
        {
            printf("FAIL f%d: %d %d %d 0x%04X %u (期望 %d %d %d 0x%04X %u)\n",
                   i, raw.vx, raw.vy, raw.omega, raw.keys, raw.estop,
                   exp[i].vx, exp[i].vy, exp[i].o, exp[i].k, exp[i].e);
            fails++;
        }
    }
    if (st.frames_ok != 4u || st.frames_bad != 0u)
    {
        printf("FAIL 计数: ok=%u bad=%u (期望 4 0)\n", st.frames_ok, st.frames_bad);
        fails++;
    }
    printf(fails == 0 ? "PARITY OK: 4/4 帧逐字段一致（12B 含 omega）\n" : "PARITY FAIL\n");
    return fails;
}
