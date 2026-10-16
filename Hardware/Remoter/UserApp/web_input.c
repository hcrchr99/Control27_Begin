/**
 * @file    web_input.c
 * @brief   网页遥控台输入源实现（纯函数，见 web_input.h）
 */
#include "web_input.h"
#include <string.h>

uint8_t WebInput_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00u;

    if (data == NULL)
    {
        return 0u;
    }
    while (len-- > 0u)
    {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x80u) != 0u)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

void WebInput_Init(WebInput_State_t *st)
{
    if (st != NULL)
    {
        memset(st, 0, sizeof *st);
    }
}

/* 错帧后的滑窗重同步：缓冲 [0]=曾当帧头的 0x5A 已证伪，从 [1..] 里找下一个
 * 0x5A 挪到队首接着攒（找不到 = 清空继续找帧头）。垃圾流里混着真帧头也能捞回 */
static void Reslide_(WebInput_State_t *st)
{
    for (uint8_t i = 1u; i < WEBINPUT_FRAME_LEN; i++)
    {
        if (st->buf[i] == WEBINPUT_HEADER_BYTE)
        {
            memmove(&st->buf[0], &st->buf[i],
                    (size_t)(WEBINPUT_FRAME_LEN - i));
            st->fill = (uint8_t)(WEBINPUT_FRAME_LEN - i);
            return;
        }
    }
    st->fill = 0u;
}

/* 结算一帧：小端逐字段解出快照（不整段 memcpy——字节序与符号显式写清，
 * 两端各自打包，pc/ARM 两侧行为一致不靠运气） */
static void Settle_(WebInput_State_t *st, uint32_t now_ms)
{
    st->snap.vx = (int16_t)(uint16_t)(st->buf[2] | ((uint16_t)st->buf[3] << 8));
    st->snap.vy = (int16_t)(uint16_t)(st->buf[4] | ((uint16_t)st->buf[5] << 8));
    st->snap.keys  = (uint16_t)(st->buf[6] | ((uint16_t)st->buf[7] << 8));
    st->snap.estop = st->buf[8];
    st->last_frame_ms = now_ms;
    st->frames_ok++;
    st->fill = 0u;
}

void WebInput_Feed(WebInput_State_t *st, const uint8_t *data, uint16_t len,
                   uint32_t now_ms)
{
    if (st == NULL || data == NULL)
    {
        return;
    }

    for (uint16_t i = 0u; i < len; i++)
    {
        uint8_t b = data[i];

        if (st->fill == 0u)                     /* 找帧头中 */
        {
            if (b == WEBINPUT_HEADER_BYTE)
            {
                st->buf[0] = b;
                st->fill = 1u;
            }
            else
            {
                st->bytes_dropped++;
            }
            continue;
        }

        st->buf[st->fill++] = b;
        if (st->fill < WEBINPUT_FRAME_LEN)
        {
            continue;
        }

        /* 攒满一帧：CRC 覆盖 [1..8]，[9] 是校验字节 */
        if (WebInput_Crc8(&st->buf[1], 8u) == st->buf[9])
        {
            Settle_(st, now_ms);
        }
        else
        {
            st->frames_bad++;
            Reslide_(st);
        }
    }
}

bool WebInput_Get(const WebInput_State_t *st, RcInput_Raw_t *out,
                  uint32_t now_ms, uint16_t watchdog_ms)
{
    if (st == NULL || out == NULL)
    {
        return false;
    }
    if (st->frames_ok == 0u)
    {
        return false;                           /* 从未收到合法帧 */
    }
    if ((now_ms - st->last_frame_ms) > (uint32_t)watchdog_ms)
    {
        return false;                           /* 超窗：上层停发 RF */
    }
    *out = st->snap;
    return true;
}
