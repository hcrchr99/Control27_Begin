/**
 * @file    web_input.h
 * @brief   网页遥控台输入源：USART1 字节流 → 线协议解帧 → RcInput_Raw_t 快照 + 看门狗
 *
 * 背景（2026-10-16 拍板，docs/网页遥控器开发规划.md）：输入硬件空窗期，网页模拟
 * 实际遥控器经 USB-TTL 喂板。本模块只替代"物理采集"这一段——解析出的快照仍是
 * RcInput_Raw_t，下游 RcInput_Build（死区/建帧/整帧清零纪律）原样复用，rc_input
 * 零改动。真件到位后 robot_config.h 切回 PHYS，本模块被 --gc-sections 回收。
 *
 * 线协议（两端契约，网页侧镜像实现见 PcTwin/web_pilot/PROTOCOL.md）：
 *   10 字节定长，小端：
 *   [0]     0x5A   帧头（重同步锚点）
 *   [1]     wseq   网页帧序号 u8（本模块不消费，留诊断）
 *   [2..3]  vx     i16  轴逻辑量 ±ROBOT_CMD_RANGE（网页直传，发送端钳位）
 *   [4..5]  vy     i16  同上
 *   [6..7]  keys   u16  方向刀位图（位布局见 rc_input.h 宏组）
 *   [8]     estop  u8   1=急停
 *   [9]     CRC8   覆盖 [1..8]：poly 0x07、初值 0x00、不反射、无异或输出
 *
 * 解析纪律：
 *  - 无应用层重传——UART 帧错了丢掉等下一拍（网页 20Hz 心跳，冗余足够），
 *    CRC 只防错帧不追帧；
 *  - 校验失败从缓冲内滑窗重同步：下一个 0x5A 起重新攒帧，垃圾流不淹没状态机；
 *  - 快照更新 = 新鲜合法帧整份替换（网页抖动不影响 100Hz 建帧节奏，与物理
 *    开关"按住不动"同语义）；
 *  - 看门狗归调用方判（WebInput_Get 传窗长）：超时不是错误态，只是"新鲜"为
 *    false——上层停发 RF，让车端 400ms 看门狗走已验的渐停+声光路径。
 *
 * 纯函数零硬件依赖：时间由调用方递入（PcTests 喂假时间），字节源由调用方递入
 * （固件=Log_Poll 拉环形缓冲）。
 */
#ifndef WEB_INPUT_H
#define WEB_INPUT_H

#include <stdint.h>
#include <stdbool.h>
#include "rc_input.h"

/* 帧长与帧头（网页侧逐字节镜像，改这里必须同步网页 PROTOCOL.md） */
#define WEBINPUT_FRAME_LEN      10u
#define WEBINPUT_HEADER_BYTE    0x5Au

/**
 * @brief  CRC8（线协议校验算法，公开给测试与网页对照）
 * @param  data  参与校验的字节串
 * @param  len   字节数
 * @retval CRC 值。多项式 0x07（x^8+x^2+x+1），初值 0x00，不反射，无异或输出。
 * @note   标准自检向量："123456789" → 0xF4
 */
uint8_t WebInput_Crc8(const uint8_t *data, uint16_t len);

/* 解析器状态（调用方持有；上电 WebInput_Init 清零即初始化）。
 * 计数器字段公开给 remoter_app 的 [RMT] 统计行直接读 */
typedef struct
{
    /* 解析器内部（勿手改） */
    uint8_t  buf[WEBINPUT_FRAME_LEN];   /* 攒帧缓冲 */
    uint8_t  fill;                      /* 已攒字节数（0=找帧头中） */

    /* 最新合法帧快照 + 到达时刻（调用方时基） */
    RcInput_Raw_t snap;
    uint32_t last_frame_ms;

    /* 诊断计数 */
    uint32_t frames_ok;                 /* 累计合法帧数（0=从未收到，Get 恒 false） */
    uint32_t frames_bad;                /* 累计 CRC 错帧数 */
    uint32_t bytes_dropped;             /* 帧头外的丢弃字节数（垃圾流诊断） */
} WebInput_State_t;

/**
 * @brief  清态：攒帧进度、计数、快照全部归零（上电/复连重置）
 */
void WebInput_Init(WebInput_State_t *st);

/**
 * @brief  喂字节流（每次收到的任意长度，攒够一帧自动结算）
 * @param  st      解析器状态
 * @param  data    字节流（可为 NULL/0 长度，等于没喂）
 * @param  len     字节数
 * @param  now_ms  当前毫秒时基（调用方递入，uint32 回绕安全：只用无符号差分）
 * @note   合法帧到达即整份替换快照并记时间戳；错帧不碰快照（保留上一帧数据，
 *         看门狗窗口内车端仍收到"最后一次已知好输入"，与失联语义分离）。
 */
void WebInput_Feed(WebInput_State_t *st, const uint8_t *data, uint16_t len,
                   uint32_t now_ms);

/**
 * @brief  取输入快照（新鲜度由调用方判）
 * @param  st           解析器状态
 * @param  out          快照输出（仅返回 true 时被写入）
 * @param  now_ms       当前毫秒时基
 * @param  watchdog_ms  新鲜窗长（remoter_app 传 robot_config 宏；测试注入任意值）
 * @retval true = 从未丢过链且最近一帧在窗长内，快照拷入 out；
 *         false = 从未收到合法帧，或最近一帧已超窗（out 不动）
 */
bool WebInput_Get(const WebInput_State_t *st, RcInput_Raw_t *out,
                  uint32_t now_ms, uint16_t watchdog_ms);

#endif /* WEB_INPUT_H */
