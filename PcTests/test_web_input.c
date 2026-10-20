/**
 * @file    test_web_input.c
 * @brief   网页遥控台输入源单测（host 编译）——线协议规格固化用例：
 *          CRC8 自检向量、合法帧逐字段小端解码（含负数轴逻辑量与 omega 轴）、
 *          分片喂入、垃圾流找帧头、错帧滑窗重同步、载荷含 0x5A 仍可判、
 *          看门狗新鲜窗边界与回绕、计数器、NULL 安全。期望值按 web_input.h
 *          线协议 v2（12B 含 omega）手排字节。
 */
#include "web_input.h"
#include "test_util.h"
#include <string.h>

/* 组一帧合法字节（buf 至少 WEBINPUT_FRAME_LEN）；返回帧长。
 * vx/vy/omega 为有符号轴逻辑量，按 i16 小端打包 */
static unsigned PackFrame(uint8_t *buf, uint8_t wseq, int16_t vx, int16_t vy,
                          int16_t omega, uint16_t keys, uint8_t estop)
{
    uint16_t ux = (uint16_t)vx;
    uint16_t uy = (uint16_t)vy;
    uint16_t uo = (uint16_t)omega;

    buf[0] = WEBINPUT_HEADER_BYTE;
    buf[1] = wseq;
    buf[2] = (uint8_t)(ux & 0xFFu);         /* 小端：低字节在前 */
    buf[3] = (uint8_t)(ux >> 8);
    buf[4] = (uint8_t)(uy & 0xFFu);
    buf[5] = (uint8_t)(uy >> 8);
    buf[6] = (uint8_t)(uo & 0xFFu);
    buf[7] = (uint8_t)(uo >> 8);
    buf[8] = (uint8_t)(keys & 0xFFu);
    buf[9] = (uint8_t)(keys >> 8);
    buf[10] = estop;
    buf[11] = WebInput_Crc8(&buf[1], 10u);
    return WEBINPUT_FRAME_LEN;
}

/* CRC8 算法对表：标准自检向量（poly 0x07/init 0x00 的公认结果） */
static void TestCrc8Vector(void)
{
    CHECK(WebInput_Crc8((const uint8_t *)"123456789", 9u) == 0xF4u);
    CHECK(WebInput_Crc8((const uint8_t *)"", 0u) == 0x00u);     /* 空串=初值 */
    uint8_t one = 0x00u;
    CHECK(WebInput_Crc8(&one, 1u) == 0x00u);                    /* 单 0 字节不变 */
    one = 0x01u;
    CHECK(WebInput_Crc8(&one, 1u) == 0x07u);                    /* 首异或即多项式 */
    CHECK(WebInput_Crc8(NULL, 5u) == 0x00u);                    /* NULL 安全 */
}

/* 合法帧：逐字段小端解码正确（含负数轴量与 omega）、计数与时间戳、Get 新鲜直读 */
static void TestGoodFrame(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          f[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    CHECK(st.frames_ok == 0u && st.fill == 0u);

    /* 从未收帧：Get 恒 false，out 不被碰 */
    memset(&raw, 0xA5, sizeof raw);
    CHECK(WebInput_Get(&st, &raw, 1000u, 500u) == false);
    CHECK(raw.vx == (int16_t)0xA5A5);

    /* 0x5AA5 考小端位序；omega/keys/estop 同帧带全 */
    (void)PackFrame(f, 7u, 0x5AA5, 4095, -77,
                    RCINPUT_KEY_K2_B | RCINPUT_KEY_K1_A, 1u);
    WebInput_Feed(&st, f, WEBINPUT_FRAME_LEN, 1000u);
    CHECK(st.frames_ok == 1u && st.frames_bad == 0u && st.bytes_dropped == 0u);
    CHECK(WebInput_Get(&st, &raw, 1000u, 500u) == true);
    CHECK(raw.vx == 0x5AA5 && raw.vy == 4095 && raw.omega == -77);
    CHECK(raw.keys == (RCINPUT_KEY_K2_B | RCINPUT_KEY_K1_A));
    CHECK(raw.estop == 1u);

    /* 负数轴量：-5 = 0xFFFB，小端字节 FB FF，解回应为 -5（三轴各验一次） */
    (void)PackFrame(f, 8u, -5, -1000, -321, 0u, 0u);
    WebInput_Feed(&st, f, WEBINPUT_FRAME_LEN, 1100u);
    CHECK(WebInput_Get(&st, &raw, 1100u, 500u) == true);
    CHECK(raw.vx == -5 && raw.vy == -1000 && raw.omega == -321);

    /* 窗内持续新鲜（差分判据，不看绝对时刻） */
    CHECK(WebInput_Get(&st, &raw, 1599u, 500u) == true);
    CHECK(WebInput_Get(&st, &raw, 1600u, 500u) == true);
    CHECK(WebInput_Get(&st, &raw, 1601u, 500u) == false);   /* 恰超窗 1ms */
}

/* 时基回绕：uint32 无符号差分判据跨回绕点不失明 */
static void TestTimeWrap(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          f[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    (void)PackFrame(f, 0u, 1000, 1000, 500, 0u, 0u);
    WebInput_Feed(&st, f, WEBINPUT_FRAME_LEN, 0xFFFFFF00u);
    CHECK(WebInput_Get(&st, &raw, 0x00000050u, 500u) == true);  /* 差分 0x150=336 */
    CHECK(WebInput_Get(&st, &raw, 0x000000F4u, 500u) == true);  /* 恰在窗沿 */
    CHECK(WebInput_Get(&st, &raw, 0x000000F5u, 500u) == false);
}

/* 分片喂入：一帧拆三次喂（粘包/半包是串口常态），结算不变形 */
static void TestChunkedFeed(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          f[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    (void)PackFrame(f, 1u, -123, 456, -500, RCINPUT_KEY_K1_A, 0u);
    WebInput_Feed(&st, f, 3u, 100u);            /* 帧头+wseq+vx 低字节 */
    CHECK(st.frames_ok == 0u);
    WebInput_Feed(&st, f + 3u, 3u, 100u);       /* vx 高字节+vy 两字节 */
    CHECK(st.frames_ok == 0u);
    WebInput_Feed(&st, f + 6u, 6u, 100u);       /* 剩余 6 字节，帧在此结算 */
    CHECK(st.frames_ok == 1u);
    CHECK(WebInput_Get(&st, &raw, 100u, 500u) == true);
    CHECK(raw.vx == -123 && raw.vy == 456 && raw.omega == -500
          && raw.keys == RCINPUT_KEY_K1_A && raw.estop == 0u);

    /* 两帧连发一次喂入（一拍内积了俩），取后到者 */
    uint8_t g[2u * WEBINPUT_FRAME_LEN];
    (void)PackFrame(g, 2u, 111, 0, 0, 0u, 0u);
    (void)PackFrame(g + WEBINPUT_FRAME_LEN, 3u, -222, 0, -1, 0u, 0u);
    WebInput_Feed(&st, g, (uint16_t)sizeof g, 200u);
    CHECK(st.frames_ok == 3u);
    CHECK(WebInput_Get(&st, &raw, 200u, 500u) == true);
    CHECK(raw.vx == -222 && raw.omega == -1);
}

/* 垃圾流找帧头：帧前垃圾计数丢弃，帧照常识别 */
static void TestGarbagePrefix(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          f[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    (void)PackFrame(f, 9u, 77, 88, 999, 0u, 0u);
    uint8_t noisy[20];
    memset(noisy, 0x33u, sizeof noisy);         /* 无 0x5A 的垃圾 */
    memcpy(noisy + 6u, f, WEBINPUT_FRAME_LEN);  /* 垃圾 6 字节后跟完整帧 */
    WebInput_Feed(&st, noisy, (uint16_t)sizeof noisy, 300u);
    CHECK(st.bytes_dropped == 6u + 2u);         /* 6 垃圾 + 帧后 2 填充垃圾 */
    CHECK(st.frames_ok == 1u);
    CHECK(WebInput_Get(&st, &raw, 300u, 500u) == true);
    CHECK(raw.vx == 77 && raw.vy == 88 && raw.omega == 999);
}

/* 错帧重同步：CRC 坏帧被拒后，滑窗从缓冲内的下一帧头继续，
 * 后续字节补齐成好帧（高频丢乱的恢复路径） */
static void TestResyncAfterBadCrc(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          bad[WEBINPUT_FRAME_LEN];
    uint8_t          good[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    (void)PackFrame(bad, 1u, 1000, 0, 0, 0u, 0u);
    bad[11] ^= 0xFFu;                           /* 只坏校验字节 */
    (void)PackFrame(good, 2u, 2000, 0, 0, 0u, 0u);

    /* 坏帧后紧跟好帧一次喂入：bad 拒收（reslide 找到 good 帧头）→ good 结算 */
    uint8_t both[2u * WEBINPUT_FRAME_LEN];
    memcpy(both, bad, WEBINPUT_FRAME_LEN);
    memcpy(both + WEBINPUT_FRAME_LEN, good, WEBINPUT_FRAME_LEN);
    WebInput_Feed(&st, both, (uint16_t)sizeof both, 400u);
    CHECK(st.frames_bad == 1u);
    CHECK(st.frames_ok == 1u);
    CHECK(WebInput_Get(&st, &raw, 400u, 500u) == true);
    CHECK(raw.vx == 2000);

    /* 坏帧内载荷含 0x5A（真帧头长错位）：reslide 逐个试帧头，中间再拒两帧
     * 碎拼（好帧头部字节在缓冲里被带着走），最终恰从好帧自己的 0x5A 对齐，
     * 一发好帧完整落账分毫无损（12B 逐字节推演：坏计数共 3） */
    WebInput_Init(&st);
    (void)PackFrame(bad, 1u, 0x5A5A, 0, 0, 0u, 0u); /* 载荷字节=5A 5A 00… */
    bad[11] ^= 0x55u;
    WebInput_Feed(&st, bad, WEBINPUT_FRAME_LEN, 500u);
    CHECK(st.frames_bad == 1u);
    WebInput_Feed(&st, good, WEBINPUT_FRAME_LEN, 510u);
    CHECK(st.frames_bad == 3u);             /* 碎拼错帧再拒两发 */
    CHECK(st.frames_ok == 1u);              /* 好帧从自己的帧头复位落账 */
    CHECK(st.fill == 0u);
    CHECK(WebInput_Get(&st, &raw, 510u, 500u) == true);
    CHECK(raw.vx == 2000);
}

/* 载荷含 0x5A 的合法帧：CRC 过了就是数据不是帧头（判据是校验不是字节值） */
static void TestHeaderByteInPayload(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          f[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    (void)PackFrame(f, 0x5Au, 0x5A5A, 0x5A5A, 0x5A5A, 0x5A5A, 0x5Au);
    WebInput_Feed(&st, f, WEBINPUT_FRAME_LEN, 600u);
    CHECK(st.frames_ok == 1u && st.frames_bad == 0u);
    CHECK(WebInput_Get(&st, &raw, 600u, 500u) == true);
    CHECK(raw.vx == 0x5A5A && raw.vy == 0x5A5A && raw.omega == 0x5A5A);
    CHECK(raw.keys == 0x5A5Au && raw.estop == 0x5Au);   /* 照单全收，语义归上层 */
}

/* 错帧不动快照：窗内上帧数据保持可读（与失联语义分离），坏帧计数独立 */
static void TestBadFrameKeepsSnapshot(void)
{
    WebInput_State_t st;
    RcInput_Raw_t    raw;
    uint8_t          f[WEBINPUT_FRAME_LEN];

    WebInput_Init(&st);
    (void)PackFrame(f, 1u, 500, 0, -250, 0u, 0u);
    WebInput_Feed(&st, f, WEBINPUT_FRAME_LEN, 100u);
    f[11] ^= 0x01u;                             /* 同帧坏校验再来一发 */
    WebInput_Feed(&st, f, WEBINPUT_FRAME_LEN, 200u);
    CHECK(st.frames_bad == 1u && st.frames_ok == 1u);
    CHECK(WebInput_Get(&st, &raw, 550u, 500u) == true);  /* 快照仍是 100ms 帧 */
    CHECK(raw.vx == 500 && raw.omega == -250);
}

/* NULL 安全：不崩、无副作用 */
static void TestNullSafety(void)
{
    WebInput_Feed(NULL, (const uint8_t *)"\x5A", 1u, 0u);       /* 不崩即可 */
    WebInput_Init(NULL);

    WebInput_State_t st;
    RcInput_Raw_t    raw;
    WebInput_Init(&st);
    WebInput_Feed(&st, NULL, 5u, 0u);           /* 空数据等于没喂 */
    CHECK(st.frames_ok == 0u && st.frames_bad == 0u);
    CHECK(WebInput_Get(NULL, &raw, 0u, 500u) == false);
    CHECK(WebInput_Get(&st, NULL, 0u, 500u) == false);
}

void TestWebInput_Run(void)
{
    printf("-- web_input\n");
    TestCrc8Vector();
    TestGoodFrame();
    TestTimeWrap();
    TestChunkedFeed();
    TestGarbagePrefix();
    TestResyncAfterBadCrc();
    TestHeaderByteInPayload();
    TestBadFrameKeepsSnapshot();
    TestNullSafety();
}
