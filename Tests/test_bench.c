/**
 * @file    test_bench.c
 * @brief   测试台调度器：强覆盖 freertos.c 的 __weak StartApp_TestBench_Task
 *
 * 任务形态：初始化选中测试一次，随后 10ms 轮询其 poll()；每 2s 打印心跳
 * 与 uxTaskGetStackHighWaterMark（栈余量自证，words 单位）。
 * 新增测试三步：写 Tests/test_xxx.c（Init/Poll）→ 本文件登记表追加一行
 * → robot_config.h 的 ROBOT_TEST_BENCH 改成对应编号。
 */
#include "test_bench.h"
#include "cmsis_os.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_gpio.h"

/* 各测试项入口（Tests/test_xxx.c 提供） */
typedef struct
{
    TestBenchId_t id;
    const char   *name;
    void        (*init)(void);
    void        (*poll)(void);
} TestEntry_t;

void Test_Gpio_Init(void);
void Test_Gpio_Poll(void);
void Test_Encoder_Init(void);
void Test_Encoder_Poll(void);
void Test_Spi_Init(void);
void Test_Spi_Poll(void);
void Test_Pwm_Init(void);
void Test_Pwm_Poll(void);
void Test_Remote_Init(void);
void Test_Remote_Poll(void);

static const TestEntry_t s_table[] = {
    { TEST_BENCH_GPIO,    "gpio",    Test_Gpio_Init,    Test_Gpio_Poll    },
    { TEST_BENCH_ENCODER, "encoder", Test_Encoder_Init, Test_Encoder_Poll },
    { TEST_BENCH_SPI,     "spi",     Test_Spi_Init,     Test_Spi_Poll     },
    { TEST_BENCH_PWM,     "pwm",     Test_Pwm_Init,     Test_Pwm_Poll     },
    { TEST_BENCH_REMOTE,  "remote",  Test_Remote_Init,  Test_Remote_Poll  },
};

bool TestBench_Active(void)
{
    return (TestBenchId_t)ROBOT_TEST_BENCH != TEST_BENCH_NONE;
}

void TestBench_Yield(const char *task_name)
{
    if (!TestBench_Active())
    {
        return;
    }
    Log_Printf("[TB] '%s' 让位（测试台激活，外设归测试台）\r\n", task_name);
    for (;;)
    {
        osDelay(60000u);    /* 永久挂起的可移植写法（不依赖 vTaskSuspend 句柄） */
    }
}

/* 心跳：LED1 1Hz 翻转（调度器与任务链路的存活证据，S0 惯例）+ 2s 日志心跳 */
static void Heartbeat(uint32_t now_ms)
{
    static uint32_t s_last_toggle;
    static uint32_t s_last_log;
    static bool     s_led;

    if ((now_ms - s_last_toggle) >= 500u)
    {
        s_last_toggle = now_ms;
        s_led = !s_led;
        (s_led ? Gpio_Set : Gpio_Reset)(PIN_LED1);
    }
    if ((now_ms - s_last_log) >= 2000u)
    {
        s_last_log = now_ms;
        Log_Printf("[TB] beat t=%ums bench=%d", (unsigned)now_ms, (int)ROBOT_TEST_BENCH);
#if INCLUDE_uxTaskGetStackHighWaterMark
        Log_Printf(" hwm=%uw", (unsigned)uxTaskGetStackHighWaterMark(NULL));
#endif
        Log_Printf("\r\n");
    }
}

void StartApp_TestBench_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);      /* 等 Bsp/Log 初始化 */

    TestBenchId_t id = (TestBenchId_t)ROBOT_TEST_BENCH;
    const TestEntry_t *entry = NULL;
    for (unsigned i = 0u; i < sizeof(s_table) / sizeof(s_table[0]); i++)
    {
        if (s_table[i].id == id)
        {
            entry = &s_table[i];
            break;
        }
    }

    if (id != TEST_BENCH_NONE && entry == NULL)
    {
        Log_Printf("[TB] ROBOT_TEST_BENCH=%d 无此项，退化为心跳\r\n", (int)id);
    }
    else if (entry != NULL)
    {
        Log_Printf("[TB] run '%s'\r\n", entry->name);
        entry->init();
    }

    for (;;)
    {
        if (entry != NULL)
        {
            entry->poll();
        }
        Heartbeat(Bsp_GetMs());
        osDelay(10u);
    }
}
