/**
 * @file    remote.h
 * @brief   无线遥控链路层：PRX（车端收包）/ PTX（遥控器发包）双模式 + 链路看门狗
 *
 * 分层与并发模型（BSP层开发规划 四.9 / 五）：
 *  - 本模块是 nRF24 器件知识与上层之间的唯一接口，上层不见寄存器/总线；
 *  - IRQ 行为注入走 bsp_gpio 的 Exti_Attach（规则 1 的唯一干净通道），
 *    回调内仅 osSignalSet（CMSIS_V1 任务信号，ISR 安全，无需信号量内核对象），
 *    帧搬运/解析全部在服务任务侧完成；
 *  - Remote_Init 必须在服务任务上下文调用：内部记录调用者线程作为 IRQ 通知目标
 *    （谁 Init 谁服务），避免裸线程 id 传递；
 *  - 车端（PRX）与遥控器端（PTX）共用本模块，robot_config.h 的
 *    CONFIG_REMOTE_UNIT 编译开关区分两端工程；
 *  - 本文件接口对上层（UserApp）冻结；include cmsis_os.h 是规划四.9/五
 *    明确授权的 Modules 层 RTOS 例外。
 */
#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    REMOTE_MODE_PTX = 0,    /* 遥控器端：周期发包 */
    REMOTE_MODE_PRX = 1,    /* 车端：收包 + 看门狗 */
} RemoteMode_t;

/**
 * @brief  初始化并进入指定模式（SPI 自检 → 寄存器配置回读校验 → 注册 IRQ）。
 * @note   必须在服务任务上下文调用。false = SPI 未就绪 / STATUS 异常 / 配置校验失败。
 */
bool Remote_Init(RemoteMode_t mode);

/**
 * @brief  服务任务循环内反复调用。PRX：阻塞等待 IRQ 信号（无 IRQ 20ms 兜底返回），
 *         内部完成 RX FIFO 搬运、入队与链路时间戳更新；
 *         PTX：等效 20ms 延时（发包走 Remote_SendPacket 阻塞路径）。
 */
void Remote_Service(void);

/**
 * @brief  【PRX】取一帧。true = 新帧拷入 buf。
 * @param  size 须 ≥ ROBOT_REMOTE_PAYLOAD；帧格式由上层约定（首字节为发送序号等）。
 * @note   队列满时丢最旧帧（保最新指令），帧队列深度 ROBOT_REMOTE_RX_QUEUE。
 */
bool Remote_ReadPacket(uint8_t *buf, uint8_t size);

/**
 * @brief  【PTX】发送一帧（≤ ROBOT_REMOTE_PAYLOAD，短帧补零到静态负载宽），
 *         阻塞至 ACK 成功或自动重传耗尽（ROBOT_REMOTE_TX_TIMEOUT_MS 兜底）。
 * @note   协议无状态：MAX_RT 后内部清 FIFO，重发节奏由上层周期驱动。
 */
bool Remote_SendPacket(const uint8_t *buf, uint8_t size);

/**
 * @brief  链路看门狗：true = 距最近一帧 < ROBOT_REMOTE_WATCHDOG_MS。
 *         上层在失联时必须进入安全停机（车端硬性要求）。
 */
bool Remote_IsLinkUp(void);

/* 诊断计数（收包率统计 / PTX 丢包诊断） */
uint32_t Remote_GetRxCount(void);
uint32_t Remote_GetTxFailCount(void);

#endif /* REMOTE_H */
