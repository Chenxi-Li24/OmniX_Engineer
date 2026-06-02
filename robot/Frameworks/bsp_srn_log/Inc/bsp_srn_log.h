#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SRN_LOG_DEBUG = 0,
    SRN_LOG_INFO  = 1,
    SRN_LOG_WARN  = 2,
    SRN_LOG_ERROR = 3,
} srn_log_level_t;

#ifndef SRN_LOG_SRC_MAX
#define SRN_LOG_SRC_MAX   24u
#endif
#ifndef SRN_LOG_BODY_MAX
#define SRN_LOG_BODY_MAX  256u
#endif

/* 日志消息内容（真正的数据块，存放在 MemoryPool 的一个 block 里） */
typedef struct {
    uint64_t        ts_ms;               /* 64-bit 单调毫秒（HAL_GetTick 扩展） */
    srn_log_level_t level;
    char            source[SRN_LOG_SRC_MAX];
    char            body[SRN_LOG_BODY_MAX];
    uint8_t         _pad[3];             /* 对齐 */
} srn_log_msg_t;

/* 初始化（保留扩展；当前仅毫秒时间基，无 DWT） */
void SRN_Log_Init(void);

/* 绑定 CubeMX 创建的队列和内存池（建议在 MX_FREERTOS_Init 之后调用一次） */
void SRN_Log_BindQueue(osMessageQueueId_t q);
void SRN_Log_BindPool (osMemoryPoolId_t   p);

/* printf 风格 API（在调用点捕获时间戳、将“指针”放入队列；非阻塞，满了丢最旧） */
osStatus_t SRN_LogPrintf(srn_log_level_t level,
                         const char* source,
                         uint32_t timeout_ticks,   /* 保留但忽略，统一非阻塞 */
                         const char* fmt, ...);

osStatus_t SRN_LogVPrintf(srn_log_level_t level,
                          const char* source,
                          uint32_t timeout_ticks,   /* 保留但忽略 */
                          const char* fmt,
                          va_list ap);

/* 便捷宏（可自行调整默认超时；这里忽略） */
#define LOGD(fmt, ...)  SRN_LogPrintf(SRN_LOG_DEBUG, NULL, 0, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...)  SRN_LogPrintf(SRN_LOG_INFO,  NULL, 0, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...)  SRN_LogPrintf(SRN_LOG_WARN,  NULL, 0, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...)  SRN_LogPrintf(SRN_LOG_ERROR, NULL, 0, fmt, ##__VA_ARGS__)

/* 串口底层（你已有） */
size_t USART1_Write(const void* data, size_t len, TickType_t timeout);
size_t USART1_WriteStr(const char* s, TickType_t timeout);

#ifdef __cplusplus
}
#endif