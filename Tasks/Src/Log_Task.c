#include "bsp_srn_log.h"
#include "bsp_usart1_dma.h"
#include "log_pool.h"
#include <string.h>
#include <stdio.h>

/* CubeMX 生成的对象 */
extern osMessageQueueId_t Log_QueueHandle;   /* Item size = 4 (uint32_t) */
extern osMemoryPoolId_t   Log_PoolHandle;    /* Block size = sizeof(srn_log_msg_t) */

/*（可选）编译期保险：检查 srn_log_msg_t 大小是否与 CubeMX Pool 配置一致 */
_Static_assert(sizeof(srn_log_msg_t) >= 280 && sizeof(srn_log_msg_t) <= 512,
               "Check Log_Pool Block size == sizeof(srn_log_msg_t)");

/* 级别转字符串 */
static inline const char* lvl_str(srn_log_level_t lvl)
{
    switch (lvl) {
        case SRN_LOG_DEBUG: return "DEBUG";
        case SRN_LOG_INFO:  return "INFO";
        case SRN_LOG_WARN:  return "WARN";
        case SRN_LOG_ERROR: return "ERROR";
        default:            return "UNK";
    }
}

/* 把 64 位毫秒转为 "[xxxxxx.nnn]"：
 * - 整数秒 <= 999999 时，宽 6 右对齐，不足空格；
 * - 超过则不限制宽度；
 * - 小数固定 3 位（毫秒）。
 */
/* 把 64 位毫秒格式化为 "[xxxxxx.nnn]"（整数秒≤999999则右对齐宽6，不足空格） */
static void fmt_ts_ms(char* out, size_t out_sz, uint64_t ms64)
{
    uint32_t sec = (uint32_t)(ms64 / 1000ULL);
    uint32_t ms  = (uint32_t)(ms64 % 1000ULL);
    if (sec <= 999999U) snprintf(out, out_sz, "[%6lu.%03lu]", (unsigned long)sec, (unsigned long)ms);
    else                 snprintf(out, out_sz, "[%lu.%03lu]",  (unsigned long)sec, (unsigned long)ms);
}

void Start_Log_Task(void *argument)
{
    (void)argument;

    /* 绑定句柄（防止 s_log_q/s_log_pool 未初始化） */
    SRN_Log_BindQueue(Log_QueueHandle);
    SRN_Log_BindPool (Log_PoolHandle);

    /* 把本任务注册为 DMA 续发服务任务 */
    USART1_DMA_RegisterServiceTask(xTaskGetCurrentTaskHandle());

    const char *boot = "[  UNKNOW  ][INFO][LOG_Task] Log_Task Started.\r\n";
    (void)USART1_Write(boot, strlen(boot), pdMS_TO_TICKS(50));


    char tsbuf[20];
    char prefix[96];


    for (;;) {
        /* 处理 DMA 完成通知，并尽快续发 */
        (void)ulTaskNotifyTake(pdTRUE, 0);
        USART1_DMA_ServiceOnce();

        /* 取“指针队列”中的一条消息 */
        uintptr_t pv;
        if (osMessageQueueGet(Log_QueueHandle, &pv, NULL, pdMS_TO_TICKS(10)) == osOK) {
            srn_log_msg_t* m = (srn_log_msg_t*)pv;

            fmt_ts_ms(tsbuf, sizeof(tsbuf), m->ts_ms);
            int n = snprintf(prefix, sizeof(prefix), "%s[%s][%s] ",
                             tsbuf, lvl_str(m->level), m->source);
            size_t plen = (size_t)((n > 0 && n < (int)sizeof(prefix)) ? n : (int)sizeof(prefix)-1);

            (void)USART1_Write(prefix, plen,  pdMS_TO_TICKS(50));
            (void)USART1_Write(m->body, strnlen(m->body, SRN_LOG_BODY_MAX), pdMS_TO_TICKS(50));
            (void)USART1_Write("\r\n", 2,     pdMS_TO_TICKS(50));

            /* 用完必须归还到内存池 */
            osMemoryPoolFree(Log_PoolHandle, m);
        }

        /* 再尝试一次，把 TX 队列抽干 */
        USART1_DMA_ServiceOnce();
    }
}