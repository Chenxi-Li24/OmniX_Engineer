#include "bsp_srn_log.h"
#include <stdio.h>
#include <string.h>

/* 由 CubeMX 生成的全局句柄（在 app_freertos.c 等处） */
extern osMessageQueueId_t Log_QueueHandle;   /* Queue item size = 4 (uint32_t) */
extern osMemoryPoolId_t   Log_PoolHandle;    /* Block size = sizeof(srn_log_msg_t) */

static osMessageQueueId_t s_log_q    = NULL; /* 实际使用的队列（存“指针”） */
static osMemoryPoolId_t   s_log_pool = NULL; /* 内存池（存消息块） */

void SRN_Log_Init(void)
{
    /* 毫秒时间基，无需额外初始化；保留扩展点 */
}

void SRN_Log_BindQueue(osMessageQueueId_t q) { s_log_q = q; }
void SRN_Log_BindPool (osMemoryPoolId_t   p) { s_log_pool = p; }

static inline const char* srn_get_source(const char* user_src)
{
    if (user_src && user_src[0]) return user_src;
    if (xPortIsInsideInterrupt() == pdTRUE) return "System";
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    const char* nm = h ? pcTaskGetName(h) : NULL;
    return (nm && nm[0]) ? nm : "System";
}

/* HAL_GetTick() 的 32 位值扩展为 64 位单调毫秒（49天回绕→64位高位计数） */
static inline uint64_t srn_capture_ms64(void)
{
    static uint32_t last32 = 0;
    static uint64_t hi     = 0;
    taskENTER_CRITICAL();
    uint32_t cur = HAL_GetTick();
    if (cur < last32) hi += (1ULL << 32);
    last32 = cur;
    uint64_t ms64 = hi | (uint64_t)cur;
    taskEXIT_CRITICAL();
    return ms64;
}

/* 辅助：立即输出一整行（兜底路径使用，包含时间戳/级别/来源） */
static void srn_direct_print_line(srn_log_level_t level, const char* source, const char* fmt, va_list ap)
{
    uint64_t ms64 = srn_capture_ms64();
    uint32_t sec  = (uint32_t)(ms64 / 1000ULL);
    uint32_t ms   = (uint32_t)(ms64 % 1000ULL);

    char tsbuf[20];
    if (sec <= 999999U) snprintf(tsbuf, sizeof(tsbuf), "[%6lu.%03lu]", (unsigned long)sec, (unsigned long)ms);
    else                 snprintf(tsbuf, sizeof(tsbuf), "[%lu.%03lu]",    (unsigned long)sec, (unsigned long)ms);

    const char* src = srn_get_source(source);
    const char* lvl = (level==SRN_LOG_DEBUG)?"DEBUG":
                      (level==SRN_LOG_INFO )?"INFO":
                      (level==SRN_LOG_WARN )?"WARN":"ERROR";

    char prefix[96];
    int n = snprintf(prefix, sizeof(prefix), "%s[%s][%s] ", tsbuf, lvl, src);
    size_t plen = (size_t)((n > 0 && n < (int)sizeof(prefix)) ? n : (int)sizeof(prefix)-1);

    char body[160];
    int bn = vsnprintf(body, sizeof(body), fmt, ap);
    size_t bl = (size_t)((bn > 0 && bn < (int)sizeof(body)) ? bn : (int)sizeof(body)-1);

    USART1_Write(prefix, plen, 0);
    USART1_Write(body,   bl,   0);
    USART1_Write("\r\n", 2,    0);
}

osStatus_t SRN_LogVPrintf(srn_log_level_t level,
                          const char* source,
                          uint32_t timeout_ticks,  /* 忽略：统一非阻塞 */
                          const char* fmt,
                          va_list ap)
{
    (void)timeout_ticks;

    /* 禁止在 ISR 里放队列（有需要请改为任务里打印） */
    if (xPortIsInsideInterrupt() == pdTRUE) {
        return osErrorISR;
    }

    /* 自动重绑定，防止被踩空 */
    if (s_log_q == NULL && Log_QueueHandle != NULL)   s_log_q    = Log_QueueHandle;
    if (s_log_pool == NULL && Log_PoolHandle != NULL) s_log_pool = Log_PoolHandle;

    /* 队列或内存池未就绪：兜底直出完整一行 */
    if (s_log_q == NULL || s_log_pool == NULL) {
        va_list ap2; va_copy(ap2, ap);
        srn_direct_print_line(level, source, fmt, ap2);
        va_end(ap2);
        return osOK;
    }

    /* —— 从内存池分配 1 个消息块（不吃调用者栈） —— */
    srn_log_msg_t* node = (srn_log_msg_t*)osMemoryPoolAlloc(s_log_pool, 0);
    if (node == NULL) {
        /* 池满：丢最旧一条（取出指针并 free），再尝试一次 */
        uintptr_t pv_old;
        if (osMessageQueueGet(s_log_q, &pv_old, NULL, 0) == osOK) {
            osMemoryPoolFree(s_log_pool, (void*)pv_old);
        }
        node = (srn_log_msg_t*)osMemoryPoolAlloc(s_log_pool, 0);
        if (node == NULL) {
            /* 仍然拿不到：降级为直出，避免卡死 */
            va_list ap2; va_copy(ap2, ap);
            srn_direct_print_line(level, source, fmt, ap2);
            va_end(ap2);
            return osErrorNoMemory;
        }
    }

    /* 填充块内容（直接写池内存，零拷贝） */
    memset(node, 0, sizeof(*node));
    node->ts_ms = srn_capture_ms64();
    node->level = level;
    const char* src = srn_get_source(source);
    strncpy(node->source, src, SRN_LOG_SRC_MAX - 1);
    node->source[SRN_LOG_SRC_MAX - 1] = '\0';
    vsnprintf(node->body, SRN_LOG_BODY_MAX, fmt, ap);
    node->body[SRN_LOG_BODY_MAX - 1] = '\0';

    /* 非阻塞入队（放“指针”）；满了就丢最旧指针并归还，再投一次 */
    uintptr_t ptr = (uintptr_t)node;
    osStatus_t st = osMessageQueuePut(s_log_q, &ptr, 0, 0);
    if (st == osOK) return osOK;

    uintptr_t pv_old;
    if (osMessageQueueGet(s_log_q, &pv_old, NULL, 0) == osOK) {
        osMemoryPoolFree(s_log_pool, (void*)pv_old);
    }
    return osMessageQueuePut(s_log_q, &ptr, 0, 0);
}

osStatus_t SRN_LogPrintf(srn_log_level_t level,
                         const char* source,
                         uint32_t timeout_ticks,
                         const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    osStatus_t st = SRN_LogVPrintf(level, source, timeout_ticks, fmt, ap);
    va_end(ap);
    return st;
}