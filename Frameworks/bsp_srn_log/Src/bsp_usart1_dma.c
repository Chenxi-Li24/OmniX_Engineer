// bsp_usart1_dma.c  —— 仅在本文件内改动，去除 StreamBuffer 出队引发的 UNALIGNED
#include "bsp_usart1_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdbool.h>

/* === D-Cache / 区段检测 === */
#ifdef OMNIX_FEATURE_MPU_DCACHE
  #if __has_include("mem_sections.h")
    #include "mem_sections.h"
    #define U1_DMA_SEC   SEC_DMA_NC_BUF   /* 非缓存区，且自带 ALIGN(32) */
  #else
    #define U1_DMA_SEC   __attribute__((aligned(32)))
  #endif
#else
  #define U1_DMA_SEC     __attribute__((aligned(32)))
#endif

/* === CubeMX 生成的句柄 === */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_tx;

/* ================= 用户可调参数 ================= */
#ifndef U1_TX_RING_SIZE
#define U1_TX_RING_SIZE        (8192u)     // 发送总队列容量（替代原 U1_TX_STREAMBUF_SIZE）
#endif
#ifndef U1_DMA_WORKBUF_SIZE
#define U1_DMA_WORKBUF_SIZE    (1024u)     // 每次搬运到 DMA 的块大小
#endif
/* =============================================== */

/* ============ 内部状态（静态分配，无动态内存） ============ */
static uint8_t         s_tx_ring[U1_TX_RING_SIZE];
static volatile size_t s_rb_head = 0;   // 写指针
static volatile size_t s_rb_tail = 0;   // 读指针

/* DMA 源缓冲：非缓存区/32B对齐 */
static uint8_t s_dma_workbuf[U1_DMA_WORKBUF_SIZE] U1_DMA_SEC;
static volatile bool        s_dma_busy = false;

static TaskHandle_t         s_service_task = NULL;  // 由上层(Log_Task)注册
/* ======================================================== */

/* ============ 本文件私有工具：逐字节 memcpy，避免对齐问题 ============ */
static inline void *memcpy8(void *dst, const void *src, size_t n)
{
    uint8_t *d = (uint8_t*)dst;
    const uint8_t *s = (const uint8_t*)src;
    for (size_t i = 0; i < n; i++) d[i] = s[i];
    return dst;
}

/* ============ 本文件私有：环形缓冲工具 ============ */
static inline size_t rb_count(void)
{
    size_t h = s_rb_head, t = s_rb_tail;
    return (h >= t) ? (h - t) : (U1_TX_RING_SIZE - (t - h));
}

static inline size_t rb_space(void)
{
    /* 预留 1 字节区分“满/空” */
    return U1_TX_RING_SIZE - 1 - rb_count();
}

/* 把用户数据写入环形（两段拷贝） */
static size_t rb_write(const uint8_t *src, size_t len)
{
    size_t space = rb_space();
    if (len > space) len = space;
    size_t h = s_rb_head;

    size_t n1 = U1_TX_RING_SIZE - h;   // 到尾部的连续空间
    if (n1 > len) n1 = len;
    memcpy8(&s_tx_ring[h], src, n1);

    size_t n2 = len - n1;
    if (n2) memcpy8(&s_tx_ring[0], src + n1, n2);

    s_rb_head = (h + len) % U1_TX_RING_SIZE;
    return len;
}

/* 从环形读最多 len 字节到 dst（两段拷贝），返回实际读出 */
static size_t rb_read(uint8_t *dst, size_t len)
{
    size_t cnt = rb_count();
    if (len > cnt) len = cnt;
    size_t t = s_rb_tail;

    size_t n1 = U1_TX_RING_SIZE - t;   // 到尾部的连续可读
    if (n1 > len) n1 = len;
    memcpy8(dst, &s_tx_ring[t], n1);

    size_t n2 = len - n1;
    if (n2) memcpy8(dst + n1, &s_tx_ring[0], n2);

    s_rb_tail = (t + len) % U1_TX_RING_SIZE;
    return len;
}

/* ISR -> 服务任务通知 */
static inline void prv_notify_service_from_isr(BaseType_t *pxHPW)
{
    if (s_service_task) {
        vTaskNotifyGiveFromISR(s_service_task, pxHPW);
    }
}

/* ============ HAL 回调（ISR 环境，保持极轻） ============ */
static void U1_TX_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) return;
    s_dma_busy = false;

    BaseType_t xHPW = pdFALSE;
    prv_notify_service_from_isr(&xHPW);
    portYIELD_FROM_ISR(xHPW);
}

static void U1_TX_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) return;
    s_dma_busy = false;

    BaseType_t xHPW = pdFALSE;
    prv_notify_service_from_isr(&xHPW);
    portYIELD_FROM_ISR(xHPW);
}

/* ============ 派发一次（任务态调用） ============ */
void USART1_DMA_ServiceOnce(void)
{
    if (s_dma_busy) return;

    /* 从环形取一块到对齐的工作缓冲（逐字节搬运，完全无对齐风险） */
    size_t want = U1_DMA_WORKBUF_SIZE;

    taskENTER_CRITICAL();
    size_t n = rb_read(s_dma_workbuf, want);
    taskEXIT_CRITICAL();

    if (n == 0) return;

    if (HAL_UART_Transmit_DMA(&huart1, s_dma_workbuf, (uint16_t)n) == HAL_OK) {
        s_dma_busy = true;
    }
}

/* ============ 对外 API ============ */
void USART1_DMA_TX_Init(void)
{
    /* 注册 HAL 回调（需要 USE_HAL_UART_REGISTER_CALLBACKS=1U） */
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, U1_TX_TxCpltCallback);
    HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID,      U1_TX_ErrorCallback);

    s_dma_busy = false;
    s_rb_head = s_rb_tail = 0;
}

void USART1_DMA_RegisterServiceTask(TaskHandle_t task)
{
    s_service_task = task;
}

/* 非阻塞写：尽力塞入环形；环满则返回已写入的字节数 */
size_t USART1_Write(const void* data, size_t len, TickType_t timeout)
{
    (void)timeout;  // 当前实现为非阻塞；如需阻塞写，可在空间不足时 vTaskDelay/等待通知
    if (!data || len == 0) return 0;

    const uint8_t* p = (const uint8_t*)data;
    size_t total = 0;

    while (total < len) {
        taskENTER_CRITICAL();
        size_t pushed = rb_write(p + total, len - total);
        taskEXIT_CRITICAL();

        total += pushed;

        /* 提醒服务任务尽快续发；如果尚未注册，则在当前任务里兜底派发一次 */
        if (s_service_task) {
            xTaskNotifyGive(s_service_task);
        } else {
            taskENTER_CRITICAL();
            USART1_DMA_ServiceOnce();
            taskEXIT_CRITICAL();
        }

        if (pushed == 0) { // 队列满
            break;
        }
    }
    return total;
}

size_t USART1_WriteStr(const char* s, TickType_t timeout)
{
    if (!s) return 0;
    size_t n = 0;
    while (s[n] != '\0') n++;
    return USART1_Write(s, n, timeout);
}

int USART1_PutChar(int ch, TickType_t timeout)
{
    uint8_t c = (uint8_t)ch;
    return (USART1_Write(&c, 1, timeout) == 1) ? ch : -1;
}