#include "bsp_dr16.h"
#include "usart.h"          /* extern UART_HandleTypeDef huart6 */
#include <string.h>

/* ====== FreeRTOS (optional) ====== */
#include "FreeRTOS.h"
#include "task.h"

/* ====== D-Cache feature & header detection (order-agnostic) ====== */
#ifdef OMNIX_FEATURE_MPU_DCACHE
  #if __has_include("mem_sections.h")
    #include "mem_sections.h"
    /* Use non-cacheable buffer when D-Cache module is present. */
    #define DR16_DMA_SEC  SEC_DMA_NC_BUF   /* carries aligned(32) inside */
  #else
    #define DR16_DMA_SEC
  #endif
#else
  #define DR16_DMA_SEC
#endif

/* ==================== Config knobs ==================== */
#ifndef DR16_DMA_BUF_SIZE
# define DR16_DMA_BUF_SIZE   (64u)        /* RX staging buffer size for To-Idle DMA */
#endif

/* FIFO size must be power-of-two for fast modulo. */
#ifndef DR16_ISR_FIFO_SIZE
# define DR16_ISR_FIFO_SIZE  (1024u)      /* bytes; power of two */
#endif

#if ((DR16_ISR_FIFO_SIZE & (DR16_ISR_FIFO_SIZE - 1)) != 0)
# error "DR16_ISR_FIFO_SIZE must be a power of two"
#endif

/* ==================== DMA staging buffer (non-cacheable) ==================== */
/* Keep it in non-cacheable RAM to avoid manual cache maintenance. */
DR16_DMA_SEC __attribute__((aligned(32)))
static uint8_t s_dr16_dma_buf[DR16_DMA_BUF_SIZE];

/* User-provided ISR chunk hook (kept for BSP compatibility) */
static dr16_rx_callback_t s_user_cb = NULL;

/* ==================== Internal: start To-Idle DMA ==================== */
static void dr16_start_receive(void)
{
    /* Start RX DMA in To-Idle mode. */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_dr16_dma_buf, DR16_DMA_BUF_SIZE);

    /* Disable HT (and optionally TC) interrupts to reduce ISR churn. */
    if (huart6.hdmarx) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
        /* If you rely solely on IDLE event, TC can be disabled too: */
        /* __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_TC); */
    }
}

/* ==================== HAL Callbacks (ISR context) ==================== */
static void DR16_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart != &huart6) return;

    if (Size == 0 || Size > DR16_DMA_BUF_SIZE) {
        dr16_start_receive();
        return;
    }

    /* Hand raw bytes to user (ISR context) */
    if (s_user_cb) {
        s_user_cb(s_dr16_dma_buf, Size);
    }

    /* Re-arm To-Idle reception */
    dr16_start_receive();
}

static void DR16_HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart6) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        dr16_start_receive();
    }
}

/* ==================== Public BSP API (compat) ==================== */
void DR16_BSP_Init(void)
{
    s_user_cb = NULL;
    HAL_UART_RegisterRxEventCallback(&huart6, DR16_HAL_UARTEx_RxEventCallback);
    HAL_UART_RegisterCallback(&huart6, HAL_UART_ERROR_CB_ID, DR16_HAL_UART_ErrorCallback);
}

void DR16_BSP_Start(void)
{
    dr16_start_receive();
}

void DR16_Rearm(void)
{
    dr16_start_receive();
}

void DR16_RegisterCallback(dr16_rx_callback_t cb)
{
    s_user_cb = cb;
}

/* ==================== Lightweight framing / decode ==================== */
static inline uint16_t u16(const uint8_t* s) { return (uint16_t)(s[0] | (s[1] << 8)); }

int DR16_FrameSanity(const uint8_t* s)
{
    if (!s) return 0;
    uint16_t ch0 = (uint16_t)((s[0] | (s[1] << 8)) & 0x07FF);
    uint8_t  s1  = (uint8_t)((s[5] >> 4) & 0x03);
    uint8_t  s2  = (uint8_t)((s[5] >> 6) & 0x03);
    if (ch0 < DR16_RC_MIN || ch0 > DR16_RC_MAX) return 0;
    if (!(s1 >= 1 && s1 <= 3)) return 0;
    if (!(s2 >= 1 && s2 <= 3)) return 0;
    return 1;
}

void DR16_Decode18(const uint8_t* s, dr16_dec_t* d)
{
    if (!s || !d) return;

    d->ch[0] = (int16_t)((s[0] | (s[1] << 8)) & 0x07FF);
    d->ch[1] = (int16_t)(((s[1] >> 3) | (s[2] << 5)) & 0x07FF);
    d->ch[2] = (int16_t)(((s[2] >> 6) | (s[3] << 2) | (s[4] << 10)) & 0x07FF);
    d->ch[3] = (int16_t)(((s[4] >> 1) | (s[5] << 7)) & 0x07FF);

    d->s1    = (uint8_t)((s[5] >> 4) & 0x03);
    d->s2    = (uint8_t)((s[5] >> 6) & 0x03);

    d->mx    = (int16_t)u16(&s[6]);
    d->my    = (int16_t)u16(&s[8]);
    d->mz    = (int16_t)u16(&s[10]);

    d->ml    = s[12];
    d->mr    = s[13];
    d->keys  = (uint16_t)u16(&s[14]);

    /* ch4 (wheel/roll): last two bytes, 11-bit like other channels */
    d->ch[4] = (int16_t)((s[16] | (s[17] << 8)) & 0x07FF);
}

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

void DR16_Normalize(const dr16_dec_t* in, dr16_norm_t* out, float deadzone)
{
    if (!in || !out) return;
    if (deadzone < 0.0f) deadzone = 0.0f;

    for (int i = 0; i < 5; ++i) {
        float v = ((float)in->ch[i] - (float)DR16_RC_OFFSET) / (float)DR16_RC_SCALE;
        v = clampf(v, -1.2f, 1.2f);
        if (v > -deadzone && v < deadzone) v = 0.0f;
        out->ch[i] = clampf(v, -1.0f, 1.0f);
    }
    out->s1 = in->s1;
    out->s2 = in->s2;
    out->mx = in->mx;
    out->my = in->my;
    out->mz = in->mz;
    out->mouse.L = (in->ml != 0);
    out->mouse.R = (in->mr != 0);
    out->keys = in->keys;
}

/* ==================== Framer ==================== */
void DR16_FramerInit(dr16_framer_t* f)
{
    if (!f) return;
    f->len = 0;
}

size_t DR16_FramerFeed(dr16_framer_t* f,
                       const uint8_t* data, size_t len,
                       dr16_raw_t* out_frames, size_t max_frames)
{
    if (!f || !data || !out_frames || max_frames == 0) return 0;

    size_t out_count = 0;
    for (size_t i = 0; i < len; ++i)
    {
        /* Fill buffer until 18B */
        if (f->len < DR16_FRAME_LEN) {
            f->buf[f->len++] = data[i];
            if (f->len < DR16_FRAME_LEN) continue;
        }

        /* Now f->len == 18, test sanity */
        if (DR16_FrameSanity(f->buf)) {
            /* Emit one frame (byte-safe, avoid memcpy) */
            for (int j = 0; j < DR16_FRAME_LEN; ++j) {
                out_frames[out_count].b[j] = f->buf[j];
            }
            out_count++;
            f->len = 0; /* reset for next frame */
            if (out_count >= max_frames) break;
        } else {
            /* Desync: drop first byte, shift left, keep last 17 */
            for (int j = 0; j < DR16_FRAME_LEN - 1; ++j) {
                f->buf[j] = f->buf[j + 1];
            }
            f->len = DR16_FRAME_LEN - 1;
        }
    }
    return out_count;
}

/* ==================== ISR-safe byte FIFO (SPSC) ==================== */
/* Single producer (ISR) / single consumer (task). No memcpy, byte-wise only. */
static volatile uint8_t  s_isr_fifo[DR16_ISR_FIFO_SIZE];
static volatile uint16_t s_isr_head = 0;
static volatile uint16_t s_isr_tail = 0;

static inline uint16_t _fifo_free_nolock(void)
{
    uint16_t h = s_isr_head, t = s_isr_tail;
    return (uint16_t)(DR16_ISR_FIFO_SIZE - ((uint16_t)(h - t) & (DR16_ISR_FIFO_SIZE - 1)) - 1);
}

static inline void _fifo_push_byte(uint8_t b)
{
    uint16_t h = s_isr_head;
    s_isr_fifo[h] = b;
    s_isr_head = (uint16_t)((h + 1) & (DR16_ISR_FIFO_SIZE - 1));
}

static size_t _fifo_pop_bytes(uint8_t* out, size_t maxlen)
{
    size_t n = 0;
    while (n < maxlen)
    {
        uint16_t t = s_isr_tail;
        if (t == s_isr_head) break; /* empty */
        out[n++] = s_isr_fifo[t];
        s_isr_tail = (uint16_t)((t + 1) & (DR16_ISR_FIFO_SIZE - 1));
    }
    return n;
}

/* ISR chunk hook: push into FIFO (no memcpy, no alignment assumptions). */
static void _dr16_isr_chunk_cb(const uint8_t* data, uint16_t len)
{
    /* Byte-wise copy to avoid any unaligned wide access in ISR. */
    uint16_t free = _fifo_free_nolock();
    uint16_t wr = (len <= free) ? len : free;  /* drop overflow silently or count it */
    for (uint16_t i = 0; i < wr; ++i) {
        _fifo_push_byte(data[i]);
    }
    /* No explicit yield here; consumer is non-blocking poll in task context. */
}

/* ==================== High-level Integration (no StreamBuffer) ==================== */
static dr16_framer_t s_framer;

void DR16_Init(void)
{
    /* Prepare framer and FIFO. */
    DR16_FramerInit(&s_framer);
    s_isr_head = s_isr_tail = 0;

    /* Bring up BSP and bind ISR -> FIFO. */
    DR16_BSP_Init();
    DR16_RegisterCallback(_dr16_isr_chunk_cb);
    DR16_BSP_Start();
}

/* Task-context poll: decode newest valid frame into dr16_dec_t (raw, center=1024) */
int DR16_PollDec(dr16_dec_t* out, uint32_t now_ms)
{
    (void)now_ms;

    uint8_t buf[128];
    size_t n = _fifo_pop_bytes(buf, sizeof(buf)); /* non-blocking */
    if (n == 0) return 0;

    dr16_raw_t frames[2];
    size_t got = DR16_FramerFeed(&s_framer, buf, n, frames, 2);
    if (got == 0) return 0;

    if (out) {
        DR16_Decode18(frames[got - 1].b, out); /* decode the newest frame */
    }
    return 1;
}
