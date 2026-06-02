#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= Configuration =================
 * USART6 RX via DMA1_Stream3 assumed configured by CubeMX.
 * DMA buffer size per shot.
 */
#ifndef DR16_DMA_BUF_SIZE
#define DR16_DMA_BUF_SIZE 256u
#endif

/* RC frame basics (DJI/DR16 18B layout) */
#define DR16_FRAME_LEN            18u
#define DR16_RC_OFFSET            1024
#define DR16_RC_MIN               364
#define DR16_RC_MAX               1684
#define DR16_RC_SCALE             660.0f   /* approx to map to [-1,1] */

/* Optional normalization params (can be overridden) */
#ifndef DR16_NORM_DEADZONE
#define DR16_NORM_DEADZONE        0.03f
#endif

/* If .dma_buffer is mapped to non-cacheable SRAM via MPU/LD,
 * define DR16_DMA_NONCACHE=1 in target to skip D-Cache maintenance.
 */
// #define DR16_DMA_NONCACHE 1

/* ================= BSP (UART RX) API ================= */

/* User chunk callback (ISR context). Keep it short. */
typedef void (*dr16_rx_callback_t)(const uint8_t* data, uint16_t len);

/* Initialize internal state (no RTOS dependency). */
void DR16_BSP_Init(void);

/* Start UART6 RX using DMA To-Idle (call once after MX_USART6_UART_Init()). */
void DR16_BSP_Start(void);

/* Register user ISR callback (optional). */
void DR16_RegisterCallback(dr16_rx_callback_t cb);

/* Manually re-arm DMA To-Idle (rare). */
void DR16_Rearm(void);

/* ================= Decode / Normalize API ================= */

typedef struct __attribute__((packed)) {
    uint8_t b[DR16_FRAME_LEN];
} dr16_raw_t;

/* Decoded fields (not normalized; raw center=1024) */
typedef struct dr16_dec_s {
    int16_t ch[5];     /* ch0..4, raw [364..1684], center 1024 */
    uint8_t s1;        /* 1:up 3:mid 2:down */
    uint8_t s2;        /* 1:up 3:mid 2:down */
    int16_t mx, my, mz;
    uint8_t ml, mr;    /* mouse buttons (0/1) */
    uint16_t keys;     /* keyboard bitmask */
} dr16_dec_t;

/* Friendly normalized output (-1..1); optional helper */
typedef struct {
    float   ch[5];     /* normalized [-1..1] with dead-zone */
    uint8_t s1, s2;    /* 1/2/3 */
    int16_t mx, my, mz;
    struct { uint8_t L:1; uint8_t R:1; } mouse;
    uint16_t keys;
} dr16_norm_t;

int  DR16_FrameSanity(const uint8_t* frame18);
void DR16_Decode18(const uint8_t* frame18, dr16_dec_t* out);
void DR16_Normalize(const dr16_dec_t* in, dr16_norm_t* out, float deadzone);

/* ================= Lightweight Framer ================= */
typedef struct {
    uint8_t buf[DR16_FRAME_LEN];
    uint8_t len;     /* bytes currently in buf */
} dr16_framer_t;

void   DR16_FramerInit(dr16_framer_t* f);
size_t DR16_FramerFeed(dr16_framer_t* f,
                       const uint8_t* data, size_t len,
                       dr16_raw_t* out_frames, size_t max_frames);

/* ================= High-level Integration (no rc_input dependency) =================
 * 创建内部 StreamBuffer，绑定 ISR 回调，开始 To-Idle。
 * 任务态调用 DR16_PollDec() 取“最新一帧的解码结果”（raw中心=1024）。
 */
void DR16_Init(void);
int  DR16_PollDec(dr16_dec_t* out, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
