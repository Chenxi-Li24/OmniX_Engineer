#pragma once
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Optional FreeRTOS dependency ===== */
#include "FreeRTOS.h"
#include "task.h"

/* ===== D-Cache / MPU non-cacheable section, aligned with bsp_dr16 ===== */
#ifdef OMNIX_FEATURE_MPU_DCACHE
  #if __has_include("mem_sections.h")
    #include "mem_sections.h"
    #define VT03_DMA_SEC  SEC_DMA_NC_BUF   /* Includes 32-byte alignment */
  #else
    #define VT03_DMA_SEC
  #endif
#else
  #define VT03_DMA_SEC
#endif

/* ==================== Config knobs ==================== */
/* DMA buffer used by ReceiveToIdle. Large enough to hold multiple frames. */
#ifndef VT03_DMA_BUF_SIZE
# define VT03_DMA_BUF_SIZE   (128u)
#endif

/* ISR-to-task single-producer FIFO. Must be a power of two. */
#ifndef VT03_ISR_FIFO_SIZE
# define VT03_ISR_FIFO_SIZE  (1024u)
#endif
#if ((VT03_ISR_FIFO_SIZE & (VT03_ISR_FIFO_SIZE - 1)) != 0)
# error "VT03_ISR_FIFO_SIZE must be a power of two"
#endif

/* Select software or hardware CRC. Hardware CRC must be configured in CubeMX. */
#ifndef VT03_USE_HWCRC
# define VT03_USE_HWCRC 0
#endif

/* CRC byte order option. Keep vendor default unless the device proves otherwise. */
#ifndef VT03_CRC_BIGENDIAN
# define VT03_CRC_BIGENDIAN 1
#endif

/* ==================== Protocol constants ==================== */
#define VT03_FRAME_LEN       21u
#define VT03_SOF0            0xA9u
#define VT03_SOF1            0x53u

/* ==================== Decoded VT03 data ==================== */
typedef struct vt03_dec_s {
    /* Four 11-bit channels, nominal range 364..1684, centered at 1024 */
    uint16_t ch[4];

    /* Switch / function bits */
    uint8_t  gear;          /* 0,1,2 */
    uint8_t  pause;         /* 0/1 */
    uint8_t  fn1;           /* 0/1 */
    uint8_t  fn2;           /* 0/1 */

    uint16_t wheel;         /* 11-bit: 364..1684 */
    uint8_t  dial;          /* Trigger / dial bit */

    /* Mouse deltas */
    int16_t  mx;
    int16_t  my;
    int16_t  mz;
    uint8_t  ml;            /* Left button, LSB only */
    uint8_t  mr;            /* Right button, LSB only */
    uint8_t  mm;            /* Middle button, LSB only */

    /* Keyboard bitmap */
    uint16_t keys;

    /* CRC values */
    uint16_t crc_frame;     /* CRC carried by the frame */
    uint16_t crc_calc;      /* CRC calculated locally */
} vt03_dec_t;

/* ISR chunk callback type */
typedef void (*vt03_rx_callback_t)(const uint8_t* data, uint16_t len);

typedef struct vt03_diag_s {
    uint8_t started;
    uint8_t reserved[3];
    uint32_t rx_evt_idle_count;
    uint32_t rx_evt_tc_count;
    uint32_t rx_hw_error_count;
    uint32_t rx_recover_count;
    uint32_t rx_boot_flush_count;
    uint32_t rx_boot_stale_flags;
    uint32_t rx_boot_start_ret;
} vt03_diag_t;

/* ======== BSP-facing UART RX API ======== */
void VT03_BSP_Init(void);                      /* Register HAL callbacks only */
void VT03_BSP_Start(void);                     /* Start ReceiveToIdle DMA */
void VT03_Rearm(void);                         /* Restart DMA after an error */
void VT03_RegisterCallback(vt03_rx_callback_t cb);
void VT03_FeedBytesFromISR(const uint8_t* data, uint16_t len);
void VT03_RegisterRawTapCallback(vt03_rx_callback_t cb);
int  VT03_ReadDiag(vt03_diag_t* out);

/* ======== Lightweight framer ======== */
typedef struct {
    uint8_t buf[VT03_FRAME_LEN];
    uint8_t len;
} vt03_framer_t;

typedef union {
    uint8_t  b[VT03_FRAME_LEN];
} vt03_raw_t;

/* Basic helpers */
void    VT03_FramerInit(vt03_framer_t* f);
size_t  VT03_FramerFeed(vt03_framer_t* f,
                        const uint8_t* data, size_t len,
                        vt03_raw_t* out_frames, size_t max_frames);

/* Returns 1 on success, 0 on SOF / CRC failure. */
int     VT03_Decode21(const uint8_t* s, vt03_dec_t* d);

/* Task-context polling helper. Returns 0 when no new frame is available. */
int     VT03_PollDec(vt03_dec_t* out, uint32_t now_ms);

void VT03_Init(void);

#ifdef __cplusplus
}
#endif
