#pragma once
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== FreeRTOS（可选）===== */
#include "FreeRTOS.h"
#include "task.h"

/* ===== D-Cache/MPU 非缓存段（与 bsp_dr16 同风格）===== */
#ifdef OMNIX_FEATURE_MPU_DCACHE
  #if __has_include("mem_sections.h")
    #include "mem_sections.h"
    #define VT03_DMA_SEC  SEC_DMA_NC_BUF   /* 内含 aligned(32) */
  #else
    #define VT03_DMA_SEC
  #endif
#else
  #define VT03_DMA_SEC
#endif

/* ==================== Config knobs ==================== */
/* To-Idle DMA 分段缓冲（和 DR16 同理念，足够容纳多帧） */
#ifndef VT03_DMA_BUF_SIZE
# define VT03_DMA_BUF_SIZE   (128u)
#endif

/* ISR → Task 单生产者环形 FIFO（字节级），必须 2^n */
#ifndef VT03_ISR_FIFO_SIZE
# define VT03_ISR_FIFO_SIZE  (1024u)
#endif
#if ((VT03_ISR_FIFO_SIZE & (VT03_ISR_FIFO_SIZE - 1)) != 0)
# error "VT03_ISR_FIFO_SIZE must be a power of two"
#endif

/* 硬件/软件 CRC 切换；硬件需在 MX 中已启用 CRC 外设且按注释配置 */
#ifndef VT03_USE_HWCRC
# define VT03_USE_HWCRC 0
#endif

/* CRC 端序：文档默认大端（高字节在前），如实机为小端改为 0 */
#ifndef VT03_CRC_BIGENDIAN
# define VT03_CRC_BIGENDIAN 1
#endif

/* ==================== 协议常量 ==================== */
#define VT03_FRAME_LEN       21u
#define VT03_SOF0            0xA9u
#define VT03_SOF1            0x53u

/* ==================== 类型定义（与 lib_remote_control 解耦的原始态） ==================== */
typedef struct vt03_dec_s {
    /* 4 路 11bit 原始值：范围 364..1684，中值 1024 */
    uint16_t ch[4];

    /* 档位/功能位 */
    uint8_t  gear;          /* 0,1,2 */
    uint8_t  pause;         /* 0/1 */
    uint8_t  fn1;           /* 0/1 */
    uint8_t  fn2;           /* 0/1 */

    uint16_t wheel;         /* 11bit: 364..1684 */
    uint8_t  dial;          /* 拨机键（76bit处），0/1 */

    /* 鼠标 */
    int16_t  mx;            /* 80..95  */
    int16_t  my;            /* 96..111 */
    int16_t  mz;            /* 112..127 */
    uint8_t  ml;            /* 左键（取最低位） */
    uint8_t  mr;            /* 右键（取最低位） */
    uint8_t  mm;            /* 中键（取最低位） */

    /* 键盘 16bit */
    uint16_t keys;

    /* CRC */
    uint16_t crc_frame;     /* 帧携带 */
    uint16_t crc_calc;      /* 本地计算 */
} vt03_dec_t;

/* ISR 分段回调类型（与 DR16 一致的风格） */
typedef void (*vt03_rx_callback_t)(const uint8_t* data, uint16_t len);

/* ======== BSP 层（与 bsp_dr16 对齐的接口）======== */
void VT03_BSP_Init(void);                      /* 注册 HAL 回调但不启动 DMA */
void VT03_BSP_Start(void);                     /* 启动 To-Idle DMA */
void VT03_Rearm(void);                         /* 重新启动 DMA（错误后） */
void VT03_RegisterCallback(vt03_rx_callback_t cb);

/* ======== 轻量帧处理 ======== */
typedef struct {
    uint8_t buf[VT03_FRAME_LEN];
    uint8_t len;
} vt03_framer_t;

typedef union {
    uint8_t  b[VT03_FRAME_LEN];
} vt03_raw_t;

/* 基础工具 */
void    VT03_FramerInit(vt03_framer_t* f);
size_t  VT03_FramerFeed(vt03_framer_t* f,
                        const uint8_t* data, size_t len,
                        vt03_raw_t* out_frames, size_t max_frames);

/* 解析：返回 1=成功 0=失败（含 CRC/帧头检查） */
int     VT03_Decode21(const uint8_t* s, vt03_dec_t* d);

/* 高层轮询（任务上下文）：把最新一帧解析为 vt03_dec_t；无新帧返回 0 */
int     VT03_PollDec(vt03_dec_t* out, uint32_t now_ms);

void VT03_Init(void);