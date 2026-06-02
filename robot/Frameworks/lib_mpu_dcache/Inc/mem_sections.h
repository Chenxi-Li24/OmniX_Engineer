//
// mem_sections.h
// Non-cacheable DMA sections for STM32H7 (D-Cache line = 32B)
//

#ifndef H723VG_V2_FREERTOS_MEM_SECTIONS_H
#define H723VG_V2_FREERTOS_MEM_SECTIONS_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    /* Toolchain attributes */
#if defined(__GNUC__)
    /* Force 32B alignment to match H7 D-Cache line size. */
#define ALIGN32           __attribute__((aligned(32)))

    /* Place small DMA descriptors (e.g., RX/TX desc) into .dma_nc.
       Linker fragment marks this section as NOLOAD and locates it in NOCACHE_RAM. */
#define SEC_DMA_NC        __attribute__((section(".dma_nc"))) ALIGN32

    /* Place large DMA buffers (e.g., ring buffers) into .dma_nc_buf.
       Also NOLOAD in the linker fragment and in the same NOCACHE_RAM region. */
#define SEC_DMA_NC_BUF    __attribute__((section(".dma_nc_buf"))) ALIGN32
#else
#define ALIGN32
#define SEC_DMA_NC
#define SEC_DMA_NC_BUF
#endif

    /* Optional: linker symbols provided by dma_nc.ld (guarded for link-time only) */
#if defined(__GNUC__)
    extern char __dma_nc_start__[];
    extern char __dma_nc_end__[];
    extern char __dma_nc_buf_start__[];
    extern char __dma_nc_buf_end__[];
#endif

    /* Helper macros to get sizes at runtime (cast to size_t). */
#include <stddef.h>
#define DMA_NC_SIZE()       ((size_t)(__dma_nc_end__      - __dma_nc_start__))
#define DMA_NC_BUF_SIZE()   ((size_t)(__dma_nc_buf_end__  - __dma_nc_buf_start__))

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* H723VG_V2_FREERTOS_MEM_SECTIONS_H */