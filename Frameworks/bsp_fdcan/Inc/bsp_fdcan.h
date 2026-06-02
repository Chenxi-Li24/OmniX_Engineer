//
// Created by sirin on 2025/10/4.
//

#ifndef H723VG_V2_FREERTOS_BSP_FDCAN_H
#define H723VG_V2_FREERTOS_BSP_FDCAN_H

#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Public configuration ===== */
#ifndef BSP_FDCAN_RX_RING_SIZE
#define BSP_FDCAN_RX_RING_SIZE   128u   /* frames per bus */
#endif

#ifndef BSP_FDCAN_ENABLE_RXFIFO1
#define BSP_FDCAN_ENABLE_RXFIFO1 0      /* set 1 if you also use FIFO1 */
#endif

/* ===== Frame container for RX side ===== */
typedef struct {
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t               data[64];    /* CAN-FD max */
    uint32_t              ts_us;       /* local timestamp (approx) */
} bsp_fdcan_rx_frame_t;

/* ===== Error/diag snapshot ===== */
typedef struct {
    volatile uint32_t rx_isr_cnt;
    volatile uint32_t rx_overrun_cnt;
    volatile uint32_t tx_ok_cnt;
    volatile uint32_t tx_busy_cnt;
    volatile uint32_t bus_off_cnt;
    volatile uint32_t last_err_code;   /* FDCAN_PSR.LEC mirror */

    volatile uint32_t last_rx_id;
    volatile uint32_t last_rx_dlc;
    volatile uint32_t last_rx_is_ext;
} bsp_fdcan_diag_t;

/* ===== API ===== */

/* Call after CubeMX's MX_FDCANx_Init() has been executed. */
void bsp_fdcan_init_all(void);
void bsp_fdcan_start_all(void);

/* Start/stop single bus (bus = 1/2/3). */
bool bsp_fdcan_start(uint8_t bus);
void bsp_fdcan_stop(uint8_t bus);

/* Non-blocking TX. Returns true if enqueued to Tx FIFO/Q. */
bool bsp_fdcan_try_tx(uint8_t               bus,
                      const FDCAN_TxHeaderTypeDef *tx_hdr,
                      const uint8_t         *payload);

/* Pop up to max_count frames from RX ring. Returns popped count. */
size_t bsp_fdcan_rx_pop(uint8_t bus,
                        bsp_fdcan_rx_frame_t *out,
                        size_t max_count);

/* Append a Standard ID filter (mask mode by default). fifo=0->FIFO0, 1->FIFO1 */
HAL_StatusTypeDef bsp_fdcan_add_std_filter(uint8_t bus,
                                           uint16_t id,
                                           uint16_t mask,
                                           uint8_t  fifo);

/* Append an Extended ID filter (mask mode). */
HAL_StatusTypeDef bsp_fdcan_add_ext_filter(uint8_t bus,
                                           uint32_t id,
                                           uint32_t mask,
                                           uint8_t  fifo);

/* Read diagnostics snapshot for a bus. */
bsp_fdcan_diag_t bsp_fdcan_get_diag(uint8_t bus);

/* ===== Weak hooks (override in upper layer if needed) ===== */
void __attribute__((weak)) bsp_fdcan_on_rx_isr(uint8_t bus);
void __attribute__((weak)) bsp_fdcan_on_busoff(uint8_t bus);
void __attribute__((weak)) bsp_fdcan_on_tx_event(uint8_t bus);

/* Utility: DLC to byte length (0..64). */
uint8_t bsp_fdcan_dlc_to_len(uint32_t dlc);

// 经典 CAN（标准 ID）非阻塞发送；len > 8 会被夹成 8；无 RTOS 依赖
bool bsp_fdcan_tx_push(uint8_t bus, uint16_t std_id, const uint8_t* data, uint8_t len);

// 经典 CAN（扩展 ID）非阻塞发送；无 RTOS 依赖
bool bsp_fdcan_tx_push_ext(uint8_t bus, uint32_t ext_id, const uint8_t* data, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_BSP_FDCAN_H
