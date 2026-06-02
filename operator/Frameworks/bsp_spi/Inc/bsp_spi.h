//
// Created by sirin on 2025/11/29.
//

#ifndef H723VG_V2_FREERTOS_BSP_SPI_H
#define H723VG_V2_FREERTOS_BSP_SPI_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Cache region compatibility
 * ============================================================ */
#ifdef OMNIX_FEATURE_MPU_DCACHE
#if __has_include("mem_sections.h")
#include "mem_sections.h"
#define BSP_SPI_DMA_SEC SEC_DMA_NC_BUF
#else
#define BSP_SPI_DMA_SEC
#warning "bsp_spi: OMNIX_FEATURE_MPU_DCACHE defined but mem_sections.h not found; DMA buffers remain cacheable!"
#endif
#else
#define BSP_SPI_DMA_SEC
#endif

/* ============================================================
 * ENUMS
 * ============================================================ */
typedef enum
{
    BSP_SPI_OK = 0,
    BSP_SPI_BUSY,
    BSP_SPI_ERROR,
    BSP_SPI_INVALID_PARAM
} bsp_spi_status_t;

typedef enum
{
    BSP_SPI_BUS_4 = 0,
    BSP_SPI_BUS_MAX
} bsp_spi_bus_id_t;

struct bsp_spi_bus;

/* ============================================================
 * SPI Device Structure
 * ============================================================ */
typedef struct
{
    struct bsp_spi_bus *bus;
    GPIO_TypeDef       *cs_port;
    uint16_t            cs_pin;

    uint32_t            max_speed_hz;
    uint8_t             mode;
    uint8_t             bits_per_word;
} bsp_spi_device_t;

/* Callback type */
typedef void (*bsp_spi_callback_t)(bsp_spi_device_t *dev, void *user_ctx, bsp_spi_status_t status);

/* ============================================================
 * PUBLIC API
 * ============================================================ */

void bsp_spi_init(void);

bsp_spi_status_t bsp_spi_device_init(bsp_spi_device_t *dev,
                                     bsp_spi_bus_id_t bus_id,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin);

/**
 * Basic async DMA transfer
 */
bsp_spi_status_t bsp_spi_transfer_dma(bsp_spi_device_t *dev,
                                      const uint8_t *tx_buf,
                                      uint8_t *rx_buf,
                                      size_t len,
                                      bsp_spi_callback_t cb,
                                      void *user_ctx);

/**
 * Old blocking wrapper (still available)
 */
bsp_spi_status_t bsp_spi_transfer_dma_blocking(bsp_spi_device_t *dev,
                                               const uint8_t *tx_buf,
                                               uint8_t *rx_buf,
                                               size_t len,
                                               uint32_t timeout_ms);

/**
 * NEW: Unified transfer (blocking or async)
 */
bsp_spi_status_t bsp_spi_transfer_ex(bsp_spi_device_t *dev,
                                     const uint8_t *tx_buf,
                                     uint8_t *rx_buf,
                                     size_t len,
                                     bsp_spi_callback_t cb,
                                     void *user_ctx,
                                     uint8_t blocking,
                                     uint32_t timeout_ms);

/**
 * NEW: Convenience wrappers
 */
static inline bsp_spi_status_t bsp_spi_transfer_blocking(bsp_spi_device_t *dev,
                                                         const uint8_t *tx,
                                                         uint8_t *rx,
                                                         size_t len,
                                                         uint32_t timeout)
{
    return bsp_spi_transfer_ex(dev, tx, rx, len, NULL, NULL, 1, timeout);
}

static inline bsp_spi_status_t bsp_spi_transfer_async(bsp_spi_device_t *dev,
                                                      const uint8_t *tx,
                                                      uint8_t *rx,
                                                      size_t len,
                                                      bsp_spi_callback_t cb,
                                                      void *ctx)
{
    return bsp_spi_transfer_ex(dev, tx, rx, len, cb, ctx, 0, 0);
}

bsp_spi_status_t bsp_spi_is_bus_busy(const bsp_spi_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_BSP_SPI_H