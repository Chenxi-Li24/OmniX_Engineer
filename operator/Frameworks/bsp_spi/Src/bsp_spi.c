//
// Created by sirin on 2025/11/29.
//
#include "bsp_spi.h"

/* External HAL handles */
extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern DMA_HandleTypeDef hdma_spi4_rx;

/* ============================================================
 * Internal bus structures
 * ============================================================ */
typedef enum
{
    BSP_SPI_XFER_IDLE = 0,
    BSP_SPI_XFER_BUSY
} bsp_spi_xfer_state_t;

typedef struct bsp_spi_bus
{
    SPI_HandleTypeDef      *hspi;
    DMA_HandleTypeDef      *hdma_tx;
    DMA_HandleTypeDef      *hdma_rx;

    volatile bsp_spi_xfer_state_t state;

    bsp_spi_device_t *current_dev;
    bsp_spi_callback_t current_cb;
    void *current_cb_ctx;

} bsp_spi_bus_t;

/* One bus for now */
static bsp_spi_bus_t s_spi_buses[BSP_SPI_BUS_MAX] =
{
    [BSP_SPI_BUS_4] =
    {
        .hspi = &hspi4,
        .hdma_tx = &hdma_spi4_tx,
        .hdma_rx = &hdma_spi4_rx,
        .state = BSP_SPI_XFER_IDLE,
        .current_dev = NULL,
        .current_cb = NULL,
        .current_cb_ctx = NULL
    }
};

/* ============================================================
 * Local helpers
 * ============================================================ */
static bsp_spi_bus_t *bsp_spi_get_bus_by_id(bsp_spi_bus_id_t id)
{
    if (id >= BSP_SPI_BUS_MAX) return NULL;
    return &s_spi_buses[id];
}

static bsp_spi_bus_t *bsp_spi_get_bus_by_handle(SPI_HandleTypeDef *hspi)
{
    for (int i = 0; i < BSP_SPI_BUS_MAX; i++)
    {
        if (s_spi_buses[i].hspi == hspi)
            return &s_spi_buses[i];
    }
    return NULL;
}

static inline void bsp_spi_cs_low(bsp_spi_device_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void bsp_spi_cs_high(bsp_spi_device_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* ============================================================
 * PUBLIC API
 * ============================================================ */
void bsp_spi_init(void)
{
    for (int i = 0; i < BSP_SPI_BUS_MAX; i++)
    {
        s_spi_buses[i].state = BSP_SPI_XFER_IDLE;
        s_spi_buses[i].current_dev = NULL;
        s_spi_buses[i].current_cb = NULL;
        s_spi_buses[i].current_cb_ctx = NULL;
    }
}

bsp_spi_status_t bsp_spi_device_init(bsp_spi_device_t *dev,
                                     bsp_spi_bus_id_t bus_id,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin)
{
    if (!dev || !cs_port) return BSP_SPI_INVALID_PARAM;

    bsp_spi_bus_t *bus = bsp_spi_get_bus_by_id(bus_id);
    if (!bus || !bus->hspi) return BSP_SPI_ERROR;

    dev->bus = bus;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    dev->max_speed_hz = 0;
    dev->mode = 0;
    dev->bits_per_word = 8;

    bsp_spi_cs_high(dev);
    return BSP_SPI_OK;
}

/* ============================================================
 * Async DMA transfer
 * ============================================================ */
bsp_spi_status_t bsp_spi_transfer_dma(bsp_spi_device_t *dev,
                                      const uint8_t *tx_buf,
                                      uint8_t *rx_buf,
                                      size_t len,
                                      bsp_spi_callback_t cb,
                                      void *user_ctx)
{
    if (!dev || !dev->bus || len == 0)
        return BSP_SPI_INVALID_PARAM;

    bsp_spi_bus_t *bus = dev->bus;

    if (bus->state == BSP_SPI_XFER_BUSY)
        return BSP_SPI_BUSY;

    bus->state = BSP_SPI_XFER_BUSY;
    bus->current_dev = dev;
    bus->current_cb = cb;
    bus->current_cb_ctx = user_ctx;

    bsp_spi_cs_low(dev);

    HAL_StatusTypeDef hal_ret;
    uint8_t *tx_ptr = (uint8_t *)tx_buf;

    if (tx_buf && rx_buf)
        hal_ret = HAL_SPI_TransmitReceive_DMA(bus->hspi, tx_ptr, rx_buf, len);
    else if (tx_buf)
        hal_ret = HAL_SPI_Transmit_DMA(bus->hspi, tx_ptr, len);
    else if (rx_buf)
        hal_ret = HAL_SPI_Receive_DMA(bus->hspi, rx_buf, len);
    else
        return BSP_SPI_INVALID_PARAM;

    if (hal_ret != HAL_OK)
    {
        bus->state = BSP_SPI_XFER_IDLE;
        bus->current_dev = NULL;
        bsp_spi_cs_high(dev);
        return BSP_SPI_ERROR;
    }

    return BSP_SPI_OK;
}

/* ============================================================
 * Blocking wrapper (old)
 * ============================================================ */
bsp_spi_status_t bsp_spi_transfer_dma_blocking(bsp_spi_device_t *dev,
                                               const uint8_t *tx_buf,
                                               uint8_t *rx_buf,
                                               size_t len,
                                               uint32_t timeout_ms)
{
    bsp_spi_status_t ret = bsp_spi_transfer_dma(dev, tx_buf, rx_buf, len, NULL, NULL);
    if (ret != BSP_SPI_OK) return ret;

    bsp_spi_bus_t *bus = dev->bus;
    uint32_t start = HAL_GetTick();

    while (bus->state == BSP_SPI_XFER_BUSY)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            HAL_SPI_Abort(bus->hspi);
            bus->state = BSP_SPI_XFER_IDLE;
            bsp_spi_cs_high(dev);
            return BSP_SPI_BUSY;
        }
    }
    return BSP_SPI_OK;
}

/* ============================================================
 * NEW unified blocking/async transfer
 * ============================================================ */
bsp_spi_status_t bsp_spi_transfer_ex(bsp_spi_device_t *dev,
                                     const uint8_t *tx_buf,
                                     uint8_t *rx_buf,
                                     size_t len,
                                     bsp_spi_callback_t cb,
                                     void *user_ctx,
                                     uint8_t blocking,
                                     uint32_t timeout_ms)
{
    bsp_spi_status_t ret = bsp_spi_transfer_dma(dev, tx_buf, rx_buf, len, cb, user_ctx);
    if (ret != BSP_SPI_OK) return ret;

    if (blocking)
    {
        bsp_spi_bus_t *bus = dev->bus;
        uint32_t start = HAL_GetTick();

        while (bus->state == BSP_SPI_XFER_BUSY)
        {
            if ((HAL_GetTick() - start) > timeout_ms)
            {
                HAL_SPI_Abort(bus->hspi);
                bus->state = BSP_SPI_XFER_IDLE;
                bsp_spi_cs_high(dev);
                return BSP_SPI_BUSY;
            }
        }
    }

    return BSP_SPI_OK;
}

/* ============================================================
 * Check bus busy
 * ============================================================ */
bsp_spi_status_t bsp_spi_is_bus_busy(const bsp_spi_device_t *dev)
{
    if (!dev || !dev->bus) return BSP_SPI_INVALID_PARAM;
    return (dev->bus->state == BSP_SPI_XFER_BUSY) ? BSP_SPI_BUSY : BSP_SPI_OK;
}

/* ============================================================
 * HAL callbacks
 * ============================================================ */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_bus_t *bus = bsp_spi_get_bus_by_handle(hspi);
    if (!bus) return;

    bsp_spi_device_t *dev = bus->current_dev;
    bsp_spi_callback_t cb = bus->current_cb;
    void *ctx = bus->current_cb_ctx;

    if (dev) bsp_spi_cs_high(dev);

    bus->state = BSP_SPI_XFER_IDLE;
    bus->current_dev = NULL;
    bus->current_cb = NULL;
    bus->current_cb_ctx = NULL;

    if (cb && dev)
        cb(dev, ctx, BSP_SPI_OK);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_bus_t *bus = bsp_spi_get_bus_by_handle(hspi);
    if (!bus) return;

    bsp_spi_device_t *dev = bus->current_dev;
    bsp_spi_callback_t cb = bus->current_cb;
    void *ctx = bus->current_cb_ctx;

    if (dev) bsp_spi_cs_high(dev);

    bus->state = BSP_SPI_XFER_IDLE;
    bus->current_dev = NULL;
    bus->current_cb = NULL;
    bus->current_cb_ctx = NULL;

    if (cb && dev)
        cb(dev, ctx, BSP_SPI_ERROR);
}