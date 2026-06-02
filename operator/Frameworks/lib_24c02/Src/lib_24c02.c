//
// Created by sirin on 2025/11/30.
//

#include "lib_24c02.h"

// WARNING：这个库现在存在问题，暂时不使用，请先使用blocking

/* Forward: internal I2C callback */
static void lib_24c02_i2c_cb(bsp_i2c_device_t *dev,
                             void *ctx,
                             bsp_i2c_status_t status);

/**
 * @brief Internal range check helper.
 */
static bsp_i2c_status_t lib_24c02_check_range(const lib_24c02_t *eeprom,
                                              uint8_t mem_addr,
                                              size_t len)
{
    if (eeprom == NULL)
    {
        return BSP_I2C_INVALID_PARAM;
    }

    if (len == 0U)
    {
        return BSP_I2C_INVALID_PARAM;
    }

    uint32_t end_addr = (uint32_t)mem_addr + (uint32_t)len;
    if (end_addr > eeprom->size_bytes)
    {
        return BSP_I2C_INVALID_PARAM;
    }

    return BSP_I2C_OK;
}

/**
 * @brief Initialize 24C02 async wrapper.
 */
bsp_i2c_status_t lib_24c02_init(lib_24c02_t *eeprom,
                                bsp_i2c_bus_id_t bus_id,
                                uint16_t i2c_addr_7bit)
{
    if (eeprom == NULL)
    {
        return BSP_I2C_INVALID_PARAM;
    }

    bsp_i2c_status_t st = bsp_i2c_device_init(&eeprom->i2c_dev,
                                              bus_id,
                                              i2c_addr_7bit);
    if (st != BSP_I2C_OK)
    {
        return st;
    }

    eeprom->size_bytes = EEPROM24C02_SIZE_BYTES;
    eeprom->page_size  = EEPROM24C02_PAGE_SIZE;

    eeprom->op   = LIB_24C02_OP_NONE;
    eeprom->busy = 0U;

    eeprom->mem_addr = 0U;
    eeprom->rx_buf   = NULL;
    eeprom->tx_buf   = NULL;
    eeprom->len      = 0U;

    eeprom->user_cb  = NULL;
    eeprom->user_ctx = NULL;

    return BSP_I2C_OK;
}

/**
 * @brief Start async read.
 */
bsp_i2c_status_t lib_24c02_read_async(lib_24c02_t *eeprom,
                                      uint8_t mem_addr,
                                      uint8_t *buf,
                                      size_t len,
                                      lib_24c02_async_cb_t cb,
                                      void *ctx)
{
    if (eeprom == NULL || buf == NULL)
    {
        return BSP_I2C_INVALID_PARAM;
    }

    if (eeprom->busy)
    {
        return BSP_I2C_BUSY;
    }

    bsp_i2c_status_t st = lib_24c02_check_range(eeprom, mem_addr, len);
    if (st != BSP_I2C_OK)
    {
        return st;
    }

    eeprom->busy     = 1U;
    eeprom->op       = LIB_24C02_OP_READ;
    eeprom->mem_addr = mem_addr;
    eeprom->rx_buf   = buf;
    eeprom->tx_buf   = NULL;
    eeprom->len      = len;
    eeprom->user_cb  = cb;
    eeprom->user_ctx = ctx;

    st = bsp_i2c_mem_read(&eeprom->i2c_dev,
                          mem_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          lib_24c02_i2c_cb,
                          eeprom);
    if (st != BSP_I2C_OK)
    {
        eeprom->busy = 0U;
        eeprom->op   = LIB_24C02_OP_NONE;
        return st;
    }

    return BSP_I2C_OK;
}

/**
 * @brief Start async write (single page).
 */
bsp_i2c_status_t lib_24c02_write_async(lib_24c02_t *eeprom,
                                       uint8_t mem_addr,
                                       const uint8_t *buf,
                                       size_t len,
                                       lib_24c02_async_cb_t cb,
                                       void *ctx)
{
    if (eeprom == NULL || buf == NULL)
    {
        return BSP_I2C_INVALID_PARAM;
    }

    if (eeprom->busy)
    {
        return BSP_I2C_BUSY;
    }

    bsp_i2c_status_t st = lib_24c02_check_range(eeprom, mem_addr, len);
    if (st != BSP_I2C_OK)
    {
        return st;
    }

    /* Enforce single-page write for this async version. */
    uint8_t page_size = eeprom->page_size;
    uint8_t page_start = (uint8_t)(mem_addr / page_size);
    uint8_t page_end   = (uint8_t)((mem_addr + (uint8_t)(len - 1U)) / page_size);
    if (page_start != page_end)
    {
        /* Cross-page write not allowed in this async API. */
        return BSP_I2C_INVALID_PARAM;
    }

    eeprom->busy     = 1U;
    eeprom->op       = LIB_24C02_OP_WRITE;
    eeprom->mem_addr = mem_addr;
    eeprom->rx_buf   = NULL;
    eeprom->tx_buf   = buf;
    eeprom->len      = len;
    eeprom->user_cb  = cb;
    eeprom->user_ctx = ctx;

    st = bsp_i2c_mem_write(&eeprom->i2c_dev,
                           mem_addr,
                           I2C_MEMADD_SIZE_8BIT,
                           buf,
                           len,
                           lib_24c02_i2c_cb,
                           eeprom);
    if (st != BSP_I2C_OK)
    {
        eeprom->busy = 0U;
        eeprom->op   = LIB_24C02_OP_NONE;
        return st;
    }

    return BSP_I2C_OK;
}

/**
 * @brief Internal wrapper callback from bsp_i2c.
 *
 * This is called in I2C IRQ context.
 */
static void lib_24c02_i2c_cb(bsp_i2c_device_t *dev,
                             void *ctx,
                             bsp_i2c_status_t status)
{
    (void)dev;

    lib_24c02_t *eeprom = (lib_24c02_t *)ctx;
    if (eeprom == NULL)
    {
        return;
    }

    lib_24c02_async_cb_t user_cb = eeprom->user_cb;
    void *user_ctx = eeprom->user_ctx;

    /* Clear async state. */
    eeprom->busy = 0U;
    eeprom->op   = LIB_24C02_OP_NONE;
    eeprom->user_cb  = NULL;
    eeprom->user_ctx = NULL;

    /* IMPORTANT:
     * Do not do heavy stuff here; this is IRQ context.
     * User callback should be lightweight (e.g. give semaphore, set flag).
     */
    if (user_cb != NULL)
    {
        user_cb(eeprom, user_ctx, status);
    }
}