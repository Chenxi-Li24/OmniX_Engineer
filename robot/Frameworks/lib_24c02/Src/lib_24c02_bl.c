//
// Created by sirin on 2025/12/1.
//
//
// Blocking 24C02 EEPROM driver implementation
//

#include "lib_24c02_bl.h"

/* ----------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------- */

/**
 * @brief Simple range check.
 */
static lib_24c02_bl_status_t lib_24c02_bl_check_range(const lib_24c02_bl_t *eeprom,
                                                      uint8_t mem_addr,
                                                      size_t len)
{
    if (eeprom == NULL)
    {
        return LIB_24C02_BL_INVALID_PARAM;
    }

    if (len == 0U)
    {
        return LIB_24C02_BL_INVALID_PARAM;
    }

    uint32_t end_addr = (uint32_t)mem_addr + (uint32_t)len;
    if (end_addr > eeprom->size_bytes)
    {
        /* Out of range */
        return LIB_24C02_BL_INVALID_PARAM;
    }

    return LIB_24C02_BL_OK;
}

/* ----------------------------------------------------------------------
 * Public APIs
 * -------------------------------------------------------------------- */

lib_24c02_bl_status_t lib_24c02_bl_init(lib_24c02_bl_t *eeprom,
                                        I2C_HandleTypeDef *hi2c,
                                        uint16_t addr_7bit)
{
    if (eeprom == NULL || hi2c == NULL)
    {
        return LIB_24C02_BL_INVALID_PARAM;
    }

    eeprom->hi2c        = hi2c;
    eeprom->dev_addr_7b = (uint16_t)(addr_7bit & 0x7FU);
    eeprom->size_bytes  = LIB_24C02_BL_SIZE_BYTES;
    eeprom->page_size   = LIB_24C02_BL_PAGE_SIZE;

    return LIB_24C02_BL_OK;
}

lib_24c02_bl_status_t lib_24c02_bl_read(lib_24c02_bl_t *eeprom,
                                        uint8_t mem_addr,
                                        uint8_t *buf,
                                        size_t len,
                                        uint32_t timeout)
{
    if (eeprom == NULL || buf == NULL)
    {
        return LIB_24C02_BL_INVALID_PARAM;
    }

    lib_24c02_bl_status_t st = lib_24c02_bl_check_range(eeprom, mem_addr, len);
    if (st != LIB_24C02_BL_OK)
    {
        return st;
    }

    /* HAL uses 8-bit address: left-shift 7-bit addr. */
    uint16_t dev_addr_8b = (uint16_t)(eeprom->dev_addr_7b << 1);

    HAL_StatusTypeDef hret = HAL_I2C_Mem_Read(eeprom->hi2c,
                                              dev_addr_8b,
                                              mem_addr,
                                              I2C_MEMADD_SIZE_8BIT,
                                              buf,
                                              (uint16_t)len,
                                              timeout);

    if (hret != HAL_OK)
    {
        return LIB_24C02_BL_ERROR;
    }

    return LIB_24C02_BL_OK;
}

lib_24c02_bl_status_t lib_24c02_bl_write(lib_24c02_bl_t *eeprom,
                                         uint8_t mem_addr,
                                         const uint8_t *buf,
                                         size_t len,
                                         uint32_t timeout,
                                         uint32_t twr_ms)
{
    if (eeprom == NULL || buf == NULL)
    {
        return LIB_24C02_BL_INVALID_PARAM;
    }

    lib_24c02_bl_status_t st = lib_24c02_bl_check_range(eeprom, mem_addr, len);
    if (st != LIB_24C02_BL_OK)
    {
        return st;
    }

    uint16_t size_bytes = eeprom->size_bytes;
    uint8_t  page_size  = eeprom->page_size;
    uint16_t dev_addr_8b = (uint16_t)(eeprom->dev_addr_7b << 1);

    uint8_t        addr      = mem_addr;
    const uint8_t *p         = buf;
    size_t         remaining = len;

    while (remaining > 0U)
    {
        uint32_t addr32 = (uint32_t)addr;
        if (addr32 >= size_bytes)
        {
            return LIB_24C02_BL_INVALID_PARAM;
        }

        /* Calculate how many bytes can be written in this page. */
        uint8_t page_offset = (uint8_t)(addr % page_size);
        uint8_t page_remain = (uint8_t)(page_size - page_offset);

        size_t chunk = remaining;
        if (chunk > page_remain)
        {
            chunk = page_remain;
        }

        HAL_StatusTypeDef hret = HAL_I2C_Mem_Write(eeprom->hi2c,
                                                   dev_addr_8b,
                                                   addr,
                                                   I2C_MEMADD_SIZE_8BIT,
                                                   (uint8_t *)p,
                                                   (uint16_t)chunk,
                                                   timeout);
        if (hret != HAL_OK)
        {
            return LIB_24C02_BL_ERROR;
        }

        /* Wait internal write cycle (tWR) */
        if (twr_ms > 0U)
        {
            HAL_Delay(twr_ms);
        }

        addr      = (uint8_t)(addr + (uint8_t)chunk);
        p        += chunk;
        remaining -= chunk;
    }

    return LIB_24C02_BL_OK;
}