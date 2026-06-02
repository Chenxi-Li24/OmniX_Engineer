//
// drv_spi.h — BSP SPI shim for MCP2518FD
//
#ifndef LIB_MCP2518FD_DRV_SPI_H
#define LIB_MCP2518FD_DRV_SPI_H

#include <stdint.h>
#include "bsp_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DRV_SPI_MAX_DEVICES
#define DRV_SPI_MAX_DEVICES       (4u)
#endif

#ifndef SPI_DEFAULT_BUFFER_LENGTH
#define SPI_DEFAULT_BUFFER_LENGTH   (256u)
#endif

#ifndef _DRV_CANFDSPI_API_H
typedef uint8_t CANFDSPI_MODULE_ID;
#endif

typedef struct
{
    uint32_t xfer_calls;
    uint32_t xfer_ok;
    uint32_t xfer_busy;
    uint32_t xfer_timeout;
    uint32_t xfer_error;
    uint32_t last_tick;
    uint32_t last_ret;
    uint16_t last_size;
    uint8_t  last_index;
} drv_spi_debug_t;

extern volatile drv_spi_debug_t g_drv_spi_debug[DRV_SPI_MAX_DEVICES];

int8_t DRV_SPI_AttachDevice(CANFDSPI_MODULE_ID index,
                            bsp_spi_bus_id_t bus_id,
                            GPIO_TypeDef *cs_port,
                            uint16_t cs_pin);

int8_t DRV_SPI_TransferData(CANFDSPI_MODULE_ID index,
                            uint8_t *spiTxData,
                            uint8_t *spiRxData,
                            uint16_t spiTransferSize);

#ifdef __cplusplus
}
#endif

#endif // LIB_MCP2518FD_DRV_SPI_H
