//
// EXFDCAN_Router_Internal.h — internal helpers for MCP2518FD routers
//
#ifndef H723VG_V2_FREERTOS_EXFDCAN_ROUTER_INTERNAL_H
#define H723VG_V2_FREERTOS_EXFDCAN_ROUTER_INTERNAL_H

#include "EXFDCAN_Router.h"
#include "cmsis_os2.h"
#include "bsp_spi.h"
#include "drv_canfdspi_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EXFDCAN_DEV_COUNT
#define EXFDCAN_DEV_COUNT          (1u)
#endif
#ifndef EXFDCAN_RX_LOG_DEPTH
#define EXFDCAN_RX_LOG_DEPTH       (64u)
#endif
#define EXFDCAN_EVT_RX             (1u << 0)
#define EXFDCAN_EVT_TX             (1u << 1)
#define EXFDCAN_EVT_INT            (1u << 2)
#ifndef EXFDCAN_RX_FIFO
#define EXFDCAN_RX_FIFO            CAN_FIFO_CH1
#endif
#ifndef EXFDCAN_TX_FIFO
#define EXFDCAN_TX_FIFO            CAN_FIFO_CH2
#endif
#ifndef EXFDCAN_TXF_SIZE
// FSIZE is "size - 1", max 31 -> 32 objects per FIFO
#define EXFDCAN_TXF_SIZE           (31u)
#endif
#ifndef EXFDCAN_RXF_SIZE
// FSIZE is "size - 1", max 31 -> 32 objects per FIFO
#define EXFDCAN_RXF_SIZE           (31u)
#endif

typedef struct
{
    uint8_t          dev_id;
    bsp_spi_bus_id_t bus_id;
    GPIO_TypeDef    *cs_port;
    uint16_t         cs_pin;
    uint16_t         int_pin;
    uint16_t         int0_pin;
    uint16_t         int1_pin;
} exfdcan_dev_cfg_t;

typedef struct
{
    osEventFlagsId_t   evt;
    volatile uint32_t  pending_isr;
    volatile exfdcan_stats_t *stats;
    exfdcan_rx_frame_t rx_log[EXFDCAN_RX_LOG_DEPTH];
    volatile uint16_t  rx_head;
    volatile uint16_t  rx_tail;
    uint8_t            init_done;
} exfdcan_ctx_t;

exfdcan_ctx_t* exfdcan_ctx_of(uint8_t dev);
const exfdcan_dev_cfg_t* exfdcan_cfg_of(uint8_t dev);
int exfdcan_hw_init(uint8_t dev);
void exfdcan_bus_lock_init(void);
void exfdcan_bus_lock(void);
void exfdcan_bus_unlock(void);

#ifdef __cplusplus
}
#endif

#endif // H723VG_V2_FREERTOS_EXFDCAN_ROUTER_INTERNAL_H
