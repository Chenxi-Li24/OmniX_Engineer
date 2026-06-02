//
// tskptt_exfdcan_device.cpp — MCP2518FD device config + EXTI dispatch
//
#include "EXFDCAN_Router_Internal.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_defines.h"
#include "drv_spi.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#ifndef EXFDCAN_SYSCLK
#define EXFDCAN_SYSCLK    CAN_SYSCLK_40M
#endif
#ifndef EXFDCAN_BITTIME
#define EXFDCAN_BITTIME   CAN_1000K_4M
#endif

static const exfdcan_dev_cfg_t s_cfg[EXFDCAN_DEV_COUNT] =
{
    {
        .dev_id   = 0u,
        .bus_id   = BSP_SPI_BUS_4,
        .cs_port  = MCP_CS_GPIO_Port,
        .cs_pin   = MCP_CS_Pin,
        .int_pin  = MCP1_INT_Pin,
        .int0_pin = MCP1_INT0_Pin,
        .int1_pin = MCP1_INT1_Pin
    }
};

extern "C" {
volatile exfdcan_stats_t g_exfdcan_stats[EXFDCAN_DEV_COUNT];
}

static exfdcan_ctx_t s_ctx[EXFDCAN_DEV_COUNT];
static osMutexId_t   s_canfdspi_mutex = NULL;

extern "C" exfdcan_ctx_t* exfdcan_ctx_of(uint8_t dev)
{
    if (dev >= EXFDCAN_DEV_COUNT)
        return NULL;

    exfdcan_ctx_t *ctx = &s_ctx[dev];
    if (!ctx->stats)
    {
        ctx->stats = &g_exfdcan_stats[dev];
    }
    return ctx;
}

extern "C" const exfdcan_dev_cfg_t* exfdcan_cfg_of(uint8_t dev)
{
    return (dev < EXFDCAN_DEV_COUNT) ? &s_cfg[dev] : NULL;
}

extern "C" void exfdcan_bus_lock_init(void)
{
    if (!s_canfdspi_mutex)
    {
        uint32_t ps = __get_PRIMASK(); __disable_irq();
        if (!s_canfdspi_mutex)
        {
            s_canfdspi_mutex = osMutexNew(NULL);
        }
        if (!ps) __enable_irq();
    }
}

extern "C" void exfdcan_bus_lock(void)
{
    if (s_canfdspi_mutex)
    {
        (void)osMutexAcquire(s_canfdspi_mutex, osWaitForever);
    }
}

extern "C" void exfdcan_bus_unlock(void)
{
    if (s_canfdspi_mutex)
    {
        (void)osMutexRelease(s_canfdspi_mutex);
    }
}

extern "C" int exfdcan_hw_init(uint8_t dev)
{
    exfdcan_ctx_t *ctx = exfdcan_ctx_of(dev);
    const exfdcan_dev_cfg_t *cfg = exfdcan_cfg_of(dev);
    if (!ctx || !cfg)
        return -1;

    exfdcan_bus_lock_init();
    exfdcan_bus_lock();
    if (ctx->init_done)
    {
        exfdcan_bus_unlock();
        return 0;
    }

    (void)DRV_SPI_AttachDevice(dev, cfg->bus_id, cfg->cs_port, cfg->cs_pin);

    (void)DRV_CANFDSPI_Reset(dev);
    HAL_Delay(2);
    (void)DRV_CANFDSPI_OperationModeSelect(dev, CAN_CONFIGURATION_MODE);

    CAN_CONFIG config;
    (void)DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.BitRateSwitchDisable = 1;
    config.RestrictReTxAttempts = 1;
    config.TXQEnable = 0;
    config.StoreInTEF = 0;
    config.TxBandWidthSharing = CAN_TXBWS_NO_DELAY;
    (void)DRV_CANFDSPI_Configure(dev, &config);

    (void)DRV_CANFDSPI_BitTimeConfigure(dev, EXFDCAN_BITTIME, CAN_SSP_MODE_OFF, EXFDCAN_SYSCLK);

    CAN_TX_FIFO_CONFIG txf;
    (void)DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txf);
    txf.TxPriority = 1;
    txf.TxAttempts = 2;
    txf.FifoSize = EXFDCAN_TXF_SIZE;
    txf.PayLoadSize = CAN_PLSIZE_8;
    txf.RTREnable = 0;
    (void)DRV_CANFDSPI_TransmitChannelConfigure(dev, EXFDCAN_TX_FIFO, &txf);

    CAN_RX_FIFO_CONFIG rxf;
    (void)DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxf);
    rxf.FifoSize = EXFDCAN_RXF_SIZE;
    rxf.PayLoadSize = CAN_PLSIZE_8;
    rxf.RxTimeStampEnable = 0;
    (void)DRV_CANFDSPI_ReceiveChannelConfigure(dev, EXFDCAN_RX_FIFO, &rxf);

    CAN_FILTEROBJ_ID fobj = {0};
    CAN_MASKOBJ_ID   mobj = {0};
    fobj.SID = 0;
    fobj.EID = 0;
    fobj.SID11 = 0;
    fobj.EXIDE = 0;
    mobj.MSID = 0;
    mobj.MEID = 0;
    mobj.MSID11 = 0;
    mobj.MIDE = 0;
    (void)DRV_CANFDSPI_FilterObjectConfigure(dev, CAN_FILTER0, &fobj);
    (void)DRV_CANFDSPI_FilterMaskConfigure(dev, CAN_FILTER0, &mobj);
    (void)DRV_CANFDSPI_FilterToFifoLink(dev, CAN_FILTER0, EXFDCAN_RX_FIFO, true);
    (void)DRV_CANFDSPI_FilterEnable(dev, CAN_FILTER0);

    (void)DRV_CANFDSPI_GpioModeConfigure(dev, GPIO_MODE_INT, GPIO_MODE_INT);
    (void)DRV_CANFDSPI_GpioDirectionConfigure(dev, GPIO_OUTPUT, GPIO_OUTPUT);
    (void)DRV_CANFDSPI_GpioInterruptPinsOpenDrainConfigure(dev, GPIO_OPEN_DRAIN);

    (void)DRV_CANFDSPI_ModuleEventEnable(dev,
                                         (CAN_MODULE_EVENT)(CAN_RX_EVENT |
                                                            CAN_TX_EVENT |
                                                            CAN_RX_OVERFLOW_EVENT));
    (void)DRV_CANFDSPI_ReceiveChannelEventEnable(dev,
                                                 EXFDCAN_RX_FIFO,
                                                 (CAN_RX_FIFO_EVENT)(CAN_RX_FIFO_HALF_FULL_EVENT |
                                                                     CAN_RX_FIFO_OVERFLOW_EVENT));
    (void)DRV_CANFDSPI_TransmitChannelEventDisable(dev, EXFDCAN_TX_FIFO, CAN_TX_FIFO_ALL_EVENTS);

    (void)DRV_CANFDSPI_OperationModeSelect(dev, CAN_NORMAL_MODE);

    ctx->init_done = 1u;
    exfdcan_bus_unlock();
    return 0;
}

extern "C" void exfdcan_on_exti(uint16_t gpio_pin)
{
    for (uint8_t i = 0; i < EXFDCAN_DEV_COUNT; ++i)
    {
        exfdcan_ctx_t *ctx = exfdcan_ctx_of(i);
        const exfdcan_dev_cfg_t *cfg = exfdcan_cfg_of(i);
        if (!ctx || !cfg)
            continue;
        uint32_t set_flags = 0;

        if (gpio_pin == cfg->int1_pin)
        {
            ctx->stats->isr_int1++;
            set_flags |= EXFDCAN_EVT_RX;
        }
        else if (gpio_pin == cfg->int0_pin)
        {
            ctx->stats->isr_int0++;
            set_flags |= EXFDCAN_EVT_TX;
        }
        else if (gpio_pin == cfg->int_pin)
        {
            ctx->stats->int_irq++;
            continue;
        }
        else
        {
            continue;
        }

        if (ctx->evt)
        {
            (void)osEventFlagsSet(ctx->evt, set_flags);
        }
        else
        {
            ctx->pending_isr |= set_flags;
        }
    }
}

extern "C" const volatile exfdcan_stats_t* exfdcan_get_stats(uint8_t dev)
{
    exfdcan_ctx_t *ctx = exfdcan_ctx_of(dev);
    return ctx ? ctx->stats : NULL;
}

extern "C" size_t exfdcan_rx_read(uint8_t dev, exfdcan_rx_frame_t *out, size_t max)
{
    exfdcan_ctx_t *ctx = exfdcan_ctx_of(dev);
    if (!ctx || !out || max == 0u)
        return 0u;

    size_t n = 0;
    uint32_t ps = __get_PRIMASK(); __disable_irq();
    while ((ctx->rx_tail != ctx->rx_head) && (n < max))
    {
        out[n] = ctx->rx_log[ctx->rx_tail];
        ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1u) % EXFDCAN_RX_LOG_DEPTH);
        n++;
    }
    if (!ps) __enable_irq();

    return n;
}
