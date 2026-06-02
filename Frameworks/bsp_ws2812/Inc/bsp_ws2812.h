//
// Created by sirin on 2025/9/29.
//

#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ================== System Config ================== */

#ifndef WS2812_LED_COUNT
#define WS2812_LED_COUNT      60u
#endif

#ifndef WS2812_RESET_SLOTS
#define WS2812_RESET_SLOTS    80u   // >= 50us
#endif

#ifndef WS2812_BITS_PER_LED
#define WS2812_BITS_PER_LED   24u   // GRB, MSB first
#endif

#ifndef WS2812_BITRATE_HZ
#define WS2812_BITRATE_HZ     800000u // 800kHz
#endif

#ifndef WS2812_T0H_NS
#define WS2812_T0H_NS         350u
#endif
#ifndef WS2812_T1H_NS
#define WS2812_T1H_NS         700u
#endif

#ifndef WS2812_GPIO_PORT
#define WS2812_GPIO_PORT      GPIOA
#endif
#ifndef WS2812_GPIO_PIN
#define WS2812_GPIO_PIN       GPIO_PIN_10
#endif
#ifndef WS2812_GPIO_AF
#define WS2812_GPIO_AF        GPIO_AF1_TIM1
#endif
#ifndef WS2812_TIM
#define WS2812_TIM            TIM1
#endif

// DMA/DMAMUX：TIM1_UP -> DMA1_Stream1
#ifndef WS2812_DMA_STREAM
#define WS2812_DMA_STREAM     DMA1_Stream1
#endif
#ifndef WS2812_DMA_REQUEST
#define WS2812_DMA_REQUEST    DMA_REQUEST_TIM1_UP
#endif
#ifndef WS2812_DMA_IRQn
#define WS2812_DMA_IRQn       DMA1_Stream1_IRQn
#endif
#ifndef WS2812_DMA_IRQ_PRIO
#define WS2812_DMA_IRQ_PRIO   5
#endif

/* ================== API & Hook ================== */

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ws2812_cb_t)(void *user);

/**
 * @brief Hook for baremetal or RTOS
 * - on_tx_done/on_tx_error：Transmission complete/error callback
 * - notify_from_isr：FromISR in FreeRTOS, or baremetal equivalent
 * - user/notify_arg：user data pointer
 */
typedef struct {
    ws2812_cb_t on_tx_done;
    ws2812_cb_t on_tx_error;
    void       *user;

    void (*notify_from_isr)(void *arg); // i.e. xSemaphoreGiveFromISR(...) or vTaskNotifyGiveFromISR(...)
    void *notify_arg;
} ws2812_hooks_t;

/** Init：tim_clk_hz = TIM1 input clock, e.g. 170MHz/170e6 */
void WS2812_Init(uint32_t tim_clk_hz);

/** Set single pixel */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/** Fast fill */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/** Change Brightness */
void WS2812_SetGlobalBrightness(uint8_t scale);

/** Commit changes to DMA */
HAL_StatusTypeDef WS2812_Commit_StartAsync(const ws2812_hooks_t *hooks);

/** If DMA Busy */
uint8_t WS2812_IsBusy(void);

/** (OPTIONAL): Commit changes and wait done */
void WS2812_WaitIdle(void);

/** IRQ for stm32h7xx_it.c */
void WS2812_DMA_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif //BSP_WS2812_H
