#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#pragma once
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 * Portability hooks (no RTOS coupling)
 * ------------------------------------------------------------
 * Default: tiny critical sections by disabling IRQ globally.
 * If you use an RTOS, override these macros to map to your
 * scheduler's critical sections (e.g., taskENTER_CRITICAL()).
 * You can override via compile flags or a project header
 * included before this one.
 * ============================================================ */
#ifndef BUZZER_ENTER_CRITICAL
  #define BUZZER_ENTER_CRITICAL()   __disable_irq()
  #define BUZZER_EXIT_CRITICAL()    __enable_irq()
#endif

/* ============================================================
 * Optional time source for duration-based playback
 * ------------------------------------------------------------
 * Weak symbol: implement in your app if you don't want to rely
 * on HAL_GetTick(). Must return a millisecond counter.
 * ============================================================ */
__attribute__((weak)) uint32_t Buzzer_TimeNowMs(void);

/* ============================================================
 * User hardware config (inlined here)
 * ------------------------------------------------------------
 * You can override any of these via -D or prior #define.
 * Example:
 *   -DBUZZER_GPIO_PORT=GPIOB -DBUZZER_PIN=10u -DBUZZER_TIM=TIM2 ...
 * ============================================================ */
#ifndef BUZZER_GPIO_PORT
  #define BUZZER_GPIO_PORT      GPIOE
#endif
#ifndef BUZZER_PIN
  #define BUZZER_PIN            13u
#endif
#ifndef BUZZER_GPIO_CLK_EN
  #define BUZZER_GPIO_CLK_EN()  __HAL_RCC_GPIOE_CLK_ENABLE()
#endif

#ifndef BUZZER_TIM
  #define BUZZER_TIM            TIM2
#endif
#ifndef BUZZER_TIM_CLK_EN
  #define BUZZER_TIM_CLK_EN()   __HAL_RCC_TIM2_CLK_ENABLE()
#endif
#ifndef BUZZER_TIM_IRQn
  #define BUZZER_TIM_IRQn       TIM2_IRQn
#endif

#ifndef BUZZER_DMA
  #define BUZZER_DMA            DMA1_Stream0
#endif
#ifndef BUZZER_DMA_REQ
  #define BUZZER_DMA_REQ        DMA_REQUEST_TIM2_UP
#endif

/* Queue capacity (only used if you use the optional queue API) */
#ifndef BUZZER_QUEUE_CAPACITY
  #define BUZZER_QUEUE_CAPACITY 16
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Core API (non-blocking, DMA-based square wave @ 50% duty)
 * ============================================================ */

/** Initialize GPIO/TIM/DMA once. Safe before scheduler start. */
void Buzzer_Init(void);

/** Start continuous tone at freq_hz (Hz). Non-blocking. freq_hz=0 ignored. */
void Buzzer_Start(uint32_t freq_hz);

/** Stop tone and drive the pin low. Non-blocking. */
void Buzzer_Stop(void);

/** Smoothly retune current tone without stopping DMA (updates TIM->ARR + EGR.UG). */
void Buzzer_SetFrequency(uint32_t freq_hz);

/** Set reference A4 pitch (default 440.0f). Clamped to [200, 500] Hz. */
void Buzzer_SetRefA4(float hz);

/** Parse and start a note name (e.g., "C4", "F#3", "Bb5", "R"/"REST"). False on parse error. */
bool Buzzer_start_note(const char *note);

/* ============================================================
 * Optional: lightweight queued playback by duration
 * ------------------------------------------------------------
 * Enqueue notes {freq_hz, dur_ms} and periodically call
 * Buzzer_Service() from any timing context (SysTick, HW timer
 * ISR, or an RTOS timer task). Library remains RTOS-agnostic.
 * ============================================================ */

typedef struct {
    uint32_t freq_hz;   /* 0 = rest (silence) */
    uint16_t dur_ms;    /* duration in milliseconds */
} BuzzerNote;

/** Enqueue one note (callable from task or ISR). Returns false if full. */
bool Buzzer_Enqueue(BuzzerNote n);

/** Enqueue an array of notes. Returns false if not all could be queued. */
bool Buzzer_EnqueueMany(const BuzzerNote *arr, uint16_t n);

/** Clear the queue and stop immediately. */
void Buzzer_Flush(void);

/** Advance playback based on time; call periodically (e.g., every 1 ms). Safe in ISR. */
void Buzzer_Service(void);

/** ISR-flavored alias (same implementation). */
static inline void Buzzer_ServiceFromISR(void) { Buzzer_Service(); }

/** True if currently generating a tone (timer running and freq>0). */
bool Buzzer_IsActive(void);

/** True if playback queue is empty. */
bool Buzzer_QueueEmpty(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUZZER_H */