//
// Created by sirin on 2025/10/4.
//

#ifndef H723VG_V2_FREERTOS_USART1_CRITCIAL_H
#define H723VG_V2_FREERTOS_USART1_CRITCIAL_H

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32h7xx.h"   // CMSIS device header (not HAL). Make sure it is in include path.

#ifdef __cplusplus
extern "C" {
#endif

    // ========================= User Config (override via -D or before include) =========================
#ifndef USART1C_TX_GPIO_PORT
#define USART1C_TX_GPIO_PORT GPIOB
#endif
#ifndef USART1C_TX_PIN
#define USART1C_TX_PIN       (14u)   // PB14
#endif
#ifndef USART1C_TX_AF
#define USART1C_TX_AF        (4u)
#endif

    // If your board uses a different TX pin (e.g., PB6), set:
    //   -DUSART1C_TX_GPIO_PORT=GPIOB -DUSART1C_TX_PIN=6 -DUSART1C_TX_AF=7

    // Optional: provide a conservative busy-wait iteration cap to avoid lockup if clocks are off.
#ifndef USART1C_SPIN_LIMIT
#define USART1C_SPIN_LIMIT   (1000000u)
#endif

    // Initialize GPIO & USART1 for polling TX at `baud` using PCLK2 frequency `pclk2_hz`.
    // Safe to call from HardFault handler. It force-resets & re-enables USART1.
    void usart1_critical_init(uint32_t pclk2_hz, uint32_t baud);

    // Write a single byte (blocking spin with timeout). Returns true on success.
    bool usart1_critical_write_byte(uint8_t b);

    // Write raw buffer (blocking). Returns number of bytes actually written before timeout.
    size_t usart1_critical_write(const void *data, size_t len);

    // Write C-string (null-terminated). Returns characters written (not counting the null).
    size_t usart1_critical_write_str(const char *s);

    // Convenience prints (hex helpers).
    void usart1_critical_write_hex_u32(uint32_t v);
    void usart1_critical_write_hex_u8(uint8_t v);

    // Force TX complete and optionally disable the peripheral (keeps GPIO config intact).
    void usart1_critical_flush(void);
    void usart1_critical_deinit(void);

    void dump_words(const char *tag, const void *addr, uint32_t words);

#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_USART1_CRITCIAL_H