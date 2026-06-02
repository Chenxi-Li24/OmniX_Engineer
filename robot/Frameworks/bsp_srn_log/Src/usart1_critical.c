//
// Created by sirin on 2025/10/4.
//
#include "usart1_critical.h"

// ---- local helpers ----
static inline void _enable_rcc_gpio(GPIO_TypeDef *port) {
    // AHB4 clock gating per port
    if (port == GPIOA) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOB) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOC) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOD) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOE) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOF) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOG) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN; (void)RCC->AHB4ENR; }
    else if (port == GPIOH) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN; (void)RCC->AHB4ENR; }
}

static inline void _gpio_pin_to_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af) {
    // Set MODER = 10 (Alternate), OSPEEDR = 11 (Very High), OTYPER=0 (Push-Pull), PUPDR=01 (PU)
    uint32_t pos = pin * 2u;
    port->MODER   = (port->MODER & ~(0x3u << pos))   | (0x2u << pos);
    port->OSPEEDR = (port->OSPEEDR & ~(0x3u << pos)) | (0x3u << pos);
    port->OTYPER &= ~(1u << pin);
    port->PUPDR   = (port->PUPDR & ~(0x3u << pos))   | (0x1u << pos);

    uint32_t afr_idx = (pin < 8u) ? 0u : 1u;
    uint32_t afr_pos = (pin & 0x7u) * 4u;
    port->AFR[afr_idx] = (port->AFR[afr_idx] & ~(0xFu << afr_pos)) | ((af & 0xFu) << afr_pos);
}

static inline void _usart1_clock_enable(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; (void)RCC->APB2ENR;
}

static inline void _usart1_force_reset(void) {
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    __DSB();
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    __DSB();
}

static inline uint32_t _div_brr(uint32_t pclk_hz, uint32_t baud) {
    // Oversampling by 16: BRR = PCLK / baud. Round to nearest.
    uint32_t div = (pclk_hz + (baud/2u)) / baud;
    if (div < 16u) div = 16u;  // avoid zero/too-small in pathological clocks
    return div;
}

void usart1_critical_init(uint32_t pclk2_hz, uint32_t baud) {
    // 1) Enable GPIO clock + set TX pin AF7
    _enable_rcc_gpio(USART1C_TX_GPIO_PORT);
    _gpio_pin_to_af(USART1C_TX_GPIO_PORT, USART1C_TX_PIN, USART1C_TX_AF);

    // 2) Enable USART1 clock & reset
    _usart1_clock_enable();
    _usart1_force_reset();

    // 3) Disable UE before configuration
    USART1->CR1 = 0;  // make sure UE=0
    __DSB();

    // 4) Set baud
    USART1->BRR = _div_brr(pclk2_hz, baud);

    // 5) 8-N-1, oversampling by 16 (defaults). Enable transmitter and USART
    // CR1: UE (bit 0), TE (bit 3)
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    // 6) Optional: clear status flags
    USART1->ICR = 0xFFFFFFFFu; // clear all clearable flags
}

bool usart1_critical_write_byte(uint8_t b) {
    uint32_t spins = 0;
    // Wait until TXE_TXFNF (TX FIFO not full) == 1
    while (!(USART1->ISR & USART_ISR_TXE_TXFNF)) {
        if (++spins > USART1C_SPIN_LIMIT) return false;
    }
    USART1->TDR = b;
    return true;
}

size_t usart1_critical_write(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    size_t i = 0;
    for (; i < len; ++i) {
        if (!usart1_critical_write_byte(p[i])) break;
    }
    return i;
}

size_t usart1_critical_write_str(const char *s) {
    size_t n = 0;
    while (*s) {
        if (!usart1_critical_write_byte((uint8_t)*s++)) break;
        ++n;
    }
    return n;
}

static inline char _hex_digit(uint32_t v) {
    v &= 0xF;
    return (char)(v < 10 ? ('0' + v) : ('A' + (v - 10)));
}

void usart1_critical_write_hex_u8(uint8_t v) {
    (void)usart1_critical_write_byte(_hex_digit(v >> 4));
    (void)usart1_critical_write_byte(_hex_digit(v));
}

void usart1_critical_write_hex_u32(uint32_t v) {
    for (int i = 7; i >= 0; --i) {
        (void)usart1_critical_write_byte(_hex_digit(v >> (i * 4)));
    }
}

void usart1_critical_flush(void) {
    uint32_t spins = 0;
    // Wait for TC (Transmission Complete)
    while (!(USART1->ISR & USART_ISR_TC)) {
        if (++spins > USART1C_SPIN_LIMIT) break;
    }
}

void usart1_critical_deinit(void) {
    usart1_critical_flush();
    USART1->CR1 &= ~(USART_CR1_TE | USART_CR1_UE);
}

// ========================= Example usage (HardFault-safe) =========================
// In your HardFault_Handler() (stm32h7xx_it.c), you can do:
//    extern void usart1_critical_init(uint32_t pclk2_hz, uint32_t baud);
//    extern size_t usart1_critical_write_str(const char *s);
//    extern void usart1_critical_write_hex_u32(uint32_t v);
//
//    // Suppose APB2 (PCLK2) = 120 MHz, baud = 115200
//    usart1_critical_init(120000000u, 115200u);
//    usart1_critical_write_str("[HF] CFSR=0x");
//    usart1_critical_write_hex_u32(SCB->CFSR);
//    usart1_critical_write_str(" SP=0x");
//    usart1_critical_write_hex_u32(__get_MSP());
//    usart1_critical_write_str("\r\n");
//
// Tip: Cache PCLK2 value during normal boot and store in a global for reuse in faults.

void dump_words(const char *tag, const void *addr, uint32_t words) {
    usart1_critical_write_str(tag);
    usart1_critical_write_str(" @0x");
    usart1_critical_write_hex_u32((uint32_t)addr);
    usart1_critical_write_str(":");
    const uint32_t *p = (const uint32_t *)addr;
    for (uint32_t i = 0; i < words; ++i) {
        usart1_critical_write_str(" ");
        usart1_critical_write_hex_u32(p[i]);
    }
    usart1_critical_write_str("\r\n");
}