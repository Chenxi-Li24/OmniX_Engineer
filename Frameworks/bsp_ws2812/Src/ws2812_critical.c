#include "ws2812_critical.h"

static uint8_t s_buf[WS2812CT_LED_COUNT*3u]; // GRB
static uint16_t s_arr, s_t0h, s_t1h;         // timer counts

static inline void _rcc_gpio_en(GPIO_TypeDef *p) {
    if (p==GPIOA) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOB) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOC) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOD) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOE) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOF) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOG) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN; (void)RCC->AHB4ENR; }
    else if (p==GPIOH) { RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN; (void)RCC->AHB4ENR; }
}

static inline void _gpio_to_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af) {
    uint32_t pos = pin*2u;
    port->MODER   = (port->MODER & ~(0x3u<<pos)) | (0x2u<<pos);   // AF
    port->OTYPER &= ~(1u<<pin);                                   // PP
    port->OSPEEDR = (port->OSPEEDR & ~(0x3u<<pos)) | (0x3u<<pos); // VeryHigh
    port->PUPDR  &= ~(0x3u<<pos);                                 // NoPull
    uint32_t idx=(pin<8)?0:1; uint32_t sh=((pin&7u)*4u);
    port->AFR[idx] = (port->AFR[idx] & ~(0xFu<<sh)) | ((af&0xFu)<<sh);
}

static inline uint32_t _ns_to_counts(uint32_t hz, uint32_t ns) {
    uint64_t num = (uint64_t)hz * (uint64_t)ns + 500000000ull;
    return (uint32_t)(num / 1000000000ull);
}

void ws2812ct_init(uint32_t tim_clk_hz)
{
    // 1) GPIO AF to TIM1_CH3
    _rcc_gpio_en(WS2812CT_GPIO_PORT);
    _gpio_to_af(WS2812CT_GPIO_PORT, WS2812CT_GPIO_PIN, WS2812CT_GPIO_AF);

    // 2) Enable TIM1 clock & basic PWM config on CH3
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; (void)RCC->APB2ENR;

    // Target: bit period = 1/800k = 1.25us => ARR+1 = tim_clk / 800k
    uint32_t arr_p1 = (tim_clk_hz + WS2812CT_BITRATE_HZ/2) / WS2812CT_BITRATE_HZ; // round
    if (arr_p1 < 2u) arr_p1 = 2u; if (arr_p1 > 0x10000u) arr_p1 = 0x10000u;
    s_arr = (uint16_t)(arr_p1 - 1u);

    // Duty counts for T0H/T1H
    uint32_t t0 = _ns_to_counts(tim_clk_hz, WS2812CT_T0H_NS);
    uint32_t t1 = _ns_to_counts(tim_clk_hz, WS2812CT_T1H_NS);
    if (t0>=arr_p1) t0 = arr_p1-1u; if (t1>=arr_p1) t1 = arr_p1-1u;
    s_t0h = (uint16_t)t0; s_t1h = (uint16_t)t1;

    // Reset timer
    WS2812CT_TIM->CR1 = 0; WS2812CT_TIM->CR2 = 0; WS2812CT_TIM->SMCR = 0;
    WS2812CT_TIM->PSC = 0;                // run at tim_clk_hz
    WS2812CT_TIM->ARR = s_arr;            // period

    // CH3 PWM1 + preload
    WS2812CT_TIM->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_CC3S_Msk);
    WS2812CT_TIM->CCMR2 |= (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; // PWM1, preload
    WS2812CT_TIM->CCER  &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    WS2812CT_TIM->CCER  |= TIM_CCER_CC3E;  // enable output

    WS2812CT_TIM->CCR3  = 0;              // idle low
    WS2812CT_TIM->BDTR |= TIM_BDTR_MOE;   // main output enable

    // Generate UG to latch
    WS2812CT_TIM->EGR = TIM_EGR_UG;

    // Clear pending flags
    WS2812CT_TIM->SR = 0;
}

void ws2812ct_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812CT_LED_COUNT) return;
    uint32_t base = (uint32_t)index*3u;
    s_buf[base+0] = g;
    s_buf[base+1] = r;
    s_buf[base+2] = b;
}

void ws2812ct_fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint32_t i=0;i<WS2812CT_LED_COUNT;i++) {
        uint32_t base=i*3u; s_buf[base]=g; s_buf[base+1]=r; s_buf[base+2]=b;
    }
}

static inline void _wait_uif_then_clear(void) {
    while ((WS2812CT_TIM->SR & TIM_SR_UIF)==0) { /*spin*/ }
    WS2812CT_TIM->SR = ~TIM_SR_UIF; // write 0 clears bit (others kept 0)
}

void ws2812ct_tx_blocking(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // 预装第一个bit占空比，然后启动定时器；之后每个周期边界更新下一bit
    const uint32_t total_bytes = WS2812CT_LED_COUNT * 3u;
    uint32_t byte_idx = 0;
    int bit = 7;

    // Program first bit duty
    uint8_t v = s_buf[0];
    WS2812CT_TIM->CCR3 = ((v>>bit)&1u) ? s_t1h : s_t0h;
    WS2812CT_TIM->EGR = TIM_EGR_UG;          // apply preload before start
    WS2812CT_TIM->CR1 |= TIM_CR1_CEN;        // start

    while (1) {
        _wait_uif_then_clear();               // wait period end
        // advance bit
        if (--bit < 0) {
            bit = 7;
            if (++byte_idx >= total_bytes) break;  // finished all bytes
            v = s_buf[byte_idx];
        }
        // program next bit for upcoming period
        WS2812CT_TIM->CCR3 = ((v>>bit)&1u) ? s_t1h : s_t0h;
    }

    // 发送复位：拉低若干周期（将 CCR3=0 保持 N 个周期）
    WS2812CT_TIM->CCR3 = 0;
    // 需要 >= 50us；周期 = 1/800k = 1.25us => 至少 40 周期，取 80 周期更稳
    const uint32_t reset_periods = (WS2812CT_RESET_US * WS2812CT_BITRATE_HZ) / 1000000u + 4u; // ≈80
    for (uint32_t i=0;i<reset_periods;i++) { _wait_uif_then_clear(); }

    // 停止定时器，输出保持低
    WS2812CT_TIM->CR1 &= ~TIM_CR1_CEN;

    if (!primask) __enable_irq();
}
