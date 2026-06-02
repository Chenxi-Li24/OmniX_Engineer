#include "bsp_ws2812.h"
#include "stm32h7xx_hal_dma.h"

/* === D-Cache feature + header availability (order-agnostic) === */
#ifdef OMNIX_FEATURE_MPU_DCACHE
  #if __has_include("mem_sections.h")
    #include "mem_sections.h"
  #else
    /* Fallback: header not visible -> degrade to normal RAM */
    #define SEC_DMA_NC
    #define SEC_DMA_NC_BUF
    #define ALIGN32
  #endif
#else
  /* Global feature off -> no-op macros */
  #define SEC_DMA_NC
  #define SEC_DMA_NC_BUF
  #define ALIGN32
#endif

static DMA_HandleTypeDef s_dma;
static volatile uint8_t  s_busy = 0;

static uint16_t  s_psc = 0, s_arr = 0;
static uint32_t  s_cnt_freq = 0;

#define WS2812_BUF_LEN   (WS2812_LED_COUNT * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS)

/* NOTE:
 * - SEC_DMA_NC_BUF already carries ALIGN(32) inside mem_sections.h (our recommended version).
 * - So do NOT add another __attribute__((aligned(32))) here, to avoid duplicate attributes.
 */
static uint16_t s_buf_front[WS2812_BUF_LEN] SEC_DMA_NC_BUF;
static uint16_t s_buf_back [WS2812_BUF_LEN] SEC_DMA_NC_BUF;

/* duty（计数） */
static uint16_t s_duty_t0h = 0;
static uint16_t s_duty_t1h = 0;


static uint8_t  s_brightness = 255;
static ws2812_hooks_t s_hooks = {0};

/* ================== tools ================== */

static void _ws_stop_hw(void)
{
    WS2812_TIM->DIER &= ~TIM_DIER_UDE;
    WS2812_TIM->CR1  &= ~TIM_CR1_CEN;
}

static void _compute_tim_params(uint32_t tim_clk_hz, uint32_t target_bit_hz,
                                uint16_t *psc_out, uint16_t *arr_out, uint32_t *cnt_freq_out)
{
    const uint32_t target_counts = 115u;
    uint32_t presc = ((uint64_t)tim_clk_hz + (uint64_t)target_bit_hz*target_counts/2) /
                     ((uint64_t)target_bit_hz*target_counts);
    if (presc == 0) presc = 1;
    presc -= 1;
    if (presc > 0xFFFFu) presc = 0xFFFFu;

    uint32_t cnt_freq = tim_clk_hz / (presc + 1u);
    uint32_t arr = ((uint64_t)cnt_freq + target_bit_hz/2) / target_bit_hz;
    if (arr == 0) arr = 1;
    arr -= 1;
    if (arr > 0xFFFFu) arr = 0xFFFFu;

    *psc_out = (uint16_t)presc;
    *arr_out = (uint16_t)arr;
    *cnt_freq_out = cnt_freq;
}

static uint16_t _ns_to_counts(uint32_t cnt_freq, uint32_t ns)
{
    uint64_t num = (uint64_t)cnt_freq * (uint64_t)ns + 500000000ull;
    return (uint16_t)(num / 1000000000ull);
}

static void _gpio_port_clk_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
}

static void _gpio_init(void)
{
    _gpio_port_clk_enable(WS2812_GPIO_PORT);

    GPIO_InitTypeDef gi = {0};
    gi.Pin       = WS2812_GPIO_PIN;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = WS2812_GPIO_AF;
    HAL_GPIO_Init(WS2812_GPIO_PORT, &gi);
}

static void _tim_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    WS2812_TIM->CR1  = TIM_CR1_ARPE;  // 仅开 ARPE
    WS2812_TIM->CR2  = 0;
    WS2812_TIM->SMCR = 0;

    WS2812_TIM->PSC = s_psc;
    WS2812_TIM->ARR = s_arr;

    WS2812_TIM->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_CC3S_Msk);
    WS2812_TIM->CCMR2 |= (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; // PWM1 + preload

    WS2812_TIM->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    WS2812_TIM->CCER |= TIM_CCER_CC3E;

    WS2812_TIM->CCR3 = 0;
    WS2812_TIM->BDTR |= TIM_BDTR_MOE;

    WS2812_TIM->EGR = TIM_EGR_UG;
}

static void _dmamux_dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
#if defined(__HAL_RCC_DMAMUX1_CLK_ENABLE)
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
#elif defined(__HAL_RCC_DMAMUX_CLK_ENABLE)
    __HAL_RCC_DMAMUX_CLK_ENABLE();
#endif

    s_dma.Instance                 = WS2812_DMA_STREAM;
    s_dma.Init.Request             = WS2812_DMA_REQUEST;
    s_dma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_dma.Init.MemInc              = DMA_MINC_ENABLE;
    s_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_dma.Init.Mode                = DMA_NORMAL;
    s_dma.Init.Priority            = DMA_PRIORITY_LOW;
    s_dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    (void)HAL_DMA_Abort(&s_dma);
    HAL_DMA_Init(&s_dma);

#if defined(HAL_DMA_RegisterCallback)
    __HAL_DMA_DISABLE_IT(&s_dma, DMA_IT_HT);
    HAL_DMA_RegisterCallback(&s_dma, HAL_DMA_XFER_CPLT_CB_ID,  NULL);
    HAL_DMA_RegisterCallback(&s_dma, HAL_DMA_XFER_ERROR_CB_ID, NULL);
#endif

    HAL_NVIC_SetPriority(WS2812_DMA_IRQn, WS2812_DMA_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(WS2812_DMA_IRQn);
}

static inline uint8_t _apply_brightness(uint8_t v)
{
    if (s_brightness == 255) return v;
    // 线性缩放（可换gamma）
    return (uint8_t)((v * (uint16_t)s_brightness) / 255u);
}

static void _encode_led_to(uint16_t *buf, uint16_t led_index, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t base = (uint32_t)led_index * WS2812_BITS_PER_LED;
    uint8_t grb[3] = { _apply_brightness(g), _apply_brightness(r), _apply_brightness(b) };

    for (uint32_t c = 0; c < 3; c++) {
        for (int bit = 7; bit >= 0; bit--) {
            buf[base++] = ((grb[c] >> bit) & 0x1) ? s_duty_t1h : s_duty_t0h;
        }
    }
}

static void _append_reset_slots_to(uint16_t *buf)
{
    for (uint32_t i = WS2812_LED_COUNT * WS2812_BITS_PER_LED; i < WS2812_BUF_LEN; i++) {
        buf[i] = 0;
    }
}

/* ================== APIs ================== */

void WS2812_Init(uint32_t tim_clk_hz)
{
    _compute_tim_params(tim_clk_hz, WS2812_BITRATE_HZ, &s_psc, &s_arr, &s_cnt_freq);

    uint32_t period_counts = (uint32_t)s_arr + 1u;
    uint16_t t0 = _ns_to_counts(s_cnt_freq, WS2812_T0H_NS);
    uint16_t t1 = _ns_to_counts(s_cnt_freq, WS2812_T1H_NS);
    if (t0 >= period_counts) t0 = (uint16_t)(period_counts - 1u);
    if (t1 >= period_counts) t1 = (uint16_t)(period_counts - 1u);
    s_duty_t0h = t0;
    s_duty_t1h = t1;

    _gpio_init();
    _tim_init();
    _dmamux_dma_init();

    // Clean both buffer
    for (uint16_t i = 0; i < WS2812_LED_COUNT; ++i) {
        _encode_led_to(s_buf_back , i, 0,0,0);
        _encode_led_to(s_buf_front, i, 0,0,0);
    }
    _append_reset_slots_to(s_buf_back);
    _append_reset_slots_to(s_buf_front);

    WS2812_TIM->DIER &= ~TIM_DIER_UDE;
}

void WS2812_SetGlobalBrightness(uint8_t scale)
{
    s_brightness = scale;
}

/**************************
 * @brief Set single pixel in "back" buffer (non-blocking)
 * @param index: index of LED, 0-based
 * @param r: Red (0-255)
 * @param g: Green (0-255)
 * @param b: Blue (0-255)
 **************************/
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_COUNT) return;
    _encode_led_to(s_buf_back, index, r, g, b);
}

/**************************
 * @brief Quickly fill all LEDs in "back" buffer (non-blocking)
 * @param r: Red (0-255)
 * @param g: Green (0-255)
 * @param b: Blue (0-255)
 **************************/
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++) {
        _encode_led_to(s_buf_back, i, r, g, b);
    }
    _append_reset_slots_to(s_buf_back);
}

HAL_StatusTypeDef WS2812_Commit_StartAsync(const ws2812_hooks_t *hooks)
{
    if (s_busy) return HAL_BUSY;
    s_busy = 1;

    // record hooks
    if (hooks) s_hooks = *hooks;
    else       s_hooks = (ws2812_hooks_t){0};

    // exchange front/back
    for (uint32_t i = 0; i < WS2812_BUF_LEN; ++i) {
        s_buf_front[i] = s_buf_back[i];
    }

    uint32_t periph_addr = (uint32_t)&(WS2812_TIM->CCR3);

    __HAL_DMA_CLEAR_FLAG(&s_dma, DMA_FLAG_TCIF1_5);
    HAL_StatusTypeDef st = HAL_DMA_Start_IT(&s_dma,
                                            (uint32_t)s_buf_front,
                                            periph_addr,
                                            WS2812_BUF_LEN);
    if (st != HAL_OK) {
        s_busy = 0;
        return st;
    }

    WS2812_TIM->DIER |= TIM_DIER_UDE;
    WS2812_TIM->CR1  |= TIM_CR1_CEN;

    return HAL_OK;
}

uint8_t WS2812_IsBusy(void) { return s_busy; }

void WS2812_WaitIdle(void)
{
    if (!s_busy) return;
    uint32_t ndtr_last = __HAL_DMA_GET_COUNTER(&s_dma);
    uint32_t watchdog  = 1000000u;

    while (watchdog--) {
        uint32_t ndtr = __HAL_DMA_GET_COUNTER(&s_dma);
        if (ndtr == 0u) break;
        if (ndtr != ndtr_last) { ndtr_last = ndtr; watchdog = 1000000u; }
        __NOP();
    }

    for (volatile int i = 0; i < 512; ++i) { __NOP(); }
    _ws_stop_hw();
    s_busy = 0;
}

/* ================== IRQ ================== */

void WS2812_DMA_IRQHandler(void)
{
    // Read and clear flags
    HAL_DMA_IRQHandler(&s_dma);

    // Decide if done
    if (__HAL_DMA_GET_COUNTER(&s_dma) == 0) {
        _ws_stop_hw();
        s_busy = 0;

        // Callback
        if (s_hooks.on_tx_done) s_hooks.on_tx_done(s_hooks.user);
        if (s_hooks.notify_from_isr) s_hooks.notify_from_isr(s_hooks.notify_arg);
        return;
    }

    // Error handling
    if (__HAL_DMA_GET_FLAG(&s_dma, DMA_FLAG_TEIF1_5)) {
        _ws_stop_hw();
        s_busy = 0;
        if (s_hooks.on_tx_error) s_hooks.on_tx_error(s_hooks.user);
        if (s_hooks.notify_from_isr) s_hooks.notify_from_isr(s_hooks.notify_arg);
        return;
    }
}