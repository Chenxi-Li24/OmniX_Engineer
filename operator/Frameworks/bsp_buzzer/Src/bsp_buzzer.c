//
// Created by sirin on 2025/10/1.
//
#include "bsp_buzzer.h"
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>

/* 可选端口适配 */
#ifdef __has_include
  #if __has_include("buzzer_port.h")
    #include "buzzer_port.h"
  #endif
#endif

/* ---------------- BSRR 双字方波表（50%占空） ---------------- */
static uint32_t bsrr_table[2] = {
    (1u << BUZZER_PIN),          /* set */
    (1u << (BUZZER_PIN + 16u))   /* reset */
};

/* ---------------- 句柄 ---------------- */
static TIM_HandleTypeDef  htim_buz;
static DMA_HandleTypeDef  hdma_buz;

/* ---------------- 运行状态 ---------------- */
static volatile uint8_t   s_running = 0;     /* TIM 正在跑 */
static volatile uint32_t  s_curr_freq = 0;   /* 当前频率，0=静音 */

static float s_ref_a4 = 440.0f;

/* ---------------- 简易环形队列（无锁化-短临界区） ---------------- */
typedef struct {
    BuzzerNote buf[BUZZER_QUEUE_CAPACITY];
    volatile uint16_t head; /* 生产者写 */
    volatile uint16_t tail; /* 消费者读 */
} buzzer_queue_t;

static buzzer_queue_t s_q = { .head = 0, .tail = 0 };

/* ---------------- 时间源（弱符号，可被覆盖） ---------------- */
__attribute__((weak))
uint32_t Buzzer_TimeNowMs(void) {
    /* 默认返回 0；建议你在应用里覆盖为：SysTick 计数、HAL_GetTick() 或 DWT->CYCCNT/时钟换算 */
    extern uint32_t HAL_GetTick(void);
    return HAL_GetTick(); /* 如果工程里有 HAL_GetTick，这里能直接工作 */
}

/* ---------------- 时钟工具 ---------------- */
static uint32_t BUZZER_GetTIMClock(void)
{
    /* 以 TIM2@APB1 为例：APB预分频!=1时，定时器时钟=2*PCLK */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clk;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clk, &latency);
    return (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (pclk1 * 2u);
}

/* ---------------- 内部：原子设置频率 -> 改 ARR 立即生效 ---------------- */
static void buzzer_apply_freq(uint32_t f_hz)
{
    if (f_hz == 0) {
        /* 静音：关 DMA，停 TIM，拉低 */
        __HAL_TIM_DISABLE_DMA(&htim_buz, TIM_DMA_UPDATE);
        HAL_TIM_Base_Stop(&htim_buz);
        HAL_DMA_Abort(&hdma_buz);
        BUZZER_GPIO_PORT->BSRR = (1u << (BUZZER_PIN + 16u));
        s_running  = 0;
        s_curr_freq= 0;
        return;
    }

    uint32_t timclk   = BUZZER_GetTIMClock();
    uint32_t updates  = f_hz * 2u;                /* 一次更新翻转一次 BSRR，2 个更新一周期 */
    if (updates == 0 || updates > timclk) {
        /* 频率过低/过高，直接静音避免除零/溢出 */
        buzzer_apply_freq(0);
        return;
    }
    uint32_t arr = (timclk / updates);
    if (arr == 0) arr = 1;                        /* 最小 ARR=1 */
    arr -= 1u;

    if (!s_running) {
        /* 首次启动：准备 TIM 基本参数 */
        __HAL_TIM_DISABLE(&htim_buz);
        __HAL_TIM_SET_PRESCALER(&htim_buz, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim_buz, arr);
        __HAL_TIM_SET_COUNTER(&htim_buz, 0);

        /* 启动 DMA 循环搬运到 GPIO BSRR */
        HAL_DMA_Start(&hdma_buz, (uint32_t)bsrr_table, (uint32_t)&BUZZER_GPIO_PORT->BSRR, 2u);
        __HAL_TIM_ENABLE_DMA(&htim_buz, TIM_DMA_UPDATE);
        HAL_TIM_Base_Start(&htim_buz);
        s_running = 1;
    } else {
        /* 运行中仅更新 ARR 并强制更新，避免停顿 */
        htim_buz.Instance->ARR = arr;
        htim_buz.Instance->EGR = TIM_EGR_UG;  /* 立即装载新 ARR；会有轻微相位跳变 */
    }

    s_curr_freq = f_hz;
}

/* ---------------- 公共 API 实现 ---------------- */

void Buzzer_Init(void)
{
    /* 1) GPIO */
    BUZZER_GPIO_CLK_EN();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = (1u << BUZZER_PIN);
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    /* 默认拉低 */
    BUZZER_GPIO_PORT->BSRR = (1u << (BUZZER_PIN + 16u));

    /* 2) TIM */
    BUZZER_TIM_CLK_EN();
    htim_buz.Instance           = BUZZER_TIM;
    htim_buz.Init.Prescaler     = 0;
    htim_buz.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim_buz.Init.Period        = 1000;
    htim_buz.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim_buz);

    /* 3) DMA + DMAMUX */
    __HAL_RCC_DMA1_CLK_ENABLE();
#if defined(__HAL_RCC_DMAMUX1_CLK_ENABLE)
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
#elif defined(__HAL_RCC_DMAMUX_CLK_ENABLE)
    __HAL_RCC_DMAMUX_CLK_ENABLE();
#else
#warning "No DMAMUX clock-enable macro found. Check HAL version."
#endif

    hdma_buz.Instance                 = BUZZER_DMA;
    hdma_buz.Init.Request             = BUZZER_DMA_REQ;
    hdma_buz.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_buz.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_buz.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_buz.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_buz.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_buz.Init.Mode                = DMA_CIRCULAR;
    hdma_buz.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_buz);

    __HAL_LINKDMA(&htim_buz, hdma[TIM_DMA_ID_UPDATE], hdma_buz);

    s_running = 0;
    s_curr_freq = 0;
}

void Buzzer_Start(uint32_t freq_hz)
{
    BUZZER_ENTER_CRITICAL();
    buzzer_apply_freq(freq_hz);
    BUZZER_EXIT_CRITICAL();
}

void Buzzer_SetFrequency(uint32_t freq_hz)
{
    BUZZER_ENTER_CRITICAL();
    buzzer_apply_freq(freq_hz);
    BUZZER_EXIT_CRITICAL();
}

void Buzzer_Stop(void)
{
    BUZZER_ENTER_CRITICAL();
    buzzer_apply_freq(0);
    BUZZER_EXIT_CRITICAL();
}

void Buzzer_SetRefA4(float hz)
{
    if (hz < 200.0f) hz = 200.0f;
    if (hz > 500.0f) hz = 500.0f;
    s_ref_a4 = hz;
}

/* ----------- 音名解析（与你原来的实现兼容） ----------- */

static int note_letter_to_index(char n)
{
    switch ((char)toupper((unsigned char)n)) {
        case 'C': return 0;
        case 'D': return 2;
        case 'E': return 4;
        case 'F': return 5;
        case 'G': return 7;
        case 'A': return 9;
        case 'B': return 11;
        default:  return -1;
    }
}

static int parse_accidental(const char *p, int *consumed)
{
    *consumed = 0;
    if (*p == '#') { *consumed = 1; return +1; }
    if (*p == 'b') { *consumed = 1; return -1; }
    return 0;
}

static bool parse_int(const char *p, int *out, int *consumed)
{
    int i = 0, sign = 1, val = 0;
    if (p[i] == '+') { i++; }
    else if (p[i] == '-') { sign = -1; i++; }
    if (!isdigit((unsigned char)p[i])) { *consumed = 0; return false; }
    while (isdigit((unsigned char)p[i])) { val = val*10 + (p[i]-'0'); i++; }
    *out = sign*val; *consumed = i; return true;
}

bool Buzzer_start_note(const char *note)
{
    if (!note) return false;

    if (toupper((unsigned char)note[0]) == 'R') {
        Buzzer_Stop();
        return true;
    }

    int idx = note_letter_to_index(note[0]);
    if (idx < 0) return false;

    int acc_c = 0;
    int acc = parse_accidental(&note[1], &acc_c);

    int octave = 4, oct_c = 0;
    if (note[1 + acc_c] != '\0') {
        (void)parse_int(&note[1 + acc_c], &octave, &oct_c);
    }

    int semitone = idx + acc;
    while (semitone < 0)  { semitone += 12; octave -= 1; }
    while (semitone > 11) { semitone -= 12; octave += 1; }

    int midi = (octave + 1) * 12 + semitone;
    if (midi < 0) midi = 0;
    if (midi > 127) midi = 127;

    double n = (double)(midi - 69) / 12.0;
    double freq = (double)s_ref_a4 * pow(2.0, n);
    if (freq < 1.0) return false;

    uint32_t f_hz = (uint32_t)(freq + 0.5);
    Buzzer_SetFrequency(f_hz);
    return true;
}

/* ---------------- 队列工具 ---------------- */

static inline uint16_t q_next(uint16_t x) { return (uint16_t)((x + 1u) % BUZZER_QUEUE_CAPACITY); }

bool Buzzer_Enqueue(BuzzerNote n)
{
    bool ok = false;
    BUZZER_ENTER_CRITICAL();
    uint16_t h = s_q.head, t = s_q.tail;
    uint16_t nh = q_next(h);
    if (nh != t) { /* not full */
        s_q.buf[h] = n;
        s_q.head = nh;
        ok = true;
    }
    BUZZER_EXIT_CRITICAL();
    return ok;
}

bool Buzzer_EnqueueMany(const BuzzerNote *arr, uint16_t n)
{
    if (!arr || n == 0) return true;
    bool ok = true;
    BUZZER_ENTER_CRITICAL();
    for (uint16_t i = 0; i < n; ++i) {
        uint16_t nh = q_next(s_q.head);
        if (nh == s_q.tail) { ok = false; break; } /* full */
        s_q.buf[s_q.head] = arr[i];
        s_q.head = nh;
    }
    BUZZER_EXIT_CRITICAL();
    return ok;
}

void Buzzer_Flush(void)
{
    BUZZER_ENTER_CRITICAL();
    s_q.head = s_q.tail = 0;
    buzzer_apply_freq(0);
    BUZZER_EXIT_CRITICAL();
}

bool Buzzer_QueueEmpty(void)
{
    bool empty;
    BUZZER_ENTER_CRITICAL();
    empty = (s_q.head == s_q.tail);
    BUZZER_EXIT_CRITICAL();
    return empty;
}

bool Buzzer_IsActive(void)
{
    return (s_running && s_curr_freq > 0);
}

/* ---------------- 软调度器 ---------------- */

void Buzzer_Service(void)
{
    /* 基于“绝对到期时间”的非阻塞推进 */
    static uint32_t s_deadline_ms = 0;
    uint32_t now = Buzzer_TimeNowMs();

    /* 若当前未发声且队列非空 -> 拉取一个并开始 */
    if (!Buzzer_IsActive()) {
        BUZZER_ENTER_CRITICAL();
        if (s_q.head != s_q.tail) {
            BuzzerNote n = s_q.buf[s_q.tail];
            s_q.tail = q_next(s_q.tail);
            BUZZER_EXIT_CRITICAL();

            Buzzer_SetFrequency(n.freq_hz);
            s_deadline_ms = now + (uint32_t)n.dur_ms;
        } else {
            BUZZER_EXIT_CRITICAL();
            return;
        }
    } else {
        /* 正在发声：检查是否到期，到了就切换到下一个 */
        if ((int32_t)(now - s_deadline_ms) >= 0) {
            BUZZER_ENTER_CRITICAL();
            if (s_q.head != s_q.tail) {
                BuzzerNote n = s_q.buf[s_q.tail];
                s_q.tail = q_next(s_q.tail);
                BUZZER_EXIT_CRITICAL();

                Buzzer_SetFrequency(n.freq_hz);
                s_deadline_ms = now + (uint32_t)n.dur_ms;
            } else {
                BUZZER_EXIT_CRITICAL();
                /* 队列空：停止 */
                Buzzer_Stop();
            }
        }
    }
}