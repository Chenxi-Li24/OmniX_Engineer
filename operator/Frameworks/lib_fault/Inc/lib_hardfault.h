// SPDX-License-Identifier: MIT
// lib_hardfault.h — Rich HardFault diagnostics for STM32H7 using usart1_critical
#pragma once
#include <stdint.h>
#include "stm32h7xx.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* ================= User config (可在编译参数或包含前覆盖) ================= */
    // USART1 kernel 时钟（你的板子：PLL2Q=25MHz）
#ifndef USART1C_KERNEL_HZ_OVERRIDE
#define USART1C_KERNEL_HZ_OVERRIDE  25000000u
#endif

    // 紧急串口波特率
#ifndef LIB_HF_USART_BAUD
#define LIB_HF_USART_BAUD           115200u
#endif

    // 发生 HardFault 后是否自动复位（1=复位；0=停机死循环）
#ifndef LIB_HF_AUTO_RESET
#define LIB_HF_AUTO_RESET           1
#endif

    /* ================= 对外 API ================= */

    // 可选：提前初始化（通常不需要，库会在 handle 里自初始化）
    void lib_hf_init(void);

    // 由异常处理入口调用的核心处理函数（r0=SP, r1=EXC_RETURN）
    void lib_hf_handle(uint32_t *stack_ptr, uint32_t exc_return);

    // 在 HAL 的 HardFault_Handler() 中调用此宏即可把 SP/EXC_RETURN 传给 lib_hf_handle()
#define LIB_HF_HANDLE_IN_HAL()                   \
__asm volatile (                             \
"tst lr, #4            \n"               \
"ite eq                \n"               \
"mrseq r0, msp         \n"               \
"mrsne r0, psp         \n"               \
"mov   r1, lr          \n"               \
"b     lib_hf_handle   \n"               \
)

#ifdef __cplusplus
}
#endif