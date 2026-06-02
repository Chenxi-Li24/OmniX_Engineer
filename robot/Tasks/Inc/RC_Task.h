#pragma once
/*
 * RC_Task - glue task for feeding lib_remote_control
 *
 * 只做三件事：
 *   1) 初始化 lib_remote_control + 两个 BSP (DR16 / VT03)
 *   2) 周期性 Poll 两个遥控器，拿到解码帧后调用 RC_UpdateFromDR16_Adapter / RC_UpdateFromVT03
 *   3) 调 RC_Tick() 并在在线状态发生变化时打印一次日志
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    /* FreeRTOS 入口 */
    void Start_RC_Task(void const * argument);

    /* 可选：默认 1ms；需要的话在编译选项里 -DRC_TASK_POLL_PERIOD_MS=2 覆盖 */
#ifndef RC_TASK_POLL_PERIOD_MS
#define RC_TASK_POLL_PERIOD_MS   (1u)
#endif

/* Unified S0 switch state for gating */
#define RC_S0_STATE_DOWN         (0u)
#define RC_S0_STATE_MID          (1u)
#define RC_S0_STATE_UP           (2u)
#ifndef RC_S0_STABLE_MS
#define RC_S0_STABLE_MS          (20u)
#endif
extern volatile uint8_t rc_s0_state;

/* DR16 logging: 1=enable 0=disable */
#ifndef RC_TASK_LOG_DR16
#define RC_TASK_LOG_DR16         (0u)
#endif

#ifdef __cplusplus
}
#endif
