// RC_Task.c - Source-agnostic remote control ingestion task
// VT03 input path is disabled to release UART9 for serial servo.

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lib_remote_control.h"
#include "RC_Task.h"

volatile uint8_t rc_s0_state = RC_S0_STATE_DOWN;

static void rc_update_s0_state(uint32_t now_ms)
{
    RC_State rc;
    RC_GetSnapshot(&rc);

    uint8_t raw = RC_S0_STATE_DOWN;
    if (switch_is_mid(rc.rc.s[0])) {
        raw = RC_S0_STATE_MID;
    } else if (switch_is_up(rc.rc.s[0])) {
        raw = RC_S0_STATE_UP;
    }

    static uint8_t last_raw = RC_S0_STATE_DOWN;
    static uint32_t last_change_ms = 0;
    if (raw != last_raw) {
        last_raw = raw;
        last_change_ms = now_ms;
    }

    if ((now_ms - last_change_ms) >= RC_S0_STABLE_MS) {
        rc_s0_state = raw;
    }
}

void Start_RC_Task(void const *argument)
{
    (void)argument;

    RC_InputInit();
    /* Keyboard/mouse from DR16 must always win; ignore the custom-controller path. */
    RC_SetSourcePriority(RC_SRC_DR16, RC_SRC_NONE);

    const TickType_t period = pdMS_TO_TICKS(RC_TASK_POLL_PERIOD_MS);

    for (;;) {
        const uint32_t now = HAL_GetTick();
        rc_update_s0_state(now);
        RC_Tick(now);
        vTaskDelay(period);
    }
}
