//
// Created by sirin on 2025/9/29.
//

#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    void Start_LED_Task(void const * argument);
    void LED_NotifyCalibStart(void);
    void LED_NotifyPauseFlash(uint32_t duration_ms, uint32_t period_ms);
    void LED_SetPauseFlash(uint8_t enable, uint32_t period_ms);
#ifdef __cplusplus
}
#endif

extern volatile int is_rc_online;

#endif /* LED_TASK_H */

