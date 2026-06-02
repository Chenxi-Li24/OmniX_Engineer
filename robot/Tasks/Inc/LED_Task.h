//
// Created by sirin on 2025/9/29.
//

#ifndef LED_TASK_H
#define LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif
    void Start_LED_Task(void const * argument);
    void LED_NotifyCalibStart(void);
#ifdef __cplusplus
}
#endif

extern volatile int is_rc_online;

#endif /* LED_TASK_H */

