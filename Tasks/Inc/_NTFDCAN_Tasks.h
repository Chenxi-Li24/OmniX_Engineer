#ifndef H723VG_V2_FREERTOS__NTFDCAN_TASKS_H
#define H723VG_V2_FREERTOS__NTFDCAN_TASKS_H

#include <stdint.h>
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

    // 任务属性（这里只声明，cpp 里“非 static”定义）
    extern const osThreadAttr_t attr_canrx1;
    extern const osThreadAttr_t attr_canrx2;
    extern const osThreadAttr_t attr_canrx3;

    // 一键启动三个 Router
    void Start_AllRouters(void);

#ifndef CAN_BUS_ARG
#define CAN_BUS_ARG(bus) ((void*)(uintptr_t)(bus))
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif // H723VG_V2_FREERTOS__NTFDCAN_TASKS_H