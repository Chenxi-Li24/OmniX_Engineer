// Tasks/Inc/Chassis_Task.h
// ----------------------------------------------------------------------------
// 底盘任务对外接口（面向其他模块的最小暴露面）
// 与 Tasks/Src/Chassis_Task.cpp 的实现配套
// ----------------------------------------------------------------------------

#pragma once
#ifndef H723VG_V2_FREERTOS_CHASSIS_TASK_H
#define H723VG_V2_FREERTOS_CHASSIS_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
    /**
     * @brief FreeRTOS 任务入口：底盘主循环（2ms节拍）
     * @param argument 传入参数，通常为 NULL
     */
    void Start_Chassis_Task(void *argument);

    /**
     * @brief 全局：8个电机（4×GM6020 + 4×C620）RX是否全部在线
     *        由 Chassis_Task 周期更新；仅供只读
     */
    extern volatile bool chassis_motor_all_online;

    /**
     * @brief 全局：底盘是否处于 Zero-Force（无力/失能）状态
     *        - 由 Chassis_Task 在每个 tick 内更新：当拨杆为 NoForce 或 RC 离线/阻塞时为 true
     *        - 其他任务（如 Shoot）可据此统一失能判据，避免重复判定导致模式不一致
     */
    extern volatile bool chassis_zero_force;

#ifdef __cplusplus
} // extern "C"
#endif

#endif // H723VG_V2_FREERTOS_CHASSIS_TASK_H
//变量的公开用extern
//函数公开在.h里写明原型（返回类型 + 函数名 + 参数列表）;不要函数体
//结构体和类要公开，必须在.h里写完整，否则其他文件不可创建实例。
//实例的内容传递，一般靠函数。在云台里公开函数可以被底盘调用，底盘传入地址类型，该函数可以访问云台的实例，将值传入指定地址。