#ifndef H723VG_V2_FREERTOS_GIMBAL_BEHAVIOR_TASK_H
#define H723VG_V2_FREERTOS_GIMBAL_BEHAVIOR_TASK_H
#pragma once
#include <stdbool.h>
#include <cstdint>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

    // 行为枚举（精简到：失能/相对角/静止）
    typedef enum {
        GIMBAL_ZERO_FORCE = 0,
        GIMBAL_RELATIVE_ANGLE,
        GIMBAL_MOTIONLESS
      } gimbal_behaviour_e;

    // 行为任务输出给执行任务的“期望快照”
    typedef struct {
        gimbal_behaviour_e behaviour; // 选择 ENCODER 相对角模式时有效
        float yaw_set_rad;            // 设定角度         //add_yaw;// 增量（rad）：行为层计算的“当前 tick 的期望变化”
        float add_pitch;// 增量（rad）
        float add_jaw;
        uint8_t  reset_zero_axis;
        uint32_t seq;                 // 快照序号（避免读写撕裂）
    } gimbal_cmd_t;

    typedef struct {
        uint16_t raw_u16[9];
        int16_t angle_cdeg[9];
        uint8_t online[9];
    } gimbal_joint_snapshot_t;

    typedef enum {
        GIMBAL_PAUSE_IDLE = 0,
        GIMBAL_PAUSE_PRE_BEEP,
        GIMBAL_PAUSE_RUN,
        GIMBAL_PAUSE_DONE_BEEP,
        GIMBAL_PAUSE_ABORT
    } gimbal_pause_phase_e;

    // 任务入口（命名延续原来“一行为一执行”）
    void Start_Gimbal_behave(void *argument);
    void Start_Gimbal_Task(void* argument);
    bool Gimbal_ReadJointSnapshot(gimbal_joint_snapshot_t* out);
    bool Gimbal_ReadPauseCmdCdeg(int16_t out_cdeg[9], uint8_t* phase);
    void Gimbal_RequestMappingReset(void);
    bool Gimbal_IsMappingResetLocalDone(void);

    // 全局旗标（与 Chassis 风格一致）
    extern volatile bool gimbal_all_online;   // 两轴都在线才 true
    extern volatile bool gimbal_zero_force;   // 统一失能判据
    extern volatile bool frontFLAG;

#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_GIMBAL_BEHAVIOR_TASK_H
