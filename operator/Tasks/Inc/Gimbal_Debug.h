#ifndef H723VG_V2_FREERTOS_GIMBAL_DEBUG_H
#define H723VG_V2_FREERTOS_GIMBAL_DEBUG_H
#pragma once

#include "../../Frameworks/lib_struct_typedef/Inc/struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  fp32 loop_target;
  fp32 loop_actual;
  fp32 pos_target;
  fp32 pos_actual;
  fp32 vel_target;
  fp32 vel_actual;
  fp32 torque_target;
  fp32 torque_actual;
  fp32 p_out;
  fp32 i_out;
  fp32 d_out;
  fp32 pid_out;
  uint8_t online;
  uint8_t mode;
  uint16_t _reserved;
} GimbalPidDebug;

extern volatile GimbalPidDebug gimbal_dbg_lk_j3;
extern volatile GimbalPidDebug gimbal_dbg_lk_j2;
extern volatile GimbalPidDebug gimbal_dbg_lk_j1;

extern volatile GimbalPidDebug gimbal_dbg_dm_j4;
extern volatile GimbalPidDebug gimbal_dbg_dm_j5;
extern volatile GimbalPidDebug gimbal_dbg_dm_j6;
extern volatile GimbalPidDebug gimbal_dbg_dm_j7;

extern volatile GimbalPidDebug gimbal_dbg_dm_j8;
extern volatile GimbalPidDebug gimbal_dbg_dm_j9;

#ifdef __cplusplus
}
#endif

#endif
