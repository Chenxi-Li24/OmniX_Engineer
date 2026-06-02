//
// Created by sirin on 2025/10/5.
//

#ifndef H723VG_V2_FREERTOS_CONF_CHASSIS_TASK_H
#define H723VG_V2_FREERTOS_CONF_CHASSIS_TASK_H

// Config/Chassis_Tunables.hpp
#pragma once
#include <cstdint>
#include <type_traits>

#include "../../Frameworks/lib_struct_typedef/Inc/struct_typedef.h"

inline constexpr uint8_t CHASSIS_6020_RB_ID = 1;
inline constexpr uint8_t CHASSIS_6020_RF_ID = 2;
inline constexpr uint8_t CHASSIS_6020_LF_ID = 3;
inline constexpr uint8_t CHASSIS_6020_LB_ID = 4;

inline constexpr uint8_t CHASSIS_6020_RB_BUS = 1;
inline constexpr uint8_t CHASSIS_6020_RF_BUS = 1;
inline constexpr uint8_t CHASSIS_6020_LF_BUS = 1;
inline constexpr uint8_t CHASSIS_6020_LB_BUS = 1;

inline constexpr uint8_t CHASSIS_C620_RB_ID = 1;
inline constexpr uint8_t CHASSIS_C620_RF_ID = 2;
inline constexpr uint8_t CHASSIS_C620_LF_ID = 3;
inline constexpr uint8_t CHASSIS_C620_LB_ID = 4;

inline constexpr uint8_t CHASSIS_C620_RB_BUS = 2;
inline constexpr uint8_t CHASSIS_C620_RF_BUS = 2;
inline constexpr uint8_t CHASSIS_C620_LF_BUS = 2;
inline constexpr uint8_t CHASSIS_C620_LB_BUS = 2;

inline constexpr uint16_t MOTOR_OFFLINE_MS = 100;

inline constexpr int32_t SERVO_INIT[4] = { 398, 2408, 7125, 6512 }; // 舵机初始编码器值  398.0 2408.0 7125.0 6512.0
static constexpr uint8_t CHASSIS_NAVISION_BUS = 1;
static constexpr uint16_t CHASSIS_NAVISION_STD_ID = NAVISION_DEFAULT_STD_ID;

// 舵机方向与零位偏置（按索引：0 RB, 1 RF, 2 LF, 3 LB）
// - SERVO_THETA_OFFSET_DEG：每个舵的角度偏置（单位deg），设置为 180.f 可实现“零位翻转180°”；默认全为 0.f。
// - SERVO_DIR：每个舵当前向号（+1 正常；-1 线序反/反装时反向电流）。默认全为 +1。
static const float SERVO_THETA_OFFSET_DEG[4] = { -90.f, 90.f, 90.f, -90.f };
static const int   SERVO_DIR[4]              = { +1,  +1,  +1,  +1  };
// 驱动轮方向系数（顺序：RB, RF, LF, LB）
// +1：保持；-1：该轮驱动方向取反（仅用于“反馈域 + 输出域”映射）
static const int WHEEL_DIR[4] = { -1, +1, -1, +1 }; // RB/LF 反向，RF/LB 正向
// 哈基米

float alpha = 1.3f;        //Sup_Cap
float alpha_slope = 4.2f;  // 飞坡



float scale_normal_list[11] = {
    1.4f, 1.4f, 1.45f, 1.5f, 1.55f,
    1.75f, 1.8f, 1.9f, 1.95f, 2.0f,
    2.1f
};
float scale_rotate_list[11] = {
    2.0f, 2.0f, 2.2f, 2.5f, 2.7f,
    3.1f, 3.2f, 3.4f, 3.8f, 3.8f,
    3.9f
};
float scale_moving_rotate_list[11] = {
    1.4f, 1.4f, 1.5f, 1.6f, 1.65f,
    1.75f, 1.8f, 1.9f, 1.95f, 2.0f,
    2.1f
};

inline constexpr fp32 PI_F = 3.14159265358979323846f;
inline constexpr fp32 SQRT2 = 1.41421356237309504880f;
inline constexpr fp32 WHEEL_DIAMETER = 0.116f;
inline constexpr fp32 ROTATE_RADIUS = 0.235484f;
inline constexpr fp32 CHASSISw_TO_CHASSISv_RATIO = ROTATE_RADIUS / SQRT2;
inline constexpr fp32 ECD_PPR_6020 = 8192.0f;
inline constexpr fp32 CHASSIS_HALF_L = 0.165f;   // 车体前后半距（wheelbase/2），单位 m
inline constexpr fp32 CHASSIS_HALF_W = 0.169f;   // 车体左右半距（trackwidth/2），单位 m
// 轮子位置（与电机 ID 对应顺序）：0 ID1, 1 ID2, 2 ID3, 3 ID4
// 坐标系：x 前为正，y 左为正
static const fp32 WHEEL_POS_X[4] = { +CHASSIS_HALF_L, +CHASSIS_HALF_L, -CHASSIS_HALF_L, -CHASSIS_HALF_L };
static const fp32 WHEEL_POS_Y[4] = { +CHASSIS_HALF_W, -CHASSIS_HALF_W, -CHASSIS_HALF_W, +CHASSIS_HALF_W };
inline constexpr fp32 CHASSIS_ANTI_HYST_DEG = 6.0f;
inline constexpr fp32 CHASSIS_SPEED_EPS = 0.05f;
inline constexpr fp32 CHASSIS_XLOCK_IDLE_VXY_EPS = CHASSIS_SPEED_EPS;
inline constexpr fp32 CHASSIS_XLOCK_IDLE_WZ_EPS = 0.05f;
inline constexpr fp32 CHASSIS_XLOCK_THETA_SHIFT_DEG = 90.0f;

// PID参数（轮电机分离，默认一致）
static const fp32 WHEEL_VELOCITY_PID_KP[4] = { 50.0f, 50.0f, 50.0f, 50.0f };
static const fp32 WHEEL_VELOCITY_PID_KI[4] = { 0.5f, 0.5f, 0.5f, 0.5f };
static const fp32 WHEEL_VELOCITY_PID_KD[4] = { 150.0f, 150.0f, 150.0f, 150.0f };
inline constexpr fp32 CHASSIS_DRIVE_GIVE_CMD_MAX = 16384.0f;
inline constexpr fp32 CHASSIS_DRIVE_GIVE_CMD_IOUT_MAX = 600.0f;
inline constexpr uint32_t CHASSIS_DRIVE_SAT_LOG_PERIOD_MS = 1000u;
static const fp32 WHEEL_VELOCITY_PID_MAX_OUT[4] = {
    CHASSIS_DRIVE_GIVE_CMD_MAX, CHASSIS_DRIVE_GIVE_CMD_MAX,
    CHASSIS_DRIVE_GIVE_CMD_MAX, CHASSIS_DRIVE_GIVE_CMD_MAX
};
static const fp32 WHEEL_VELOCITY_PID_MAX_IOUT[4] = {
    CHASSIS_DRIVE_GIVE_CMD_IOUT_MAX, CHASSIS_DRIVE_GIVE_CMD_IOUT_MAX,
    CHASSIS_DRIVE_GIVE_CMD_IOUT_MAX, CHASSIS_DRIVE_GIVE_CMD_IOUT_MAX
};

inline constexpr fp32 SERVO_VELOCITY_PID_KP = 10.0f;
inline constexpr fp32 SERVO_VELOCITY_PID_KI = 1.0f;
inline constexpr fp32 SERVO_VELOCITY_PID_KD = 0.0f;
inline constexpr fp32 SERVO_VELOCITY_PID_MAX_OUT = 15000.0f;
inline constexpr fp32 SERVO_VELOCITY_PID_MAX_IOUT = 100.0f;

inline constexpr fp32 SERVO_ANGLE_PID_KP = 70.0f;
inline constexpr fp32 SERVO_ANGLE_PID_KI = 0.0f;
inline constexpr fp32 SERVO_ANGLE_PID_KD = 0.0f;
inline constexpr fp32 SERVO_ANGLE_PID_MAX_OUT = 3000.0f;
inline constexpr fp32 SERVO_ANGLE_PID_MAX_IOUT = 0.0f;


///////////////////////// follow /////////////////////////
inline constexpr fp32 YAW_VEL_KP       = 150.0f;
inline constexpr fp32 YAW_VEL_KI       = 0.0f;
inline constexpr fp32 YAW_VEL_KD       = 0.2f;
inline constexpr fp32 YAW_VEL_MAX_OUT  = 100.0f;
inline constexpr fp32 YAW_VEL_MAX_IOUT = 0.0f;

inline constexpr fp32 YAW_ANGLE_KP       = 15.0f;
inline constexpr fp32 YAW_ANGLE_KI       = 0.0f;
inline constexpr fp32 YAW_ANGLE_KD       = 0.35f;
inline constexpr fp32 YAW_ANGLE_MAX_OUT  = 8.0f;     // rad/s ≈ 458°/s
inline constexpr fp32 YAW_ANGLE_MAX_IOUT = 0.0f;
/*
#define YAW_ANGLE_KP        50.0f
#define YAW_ANGLE_KI        0.0f
#define YAW_ANGLE_KD        100.0f
#define YAW_ANGLE_MAX_OUT   400.0f
#define YAW_ANGLE_MAX_IOUT  0.0f
*/
// 常用宏
#ifdef MAX
#undef MAX
#endif
#ifdef MIN
#undef MIN
#endif
#ifdef ABS
#undef ABS
#endif
#ifdef NORM_ANGLE
#undef NORM_ANGLE
#endif
#ifdef NORM_ANGLE_RAD
#undef NORM_ANGLE_RAD
#endif
#ifdef CLIP
#undef CLIP
#endif
#ifdef COMPARE_ABS
#undef COMPARE_ABS
#endif

template <typename T, typename U>
inline constexpr auto MAX(T a, U b) -> std::common_type_t<T, U> {
  using R = std::common_type_t<T, U>;
                     return (a > b) ? static_cast<R>(a) : static_cast<R>(b);
}

template <typename T, typename U>
inline constexpr auto MIN(T a, U b) -> std::common_type_t<T, U> {
  using R = std::common_type_t<T, U>;
  return (a < b) ? static_cast<R>(a) : static_cast<R>(b);
}

template <typename T>
inline constexpr T ABS(T x) {
  return (x > static_cast<T>(0)) ? x : static_cast<T>(-x);
}

inline constexpr fp32 NORM_ANGLE(fp32 x) {
  return (x > 180.0f) ? (x - 360.0f) : ((x < -180.0f) ? (x + 360.0f) : x);
}

inline constexpr fp32 NORM_ANGLE_RAD(fp32 x) {
  return (x > PI_F) ? (x - 2.0f * PI_F) : ((x < -PI_F) ? (x + 2.0f * PI_F) : x);
}

template <typename T, typename U, typename V>
inline constexpr auto CLIP(T value, U min_value, V max_value) -> std::common_type_t<T, U, V> {
  using R = std::common_type_t<T, U, V>;
  const R v = static_cast<R>(value);
  const R lo = static_cast<R>(min_value);
  const R hi = static_cast<R>(max_value);
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

inline constexpr fp32 COMPARE_ABS(fp32 a, fp32 b, fp32 c) {
  return (ABS(NORM_ANGLE_RAD(a - c)) > ABS(NORM_ANGLE_RAD(b - c))) ? b : a;
}

// 遥控器相关宏
inline constexpr fp32 MAX_VELOCITY_FORWARD = 5.0f;
inline constexpr fp32 MAX_VELOCITY_RIGHT   = 5.0f;
inline constexpr fp32 MAX_OMEGA            = 18.0f * PI_F;
inline constexpr fp32 MAX_VELOCITY_ROTATE  = MAX_OMEGA * ROTATE_RADIUS;
inline constexpr fp32 RC_SEN_FORWARD       = MAX_VELOCITY_FORWARD / -660.0f;
inline constexpr fp32 RC_SEN_RIGHT         = MAX_VELOCITY_RIGHT  / -660.0f;
inline constexpr fp32 RC_SEN_ROTATE        = MAX_VELOCITY_ROTATE / 800.0f;

// 主yaw电机相关
inline constexpr uint16_t P_YAW_INIT_ECD         = 28260;
inline constexpr fp32     P_YAW_IMU_VEL_KP       = 1.5f;
inline constexpr fp32     P_YAW_IMU_VEL_KI       = 0.0f;
inline constexpr fp32     P_YAW_IMU_VEL_KD       = 0.0f;
inline constexpr fp32     P_YAW_IMU_VEL_MAX_OUT  = 550.0f;
inline constexpr fp32     P_YAW_IMU_VEL_MAX_IOUT = 0.0f;
inline constexpr fp32 P_YAW_IMU_ANGLE_KP       = 50.0f;
inline constexpr fp32 P_YAW_IMU_ANGLE_KI       = 0.0f;
inline constexpr fp32 P_YAW_IMU_ANGLE_KD       = 100.0f;
inline constexpr fp32 P_YAW_IMU_ANGLE_MAX_OUT  = 400.0f;
inline constexpr fp32 P_YAW_IMU_ANGLE_MAX_IOUT = 0.0f;

inline constexpr fp32 P_YAW_ECD_ANGLE_KP       = 0.3f;
inline constexpr fp32 P_YAW_ECD_ANGLE_KI       = 0.0f;
inline constexpr fp32 P_YAW_ECD_ANGLE_KD       = 0.0f;
inline constexpr fp32 P_YAW_ECD_ANGLE_MAX_OUT  = 300.0f;
inline constexpr fp32 P_YAW_ECD_ANGLE_MAX_IOUT = 0.0f;

inline constexpr fp32 CHASSIS_ACCEL_VX_NUM = 0.3333333333f;
inline constexpr fp32 CHASSIS_ACCEL_VY_NUM = 0.3333333333f;
inline constexpr fp32 CHASSIS_ACCEL_WZ_NUM = 0.3333333333f;

inline constexpr fp32 CHASSIS_FOLLOW_KP = 4.0f;

#ifndef CHASSIS_SAFE_HOLD_LOG_GIMBAL_J1_J9
#define CHASSIS_SAFE_HOLD_LOG_GIMBAL_J1_J9 1
#endif

#ifndef CHASSIS_DIAG_ENABLE
#define CHASSIS_DIAG_ENABLE 1
#endif

#endif //H723VG_V2_FREERTOS_CONF_CHASSIS_TASK_H
