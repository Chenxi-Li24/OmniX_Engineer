#ifndef H723VG_V2_FREERTOS_CONF_GIMBAL_TASK_H
#define H723VG_V2_FREERTOS_CONF_GIMBAL_TASK_H
#pragma once

#include <cmath>
#include <cstdint>

#include "../../Frameworks/lib_adp_dm_4310/Inc/lib_adp_dm_4310.h"
#include "../../Frameworks/lib_adp_lk_mg8016e/Inc/lib_adp_lk_mg8016e.h"
#include "CONF_Gimbal_Zero.h"
#include "Servo_Joint_Angle_Protocol.h"

// Motor layout:
// J1/J2/J3 -> LK8016E
// J5/J6/J7/J9 -> DM4310
// J4/J8 -> DM4340

enum class GimbalMotorPosRefMode : uint8_t {
  RelativeMultiTurn = 0u,
  AbsoluteSingleTurn = 1u,
  AbsoluteFiniteTurn = 2u,
};

// ------------------- DM motor ID/Bus -------------------
inline constexpr uint16_t GIMBAL_DM_J5_ID  = 5;
inline constexpr uint16_t GIMBAL_DM_J5_BUS = 3;

inline constexpr uint16_t GIMBAL_DM_J6_ID  = 6;
inline constexpr uint16_t GIMBAL_DM_J6_BUS = 3;

inline constexpr uint16_t GIMBAL_DM_J7_ID  = 7;
inline constexpr uint16_t GIMBAL_DM_J7_BUS = 3;

inline constexpr uint16_t GIMBAL_DM_J4_ID  = 4;
inline constexpr uint16_t GIMBAL_DM_J4_BUS = 1;

inline constexpr uint16_t GIMBAL_DM_J8_ID  = 8;
inline constexpr uint16_t GIMBAL_DM_J8_BUS = 3;

inline constexpr uint16_t GIMBAL_DM_J9_ID  = 9;
inline constexpr uint16_t GIMBAL_DM_J9_BUS = 3;

// ------------------- LK motor ID/Bus -------------------
inline constexpr uint16_t GIMBAL_LK_J3_ID  = 3;
inline constexpr uint16_t GIMBAL_LK_J3_BUS = 1;
inline constexpr uint16_t GIMBAL_LK_J2_ID  = 2;
inline constexpr uint16_t GIMBAL_LK_J2_BUS = 3;
inline constexpr uint16_t GIMBAL_LK_J1_ID  = 1;
inline constexpr uint16_t GIMBAL_LK_J1_BUS = 2;

inline constexpr uint32_t GIMBAL_LK_TX_PERIOD_MS        = 2;
inline constexpr uint32_t GIMBAL_LK_ENABLE_RESEND_MS    = 100;

inline constexpr uint16_t DM4310_FB_MASTER_SID = 0x00;
inline constexpr uint16_t DM4340_FB_MASTER_SID = 0x00;

inline constexpr uint16_t GIMBAL_DM4310_FB_SID = DM4310_FB_MASTER_SID;
inline constexpr uint16_t GIMBAL_DM_J4_FB_SID = DM4340_FB_MASTER_SID;
inline constexpr uint16_t GIMBAL_DM_J8_FB_SID = DM4340_FB_MASTER_SID;
// Legacy shared alias (kept for compatibility with existing code paths).
inline constexpr uint16_t GIMBAL_DM4340_FB_SID = DM4340_FB_MASTER_SID;

// ------------------- Power-on zero encoder config (raw) -------------------
#define GIMBAL_DM_J5_POWERON_ZERO_RAW GIMBAL_DM_J5_ABS_ZERO_RAW
#define GIMBAL_DM_J6_POWERON_ZERO_RAW GIMBAL_DM_J6_ABS_ZERO_RAW
#define GIMBAL_DM_J7_POWERON_ZERO_RAW GIMBAL_DM_J7_ABS_ZERO_RAW
#define GIMBAL_DM_J4_POWERON_ZERO_RAW GIMBAL_DM_J4_ABS_ZERO_RAW

// DM power-on home uses the absolute single-turn encoder zero above.
// This is startup auto-alignment to configured zero, not mechanical limit homing.
inline constexpr bool GIMBAL_DM_J5_POWERON_HOME_ENABLE = true;
inline constexpr bool GIMBAL_DM_J6_POWERON_HOME_ENABLE = true;
inline constexpr bool GIMBAL_DM_J7_POWERON_HOME_ENABLE = false;
inline constexpr bool GIMBAL_DM_J4_POWERON_HOME_ENABLE = true;
inline constexpr bool GIMBAL_DM_J8_POWERON_HOME_ENABLE = true;
inline constexpr bool GIMBAL_DM_J9_POWERON_HOME_ENABLE = true;
inline constexpr uint32_t GIMBAL_DM_J8_POWERON_HOME_MS = 3000u;
inline constexpr uint32_t GIMBAL_DM_J9_POWERON_HOME_MS = 3000u;

// Backward-compatible zero aliases. Canonical values live in CONF_Gimbal_Zero.h.
#define GIMBAL_LK_J3_POWERON_ZERO_RAW GIMBAL_LK_J3_ABS_ZERO_RAW
#define GIMBAL_LK_J2_POWERON_ZERO_RAW GIMBAL_LK_J2_ABS_ZERO_RAW
#define GIMBAL_LK_J1_POWERON_ZERO_RAW GIMBAL_LK_J1_ABS_ZERO_RAW

// ------------------- Control-layer zero references -------------------
#define GIMBAL_DM_J5_ZERO_OFFSET_RAD GIMBAL_DM_J5_ABS_ZERO_RAD
inline constexpr fp32 GIMBAL_DM_J5_NEGATIVE_ZERO_OFFSET_RAD = 2.93f; // legacy reference only
#define GIMBAL_DM_J6_ZERO_OFFSET_RAD GIMBAL_DM_J6_ABS_ZERO_RAD
inline constexpr uint16_t GIMBAL_DM_J7_ZERO_REF_RAW = GIMBAL_DM_J7_ABS_ZERO_RAW;
#define GIMBAL_DM_J7_ZERO_OFFSET_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J7_ZERO_REF_RAW))
#define GIMBAL_DM_J4_ZERO_OFFSET_RAD GIMBAL_DM_J4_ABS_ZERO_RAD
inline constexpr fp32 GIMBAL_DM_SINGLE_TURN_SPAN_RAD = DM4310_POS_RAD_MAX - DM4310_POS_RAD_MIN;

inline constexpr GimbalMotorPosRefMode GIMBAL_DM_J5_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteFiniteTurn;
inline constexpr GimbalMotorPosRefMode GIMBAL_DM_J6_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteSingleTurn;
inline constexpr GimbalMotorPosRefMode GIMBAL_DM_J7_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteSingleTurn;
inline constexpr GimbalMotorPosRefMode GIMBAL_DM_J4_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteSingleTurn;

inline constexpr fp32 GIMBAL_DM_J5_ABS_WINDOW_TURNS = 3.0f;
inline constexpr fp32 GIMBAL_DM_J5_ABS_WINDOW_HALF_RAD =
    0.5f * GIMBAL_DM_J5_ABS_WINDOW_TURNS * GIMBAL_DM_SINGLE_TURN_SPAN_RAD;
inline constexpr fp32 GIMBAL_DM_J5_ABS_MIN_REL_RAD = -GIMBAL_DM_J5_ABS_WINDOW_HALF_RAD;
inline constexpr fp32 GIMBAL_DM_J5_ABS_MAX_REL_RAD =  GIMBAL_DM_J5_ABS_WINDOW_HALF_RAD;

// ------------------- Relative angle limits (rad) -------------------
inline constexpr fp32 GIMBAL_DM_HALF_TURN_HALF_RAD = 1.57079632679f;
inline constexpr fp32 GIMBAL_DM_J5_MIN_REL_RAD = -1145.0f;
inline constexpr fp32 GIMBAL_DM_J5_MAX_REL_RAD =  1145.0f;
inline constexpr fp32 GIMBAL_DM_J5_ZERO_SHIFT_RAD = -4.367f; // legacy manual trim, not power-on authority

inline constexpr fp32 GIMBAL_DM_J6_MIN_REL_RAD = -GIMBAL_DM_HALF_TURN_HALF_RAD;
inline constexpr fp32 GIMBAL_DM_J6_MAX_REL_RAD =  GIMBAL_DM_HALF_TURN_HALF_RAD;
inline constexpr fp32 GIMBAL_DM_J6_ZERO_SHIFT_RAD = -0.261f; // legacy manual trim

inline constexpr fp32 GIMBAL_DM_J7_MIN_REL_RAD =
    DM4310_PosRawToRad(29670u) - GIMBAL_DM_J7_ZERO_OFFSET_RAD;
inline constexpr fp32 GIMBAL_DM_J7_MAX_REL_RAD =
    DM4310_PosRawToRad(63108u) - GIMBAL_DM_J7_ZERO_OFFSET_RAD;
inline constexpr fp32 GIMBAL_DM_J7_ZERO_SHIFT_RAD = -0.367f; // legacy manual trim

inline constexpr fp32 GIMBAL_DM_J4_MIN_REL_RAD = -GIMBAL_DM_HALF_TURN_HALF_RAD;
inline constexpr fp32 GIMBAL_DM_J4_MAX_REL_RAD =  GIMBAL_DM_HALF_TURN_HALF_RAD;
inline constexpr fp32 GIMBAL_DM_J4_ZERO_SHIFT_RAD = -0.291f; // legacy manual trim

// ------------------- Angle-loop PID -------------------
inline constexpr fp32 GIMBAL_DM_J5_ANG_KP       = 30.0f;
inline constexpr fp32 GIMBAL_DM_J5_ANG_KI       = 0.02f;
inline constexpr fp32 GIMBAL_DM_J5_ANG_KD       = 0.3f;
inline constexpr fp32 GIMBAL_DM_J5_ANG_MAX_IOUT = 3.0f;
inline constexpr fp32 GIMBAL_DM_J5_ANG_MAX_OUT  = 10.0f;

inline constexpr fp32 GIMBAL_DM_J6_ANG_KP       = 3.5f;
inline constexpr fp32 GIMBAL_DM_J6_ANG_KI       = 0.001f;
inline constexpr fp32 GIMBAL_DM_J6_ANG_KD       = 0.12f;
inline constexpr fp32 GIMBAL_DM_J6_ANG_MAX_IOUT = 2.0f;
inline constexpr fp32 GIMBAL_DM_J6_ANG_MAX_OUT  = 10.0f;

inline constexpr fp32 GIMBAL_DM_J7_ANG_KP       = 3.5f;
inline constexpr fp32 GIMBAL_DM_J7_ANG_KI       = 0.0f;
inline constexpr fp32 GIMBAL_DM_J7_ANG_KD       = 0.08f;
inline constexpr fp32 GIMBAL_DM_J7_ANG_MAX_IOUT = 0.0f;
inline constexpr fp32 GIMBAL_DM_J7_ANG_MAX_OUT  = 4.0f;

inline constexpr fp32 GIMBAL_DM_J4_ANG_KP       = 3.5f;
inline constexpr fp32 GIMBAL_DM_J4_ANG_KI       = 0.001f;
inline constexpr fp32 GIMBAL_DM_J4_ANG_KD       = 0.12f;
inline constexpr fp32 GIMBAL_DM_J4_ANG_MAX_IOUT = 2.0f;
inline constexpr fp32 GIMBAL_DM_J4_ANG_MAX_OUT  = 10.0f;

// ------------------- Input sensitivity / deadband -------------------
inline constexpr uint16_t RC_DEADBAND = 20;
inline constexpr fp32 GIMBAL_DM_J5_RC_SEN = 0.000005f;
inline constexpr fp32 GIMBAL_DM_J5_RC_SEN_ANG = 6.28f / 660.0f;
inline constexpr fp32 GIMBAL_DM_J6_RC_SEN = -0.00005f;
inline constexpr fp32 GIMBAL_DM_J7_RC_SEN = 0.00005f;
inline constexpr fp32 GIMBAL_DM_J4_RC_SEN = 0.00003f;
inline constexpr fp32 GIMBAL_DM_J7_VT03_WHEEL_MAX_RATE_DPS = 60.0f;

inline constexpr uint16_t GIMBAL_DM_J5_DEADBAND = RC_DEADBAND;
inline constexpr uint16_t GIMBAL_DM_J6_DEADBAND = RC_DEADBAND;
inline constexpr uint16_t GIMBAL_DM_J4_DEADBAND = RC_DEADBAND;
inline constexpr uint16_t GIMBAL_DM_J7_DEADBAND = RC_DEADBAND;
inline constexpr uint32_t GIMBAL_DM_J7_OVERCURRENT_CLEAR_STABLE_MS = 300u;
inline constexpr int16_t GIMBAL_CUSTOM_DM_SYNC_WINDOW_CDEG = 200;
inline constexpr int16_t GIMBAL_CUSTOM_DM_RUN_DEADBAND_CDEG = 200;

struct GimbalRawLinearMapCfg {
  uint16_t controller_raw_min;
  uint16_t controller_raw_max;
  uint16_t motor_raw_min;
  uint16_t motor_raw_max;
  bool wrap_enable;
};

struct GimbalRawPiecewiseMapCfg {
  uint16_t controller_raw_min;
  uint16_t controller_raw_mid;
  uint16_t controller_raw_max;
  uint16_t motor_raw_min;
  uint16_t motor_raw_mid;
  uint16_t motor_raw_max;
  bool wrap_enable;
};

struct GimbalRawCenteredMapCfg {
  uint16_t controller_raw_min;
  uint16_t controller_raw_neutral;
  uint16_t controller_raw_max;
  uint16_t motor_raw_min;
  uint16_t motor_raw_neutral;
  uint16_t motor_raw_max;
};

struct GimbalRawCenteredMapResult {
  uint16_t motor_raw;
  fp32 controller_rel;
  fp32 motor_rel;
};

struct GimbalExternalBidirRelAngleMapCfg {
  omnix_controller_joint_raw_calib_t controller_calib;
  fp32 target_rel_min_rad;
  fp32 target_rel_max_rad;
  uint16_t external_deadband_raw;
};

struct GimbalExternalHalfRangeReverseMapCfg {
  uint16_t raw_at_motor_max;
  uint16_t raw_at_motor_min;
  fp32 target_at_raw_max_rel_rad;
  fp32 target_at_raw_min_rel_rad;
  uint16_t external_deadband_raw;
};

struct GimbalExternalBidirRelAngleMapResult {
  uint16_t clamped_raw;
  int32_t delta_raw;
  fp32 controller_rel_deg;
  fp32 target_rel_rad;
  bool hold_last_target;
};

inline constexpr uint16_t GIMBAL_LK_J1_ABS_LIMIT_MIN_RAW = 0u;
inline constexpr uint16_t GIMBAL_LK_J1_ABS_LIMIT_MAX_RAW = 60075u;
inline constexpr fp32 GIMBAL_LK_J1_GEAR_RATIO = 6.0f;
inline constexpr fp32 GIMBAL_LK_J1_ABS_LIMIT_MIN_DEG =
    LK8016E_EncoderRawToDeg(GIMBAL_LK_J1_ABS_LIMIT_MIN_RAW);
inline constexpr fp32 GIMBAL_LK_J1_ABS_LIMIT_MAX_DEG =
    LK8016E_EncoderRawToDeg(GIMBAL_LK_J1_ABS_LIMIT_MAX_RAW);
inline constexpr uint16_t GIMBAL_LK_J1_CAL_ZERO_RAW = 30004u;
inline constexpr fp32 GIMBAL_LK_J1_MECH_ABS_LIMIT_MIN_DEG =
    GIMBAL_LK_J1_ABS_LIMIT_MIN_DEG * GIMBAL_LK_J1_GEAR_RATIO;
inline constexpr fp32 GIMBAL_LK_J1_MECH_ABS_ZERO_DEG = 988.91f;
inline constexpr fp32 GIMBAL_LK_J1_MECH_ABS_LIMIT_MAX_DEG =
    GIMBAL_LK_J1_ABS_LIMIT_MAX_DEG * GIMBAL_LK_J1_GEAR_RATIO;
inline constexpr fp32 GIMBAL_LK_J1_MECH_MIN_REL_DEG =
    GIMBAL_LK_J1_MECH_ABS_LIMIT_MIN_DEG - GIMBAL_LK_J1_MECH_ABS_ZERO_DEG;
inline constexpr fp32 GIMBAL_LK_J1_MECH_MAX_REL_DEG =
    GIMBAL_LK_J1_MECH_ABS_LIMIT_MAX_DEG - GIMBAL_LK_J1_MECH_ABS_ZERO_DEG;

inline constexpr uint16_t GIMBAL_LK_J2_ABS_LIMIT_MIN_RAW = 0u;
inline constexpr uint16_t GIMBAL_LK_J2_ABS_LIMIT_MAX_RAW = 21607u;
inline constexpr fp32 GIMBAL_LK_J2_ABS_LIMIT_MIN_DEG =
    LK8016E_EncoderRawToDeg(GIMBAL_LK_J2_ABS_LIMIT_MIN_RAW);
inline constexpr fp32 GIMBAL_LK_J2_ABS_LIMIT_MAX_DEG =
    LK8016E_EncoderRawToDeg(GIMBAL_LK_J2_ABS_LIMIT_MAX_RAW);
inline constexpr fp32 GIMBAL_LK_J2_GEAR_RATIO =
    782.00f / GIMBAL_LK_J2_ABS_LIMIT_MAX_DEG;
inline constexpr fp32 GIMBAL_LK_J2_MECH_ABS_LIMIT_MIN_DEG =
    GIMBAL_LK_J2_ABS_LIMIT_MIN_DEG * GIMBAL_LK_J2_GEAR_RATIO;
inline constexpr fp32 GIMBAL_LK_J2_MECH_ABS_ZERO_DEG = 0.0f;
inline constexpr fp32 GIMBAL_LK_J2_MECH_ABS_LIMIT_MAX_DEG =
    GIMBAL_LK_J2_ABS_LIMIT_MAX_DEG * GIMBAL_LK_J2_GEAR_RATIO;
inline constexpr fp32 GIMBAL_LK_J2_MECH_MIN_REL_DEG =
    GIMBAL_LK_J2_MECH_ABS_LIMIT_MIN_DEG - GIMBAL_LK_J2_MECH_ABS_ZERO_DEG;
inline constexpr fp32 GIMBAL_LK_J2_MECH_MAX_REL_DEG =
    GIMBAL_LK_J2_MECH_ABS_LIMIT_MAX_DEG - GIMBAL_LK_J2_MECH_ABS_ZERO_DEG;

inline constexpr uint16_t GIMBAL_LK_J3_ABS_LIMIT_WRAP_MIN_RAW = 0u;
inline constexpr uint16_t GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_RAW = 23275u;
inline constexpr fp32 GIMBAL_LK_J3_ABS_LIMIT_WRAP_MIN_DEG =
    LK8016E_EncoderRawToDeg(GIMBAL_LK_J3_ABS_LIMIT_WRAP_MIN_RAW);
inline constexpr fp32 GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_DEG =
    LK8016E_EncoderRawToDeg(GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_RAW);
inline constexpr fp32 GIMBAL_LK_J3_GEAR_RATIO =
    767.14f / GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_DEG;
inline constexpr fp32 GIMBAL_LK_J3_MECH_ABS_LIMIT_MIN_DEG =
    GIMBAL_LK_J3_ABS_LIMIT_WRAP_MIN_DEG * GIMBAL_LK_J3_GEAR_RATIO;
inline constexpr fp32 GIMBAL_LK_J3_MECH_ABS_ZERO_DEG = 0.0f;
inline constexpr fp32 GIMBAL_LK_J3_MECH_ABS_LIMIT_MAX_DEG =
    GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_DEG * GIMBAL_LK_J3_GEAR_RATIO;
inline constexpr fp32 GIMBAL_LK_J3_MECH_MIN_REL_DEG =
    GIMBAL_LK_J3_MECH_ABS_LIMIT_MIN_DEG - GIMBAL_LK_J3_MECH_ABS_ZERO_DEG;
inline constexpr fp32 GIMBAL_LK_J3_MECH_MAX_REL_DEG =
    GIMBAL_LK_J3_MECH_ABS_LIMIT_MAX_DEG - GIMBAL_LK_J3_MECH_ABS_ZERO_DEG;

inline constexpr GimbalRawCenteredMapCfg GIMBAL_J1_RAW_MAP_CFG = {
    OMNIX_CTRL_J1_RAW_MIN, OMNIX_CTRL_J1_RAW_ZERO, OMNIX_CTRL_J1_RAW_MAX,
    GIMBAL_LK_J1_ABS_LIMIT_MIN_RAW, GIMBAL_LK_J1_CAL_ZERO_RAW, GIMBAL_LK_J1_ABS_LIMIT_MAX_RAW
};
inline constexpr GimbalRawLinearMapCfg GIMBAL_J2_RAW_MAP_CFG = {
    OMNIX_CTRL_J2_RAW_MIN, OMNIX_CTRL_J2_RAW_MAX,
    GIMBAL_LK_J2_ABS_LIMIT_MIN_RAW, GIMBAL_LK_J2_ABS_LIMIT_MAX_RAW,
    false
};
inline constexpr GimbalRawLinearMapCfg GIMBAL_J3_RAW_MAP_CFG = {
    OMNIX_CTRL_J3_RAW_MIN, OMNIX_CTRL_J3_RAW_MAX,
    GIMBAL_LK_J3_ABS_LIMIT_WRAP_MIN_RAW, GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_RAW,
    false
};
inline constexpr GimbalRawCenteredMapCfg GIMBAL_J4_RAW_MAP_CFG = {
    OMNIX_CTRL_J4_RAW_MIN, OMNIX_CTRL_J4_RAW_ZERO, OMNIX_CTRL_J4_RAW_MAX,
    45413u, 43786u, 10783u
};
inline constexpr GimbalRawCenteredMapCfg GIMBAL_J5_RAW_MAP_CFG = {
    OMNIX_CTRL_J5_RAW_MIN, OMNIX_CTRL_J5_RAW_ZERO, OMNIX_CTRL_J5_RAW_MAX,
    56718u, GIMBAL_DM_J5_ABS_ZERO_RAW, 12127u
};
inline constexpr GimbalRawCenteredMapCfg GIMBAL_J6_RAW_MAP_CFG = {
    OMNIX_CTRL_J6_RAW_MIN, OMNIX_CTRL_J6_RAW_ZERO, OMNIX_CTRL_J6_RAW_MAX,
    48233u, 40280u, 14916u
};

inline constexpr int32_t Gimbal_RawWrapSpan(uint16_t start_raw, uint16_t end_raw)
{
  const int32_t start = static_cast<int32_t>(start_raw);
  const int32_t end = static_cast<int32_t>(end_raw);
  return (end >= start) ? (end - start) : (end + 65536 - start);
}

inline constexpr int32_t Gimbal_RawWrapOffset(uint16_t start_raw, uint16_t value_raw)
{
  const int32_t start = static_cast<int32_t>(start_raw);
  const int32_t value = static_cast<int32_t>(value_raw);
  return (value >= start) ? (value - start) : (value + 65536 - start);
}

inline constexpr uint16_t Gimbal_RawWrapCompose(uint16_t start_raw, int32_t offset)
{
  int32_t value = static_cast<int32_t>(start_raw) + offset;
  while (value < 0) {
    value += 65536;
  }
  while (value >= 65536) {
    value -= 65536;
  }
  return static_cast<uint16_t>(value);
}

inline constexpr uint16_t Gimbal_MapRawLinearU16(uint16_t controller_raw, const GimbalRawLinearMapCfg& cfg)
{
  if (!cfg.wrap_enable) {
    const int32_t in_min = static_cast<int32_t>(cfg.controller_raw_min);
    const int32_t in_max = static_cast<int32_t>(cfg.controller_raw_max);
    int32_t in = static_cast<int32_t>(controller_raw);
    if (in <= in_min) {
      return cfg.motor_raw_min;
    }
    if (in >= in_max) {
      return cfg.motor_raw_max;
    }
    const int32_t in_span = in_max - in_min;
    const int32_t out_min = static_cast<int32_t>(cfg.motor_raw_min);
    const int32_t out_max = static_cast<int32_t>(cfg.motor_raw_max);
    const int32_t out_span = out_max - out_min;
    if (in_span == 0) {
      return cfg.motor_raw_min;
    }
    const int32_t mapped = out_min + ((in - in_min) * out_span + (in_span / 2)) / in_span;
    return static_cast<uint16_t>(mapped);
  }

  const int32_t in_span = Gimbal_RawWrapSpan(cfg.controller_raw_min, cfg.controller_raw_max);
  const int32_t out_span = Gimbal_RawWrapSpan(cfg.motor_raw_min, cfg.motor_raw_max);
  if (in_span == 0) {
    return cfg.motor_raw_min;
  }

  int32_t in_offset = Gimbal_RawWrapOffset(cfg.controller_raw_min, controller_raw);
  if (in_offset < 0) {
    in_offset = 0;
  }
  if (in_offset > in_span) {
    in_offset = in_span;
  }
  const int32_t out_offset = (in_offset * out_span + (in_span / 2)) / in_span;
  return Gimbal_RawWrapCompose(cfg.motor_raw_min, out_offset);
}

inline constexpr uint16_t Gimbal_MapRawPiecewiseU16(uint16_t controller_raw, const GimbalRawPiecewiseMapCfg& cfg)
{
  if (controller_raw <= cfg.controller_raw_mid) {
    return Gimbal_MapRawLinearU16(controller_raw, GimbalRawLinearMapCfg{
        cfg.controller_raw_min,
        cfg.controller_raw_mid,
        cfg.motor_raw_min,
        cfg.motor_raw_mid,
        cfg.wrap_enable
    });
  }

  return Gimbal_MapRawLinearU16(controller_raw, GimbalRawLinearMapCfg{
      cfg.controller_raw_mid,
      cfg.controller_raw_max,
      cfg.motor_raw_mid,
      cfg.motor_raw_max,
      cfg.wrap_enable
  });
}

inline fp32 Gimbal_NormalizeRawCenteredNoWrap(uint16_t raw,
                                              uint16_t raw_min,
                                              uint16_t raw_neutral,
                                              uint16_t raw_max)
{
  if (raw <= raw_neutral) {
    if (raw_neutral <= raw_min) {
      return 0.0f;
    }
    if (raw <= raw_min) {
      return -1.0f;
    }
    return -static_cast<fp32>(raw_neutral - raw) /
           static_cast<fp32>(raw_neutral - raw_min);
  }

  if (raw_max <= raw_neutral) {
    return 0.0f;
  }
  if (raw >= raw_max) {
    return 1.0f;
  }
  return static_cast<fp32>(raw - raw_neutral) /
         static_cast<fp32>(raw_max - raw_neutral);
}

inline constexpr int32_t Gimbal_RawShortestAbsSpan(uint16_t a_raw, uint16_t b_raw)
{
  const int32_t forward = Gimbal_RawWrapSpan(a_raw, b_raw);
  const int32_t backward = Gimbal_RawWrapSpan(b_raw, a_raw);
  return (forward <= backward) ? forward : backward;
}

inline GimbalRawCenteredMapResult Gimbal_MapRawCenteredU16(uint16_t controller_raw,
                                                           const GimbalRawCenteredMapCfg& cfg)
{
  const fp32 controller_rel = Gimbal_NormalizeRawCenteredNoWrap(
      controller_raw,
      cfg.controller_raw_min,
      cfg.controller_raw_neutral,
      cfg.controller_raw_max);

  if (controller_rel <= 0.0f) {
    const fp32 alpha = -controller_rel;
    const int32_t motor_delta =
        static_cast<int32_t>(cfg.motor_raw_min) - static_cast<int32_t>(cfg.motor_raw_neutral);
    const int32_t motor_raw = static_cast<int32_t>(cfg.motor_raw_neutral) +
                              static_cast<int32_t>(alpha * static_cast<fp32>(motor_delta) +
                                                   ((motor_delta >= 0) ? 0.5f : -0.5f));
    const fp32 motor_rel = -alpha * static_cast<fp32>(
        Gimbal_RawShortestAbsSpan(cfg.motor_raw_neutral, cfg.motor_raw_min));
    return {
        static_cast<uint16_t>(motor_raw),
        controller_rel,
        motor_rel
    };
  }

  const fp32 alpha = controller_rel;
  const int32_t motor_delta =
      static_cast<int32_t>(cfg.motor_raw_max) - static_cast<int32_t>(cfg.motor_raw_neutral);
  const int32_t motor_raw = static_cast<int32_t>(cfg.motor_raw_neutral) +
                            static_cast<int32_t>(alpha * static_cast<fp32>(motor_delta) +
                                                 ((motor_delta >= 0) ? 0.5f : -0.5f));
  const fp32 motor_rel = alpha * static_cast<fp32>(
      Gimbal_RawShortestAbsSpan(cfg.motor_raw_neutral, cfg.motor_raw_max));
  return {
      static_cast<uint16_t>(motor_raw),
      controller_rel,
      motor_rel
  };
}

inline fp32 Gimbal_MotorRawCenteredRel(uint16_t motor_raw, const GimbalRawCenteredMapCfg& cfg)
{
  return Gimbal_NormalizeRawCenteredNoWrap(
      motor_raw,
      cfg.motor_raw_min,
      cfg.motor_raw_neutral,
      cfg.motor_raw_max);
}

inline GimbalExternalBidirRelAngleMapResult Gimbal_MapExternalRawToBidirRelAngle(
    uint16_t controller_raw,
    const GimbalExternalBidirRelAngleMapCfg& cfg)
{
  const uint16_t clamped_raw = Omnix_ControllerClampRaw(controller_raw, &cfg.controller_calib);
  const int32_t delta_raw =
      static_cast<int32_t>(clamped_raw) - static_cast<int32_t>(cfg.controller_calib.raw_zero);
  const fp32 controller_rel_deg = Omnix_ControllerRawToRelDeg(clamped_raw, &cfg.controller_calib);

  if (std::abs(delta_raw) <= static_cast<int32_t>(cfg.external_deadband_raw)) {
    return {
        clamped_raw,
        delta_raw,
        controller_rel_deg,
        0.0f,
        true
    };
  }

  if (delta_raw < 0) {
    const int32_t neg_span_raw = Omnix_ControllerNegativeSpanRaw(&cfg.controller_calib);
    if (neg_span_raw <= static_cast<int32_t>(cfg.external_deadband_raw)) {
      return {
          clamped_raw,
          delta_raw,
          controller_rel_deg,
          0.0f,
          true
      };
    }
    return {
        clamped_raw,
        delta_raw,
        controller_rel_deg,
        (static_cast<fp32>(-delta_raw) / static_cast<fp32>(neg_span_raw)) * cfg.target_rel_min_rad,
        false
    };
  }

  {
    const int32_t pos_span_raw = Omnix_ControllerPositiveSpanRaw(&cfg.controller_calib);
    if (pos_span_raw <= static_cast<int32_t>(cfg.external_deadband_raw)) {
      return {
          clamped_raw,
          delta_raw,
          controller_rel_deg,
          0.0f,
          true
      };
    }
    return {
        clamped_raw,
        delta_raw,
        controller_rel_deg,
        (static_cast<fp32>(delta_raw) / static_cast<fp32>(pos_span_raw)) * cfg.target_rel_max_rad,
        false
    };
  }
}

inline GimbalExternalBidirRelAngleMapResult Gimbal_MapExternalRawToHalfRangeReverseRelAngle(
    uint16_t controller_raw,
    const GimbalExternalHalfRangeReverseMapCfg& cfg)
{
  uint16_t clamped_raw = controller_raw;
  if (clamped_raw <= cfg.raw_at_motor_max) {
    clamped_raw = cfg.raw_at_motor_max;
  } else if (clamped_raw >= cfg.raw_at_motor_min) {
    clamped_raw = cfg.raw_at_motor_min;
  }

  const int32_t delta_raw =
      static_cast<int32_t>(clamped_raw) - static_cast<int32_t>(cfg.raw_at_motor_max);
  const fp32 controller_rel_deg = static_cast<fp32>(delta_raw) * OMNIX_CONTROLLER_SERVO_DEG_PER_CODE;

  const int32_t span_raw =
      static_cast<int32_t>(cfg.raw_at_motor_min) - static_cast<int32_t>(cfg.raw_at_motor_max);
  if (span_raw <= 0) {
    return {
        clamped_raw,
        delta_raw,
        controller_rel_deg,
        cfg.target_at_raw_max_rel_rad,
        false
    };
  }

  // Half-range reverse mapping uses raw_at_motor_min as the neutral/hold side.
  // Apply deadband around that side so minor external-controller jitter does not
  // update the absolute target every 2 ms tick.
  const int32_t neutral_delta_raw =
      static_cast<int32_t>(cfg.raw_at_motor_min) - static_cast<int32_t>(clamped_raw);
  if (std::abs(neutral_delta_raw) <= static_cast<int32_t>(cfg.external_deadband_raw)) {
    return {
        clamped_raw,
        delta_raw,
        controller_rel_deg,
        cfg.target_at_raw_min_rel_rad,
        true
    };
  }

  const fp32 alpha = static_cast<fp32>(delta_raw) / static_cast<fp32>(span_raw);
  const fp32 target_rel_rad = cfg.target_at_raw_max_rel_rad +
                              alpha * (cfg.target_at_raw_min_rel_rad - cfg.target_at_raw_max_rel_rad);

  return {
      clamped_raw,
      delta_raw,
      controller_rel_deg,
      target_rel_rad,
      false
  };
}

inline uint16_t Gimbal_MapHalfRangeReverseRelAngleToExternalRaw(
    fp32 target_rel_rad,
    const GimbalExternalHalfRangeReverseMapCfg& cfg)
{
  const fp32 target_span = cfg.target_at_raw_min_rel_rad - cfg.target_at_raw_max_rel_rad;
  if ((target_span < 1e-6f) && (target_span > -1e-6f)) {
    return cfg.raw_at_motor_max;
  }

  fp32 alpha = (target_rel_rad - cfg.target_at_raw_max_rel_rad) / target_span;
  if (alpha < 0.0f) {
    alpha = 0.0f;
  } else if (alpha > 1.0f) {
    alpha = 1.0f;
  }

  const int32_t raw_span =
      static_cast<int32_t>(cfg.raw_at_motor_min) - static_cast<int32_t>(cfg.raw_at_motor_max);
  if (raw_span <= 0) {
    return cfg.raw_at_motor_max;
  }

  const int32_t mapped_raw =
      static_cast<int32_t>(cfg.raw_at_motor_max) +
      Omnix_RoundToI32(alpha * static_cast<fp32>(raw_span));
  if (mapped_raw <= static_cast<int32_t>(cfg.raw_at_motor_max)) {
    return cfg.raw_at_motor_max;
  }
  if (mapped_raw >= static_cast<int32_t>(cfg.raw_at_motor_min)) {
    return cfg.raw_at_motor_min;
  }
  return static_cast<uint16_t>(mapped_raw);
}

inline uint16_t Gimbal_MapRelAngleToExternalRaw(fp32 target_rel_rad,
                                                const GimbalExternalBidirRelAngleMapCfg& cfg)
{
  if (target_rel_rad <= 0.0f) {
    const int32_t neg_span_raw = Omnix_ControllerNegativeSpanRaw(&cfg.controller_calib);
    if (neg_span_raw <= static_cast<int32_t>(cfg.external_deadband_raw) ||
        (((cfg.target_rel_min_rad < 0.0f) ? -cfg.target_rel_min_rad : cfg.target_rel_min_rad) <= 1e-6f)) {
      return cfg.controller_calib.raw_zero;
    }
    {
      fp32 alpha = target_rel_rad / cfg.target_rel_min_rad;
      if (alpha < 0.0f) {
        alpha = 0.0f;
      } else if (alpha > 1.0f) {
        alpha = 1.0f;
      }
      return static_cast<uint16_t>(
          static_cast<int32_t>(cfg.controller_calib.raw_zero) -
          Omnix_RoundToI32(alpha * static_cast<fp32>(neg_span_raw)));
    }
  }

  {
    const int32_t pos_span_raw = Omnix_ControllerPositiveSpanRaw(&cfg.controller_calib);
    if (pos_span_raw <= static_cast<int32_t>(cfg.external_deadband_raw) ||
        (((cfg.target_rel_max_rad < 0.0f) ? -cfg.target_rel_max_rad : cfg.target_rel_max_rad) <= 1e-6f)) {
      return cfg.controller_calib.raw_zero;
    }
    {
      fp32 alpha = target_rel_rad / cfg.target_rel_max_rad;
      if (alpha < 0.0f) {
        alpha = 0.0f;
      } else if (alpha > 1.0f) {
        alpha = 1.0f;
      }
      return static_cast<uint16_t>(
          static_cast<int32_t>(cfg.controller_calib.raw_zero) +
          Omnix_RoundToI32(alpha * static_cast<fp32>(pos_span_raw)));
    }
  }
}

inline constexpr fp32 Gimbal_AbsFp32(fp32 v)
{
  return (v < 0.0f) ? -v : v;
}

inline constexpr fp32 Gimbal_MinFp32(fp32 a, fp32 b)
{
  return (a < b) ? a : b;
}

inline constexpr fp32 Gimbal_MaxFp32(fp32 a, fp32 b)
{
  return (a > b) ? a : b;
}

inline constexpr bool GIMBAL_J7_DIRECT_FOLLOW_TEST_ENABLE = true;
inline constexpr bool GIMBAL_J7_DIRECT_CAN_BYPASS_ENABLE = true;
inline constexpr bool GIMBAL_J7_HARD_OVERRIDE_ENABLE = true;
inline constexpr bool GIMBAL_J7_HARD_OVERRIDE_KEEP_LAST_TARGET = true;
inline constexpr bool GIMBAL_J7_GEAR_DISABLE_ENABLE = true;
inline constexpr bool GIMBAL_J7_GEAR_ALLOW_CHASSIS = true;
inline constexpr bool GIMBAL_J7_GEAR_ALLOW_GIMBAL = true;
inline constexpr bool GIMBAL_J7_GEAR_CLEAR_LATCH_ON_DISABLE = false;
inline constexpr bool GIMBAL_J7_DIRECT_MIT_ENABLE = true;
inline constexpr bool GIMBAL_J7_DIRECT_USE_ZERO_RAW = false;
inline constexpr bool GIMBAL_J7_DIRECT_ABS_RAW_TARGET_ENABLE = true;
inline constexpr bool GIMBAL_J6_DIRECT_SINGLE_TURN_SHORTEST_WRAP_ENABLE = false;
inline constexpr bool GIMBAL_J4_DIRECT_SINGLE_TURN_SHORTEST_WRAP_ENABLE = false;
inline constexpr bool GIMBAL_J7_DIRECT_SINGLE_TURN_SHORTEST_WRAP_ENABLE = false;
inline constexpr fp32 GIMBAL_J7_DIRECT_MIT_KP = 5.0f;
inline constexpr fp32 GIMBAL_J7_DIRECT_MIT_KD = 0.6f;
// J7 direct-test calibration:
// 1. Servo raw is clamped to the measured controller endpoints.
// 2. DM raw is obtained by linear interpolation between the measured motor endpoints.
// 3. In absolute-raw direct mode, mapped DM raw is the final target authority.
inline constexpr int16_t GIMBAL_J7_DIRECT_MAP_SERVO_RAW_MIN = OMNIX_CTRL_J7_RAW_MIN;
inline constexpr int16_t GIMBAL_J7_DIRECT_MAP_SERVO_RAW_MAX = OMNIX_CTRL_J7_RAW_MAX;
inline constexpr uint16_t GIMBAL_J7_DIRECT_MAP_DM_RAW_MIN = 62483u;
inline constexpr uint16_t GIMBAL_J7_DIRECT_MAP_DM_RAW_MAX = 30309u;
inline constexpr uint16_t GIMBAL_J7_DIRECT_MAP_DM_ZERO_RAW = GIMBAL_DM_J7_ZERO_REF_RAW;

// Backward-compatible aliases for existing direct-test code paths.
inline constexpr int16_t GIMBAL_J7_DIRECT_TEST_SERVO_RAW_MIN = GIMBAL_J7_DIRECT_MAP_SERVO_RAW_MIN;
inline constexpr int16_t GIMBAL_J7_DIRECT_TEST_SERVO_RAW_MAX = GIMBAL_J7_DIRECT_MAP_SERVO_RAW_MAX;
inline constexpr uint16_t GIMBAL_J7_DIRECT_TEST_DM_RAW_MIN = GIMBAL_J7_DIRECT_MAP_DM_RAW_MIN;
inline constexpr uint16_t GIMBAL_J7_DIRECT_TEST_DM_RAW_MAX = GIMBAL_J7_DIRECT_MAP_DM_RAW_MAX;
inline constexpr uint32_t GIMBAL_J7_DIRECT_TEST_LOG_PERIOD_MS = 250u;
inline constexpr GimbalRawLinearMapCfg GIMBAL_J7_RAW_MAP_CFG = {
    static_cast<uint16_t>(GIMBAL_J7_DIRECT_MAP_SERVO_RAW_MIN),
    static_cast<uint16_t>(GIMBAL_J7_DIRECT_MAP_SERVO_RAW_MAX),
    GIMBAL_J7_DIRECT_MAP_DM_RAW_MIN,
    GIMBAL_J7_DIRECT_MAP_DM_RAW_MAX,
    false
};

inline constexpr uint16_t GIMBAL_J1_EXTERNAL_DEADBAND_RAW = 10u;
inline constexpr bool GIMBAL_LK_J1_EXT_FILTER_ENABLE = false;
inline constexpr fp32 GIMBAL_LK_J1_EXT_FILTER_TAU_S = 0.01f;
inline constexpr fp32 GIMBAL_LK_J1_EXT_MAX_DPS = 0.0f;
inline constexpr uint16_t GIMBAL_J2_EXTERNAL_DEADBAND_RAW = 10u;
inline constexpr uint16_t GIMBAL_J3_EXTERNAL_DEADBAND_RAW = 10u;
inline constexpr uint16_t GIMBAL_J4_EXTERNAL_DEADBAND_RAW = 20u;
inline constexpr uint16_t GIMBAL_J5_EXTERNAL_DEADBAND_RAW = 20u;
inline constexpr uint16_t GIMBAL_J6_EXTERNAL_DEADBAND_RAW = 20u;
inline constexpr uint16_t GIMBAL_J7_EXTERNAL_DEADBAND_RAW = 0u;
inline constexpr bool GIMBAL_J5_EXT_INVERT_WHEN_J4_FLIPPED = true;
inline constexpr bool GIMBAL_J6_EXT_INVERT_WHEN_J4_FLIPPED = false;
inline constexpr uint16_t GIMBAL_LK_J2_CROSS_ZERO_HIGH_RAW_TO_ZERO_THRESHOLD = 65000u;
inline constexpr uint16_t GIMBAL_LK_J3_CROSS_ZERO_HIGH_RAW_TO_ZERO_THRESHOLD = 65000u;
inline constexpr uint16_t GIMBAL_J7_HALF_RANGE_RAW_AT_MOTOR_MAX = 285u;
inline constexpr uint16_t GIMBAL_J7_HALF_RANGE_RAW_AT_MOTOR_MIN = 520u;
inline constexpr fp32 GIMBAL_J5_EXT_TARGET_AT_CTRL_MIN_REL_RAD =
    DM4310_PosRawToRad(56718u) - GIMBAL_DM_J5_ZERO_OFFSET_RAD;
inline constexpr fp32 GIMBAL_J5_EXT_TARGET_AT_CTRL_MAX_REL_RAD =
    DM4310_PosRawToRad(12127u) - GIMBAL_DM_J5_ZERO_OFFSET_RAD;
inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J4_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J4_RAW_MIN, OMNIX_CTRL_J4_RAW_ZERO, OMNIX_CTRL_J4_RAW_MAX},
    GIMBAL_DM_J4_MIN_REL_RAD,
    GIMBAL_DM_J4_MAX_REL_RAD,
    GIMBAL_J4_EXTERNAL_DEADBAND_RAW
};

inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J5_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J5_RAW_MIN, OMNIX_CTRL_J5_RAW_ZERO, OMNIX_CTRL_J5_RAW_MAX},
    GIMBAL_J5_EXT_TARGET_AT_CTRL_MIN_REL_RAD,
    GIMBAL_J5_EXT_TARGET_AT_CTRL_MAX_REL_RAD,
    GIMBAL_J5_EXTERNAL_DEADBAND_RAW
};

inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J6_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J6_RAW_MIN, OMNIX_CTRL_J6_RAW_ZERO, OMNIX_CTRL_J6_RAW_MAX},
    GIMBAL_DM_J6_MIN_REL_RAD,
    GIMBAL_DM_J6_MAX_REL_RAD,
    GIMBAL_J6_EXTERNAL_DEADBAND_RAW
};

inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J7_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J7_RAW_MIN, OMNIX_CTRL_J7_RAW_ZERO, OMNIX_CTRL_J7_RAW_MAX},
    GIMBAL_DM_J7_MIN_REL_RAD,
    GIMBAL_DM_J7_MAX_REL_RAD,
    GIMBAL_J7_EXTERNAL_DEADBAND_RAW
};
inline constexpr GimbalExternalHalfRangeReverseMapCfg GIMBAL_J7_EXT_HALF_RANGE_REVERSE_MAP_CFG = {
    GIMBAL_J7_HALF_RANGE_RAW_AT_MOTOR_MAX,
    GIMBAL_J7_HALF_RANGE_RAW_AT_MOTOR_MIN,
    GIMBAL_DM_J7_MAX_REL_RAD,
    GIMBAL_DM_J7_MIN_REL_RAD,
    GIMBAL_J7_EXTERNAL_DEADBAND_RAW
};

inline constexpr fp32 GIMBAL_DM_J5_MOUSE_SEN = 0.00005f;
inline constexpr fp32 GIMBAL_DM_J6_MOUSE_SEN = -0.00005f;

inline constexpr fp32 GIMBAL_DM_J5_RC_SEN_W    = 5.0f / 660.0f;
inline constexpr fp32 GIMBAL_DM_J5_MOUSE_SEN_W = 0.002f;

// ------------------- RC mapping -------------------
inline constexpr uint8_t GIMBAL_RC_S0_INDEX = 0;
inline constexpr uint8_t GIMBAL_RC_S1_INDEX = 1;

enum class GimbalRcSwitchPos : uint8_t {
    Up,
    Mid,
    Down
};

inline constexpr GimbalRcSwitchPos GIMBAL_RC_DM_ENABLE_S0    = GimbalRcSwitchPos::Up;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_DM_ENABLE_S1    = GimbalRcSwitchPos::Up;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_LK_ENABLE_S1    = GimbalRcSwitchPos::Mid;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_ZERO_FORCE_S0   = GimbalRcSwitchPos::Down;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_HOLD_S0         = GimbalRcSwitchPos::Mid;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_ROLL_RATE_S0    = GimbalRcSwitchPos::Up;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_ROLL_RATE_S1    = GimbalRcSwitchPos::Up;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_ZERO_LOG_S0     = GimbalRcSwitchPos::Up;
inline constexpr GimbalRcSwitchPos GIMBAL_RC_ZERO_LOG_S1     = GimbalRcSwitchPos::Down;
inline constexpr uint32_t GIMBAL_ZERO_LOG_PERIOD_MS          = 200;

#ifndef GIMBAL_LOG_ZERO_DM
#define GIMBAL_LOG_ZERO_DM 0
#endif

#ifndef GIMBAL_LOG_ZERO_LK
#define GIMBAL_LOG_ZERO_LK 0
#endif

#ifndef GIMBAL_LOG_ALL_MOTOR_POS
#define GIMBAL_LOG_ALL_MOTOR_POS 0
#endif

#ifndef GIMBAL_LOG_ALL_MOTOR_POS_PERIOD_MS
#define GIMBAL_LOG_ALL_MOTOR_POS_PERIOD_MS 1000u
#endif

#ifndef GIMBAL_LK_CAN_RAW_LOG_ENABLE
#define GIMBAL_LK_CAN_RAW_LOG_ENABLE 0
#endif

#ifndef GIMBAL_LK_CAN_RAW_LOG_PERIOD_MS
#define GIMBAL_LK_CAN_RAW_LOG_PERIOD_MS 100u
#endif

#ifndef GIMBAL_LK_LOCAL_HOLD_USE_A6
#define GIMBAL_LK_LOCAL_HOLD_USE_A6 0
#endif

#ifndef GIMBAL_LK_EXEC_DIAG_ENABLE
#define GIMBAL_LK_EXEC_DIAG_ENABLE 1
#endif

#ifndef GIMBAL_LK_EXEC_DIAG_PERIOD_MS
#define GIMBAL_LK_EXEC_DIAG_PERIOD_MS 500u
#endif

#ifndef GIMBAL_LK_A6_LOCAL_HOLD_MAX_DPS
#define GIMBAL_LK_A6_LOCAL_HOLD_MAX_DPS 40.0f
#endif

#ifndef GIMBAL_LK_J2_A8_LOCAL_RATE_SCALE
#define GIMBAL_LK_J2_A8_LOCAL_RATE_SCALE 1.0f
#endif

#ifndef GIMBAL_LK_J3_A8_LOCAL_RATE_SCALE
#define GIMBAL_LK_J3_A8_LOCAL_RATE_SCALE 1.0f
#endif

#ifndef GIMBAL_LK_J1_A8_LOCAL_RATE_SCALE
#define GIMBAL_LK_J1_A8_LOCAL_RATE_SCALE 2.0f
#endif

#ifndef GIMBAL_LK_J2_A8_LOCAL_MAX_DPS_SCALE
#define GIMBAL_LK_J2_A8_LOCAL_MAX_DPS_SCALE 1.0f
#endif

#ifndef GIMBAL_LK_J3_A8_LOCAL_MAX_DPS_SCALE
#define GIMBAL_LK_J3_A8_LOCAL_MAX_DPS_SCALE 1.0f
#endif

#ifndef GIMBAL_LK_J1_A8_LOCAL_MAX_DPS_SCALE
#define GIMBAL_LK_J1_A8_LOCAL_MAX_DPS_SCALE 2.0f
#endif

// ------------------- VT03 pause sequence -------------------
inline constexpr uint32_t GIMBAL_PAUSE_RUN_MS = 3000u;
inline constexpr uint32_t GIMBAL_PAUSE_BEEP_HZ = 2000u;
inline constexpr uint32_t GIMBAL_PAUSE_BEEP_ON_MS = 100u;
inline constexpr uint32_t GIMBAL_PAUSE_BEEP_OFF_MS = 100u;
inline constexpr uint8_t GIMBAL_PAUSE_BEEP_COUNT = 3u;
inline constexpr uint32_t GIMBAL_PAUSE_DONE_BEEP_MS = 120u;
inline constexpr uint32_t GIMBAL_PAUSE_LED_FLASH_PERIOD_MS = 80u;

// J1..J9 pause target, unit: cdeg.
// J1-J7 all use zero-relative targets so pause interpolation stays consistent with
// Gimbal_ReadJointSnapshot() and the live control path.
// J8-J9 keep absolute single-turn targets because pause drives MIT position hold directly.
inline const int16_t GIMBAL_PAUSE_TARGET_CDEG[9] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    static_cast<int16_t>(GIMBAL_DM_J8_HAS_ABS_ZERO ? Omnix_RadToCdeg(GIMBAL_DM_J8_ABS_ZERO_RAD) : 0),
    static_cast<int16_t>(GIMBAL_DM_J9_HAS_ABS_ZERO ? Omnix_RadToCdeg(GIMBAL_DM_J9_ABS_ZERO_RAD) : 0)
};

// Pause MIT gains for J8/J9 position hold (slow/stable).
inline constexpr fp32 GIMBAL_PAUSE_DM_J8_MIT_KP = 5.0f;
inline constexpr fp32 GIMBAL_PAUSE_DM_J8_MIT_KD = 0.6f;
inline constexpr fp32 GIMBAL_PAUSE_DM_J9_MIT_KP = 5.0f;
inline constexpr fp32 GIMBAL_PAUSE_DM_J9_MIT_KD = 0.6f;

inline constexpr uint8_t GIMBAL_DM_J5_RC_CH = 3;
inline constexpr uint8_t GIMBAL_DM_J6_RC_CH = 2;
inline constexpr uint8_t GIMBAL_DM_J7_RC_CH = 4;
inline constexpr uint8_t GIMBAL_DM_J4_RC_CH = 0;

inline constexpr int8_t GIMBAL_DM_J5_RC_SIGN = +1;
inline constexpr int8_t GIMBAL_DM_J6_RC_SIGN = +1;
inline constexpr int8_t GIMBAL_DM_J7_RC_SIGN = +1;
inline constexpr int8_t GIMBAL_DM_J4_RC_SIGN = -1;

inline constexpr uint8_t GIMBAL_LK_J3_RC_CH = 1;
inline constexpr uint8_t GIMBAL_LK_J2_RC_CH = 1;
inline constexpr uint8_t GIMBAL_LK_J1_RC_CH = 0;
inline constexpr int8_t GIMBAL_LK_J3_RC_SIGN = +1;
inline constexpr int8_t GIMBAL_LK_J2_RC_SIGN = +1;
inline constexpr int8_t GIMBAL_LK_J1_RC_SIGN = +1;

inline constexpr uint8_t GIMBAL_LINK_J5_RC_CH = 0;
inline constexpr int8_t GIMBAL_LINK_J5_RC_SIGN = GIMBAL_DM_J5_RC_SIGN;
inline constexpr uint8_t GIMBAL_LINK_J3_RC_CH = 1;
inline constexpr int8_t GIMBAL_LINK_J3_RC_SIGN = GIMBAL_LK_J3_RC_SIGN;
inline constexpr uint8_t GIMBAL_LINK_J1_RC_CH = 2;
inline constexpr int8_t GIMBAL_LINK_J1_RC_SIGN = GIMBAL_LK_J1_RC_SIGN;
inline constexpr uint8_t GIMBAL_LINK_J2_RC_CH = 3;
inline constexpr int8_t GIMBAL_LINK_J2_RC_SIGN = GIMBAL_LK_J2_RC_SIGN;

#define GIMBAL_LK_J3_ZERO_DEG GIMBAL_LK_J3_ABS_ZERO_DEG
#define GIMBAL_LK_J2_ZERO_DEG GIMBAL_LK_J2_ABS_ZERO_DEG
#define GIMBAL_LK_J1_ZERO_DEG GIMBAL_LK_J1_ABS_ZERO_DEG
inline constexpr GimbalMotorPosRefMode GIMBAL_LK_J3_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteSingleTurn;
inline constexpr GimbalMotorPosRefMode GIMBAL_LK_J2_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteSingleTurn;
inline constexpr GimbalMotorPosRefMode GIMBAL_LK_J1_POS_REF_MODE = GimbalMotorPosRefMode::AbsoluteSingleTurn;

struct GimbalLkPosSingleTurnCfg {
  uint16_t motor_raw_zero;
  uint16_t abs_limit_min_raw;
  uint16_t abs_limit_max_raw;
  bool a6_raw_increase_cmd_ccw;
  fp32 mech_gear_ratio;
  fp32 cfg_zero_deg;
  fp32 min_rel_deg;
  fp32 max_rel_deg;
  fp32 manual_rate_dps;
  fp32 manual_min_step_deg;
  uint16_t rc_deadband_raw;
  uint16_t link_deadband_raw;
  fp32 max_dps;
  bool center_trim_enable;
  fp32 poweron_home_target_rel_deg;
  bool forbid_zero_crossing;
  fp32 abs_limit_min_deg;
  fp32 abs_limit_max_deg;
};

inline constexpr fp32 Gimbal_LkRelDegFromRaw(uint16_t motor_raw, uint16_t motor_raw_zero)
{
  const int32_t delta_raw =
      static_cast<int32_t>(motor_raw) - static_cast<int32_t>(motor_raw_zero);
  return static_cast<fp32>(delta_raw) * LK8016E_ENCODER_RAW_TO_DEG;
}

inline constexpr fp32 Gimbal_LkClampRelDeg(fp32 rel_deg, const GimbalLkPosSingleTurnCfg& cfg)
{
  return Gimbal_MaxFp32(cfg.min_rel_deg, Gimbal_MinFp32(cfg.max_rel_deg, rel_deg));
}

inline constexpr fp32 Gimbal_LkRelSpanDeg(uint16_t motor_raw_a, uint16_t motor_raw_b)
{
  const int32_t delta_raw =
      static_cast<int32_t>(motor_raw_a) - static_cast<int32_t>(motor_raw_b);
  return static_cast<fp32>(delta_raw) * LK8016E_ENCODER_RAW_TO_DEG;
}

inline constexpr fp32 Gimbal_LkRelSpanDegWrapNegative(uint16_t motor_raw_wrap_min,
                                                      uint16_t motor_raw_zero)
{
  const int32_t delta_raw =
      static_cast<int32_t>(motor_raw_zero) + 65536 -
      static_cast<int32_t>(motor_raw_wrap_min);
  return -static_cast<fp32>(delta_raw) * LK8016E_ENCODER_RAW_TO_DEG;
}

// ------------------- LK torque control scaling -------------------
inline constexpr fp32 GIMBAL_LK_TORQUE_CURRENT_MAX_A = 33.0f;
inline constexpr fp32 GIMBAL_LK_J3_IQ_MAX_A = 25.0f;
inline constexpr fp32 GIMBAL_LK_J2_IQ_MAX_A = 25.0f;
inline constexpr fp32 GIMBAL_LK_J1_IQ_MAX_A = 20.0f;

inline constexpr bool GIMBAL_LK_USE_TORQUE_POS_HOLD = true;
inline constexpr bool GIMBAL_LK_J1_USE_POS_HOLD = false;

inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_KP = 0.32f;
inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_KD = 0.018f;
inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_KI = 0.0f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_KP = 0.34f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_KD = 0.016f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_KI = 0.0f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_KP = 0.12f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_KD = 0.010f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_KI = 0.0f;

inline constexpr fp32 GIMBAL_LK_TORQUE_POS_MAX_A = 28.0f;
inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_MAX_A = 9.0f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_MAX_A = 12.0f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_MAX_A = 2.5f;
inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_I_MAX_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_I_MAX_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_I_MAX_A = 0.0f;

inline constexpr fp32 GIMBAL_LK_TORQUE_POS_RATE_DPS = 15.0f;
inline constexpr fp32 GIMBAL_LK_J3_RATE_SCALE = 12.0f;
inline constexpr fp32 GIMBAL_LK_J2_RATE_SCALE = 20.0f;
inline constexpr fp32 GIMBAL_LK_J1_RATE_SCALE = 5.0f;
inline constexpr bool GIMBAL_LK_J3_POS_DIR_INVERT = true;
inline constexpr bool GIMBAL_LK_J2_POS_DIR_INVERT = true;
inline constexpr bool GIMBAL_LK_J1_POS_DIR_INVERT = false;
inline constexpr fp32 GIMBAL_LK_TORQUE_POS_MIN_STEP_DEG = 0.02f;
inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_DB_DEG = 0.60f;
inline constexpr fp32 GIMBAL_LK_J3_TORQUE_POS_DB_DPS = 7.0f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_DB_DEG = 0.45f;
inline constexpr fp32 GIMBAL_LK_J2_TORQUE_POS_DB_DPS = 6.0f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_DB_DEG = 0.50f;
inline constexpr fp32 GIMBAL_LK_J1_TORQUE_POS_DB_DPS = 8.0f;

inline constexpr bool GIMBAL_LK_TORQUE_POS_USE_GCOMP = false;
inline constexpr bool GIMBAL_LK_GCOMP_USE_3LINK = false;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_SCALE = 0.25f;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_K1_A = 9.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_K2_A = 4.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_K3_A = 0.5f;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_K4_A = 4.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_K5_A = 0.5f;
inline constexpr fp32 GIMBAL_LK_GCOMP_3LINK_K6_A = 0.0f;

inline constexpr bool GIMBAL_LK_GCOMP_USE_2LINK = false;
inline constexpr fp32 GIMBAL_LK_J3_GCOMP_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_J3_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_LK_J2_GCOMP_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_J2_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_LK_J1_GCOMP_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_J1_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_2LINK_K1_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_2LINK_K2_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_2LINK_K3_A = 0.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_2LINK_PHASE1_DEG = 0.0f;
inline constexpr fp32 GIMBAL_LK_GCOMP_2LINK_PHASE12_DEG = 0.0f;

inline constexpr bool GIMBAL_LK_USE_POS_HOLD = true;
inline constexpr fp32 GIMBAL_LK_POS_RATE_DPS = 180.0f;
inline constexpr fp32 GIMBAL_LK_POS_MAX_DPS  = 300.0f;
inline constexpr fp32 GIMBAL_LK_J1_POS_MAX_DPS = 300.0f;
inline constexpr fp32 GIMBAL_LK_J1_A4_MAX_DPS = 450.0f;
inline constexpr fp32 GIMBAL_LK_POS_MIN_STEP_DEG = 0.008f;
inline constexpr uint16_t GIMBAL_LK_POS_RC_DEADBAND = 10;
inline constexpr uint16_t GIMBAL_LK_POS_LINK_DEADBAND = 10;
inline constexpr uint16_t GIMBAL_LK_J1_POS_RC_DEADBAND = 0;
inline constexpr uint16_t GIMBAL_LK_J1_POS_LINK_DEADBAND = 0;
inline constexpr fp32 GIMBAL_LK_A6_HOLD_ERR_DB_DEG = 0.4f;
inline constexpr fp32 GIMBAL_LK_A6_HOLD_SPD_DB_DPS = 4.0f;
inline constexpr fp32 GIMBAL_LK_A6_DIR_HYST_DEG = 0.15f;

inline constexpr fp32 GIMBAL_LK_J3_MIN_REL_DEG = GIMBAL_LK_J3_MECH_MIN_REL_DEG;
inline constexpr fp32 GIMBAL_LK_J3_MAX_REL_DEG = GIMBAL_LK_J3_MECH_MAX_REL_DEG;
inline constexpr fp32 GIMBAL_LK_J2_MIN_REL_DEG = GIMBAL_LK_J2_MECH_MIN_REL_DEG;
inline constexpr fp32 GIMBAL_LK_J2_MAX_REL_DEG = GIMBAL_LK_J2_MECH_MAX_REL_DEG;
inline constexpr fp32 GIMBAL_LK_J1_MIN_REL_DEG = GIMBAL_LK_J1_MECH_MIN_REL_DEG;
inline constexpr fp32 GIMBAL_LK_J1_MAX_REL_DEG = GIMBAL_LK_J1_MECH_MAX_REL_DEG;

inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J1_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J1_RAW_MIN, OMNIX_CTRL_J1_RAW_ZERO, OMNIX_CTRL_J1_RAW_MAX},
    GIMBAL_LK_J1_MIN_REL_DEG * 0.01745329251994329577f,
    GIMBAL_LK_J1_MAX_REL_DEG * 0.01745329251994329577f,
    GIMBAL_J1_EXTERNAL_DEADBAND_RAW
};

inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J3_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J3_RAW_MIN, OMNIX_CTRL_J3_RAW_ZERO, OMNIX_CTRL_J3_RAW_MAX},
    GIMBAL_LK_J3_MIN_REL_DEG * 0.01745329251994329577f,
    GIMBAL_LK_J3_MAX_REL_DEG * 0.01745329251994329577f,
    GIMBAL_J3_EXTERNAL_DEADBAND_RAW
};
inline constexpr GimbalExternalHalfRangeReverseMapCfg GIMBAL_J3_EXT_HALF_RANGE_REVERSE_MAP_CFG = {
    OMNIX_CTRL_J3_RAW_MIN,
    OMNIX_CTRL_J3_RAW_MAX,
    GIMBAL_LK_J3_MAX_REL_DEG * 0.01745329251994329577f,
    GIMBAL_LK_J3_MIN_REL_DEG * 0.01745329251994329577f,
    GIMBAL_J3_EXTERNAL_DEADBAND_RAW
};

inline constexpr GimbalExternalBidirRelAngleMapCfg GIMBAL_J2_EXT_BIDIR_REL_ANGLE_MAP_CFG = {
    {OMNIX_CTRL_J2_RAW_MIN, OMNIX_CTRL_J2_RAW_ZERO, OMNIX_CTRL_J2_RAW_MAX},
    GIMBAL_LK_J2_MIN_REL_DEG * 0.01745329251994329577f,
    GIMBAL_LK_J2_MAX_REL_DEG * 0.01745329251994329577f,
    GIMBAL_J2_EXTERNAL_DEADBAND_RAW
};
inline constexpr GimbalExternalHalfRangeReverseMapCfg GIMBAL_J2_EXT_HALF_RANGE_REVERSE_MAP_CFG = {
    OMNIX_CTRL_J2_RAW_MIN,
    OMNIX_CTRL_J2_RAW_MAX,
    GIMBAL_LK_J2_MAX_REL_DEG * 0.01745329251994329577f,
    GIMBAL_LK_J2_MIN_REL_DEG * 0.01745329251994329577f,
    GIMBAL_J2_EXTERNAL_DEADBAND_RAW
};
inline constexpr GimbalLkPosSingleTurnCfg GIMBAL_LK_J3_POS_CFG = {
    GIMBAL_LK_J3_POWERON_ZERO_RAW,
    GIMBAL_LK_J3_ABS_LIMIT_WRAP_MIN_RAW,
    GIMBAL_LK_J3_ABS_LIMIT_WRAP_MAX_RAW,
    true,
    GIMBAL_LK_J3_GEAR_RATIO,
    GIMBAL_LK_J3_MECH_ABS_ZERO_DEG,
    GIMBAL_LK_J3_MIN_REL_DEG,
    GIMBAL_LK_J3_MAX_REL_DEG,
    GIMBAL_LK_POS_RATE_DPS * GIMBAL_LK_J3_RATE_SCALE,
    GIMBAL_LK_POS_MIN_STEP_DEG,
    GIMBAL_LK_POS_RC_DEADBAND,
    GIMBAL_LK_POS_LINK_DEADBAND,
    GIMBAL_LK_POS_MAX_DPS,
    false,
    0.0f,
    true,
    GIMBAL_LK_J3_MECH_ABS_LIMIT_MIN_DEG,
    GIMBAL_LK_J3_MECH_ABS_LIMIT_MAX_DEG
};

inline constexpr GimbalLkPosSingleTurnCfg GIMBAL_LK_J2_POS_CFG = {
    GIMBAL_LK_J2_POWERON_ZERO_RAW,
    GIMBAL_LK_J2_ABS_LIMIT_MIN_RAW,
    GIMBAL_LK_J2_ABS_LIMIT_MAX_RAW,
    true,
    GIMBAL_LK_J2_GEAR_RATIO,
    GIMBAL_LK_J2_MECH_ABS_ZERO_DEG,
    GIMBAL_LK_J2_MIN_REL_DEG,
    GIMBAL_LK_J2_MAX_REL_DEG,
    GIMBAL_LK_POS_RATE_DPS * GIMBAL_LK_J2_RATE_SCALE,
    GIMBAL_LK_POS_MIN_STEP_DEG,
    GIMBAL_LK_POS_RC_DEADBAND,
    GIMBAL_LK_POS_LINK_DEADBAND,
    GIMBAL_LK_POS_MAX_DPS,
    false,
    0.0f,
    true,
    GIMBAL_LK_J2_MECH_ABS_LIMIT_MIN_DEG,
    GIMBAL_LK_J2_MECH_ABS_LIMIT_MAX_DEG
};

inline constexpr GimbalLkPosSingleTurnCfg GIMBAL_LK_J1_POS_CFG = {
    GIMBAL_LK_J1_CAL_ZERO_RAW,
    GIMBAL_LK_J1_ABS_LIMIT_MIN_RAW,
    GIMBAL_LK_J1_ABS_LIMIT_MAX_RAW,
    false,
    GIMBAL_LK_J1_GEAR_RATIO,
    GIMBAL_LK_J1_MECH_ABS_ZERO_DEG,
    GIMBAL_LK_J1_MIN_REL_DEG,
    GIMBAL_LK_J1_MAX_REL_DEG,
    GIMBAL_LK_POS_RATE_DPS * GIMBAL_LK_J1_RATE_SCALE,
    GIMBAL_LK_POS_MIN_STEP_DEG,
    GIMBAL_LK_J1_POS_RC_DEADBAND,
    GIMBAL_LK_J1_POS_LINK_DEADBAND,
    GIMBAL_LK_J1_POS_MAX_DPS,
    true,
    0.0f,
    true,
    GIMBAL_LK_J1_MECH_ABS_LIMIT_MIN_DEG,
    GIMBAL_LK_J1_MECH_ABS_LIMIT_MAX_DEG
};

inline constexpr bool GIMBAL_LK_J1_POWERON_HOME_ENABLE = true;
inline constexpr uint32_t GIMBAL_LK_J1_POWERON_HOME_MS = 3000u;
inline constexpr fp32 GIMBAL_LK_J1_POWERON_EXIT_DB_DEG = 1.0f;
inline constexpr fp32 GIMBAL_LK_J1_POWERON_EXIT_DB_DPS = 2.0f;
inline constexpr fp32 GIMBAL_LK_J1_ZERO_HOLD_ENTRY_DB_DEG = 1.0f;
inline constexpr fp32 GIMBAL_LK_J1_ZERO_HOLD_ENTRY_DB_DPS = 2.0f;
inline constexpr uint32_t GIMBAL_LK_J1_ZERO_HOLD_LOG_PERIOD_MS = 500u;

// ------------------- DM4340 J8 MIT mapping -------------------
inline constexpr fp32 GIMBAL_DM_J8_PMAX_RAD   = 3.2f;
inline constexpr fp32 GIMBAL_DM_J8_VMAX_RAD_S = 45.0f;
inline constexpr fp32 GIMBAL_DM_J8_TMAX_NM    = 18.0f;
inline constexpr fp32 GIMBAL_DM_J4_PMAX_RAD   = 3.2f;
inline constexpr fp32 GIMBAL_DM_J4_VMAX_RAD_S = 45.0f;
inline constexpr fp32 GIMBAL_DM_J4_TMAX_NM    = 18.0f;

// ------------------- DM4310 MIT tuning -------------------
inline constexpr bool GIMBAL_DM4310_USE_MIT = true;
inline constexpr fp32 GIMBAL_DM_MIT_KP = 0.0f;
inline constexpr fp32 GIMBAL_DM_MIT_KD = 0.5f;
inline constexpr fp32 GIMBAL_DM_J4_MIT_KP = 10.0f;
inline constexpr fp32 GIMBAL_DM_J4_MIT_KD = 1.0f;

// ------------------- DM J8/J9 torque compensation -------------------
inline constexpr bool GIMBAL_DM_J89_EXTERNAL_FILTER_ENABLE = true;
struct GimbalDmFollowCompCfg {
  fp32 follow_cmd_db_A;
  fp32 external_cmd_db_A;
  fp32 external_lpf_tau_s;
  fp32 external_follow_db_enter_A;
  fp32 external_follow_db_exit_A;
  fp32 torque_share;
  fp32 torque_max_nm;
  fp32 torque_slew_nm_s;
  int8_t torque_sign;
  fp32 torque_nm_per_A;
};

inline constexpr GimbalDmFollowCompCfg GIMBAL_DM_J8_FOLLOW_COMP_CFG = {
    0.40f, // follow_cmd_db_A
    0.20f, // external_cmd_db_A
    0.0880f, // external_lpf_tau_s
    2.00f, // external_follow_db_enter_A
    1.00f, // external_follow_db_exit_A
    5.00f, // torque_share
    15.00f, // torque_max_nm
    10.0f, // torque_slew_nm_s
    1,     // torque_sign
    1.00f  // torque_nm_per_A
};

inline constexpr GimbalDmFollowCompCfg GIMBAL_DM_J9_FOLLOW_COMP_CFG = {
    0.40f, // follow_cmd_db_A
    0.00f, // external_cmd_db_A
    0.040f, // external_lpf_tau_s
    1.00f, // external_follow_db_enter_A
    0.60f, // external_follow_db_exit_A
    2.00f, // torque_share
    6.00f, // torque_max_nm
    30.0f, // torque_slew_nm_s
    1,     // torque_sign
    1.00f  // torque_nm_per_A
};

inline constexpr fp32 GIMBAL_DM_J8_GCOMP_NM = 2.0f;
inline constexpr fp32 GIMBAL_DM_J8_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_DM_J9_GCOMP_NM = 2.0f;
inline constexpr fp32 GIMBAL_DM_J9_GCOMP_PHASE_DEG = 0.0f;

// ------------------- DM gravity compensation -------------------
inline constexpr bool GIMBAL_DM_GCOMP_ENABLE = true;
inline constexpr fp32 GIMBAL_DM_GCOMP_SCALE = 1.0f;
inline constexpr fp32 GIMBAL_DM_GCOMP_MAX_NM = 4.0f;
inline constexpr fp32 GIMBAL_DM_J5_GCOMP_NM = 0.6f;
inline constexpr fp32 GIMBAL_DM_J5_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_DM_J6_GCOMP_NM = 0.0f;
inline constexpr fp32 GIMBAL_DM_J6_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_DM_J7_GCOMP_NM = 0.0f;
inline constexpr fp32 GIMBAL_DM_J7_GCOMP_PHASE_DEG = 0.0f;
inline constexpr fp32 GIMBAL_DM_J4_GCOMP_NM = 0.0f;
inline constexpr fp32 GIMBAL_DM_J4_GCOMP_PHASE_DEG = 0.0f;

#endif // H723VG_V2_FREERTOS_CONF_GIMBAL_TASK_H
GIMBAL_DM_J6_ANG_KP