#pragma once

#include <cstdint>

#include "RC_Task.h"
#include "lib_remote_control.h"

enum class RcMainMode : uint8_t {
  ZeroForce = 0,
  Chassis,
  Gimbal,
};

enum class RcGimbalMode : uint8_t {
  DM = 0,
  LK,
  Link,
};

typedef struct {
  rc_source_t active_src;
  uint8_t s0_state;
  RcMainMode main_mode;
  RcGimbalMode gimbal_mode;
  bool dr16_active;
  bool vt03_active;
} RcControlMode;

static inline bool Rc_AxisIdle(int16_t value, int16_t deadband)
{
  return (value > -deadband) && (value < deadband);
}

static inline RcMainMode Rc_MainModeFromS0(uint8_t s0_state)
{
  switch (s0_state) {
    case RC_S0_STATE_MID:
      return RcMainMode::Chassis;
    case RC_S0_STATE_UP:
      return RcMainMode::Gimbal;
    case RC_S0_STATE_DOWN:
    default:
      return RcMainMode::ZeroForce;
  }
}

static inline uint8_t Rc_ResolveS0State(const RC_State& rc, const RC_Status& st)
{
  if (st.active_src == RC_SRC_DR16) {
    return rc_s0_state;
  }
  if (switch_is_up(rc.rc.s[0])) {
    return RC_S0_STATE_UP;
  }
  if (switch_is_mid(rc.rc.s[0])) {
    return RC_S0_STATE_MID;
  }
  return RC_S0_STATE_DOWN;
}

static inline RcGimbalMode Rc_ResolveGimbalMode(const RC_State& rc, const RC_Status& st)
{
  if (st.active_src == RC_SRC_VT03) {
    const bool fn1 = (rc.key.v_ext & KEY_EXT_VT03_BTN_L) != 0u;
    const bool fn2 = (rc.key.v_ext & KEY_EXT_VT03_BTN_R) != 0u;
    if (fn1 && fn2) {
      return RcGimbalMode::Link;
    }
    if (fn2) {
      return RcGimbalMode::LK;
    }
    return RcGimbalMode::DM;
  }

  if (switch_is_mid(rc.rc.s[1])) {
    return RcGimbalMode::LK;
  }
  if (switch_is_down(rc.rc.s[1])) {
    return RcGimbalMode::Link;
  }
  return RcGimbalMode::DM;
}

static inline RcControlMode Rc_ResolveControlMode(const RC_State& rc, const RC_Status& st)
{
  RcControlMode out{};
  out.active_src = st.active_src;
  out.dr16_active = (st.active_src == RC_SRC_DR16);
  out.vt03_active = (st.active_src == RC_SRC_VT03);
  out.s0_state = Rc_ResolveS0State(rc, st);
  out.main_mode = Rc_MainModeFromS0(out.s0_state);
  out.gimbal_mode = Rc_ResolveGimbalMode(rc, st);
  return out;
}
