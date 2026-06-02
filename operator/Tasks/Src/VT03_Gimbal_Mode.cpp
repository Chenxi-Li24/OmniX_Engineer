#include "VT03_Gimbal_Mode.h"

namespace {

struct VT03_GimbalState {
  bool prev_active{false};
  bool prev_btn_l{false};
  bool prev_btn_r{false};
  RcGimbalMode latched_mode{RcGimbalMode::DM};
};

VT03_GimbalState g_vt03_gimbal_state{};

void VT03_Gimbal_ResetState()
{
  g_vt03_gimbal_state.prev_active = false;
  g_vt03_gimbal_state.prev_btn_l = false;
  g_vt03_gimbal_state.prev_btn_r = false;
  g_vt03_gimbal_state.latched_mode = RcGimbalMode::DM;
}

}  // namespace

RcControlMode VT03_Gimbal_ResolveControlMode(const RC_State& rc, const RC_Status& st)
{
  RcControlMode out = Rc_ResolveControlMode(rc, st);

  if (!st.vt03_online) {
    VT03_Gimbal_ResetState();
    return out;
  }

  if (st.active_src != RC_SRC_VT03) {
    g_vt03_gimbal_state.prev_active = false;
    return out;
  }

  const bool btn_l = (rc.key.v_ext & KEY_EXT_VT03_BTN_L) != 0u;
  const bool btn_r = (rc.key.v_ext & KEY_EXT_VT03_BTN_R) != 0u;

  if (!g_vt03_gimbal_state.prev_active) {
    g_vt03_gimbal_state.prev_active = true;
    g_vt03_gimbal_state.prev_btn_l = btn_l;
    g_vt03_gimbal_state.prev_btn_r = btn_r;
  } else {
    const bool btn_l_rise = btn_l && !g_vt03_gimbal_state.prev_btn_l;
    const bool btn_r_rise = btn_r && !g_vt03_gimbal_state.prev_btn_r;

    if (btn_l && btn_r) {
      g_vt03_gimbal_state.latched_mode = RcGimbalMode::Link;
    } else if (btn_l_rise) {
      g_vt03_gimbal_state.latched_mode = RcGimbalMode::LK;
    } else if (btn_r_rise) {
      g_vt03_gimbal_state.latched_mode = RcGimbalMode::DM;
    }

    g_vt03_gimbal_state.prev_btn_l = btn_l;
    g_vt03_gimbal_state.prev_btn_r = btn_r;
  }

  out.gimbal_mode = g_vt03_gimbal_state.latched_mode;
  return out;
}
