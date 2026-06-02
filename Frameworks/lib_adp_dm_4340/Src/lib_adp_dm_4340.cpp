#include "lib_adp_dm_4340.h"
#include <cstring>

namespace {
  static inline uint32_t f32_to_u32(float v)
  {
    static_assert(sizeof(float) == 4, "float must be 32-bit IEEE754");
    uint32_t u;
    std::memcpy(&u, &v, 4);
    return u;
  }

  static inline float u32_to_f32(uint32_t u)
  {
    static_assert(sizeof(float) == 4, "float must be 32-bit IEEE754");
    float v;
    std::memcpy(&v, &u, 4);
    return v;
  }

  static inline void wr_le_f32(uint8_t* p, float v)
  {
    uint32_t u = f32_to_u32(v);
    p[0] = static_cast<uint8_t>(u >> 0);
    p[1] = static_cast<uint8_t>(u >> 8);
    p[2] = static_cast<uint8_t>(u >> 16);
    p[3] = static_cast<uint8_t>(u >> 24);
  }

  static inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
  {
    const float span = x_max - x_min;
    float offset = x - x_min;
    if (offset < 0.0f) offset = 0.0f;
    if (offset > span) offset = span;
    const uint32_t levels = (1u << bits) - 1u;
    uint32_t ret = static_cast<uint32_t>(offset * static_cast<float>(levels) / span + 0.5f);
    if (ret > levels) ret = levels;
    return static_cast<uint16_t>(ret);
  }

  static inline float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
  {
    const float span = x_max - x_min;
    const float levels = static_cast<float>((1u << bits) - 1u);
    return (static_cast<float>(x_int) * span / levels) + x_min;
  }
} // namespace

DM4340::DM4340(uint8_t can_id, uint16_t fb_master_sid)
: can_id_(can_id), fb_master_sid_(fb_master_sid)
{
  // Conservative defaults; must match motor-side PMAX/VMAX/TMAX for correct scaling.
  set_mit_limits(3.2f, 45.0f, 18.0f);
}

void DM4340::set_mit_limits(fp32 pmax_rad, fp32 vmax_rad_s, fp32 tmax_nm)
{
  pmax_u32_.store(f32_to_u32(static_cast<float>(pmax_rad)), std::memory_order_relaxed);
  vmax_u32_.store(f32_to_u32(static_cast<float>(vmax_rad_s)), std::memory_order_relaxed);
  tmax_u32_.store(f32_to_u32(static_cast<float>(tmax_nm)), std::memory_order_relaxed);
}

void DM4340::set_mit_cmd(fp32 pos_rad, fp32 vel_rad_s, fp32 kp, fp32 kd, fp32 torque_nm)
{
  cmd_mit_pos_u32_.store(f32_to_u32(static_cast<float>(pos_rad)), std::memory_order_relaxed);
  cmd_mit_vel_u32_.store(f32_to_u32(static_cast<float>(vel_rad_s)), std::memory_order_relaxed);
  cmd_mit_kp_u32_.store(f32_to_u32(static_cast<float>(kp)), std::memory_order_relaxed);
  cmd_mit_kd_u32_.store(f32_to_u32(static_cast<float>(kd)), std::memory_order_relaxed);
  cmd_mit_torque_u32_.store(f32_to_u32(static_cast<float>(torque_nm)), std::memory_order_relaxed);
  mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4340::set_mit_pos_rad(fp32 pos_rad)
{
  cmd_mit_pos_u32_.store(f32_to_u32(static_cast<float>(pos_rad)), std::memory_order_relaxed);
  mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4340::set_mit_vel_rad_s(fp32 vel_rad_s)
{
  cmd_mit_vel_u32_.store(f32_to_u32(static_cast<float>(vel_rad_s)), std::memory_order_relaxed);
  mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4340::set_mit_kp(fp32 kp)
{
  cmd_mit_kp_u32_.store(f32_to_u32(static_cast<float>(kp)), std::memory_order_relaxed);
  mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4340::set_mit_kd(fp32 kd)
{
  cmd_mit_kd_u32_.store(f32_to_u32(static_cast<float>(kd)), std::memory_order_relaxed);
  mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4340::set_mit_torque_nm(fp32 torque_nm)
{
  cmd_mit_torque_u32_.store(f32_to_u32(static_cast<float>(torque_nm)), std::memory_order_relaxed);
  mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4340::set_posvel_cmd(fp32 pos_rad, fp32 vel_rad_s)
{
  cmd_posvel_pos_u32_.store(f32_to_u32(static_cast<float>(pos_rad)), std::memory_order_relaxed);
  cmd_posvel_vel_u32_.store(f32_to_u32(static_cast<float>(vel_rad_s)), std::memory_order_relaxed);
}

void DM4340::set_velocity_cmd(fp32 vel_rad_s)
{
  cmd_vel_u32_.store(f32_to_u32(static_cast<float>(vel_rad_s)), std::memory_order_relaxed);
}

void DM4340::set_posvel_torque_cmd(fp32 pos_rad, fp32 vlim_rad_s, fp32 ilimit_norm)
{
  // manual: v_des scaled by 100, range 0..10000 => 0..100 rad/s
  float v = static_cast<float>(vlim_rad_s) * 100.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 10000.0f) v = 10000.0f;
  uint16_t v_u16 = static_cast<uint16_t>(v + 0.5f);

  // manual: i_des scaled by 10000, range 0..10000 => 0..1.0 (normalized)
  float i = static_cast<float>(ilimit_norm) * 10000.0f;
  if (i < 0.0f) i = 0.0f;
  if (i > 10000.0f) i = 10000.0f;
  uint16_t i_u16 = static_cast<uint16_t>(i + 0.5f);

  cmd_pvt_pos_u32_.store(f32_to_u32(static_cast<float>(pos_rad)), std::memory_order_relaxed);
  cmd_pvt_vlim_u16_.store(v_u16, std::memory_order_relaxed);
  cmd_pvt_ilim_u16_.store(i_u16, std::memory_order_relaxed);
}

void DM4340::onRxFeedback(const CanFrame& f, uint32_t now_ms)
{
  if (f.is_ext || f.dlc < 8) {
    return;
  }

  const uint16_t sid = static_cast<uint16_t>(f.id & 0x7FFu);
  const bool sid_ok =
      (sid == fb_master_sid_) ||
      (sid == static_cast<uint16_t>(0x000u + can_id_)) ||
      (sid == static_cast<uint16_t>(0x100u + can_id_)) ||
      (sid == static_cast<uint16_t>(0x200u + can_id_)) ||
      (sid == static_cast<uint16_t>(0x300u + can_id_));
  if (!sid_ok) {
    return;
  }

  const uint8_t* d = f.data;
  const uint8_t id_in = payload_id(d[0]);
  const uint8_t err   = payload_err(d[0]);

  if (id_in != (can_id_ & 0x0F) && id_in != can_id_) {
    return;
  }

  __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);

  DM4340State s = state_;
  const bool was_online = s.online;
  (void)was_online;

  const int16_t pos_q16 = static_cast<int16_t>(
    (static_cast<uint16_t>(d[1]) << 8) | d[2]
  );
  const int16_t vel_q12 = dm_signext12(
    (static_cast<uint16_t>(d[3]) << 4) | (d[4] >> 4)
  );
  const int16_t tq_q12  = dm_signext12(
    (static_cast<uint16_t>(d[4] & 0x0F) << 8) | d[5]
  );

  const float pmax = u32_to_f32(pmax_u32_.load(std::memory_order_relaxed));
  const float vmax = u32_to_f32(vmax_u32_.load(std::memory_order_relaxed));
  const float tmax = u32_to_f32(tmax_u32_.load(std::memory_order_relaxed));

  const uint16_t pos_u16 = static_cast<uint16_t>(pos_q16);
  const uint16_t vel_u12 = static_cast<uint16_t>(vel_q12) & 0x0FFFu;
  const uint16_t tq_u12  = static_cast<uint16_t>(tq_q12) & 0x0FFFu;

  s.motor_id      = id_in;
  s.error_code    = err;
  // For MIT protocol, motor uses unsigned mapping over [min,max]. Keep "raw" as signed-centered for debug/API.
  s.pos_raw       = static_cast<int16_t>(static_cast<int32_t>(pos_u16) - 32768);
  s.pos_raw_total = s.pos_raw;
  s.pos_rad       = static_cast<fp32>(uint_to_float(pos_u16, -pmax, +pmax, 16));
  s.pos_rad_total = s.pos_rad;
  s.speed_raw     = static_cast<int16_t>(static_cast<int32_t>(vel_u12) - 2048);
  s.speed_rad_s   = static_cast<fp32>(uint_to_float(vel_u12, -vmax, +vmax, 12));
  s.torque_raw    = static_cast<int16_t>(static_cast<int32_t>(tq_u12) - 2048);
  s.torque_nm     = static_cast<fp32>(uint_to_float(tq_u12, -tmax, +tmax, 12));
  s.temp_mos_C    = d[6];
  s.temp_rotor_C  = d[7];
  s.last_rx_ms    = now_ms;
  s.online        = true;

  state_ = s;
  __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
}

void DM4340::tick(uint32_t now_ms)
{
  const uint32_t last = __atomic_load_n(&state_.last_rx_ms, __ATOMIC_RELAXED);
  const bool was_online = __atomic_load_n(&state_.online, __ATOMIC_RELAXED);

  if (was_online && (now_ms - last > offline_ms_)) {
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);
    state_.online = false;
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
  }
}

bool DM4340::snapshot(DM4340State& out) const
{
  for (int i = 0; i < 3; ++i) {
    uint32_t s1 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
    if (s1 & 1u) continue;
    DM4340State tmp = state_;
    uint32_t s2 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
    if (s1 == s2) { out = tmp; return true; }
  }
  out = state_;
  return false;
}

bool DM4340::exportTxRaw8(uint16_t* sid, uint8_t out[8]) const
{
  if (!sid || !out) return false;

  const DM4340Mode m = mode();
  uint16_t base = 0x000u;
  switch (m) {
    case DM4340Mode::MIT:          base = 0x000u; break;
    case DM4340Mode::PosVel:       base = 0x100u; break;
    case DM4340Mode::Velocity:     base = 0x200u; break;
    case DM4340Mode::PosVelTorque: base = 0x300u; break;
    default:                       base = 0x000u; break;
  }
  *sid = static_cast<uint16_t>(base + can_id_);

  for (int i = 0; i < 8; ++i) out[i] = 0;

  if (m == DM4340Mode::MIT) {
    const float pmax = u32_to_f32(pmax_u32_.load(std::memory_order_relaxed));
    const float vmax = u32_to_f32(vmax_u32_.load(std::memory_order_relaxed));
    const float tmax = u32_to_f32(tmax_u32_.load(std::memory_order_relaxed));

    const float p = u32_to_f32(cmd_mit_pos_u32_.load(std::memory_order_relaxed));
    const float v = u32_to_f32(cmd_mit_vel_u32_.load(std::memory_order_relaxed));
    const float kp = u32_to_f32(cmd_mit_kp_u32_.load(std::memory_order_relaxed));
    const float kd = u32_to_f32(cmd_mit_kd_u32_.load(std::memory_order_relaxed));
    const float tq = u32_to_f32(cmd_mit_torque_u32_.load(std::memory_order_relaxed));

    const uint16_t p_u16 = float_to_uint(p, -pmax, +pmax, 16);
    const uint16_t v_u12 = float_to_uint(v, -vmax, +vmax, 12);
    const uint16_t kp_u = float_to_uint(kp, 0.0f, 500.0f, 12);
    const uint16_t kd_u = float_to_uint(kd, 0.0f, 5.0f, 12);
    const uint16_t t_u12 = float_to_uint(tq, -tmax, +tmax, 12);

    // Pack 8B according to the MIT protocol (same 8B layout as DM4310 adaptor).
    // NOTE: The motor ID is carried by CAN SID (0x000 + can_id), not in the data bytes.
    out[0] = static_cast<uint8_t>(p_u16 >> 8);
    out[1] = static_cast<uint8_t>(p_u16 & 0xFFu);
    out[2] = static_cast<uint8_t>(v_u12 >> 4);
    out[3] = static_cast<uint8_t>(((v_u12 & 0x0Fu) << 4) | ((kp_u >> 8) & 0x0Fu));
    out[4] = static_cast<uint8_t>(kp_u & 0xFFu);
    out[5] = static_cast<uint8_t>((kd_u >> 4) & 0xFFu);
    out[6] = static_cast<uint8_t>(((kd_u & 0x0Fu) << 4) | ((t_u12 >> 8) & 0x0Fu));
    out[7] = static_cast<uint8_t>(t_u12 & 0xFFu);
    return true;
  }

  if (m == DM4340Mode::PosVel) {
    wr_le_f32(&out[0], u32_to_f32(cmd_posvel_pos_u32_.load(std::memory_order_relaxed)));
    wr_le_f32(&out[4], u32_to_f32(cmd_posvel_vel_u32_.load(std::memory_order_relaxed)));
    return true;
  }

  if (m == DM4340Mode::Velocity) {
    wr_le_f32(&out[0], u32_to_f32(cmd_vel_u32_.load(std::memory_order_relaxed)));
    return true;
  }

  if (m == DM4340Mode::PosVelTorque) {
    wr_le_f32(&out[0], u32_to_f32(cmd_pvt_pos_u32_.load(std::memory_order_relaxed)));
    const uint16_t v_u16 = cmd_pvt_vlim_u16_.load(std::memory_order_relaxed);
    const uint16_t i_u16 = cmd_pvt_ilim_u16_.load(std::memory_order_relaxed);
    out[4] = static_cast<uint8_t>(v_u16 & 0xFFu);
    out[5] = static_cast<uint8_t>((v_u16 >> 8) & 0xFFu);
    out[6] = static_cast<uint8_t>(i_u16 & 0xFFu);
    out[7] = static_cast<uint8_t>((i_u16 >> 8) & 0xFFu);
    return true;
  }

  return false;
}
