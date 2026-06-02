// lib_adp_lk_mg8016e.cpp
// MG-supported commands: 0x80/0x88/0x81/0xA1/0xA2/0xA3..0xA8/0x280
// Note: 0xA0 open-loop power/voltage is MS-only and not supported by MG.
#include "lib_adp_lk_mg8016e.h"
#include <cstring>
#include <cmath>

namespace {
  static inline int16_t rd_le_i16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
  }
  static inline uint16_t rd_le_u16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
  }
  static inline int64_t rd_le_i56_sext(const uint8_t* p) {
    uint64_t v = 0u;
    for (int i = 0; i < 7; ++i) {
      v |= (static_cast<uint64_t>(p[i]) << (8 * i));
    }
    if ((p[6] & 0x80u) != 0u) {
      v |= 0xFF00000000000000ull;
    }
    return static_cast<int64_t>(v);
  }
  static inline void wr_le_i16(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }
  static inline void wr_le_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }
  static inline void wr_le_i32(uint8_t* p, int32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  }
  static inline void wr_le_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  }
  static inline int16_t clamp_iq_broadcast(int32_t v) {
    if (v > 2000) v = 2000;
    if (v < -2000) v = -2000;
    return static_cast<int16_t>(v);
  }
  static inline uint16_t clamp_u16_from_f(fp32 v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return static_cast<uint16_t>(v + 0.5f);
  }
  static inline fp32 wrap_0_360_deg(fp32 deg) {
    fp32 d = std::fmod(deg, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d;
  }
} // namespace

LK8016E::LK8016E(uint8_t id)
: motor_id_(id) {}

void LK8016E::set_iq_raw(int16_t iq_raw)
{
  cmd_iq_raw_.store(clamp_iq_raw(iq_raw), std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::Torque), std::memory_order_relaxed);
}

void LK8016E::set_iq_A(fp32 iq_A)
{
  fp32 max_A = torque_current_max_A_;
  if (max_A <= 0.0f) {
    max_A = 33.0f;
  }
  const fp32 raw = (iq_A / max_A) * 2048.0f;
  set_iq_raw(clamp_iq_raw(static_cast<int32_t>(raw)));
}

void LK8016E::set_speed_control_raw(int32_t sc)
{
  cmd_speed_raw_.store(sc, std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::Speed), std::memory_order_relaxed);
}

void LK8016E::set_speed_dps(fp32 dps)
{
  const fp32 raw = dps * 100.0f;
  set_speed_control_raw(static_cast<int32_t>(raw));
}

void LK8016E::set_angle_control_raw(int32_t angle_raw)
{
  pos_multi_raw_.store(angle_raw, std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::PosMulti), std::memory_order_relaxed);
}

void LK8016E::set_angle_deg(fp32 deg)
{
  const int32_t raw = static_cast<int32_t>(deg * 100.0f);
  set_angle_control_raw(raw);
}

void LK8016E::set_angle_with_max_speed_raw(int32_t angle_raw, uint16_t max_dps)
{
  pos_multi_raw_.store(angle_raw, std::memory_order_relaxed);
  pos_multi_maxspd_.store(max_dps, std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::PosMultiMaxSpeed), std::memory_order_relaxed);
}

void LK8016E::set_angle_deg_max_speed(fp32 deg, fp32 max_dps)
{
  const int32_t raw = static_cast<int32_t>(deg * 100.0f);
  set_angle_with_max_speed_raw(raw, clamp_u16_from_f(max_dps));
}

void LK8016E::set_circle_angle_raw(uint32_t angle_raw, uint8_t dir)
{
  pos_single_raw_.store(angle_raw, std::memory_order_relaxed);
  pos_single_dir_.store(static_cast<uint8_t>(dir ? 1u : 0u), std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::PosSingle), std::memory_order_relaxed);
}

void LK8016E::set_circle_angle_deg(fp32 deg, bool ccw)
{
  const fp32 d = wrap_0_360_deg(deg);
  const uint32_t raw = static_cast<uint32_t>(d * 100.0f);
  set_circle_angle_raw(raw, ccw ? 1u : 0u);
}

void LK8016E::set_circle_angle_raw_max_speed(uint32_t angle_raw, uint8_t dir, uint16_t max_dps)
{
  pos_single_raw_.store(angle_raw, std::memory_order_relaxed);
  pos_single_dir_.store(static_cast<uint8_t>(dir ? 1u : 0u), std::memory_order_relaxed);
  pos_single_maxspd_.store(max_dps, std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::PosSingleMaxSpeed), std::memory_order_relaxed);
}

void LK8016E::set_circle_angle_deg_max_speed(fp32 deg, bool ccw, fp32 max_dps)
{
  const fp32 d = wrap_0_360_deg(deg);
  const uint32_t raw = static_cast<uint32_t>(d * 100.0f);
  set_circle_angle_raw_max_speed(raw, ccw ? 1u : 0u, clamp_u16_from_f(max_dps));
}

void LK8016E::set_inc_angle_raw(int32_t inc_raw)
{
  pos_inc_raw_.store(inc_raw, std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::PosInc), std::memory_order_relaxed);
}

void LK8016E::set_inc_angle_deg(fp32 deg)
{
  const int32_t raw = static_cast<int32_t>(deg * 100.0f);
  set_inc_angle_raw(raw);
}

void LK8016E::set_inc_angle_raw_max_speed(int32_t inc_raw, uint16_t max_dps)
{
  pos_inc_raw_.store(inc_raw, std::memory_order_relaxed);
  pos_inc_maxspd_.store(max_dps, std::memory_order_relaxed);
  ctrl_mode_.store(static_cast<uint8_t>(LK8016EMode::PosIncMaxSpeed), std::memory_order_relaxed);
}

void LK8016E::set_inc_angle_deg_max_speed(fp32 deg, fp32 max_dps)
{
  const int32_t raw = static_cast<int32_t>(deg * 100.0f);
  set_inc_angle_raw_max_speed(raw, clamp_u16_from_f(max_dps));
}

void LK8016E::request_enable()
{
  pending_cmd_.store(0x88, std::memory_order_relaxed);
}

void LK8016E::request_disable()
{
  pending_cmd_.store(0x80, std::memory_order_relaxed);
}

void LK8016E::request_stop()
{
  pending_cmd_.store(0x81, std::memory_order_relaxed);
}

void LK8016E::request_read_single_turn_encoder()
{
  pending_cmd_.store(0x90, std::memory_order_relaxed);
}

void LK8016E::request_read_multi_turn_angle()
{
  pending_cmd_.store(0x92, std::memory_order_relaxed);
}

void LK8016E::onRxFeedback(const CanFrame& f, uint32_t now_ms)
{
  if (f.is_ext || f.dlc < 8 || f.id != rxId()) {
    return;
  }

  const uint8_t cmd = f.data[0];
  const bool is_mg_status = (cmd >= 0xA1u && cmd <= 0xA8u);
  const bool is_basic_ack = (cmd == 0x80u || cmd == 0x81u || cmd == 0x88u);
  const bool is_encoder_read = (cmd == 0x90u);
  const bool is_multi_turn_read = (cmd == 0x92u);
  if (!is_mg_status && !is_basic_ack && !is_encoder_read && !is_multi_turn_read) {
    return;
  }

  __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);

  LK8016EState s = state_;

  s.motor_id = motor_id_;
  s.last_rx_ms = now_ms;
  s.online = true;
  s.rx_count++;

  if (is_mg_status) {
    s.temperature_C = static_cast<int8_t>(f.data[1]);
    s.iq_raw = rd_le_i16(&f.data[2]);
    fp32 max_A = torque_current_max_A_;
    if (max_A <= 0.0f) {
      max_A = 33.0f;
    }
    s.iq_A = static_cast<fp32>(s.iq_raw) / 2048.0f * max_A;
    s.speed_dps = rd_le_i16(&f.data[4]);
    s.encoder_raw = rd_le_u16(&f.data[6]);
    s.last_encoder_rx_ms = now_ms;
    s.encoder_rx_count++;
  } else if (is_encoder_read) {
    s.encoder_raw = rd_le_u16(&f.data[6]);
    s.last_encoder_rx_ms = now_ms;
    s.encoder_rx_count++;
  } else if (is_multi_turn_read) {
    s.multi_turn_angle_cdeg = rd_le_i56_sext(&f.data[1]);
    s.multi_turn_angle_deg = static_cast<fp32>(s.multi_turn_angle_cdeg) * 0.01f;
    s.multi_turn_valid = true;
    s.last_multi_turn_rx_ms = now_ms;
    s.multi_turn_rx_count++;
  }
  s.last_cmd = cmd;

  state_ = s;

  __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
}

void LK8016E::tick(uint32_t now_ms)
{
  const uint32_t last = __atomic_load_n(&state_.last_rx_ms, __ATOMIC_RELAXED);
  const bool was_online = __atomic_load_n(&state_.online, __ATOMIC_RELAXED);

  if (was_online && (now_ms - last > offline_ms_)) {
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);
    state_.online = false;
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
  }
}

bool LK8016E::snapshot(LK8016EState& out) const
{
  for (int i = 0; i < 3; ++i) {
    uint32_t s1 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
    if (s1 & 1u) {
      continue;
    }

    LK8016EState tmp;
    std::memcpy(&tmp, &state_, sizeof(tmp));

    uint32_t s2 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
    if (s1 == s2) {
      out = tmp;
      return true;
    }
  }
  return false;
}

bool LK8016E::exportTxRaw8(uint16_t* sid, uint8_t out[8]) const
{
  if (!sid || !out) {
    return false;
  }

  *sid = rxId();

  uint8_t cmd = pending_cmd_.exchange(0, std::memory_order_acq_rel);
  if (cmd == 0x80 || cmd == 0x88 || cmd == 0x81 || cmd == 0x90 || cmd == 0x92) {
    out[0] = cmd;
    for (int i = 1; i < 8; ++i) out[i] = 0;
    return true;
  }

  const auto mode = static_cast<LK8016EMode>(ctrl_mode_.load(std::memory_order_relaxed));
  switch (mode) {
    case LK8016EMode::Speed: {
      const int32_t sc = cmd_speed_raw_.load(std::memory_order_relaxed);
      out[0] = 0xA2;
      out[1] = 0;
      out[2] = 0;
      out[3] = 0;
      wr_le_i32(&out[4], sc);
      return true;
    }
    case LK8016EMode::PosMulti: {
      const int32_t ang = pos_multi_raw_.load(std::memory_order_relaxed);
      out[0] = 0xA3;
      out[1] = 0;
      out[2] = 0;
      out[3] = 0;
      wr_le_i32(&out[4], ang);
      return true;
    }
    case LK8016EMode::PosMultiMaxSpeed: {
      const int32_t ang = pos_multi_raw_.load(std::memory_order_relaxed);
      const uint16_t ms = pos_multi_maxspd_.load(std::memory_order_relaxed);
      out[0] = 0xA4;
      out[1] = 0;
      wr_le_u16(&out[2], ms);
      wr_le_i32(&out[4], ang);
      return true;
    }
    case LK8016EMode::PosSingle: {
      const uint32_t ang = pos_single_raw_.load(std::memory_order_relaxed);
      const uint8_t dir = pos_single_dir_.load(std::memory_order_relaxed);
      out[0] = 0xA5;
      out[1] = dir;
      out[2] = 0;
      out[3] = 0;
      wr_le_u32(&out[4], ang);
      return true;
    }
    case LK8016EMode::PosSingleMaxSpeed: {
      const uint32_t ang = pos_single_raw_.load(std::memory_order_relaxed);
      const uint8_t dir = pos_single_dir_.load(std::memory_order_relaxed);
      const uint16_t ms = pos_single_maxspd_.load(std::memory_order_relaxed);
      out[0] = 0xA6;
      out[1] = dir;
      wr_le_u16(&out[2], ms);
      wr_le_u32(&out[4], ang);
      return true;
    }
    case LK8016EMode::PosInc: {
      const int32_t ang = pos_inc_raw_.load(std::memory_order_relaxed);
      out[0] = 0xA7;
      out[1] = 0;
      out[2] = 0;
      out[3] = 0;
      wr_le_i32(&out[4], ang);
      return true;
    }
    case LK8016EMode::PosIncMaxSpeed: {
      const int32_t ang = pos_inc_raw_.load(std::memory_order_relaxed);
      const uint16_t ms = pos_inc_maxspd_.load(std::memory_order_relaxed);
      out[0] = 0xA8;
      out[1] = 0;
      // Protocol text may mention uint32, but the frame only carries 2 bytes at data[2..3].
      wr_le_u16(&out[2], ms);
      wr_le_i32(&out[4], ang);
      return true;
    }
    case LK8016EMode::Torque:
    default:
      break;
  }

  const int16_t iq = clamp_iq_raw(cmd_iq_raw_.load(std::memory_order_relaxed));
  out[0] = 0xA1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  wr_le_i16(&out[4], iq);
  out[6] = 0;
  out[7] = 0;
  return true;
}

void LK8016E_CollectTxFrames(LK8016E* const motors[],
                             std::size_t n,
                             CanFrame out[],
                             std::size_t& out_count)
{
  std::size_t idx = 0;
  for (std::size_t i = 0; i < n; ++i) {
    LK8016E* m = motors[i];
    if (!m) {
      continue;
    }

    uint16_t sid;
    uint8_t payload[8];
    if (!m->exportTxRaw8(&sid, payload)) {
      continue;
    }

    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    std::memcpy(f.data, payload, 8);

    out[idx++] = f;
  }
  out_count = idx;
}

bool LK8016E_ExportBroadcast280(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, CanFrame* out)
{
  if (!out) return false;

  out->id = 0x280;
  out->is_ext = false;
  out->dlc = 8;

  // Broadcast uses [-2000,2000] raw (~[-32A,32A]); single motor uses [-2048,2048] (~[-33A,33A]).
  // Note: broadcast requires host config and should not be mixed with single-motor commands.
  const int16_t c1 = clamp_iq_broadcast(iq1);
  const int16_t c2 = clamp_iq_broadcast(iq2);
  const int16_t c3 = clamp_iq_broadcast(iq3);
  const int16_t c4 = clamp_iq_broadcast(iq4);

  wr_le_i16(&out->data[0], c1);
  wr_le_i16(&out->data[2], c2);
  wr_le_i16(&out->data[4], c3);
  wr_le_i16(&out->data[6], c4);
  return true;
}
