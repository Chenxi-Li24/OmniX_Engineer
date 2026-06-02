#ifndef H723VG_V2_FREERTOS_LIB_ADP_DM_4310_H
#define H723VG_V2_FREERTOS_LIB_ADP_DM_4310_H
#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <cstring> // Added for memcpy

#include "../../lib_struct_typedef/Inc/struct_typedef.h"

#ifndef ADP_CANFRAME_DEFINED
#define ADP_CANFRAME_DEFINED
struct CanFrame {
  uint32_t id;
  uint8_t  dlc;
  bool     is_ext;
  uint8_t  data[8];
};
#endif

// 4310 feedback snapshot
struct DM4310State {
  uint8_t  motor_id = 0;
  uint8_t  error_code = 0;    /* error code: 8=overvoltage, 9=undervoltage, A=overcurrent,
                              B=MOS overheat, C=rotor overheat, D=loss comm, E=overload*/

  // feedback (raw + derived)
  int16_t  pos_raw = 0;          // raw position ticks
  int32_t  pos_raw_total = 0;    // new: multi-turn raw accumulator
  fp32     pos_rad = 0.0f;       // position in rad
  fp32     pos_rad_total = 0.0f; // new: multi-turn rad
  int16_t  speed_rpm = 0;        // raw speed in rpm
  fp32     speed_rad_s = 0.0f;   // speed in rad/s
  int16_t  torque_raw = 0;       // raw torque unit
  uint8_t  temp_mos_C = 0;       // MOS temperature
  uint8_t  temp_rotor_C = 0;     // rotor temperature

  // meta
  uint32_t last_rx_ms = 0;
  bool     online = false;

  // previous snapshot
  int16_t  prev_pos_raw = 0;
  int16_t  prev_speed_raw = 0;
  int16_t  prev_torque_raw = 0;
};

/**
 * @brief Control mode: decides CAN SID mapping
 *
 * Mode → standard ID (SID) mapping:
 * @li MIT      -> SID = 0x000 + can_id
 * @li PosVel   -> SID = 0x100 + can_id
 * @li Velocity -> SID = 0x200 + can_id
 */
enum class DM4310Mode : uint8_t {
  MIT      = 0,
  PosVel   = 1,
  Velocity = 2,
};

class DM4310 {
public:
  // can_id: device ID (0..255, typically 0..127)
  // fb_master_sid: standard ID used for feedback frames
  explicit DM4310(uint8_t can_id, uint16_t fb_master_sid = 0);

  // fixed info / read-only
  uint8_t   canId()       const { return can_id_; }
  uint16_t  fbMasterSid() const { return fb_master_sid_; }
  DM4310Mode mode()       const {
    return static_cast<DM4310Mode>(mode_.load(std::memory_order_relaxed));
  }

  // control interface
  void set_mode(DM4310Mode m) {
    mode_.store(static_cast<uint8_t>(m), std::memory_order_relaxed);
  }

  // MIT-mode control (physical units)
  void set_mit_cmd(fp32 pos_rad, fp32 vel_rad_s, fp32 kp, fp32 kd, fp32 torque_nm);
  void set_mit_pos_rad(fp32 pos_rad);
  void set_mit_vel_rad_s(fp32 vel_rad_s);
  void set_mit_kp(fp32 kp);
  void set_mit_kd(fp32 kd);
  void set_mit_torque_nm(fp32 torque_nm);

  // New float interface (Thread-safe bit-casting)
  void set_pos_rad(float p) { cmd_pos_f32_.store(f2u(p), std::memory_order_relaxed); }
  void set_vel_lim(float v) { cmd_vel_f32_.store(f2u(v), std::memory_order_relaxed); }

  // Getters for internal use
  float get_pos_cmd_rad() const { return u2f(cmd_pos_f32_.load(std::memory_order_relaxed)); }
  float get_vel_cmd_lim() const { return u2f(cmd_vel_f32_.load(std::memory_order_relaxed)); }

  // Legacy/Raw interface adaptation (Proxy to float storage)
  void set_pos_raw(int16_t pos)   { set_pos_rad(static_cast<float>(pos)); }
  void set_vel_raw(int16_t vel)   { set_vel_lim(static_cast<float>(vel)); }
  void set_torque_raw(int16_t tq) { cmd_torque_raw_.store(tq, std::memory_order_relaxed); }

  // feedback handler / online timeout
  void onRxFeedback(const CanFrame& f, uint32_t now_ms);
  void set_offline_timeout(uint32_t ms) { offline_ms_ = ms; }
  void tick(uint32_t now_ms);

  // snapshot read (seqlock style)
  bool snapshot(DM4310State& out) const;
  DM4310State state() const noexcept {
    DM4310State s{};
    (void)snapshot(s);
    return s;
  }

  // export one 8-byte TX frame according to current mode()
  bool exportTxRaw8(uint16_t* sid, uint8_t out[8]) const;

private:
  // 12-bit sign extension
  static inline int16_t dm4310_signext12(uint16_t q12) {
    q12 &= 0x0FFFu;
    if (q12 & 0x0800u) q12 |= 0xF000u;
    return static_cast<int16_t>(q12);
  }

  // D0 byte: low 4 bits = device ID, high 4 bits = error code
  static inline uint8_t dm4310_payload_id(uint8_t d0)  { return (d0 & 0x0Fu); }
  static inline uint8_t dm4310_payload_err(uint8_t d0) { return (d0 >> 4); }

  // Helper for float bitwise atomics
  static inline uint32_t f2u(float x) { uint32_t u; std::memcpy(&u, &x, 4); return u; }
  static inline float u2f(uint32_t u) { float x; std::memcpy(&x, &u, 4); return x; }

  // fixed metadata
  const uint8_t  can_id_;
  const uint16_t fb_master_sid_;

  // command registers
  std::atomic<uint8_t>  mode_{static_cast<uint8_t>(DM4310Mode::MIT)};

  // Replaced int16_t raw storage with uint32_t (float bit pattern)
  std::atomic<uint32_t> cmd_pos_f32_{0};
  std::atomic<uint32_t> cmd_vel_f32_{0};

  std::atomic<int16_t>  cmd_torque_raw_{0};

  std::atomic<uint32_t> cmd_mit_pos_u32_{0};
  std::atomic<uint32_t> cmd_mit_vel_u32_{0};
  std::atomic<uint32_t> cmd_mit_kp_u32_{0};
  std::atomic<uint32_t> cmd_mit_kd_u32_{0};
  std::atomic<uint32_t> cmd_mit_torque_u32_{0};
  std::atomic<uint8_t>  mit_cmd_valid_{0};

  // state (seqlock: even = stable, odd = being written)
  mutable volatile uint32_t state_seq_{0};
  DM4310State state_{};

  uint32_t offline_ms_ = 100;    // offline threshold in ms
};

/**
 * @brief Collect TX frames for an array of DM4310 motors.
 *
 * @param motors    motor pointer array
 * @param n         number of entries in motors[]
 * @param out       output frame array (must hold at least n frames)
 * @param out_count number of valid frames written to out[]
 */
void DM4310_CollectTxFrames(DM4310* const motors[],
                            std::size_t n,
                            CanFrame out[],
                            std::size_t& out_count);

#endif // H723VG_V2_FREERTOS_LIB_ADP_DM_4310_H
