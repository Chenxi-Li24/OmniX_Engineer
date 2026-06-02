#ifndef H723VG_V2_FREERTOS_LIB_ADP_DM_4340_H
#define H723VG_V2_FREERTOS_LIB_ADP_DM_4340_H
#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

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

// DM4340 feedback snapshot (MIT protocol per DM manual)
struct DM4340State {
  uint8_t  motor_id = 0;
  uint8_t  error_code = 0;

  int16_t  pos_raw = 0;          // signed 16-bit mapped position
  int32_t  pos_raw_total = 0;    // kept for API compatibility (no multi-turn by default)
  fp32     pos_rad = 0.0f;       // mapped position (rad)
  fp32     pos_rad_total = 0.0f; // same as pos_rad unless user adds unwrap

  int16_t  speed_raw = 0;        // signed 12-bit mapped speed (stored in int16)
  fp32     speed_rad_s = 0.0f;   // mapped speed (rad/s)

  int16_t  torque_raw = 0;       // signed 12-bit mapped torque (stored in int16)
  fp32     torque_nm = 0.0f;     // mapped torque (Nm)

  uint8_t  temp_mos_C = 0;
  uint8_t  temp_rotor_C = 0;

  uint32_t last_rx_ms = 0;
  bool     online = false;
};

/**
 * @brief Control mode for DM4340 V4 driver (per DM-J4340-2EC manual)
 *
 * Mode -> CAN SID mapping:
 *   MIT          : SID = 0x000 + can_id
 *   PosVel       : SID = 0x100 + can_id
 *   Velocity     : SID = 0x200 + can_id
 *   PosVelTorque : SID = 0x300 + can_id
 */
enum class DM4340Mode : uint8_t {
  MIT          = 1,
  PosVel       = 2,
  Velocity     = 3,
  PosVelTorque = 4,
};

class DM4340 {
public:
  explicit DM4340(uint8_t can_id, uint16_t fb_master_sid = 0);

  uint8_t   canId()       const { return can_id_; }
  uint16_t  fbMasterSid() const { return fb_master_sid_; }
  DM4340Mode mode()       const {
    return static_cast<DM4340Mode>(mode_.load(std::memory_order_relaxed));
  }

  void set_mode(DM4340Mode m) {
    mode_.store(static_cast<uint8_t>(m), std::memory_order_relaxed);
  }

  // MIT mapping limits (must match motor-side PMAX/VMAX/TMAX)
  void set_mit_limits(fp32 pmax_rad, fp32 vmax_rad_s, fp32 tmax_nm);

  // MIT-mode control (physical units)
  void set_mit_cmd(fp32 pos_rad, fp32 vel_rad_s, fp32 kp, fp32 kd, fp32 torque_nm);
  void set_mit_pos_rad(fp32 pos_rad);
  void set_mit_vel_rad_s(fp32 vel_rad_s);
  void set_mit_kp(fp32 kp);
  void set_mit_kd(fp32 kd);
  void set_mit_torque_nm(fp32 torque_nm);

  // PosVel / Velocity / PosVelTorque control (manual-defined encodings)
  void set_posvel_cmd(fp32 pos_rad, fp32 vel_rad_s);
  void set_velocity_cmd(fp32 vel_rad_s);
  void set_posvel_torque_cmd(fp32 pos_rad, fp32 vlim_rad_s, fp32 ilimit_norm);

  // feedback handler / online timeout
  void onRxFeedback(const CanFrame& f, uint32_t now_ms);
  void set_offline_timeout(uint32_t ms) { offline_ms_ = ms; }
  void tick(uint32_t now_ms);

  bool snapshot(DM4340State& out) const;
  DM4340State state() const noexcept {
    DM4340State s{};
    (void)snapshot(s);
    return s;
  }

  bool exportTxRaw8(uint16_t* sid, uint8_t out[8]) const;

private:
  static inline int16_t dm_signext12(uint16_t q12) {
    q12 &= 0x0FFFu;
    if (q12 & 0x0800u) q12 |= 0xF000u;
    return static_cast<int16_t>(q12);
  }
  static inline uint8_t payload_id(uint8_t d0)  { return (d0 & 0x0Fu); }
  static inline uint8_t payload_err(uint8_t d0) { return (d0 >> 4); }

  const uint8_t  can_id_;
  const uint16_t fb_master_sid_;

  std::atomic<uint8_t>  mode_{static_cast<uint8_t>(DM4340Mode::MIT)};
  std::atomic<uint32_t> cmd_mit_pos_u32_{0};
  std::atomic<uint32_t> cmd_mit_vel_u32_{0};
  std::atomic<uint32_t> cmd_mit_kp_u32_{0};
  std::atomic<uint32_t> cmd_mit_kd_u32_{0};
  std::atomic<uint32_t> cmd_mit_torque_u32_{0};
  std::atomic<uint8_t>  mit_cmd_valid_{0};

  std::atomic<uint32_t> cmd_posvel_pos_u32_{0};
  std::atomic<uint32_t> cmd_posvel_vel_u32_{0};
  std::atomic<uint32_t> cmd_vel_u32_{0};

  std::atomic<uint32_t> cmd_pvt_pos_u32_{0};
  std::atomic<uint16_t> cmd_pvt_vlim_u16_{0};
  std::atomic<uint16_t> cmd_pvt_ilim_u16_{0};

  std::atomic<uint32_t> pmax_u32_{0};
  std::atomic<uint32_t> vmax_u32_{0};
  std::atomic<uint32_t> tmax_u32_{0};

  mutable volatile uint32_t state_seq_{0};
  DM4340State state_{};
  uint32_t offline_ms_ = 100;
};

#endif // H723VG_V2_FREERTOS_LIB_ADP_DM_4340_H

