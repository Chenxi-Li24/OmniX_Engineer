#ifndef H723VG_V2_FREERTOS_LIB_ADP_LK_MG8016E_H
#define H723VG_V2_FREERTOS_LIB_ADP_LK_MG8016E_H
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

/**
 * @brief Snapshot of LK8016E feedback (MG series).
 * All command replies A1/A2/A3..A8 share the same data layout.
 */
// MG-supported commands: 0x80/0x88/0x81/0xA1/0xA2/0xA3..0xA8/0x280
// Note: 0xA0 open-loop power/voltage is MS-only and not supported by MG.

struct LK8016EState {
  uint8_t  motor_id = 0;
  bool     online = false;
  uint32_t last_rx_ms = 0;
  int8_t   temperature_C = 0;   // 1 C/LSB
  int16_t  iq_raw = 0;          // [-2048,2048]
  fp32     iq_A = 0.0f;         // iq_raw/2048*33A
  int16_t  speed_dps = 0;       // 1 dps/LSB
  uint16_t encoder_raw = 0;
  uint8_t  last_cmd = 0;        // last feedback cmd (A1..A8)
  uint32_t rx_count = 0;
};

/**
 * @brief LK8016E control modes (MG series).
 * Note: 0xA0 open-loop power/voltage is MS-only and not supported by MG.
 */
enum class LK8016EMode : uint8_t {
  Torque            = 0, // 0xA1
  Speed             = 1, // 0xA2
  PosMulti          = 2, // 0xA3
  PosMultiMaxSpeed  = 3, // 0xA4
  PosSingle         = 4, // 0xA5
  PosSingleMaxSpeed = 5, // 0xA6
  PosInc            = 6, // 0xA7
  PosIncMaxSpeed    = 7, // 0xA8
};

/**
 * @brief LK8016E MG adapter.
 * Units/scale:
 * - iq_raw: [-2048,2048] -> [-torque_current_max_A, torque_current_max_A]
 * - speed control: 0.01 dps/LSB
 * - angle control: 0.01 deg/LSB
 * - max speed: 1 dps/LSB
 */
class LK8016E {
public:
  explicit LK8016E(uint8_t id);

  uint8_t  id()   const { return motor_id_; }
  uint16_t rxId() const { return static_cast<uint16_t>(0x140u + motor_id_); }

  void set_offline_timeout(uint32_t ms) { offline_ms_ = ms; }
  void set_torque_current_max_A(fp32 max_A) { torque_current_max_A_ = max_A; }
  void tick(uint32_t now_ms);
  bool snapshot(LK8016EState& out) const;
  LK8016EState state() const noexcept {
    LK8016EState s{};
    (void)snapshot(s);
    return s;
  }

  /** @brief Reference DM style mode control. */
  void set_mode(LK8016EMode m) {
    ctrl_mode_.store(static_cast<uint8_t>(m), std::memory_order_relaxed);
  }

  /** @brief Torque control (0xA1). */
  void set_iq_raw(int16_t iq_raw);
  void set_iq_A(fp32 iq_A);          // A -> raw (torque_current_max_A full scale)

  /** @brief Speed control (0xA2, 0.01 dps/LSB). */
  void set_speed_control_raw(int32_t sc);
  void set_speed_dps(fp32 dps);

  /** @brief Multi-turn position control (0xA3/A4, 0.01 deg/LSB). */
  void set_angle_control_raw(int32_t angle_raw);
  void set_angle_deg(fp32 deg);
  void set_angle_with_max_speed_raw(int32_t angle_raw, uint16_t max_dps);
  void set_angle_deg_max_speed(fp32 deg, fp32 max_dps);

  /** @brief Single-turn position control (0xA5/A6, 0.01 deg/LSB), dir: 0=CW 1=CCW. */
  void set_circle_angle_raw(uint32_t angle_raw, uint8_t dir);
  void set_circle_angle_deg(fp32 deg, bool ccw);
  void set_circle_angle_raw_max_speed(uint32_t angle_raw, uint8_t dir, uint16_t max_dps);
  void set_circle_angle_deg_max_speed(fp32 deg, bool ccw, fp32 max_dps);

  /** @brief Incremental position control (0xA7/A8, 0.01 deg/LSB). */
  void set_inc_angle_raw(int32_t inc_raw);
  void set_inc_angle_deg(fp32 deg);
  void set_inc_angle_raw_max_speed(int32_t inc_raw, uint16_t max_dps);
  void set_inc_angle_deg_max_speed(fp32 deg, fp32 max_dps);

  /** @brief Enable/disable/stop (0x88/0x80/0x81). */
  void request_enable();
  void request_disable();
  void request_stop();

  void onRxFeedback(const CanFrame& f, uint32_t now_ms);
  bool exportTxRaw8(uint16_t* sid, uint8_t out[8]) const;

private:
  static inline int16_t clamp_iq_raw(int32_t v) {
    if (v > 2048) v = 2048;
    if (v < -2048) v = -2048;
    return static_cast<int16_t>(v);
  }

  const uint8_t motor_id_;

  std::atomic<uint8_t>  ctrl_mode_{static_cast<uint8_t>(LK8016EMode::Torque)};
  std::atomic<int16_t>  cmd_iq_raw_{0};
  std::atomic<int32_t>  cmd_speed_raw_{0};
  std::atomic<int32_t>  pos_multi_raw_{0};
  std::atomic<uint16_t> pos_multi_maxspd_{0};
  std::atomic<uint8_t>  pos_single_dir_{0};
  std::atomic<uint32_t> pos_single_raw_{0};
  std::atomic<uint16_t> pos_single_maxspd_{0};
  std::atomic<int32_t>  pos_inc_raw_{0};
  std::atomic<uint16_t> pos_inc_maxspd_{0};
  mutable std::atomic<uint8_t>  pending_cmd_{0};

  mutable volatile uint32_t state_seq_{0};
  LK8016EState state_{};

  uint32_t offline_ms_ = 100;
  fp32 torque_current_max_A_ = 33.0f;
};

void LK8016E_CollectTxFrames(LK8016E* const motors[],
                             std::size_t n,
                             CanFrame out[],
                             std::size_t& out_count);

/**
 * @brief Export MG broadcast torque frame (0x280).
 * Range uses [-2000,2000] raw (~[-32A,32A]); single motor uses [-2048,2048] (~[-33A,33A]).
 * Note: broadcast requires host config and should not be mixed with single-motor commands.
 */
bool LK8016E_ExportBroadcast280(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, CanFrame* out);

#endif // H723VG_V2_FREERTOS_LIB_ADP_LK_MG8016E_H
