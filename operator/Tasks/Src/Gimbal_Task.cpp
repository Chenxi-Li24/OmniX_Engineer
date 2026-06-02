#include "../Inc/Gimbal_Task.h"

#include "bsp_srn_log.h"
#include "CONF_Gimbal_Task.h"

#include "Gimbal_Task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "Gimbal_behavior_Task.h"
#include "Gimbal_Debug.h"
#include "LED_Task.h"
#include "Referee_Task.h"
#include "bsp_buzzer.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#include "NTFDCAN_Router.h"
#include "bsp_fdcan.h"
#include "lib_adp_dm_4310.h"
#include "lib_adp_dm_4340.h"
#include "lib_remote_control.h"
#include "RC_Control_Mode.h"
#include "RC_Task.h"
#include "VT03_Gimbal_Mode.h"
#include "pid.h"
#include <cmath>


float gimbal_yaw_rad = 0.0f;
extern bool read_gimbal_cmd(gimbal_cmd_t* out); // from Behaviour TU
extern volatile bool frontFLAG;


volatile bool gimbal_all_online = false;
//float yaw_transport   = 0.0f;
 gimbal_feedback_t gimbal_feedback;

// 姣忚酱鐙珛 PID 鍙傛暟鎵撳寘
struct AxisPidCfg {
  fp32 ang[3];
  fp32 ang_max_out;
  fp32 ang_max_iout;
  fp32 mit_kp;
  fp32 mit_kd;
};

namespace {
constexpr fp32 GIMBAL_PI_RAD = 3.14159265358979323846f;
constexpr uint32_t GIMBAL_J4_TRIGGER_BEEP_HZ = 1800u;
constexpr uint32_t GIMBAL_J4_TRIGGER_BEEP_MS = 120u;

struct DmJ4J7Snapshot {
  uint16_t raw_u16[4];
  int16_t angle_cdeg[4];
  uint8_t online[4];
};

volatile uint32_t g_dm_j4j7_guard = 0u;
DmJ4J7Snapshot g_dm_j4j7_snap{};

inline uint16_t dm4310_raw_u16_from_pos_raw(int16_t raw)
{
  return DM4310_PosRawToU16(raw);
}

inline uint16_t dm_pos_rad_to_u16(fp32 pos_rad, fp32 pos_rad_abs_max)
{
  if (pos_rad_abs_max <= 0.0f) {
    return 0u;
  }
  const fp32 min_rad = -pos_rad_abs_max;
  const fp32 max_rad = pos_rad_abs_max;
  fp32 clamped = pos_rad;
  if (clamped < min_rad) clamped = min_rad;
  if (clamped > max_rad) clamped = max_rad;
  const fp32 norm = (clamped - min_rad) / (max_rad - min_rad);
  uint32_t raw_u32 = static_cast<uint32_t>(norm * 65535.0f + 0.5f);
  if (raw_u32 > 65535u) {
    raw_u32 = 65535u;
  }
  return static_cast<uint16_t>(raw_u32);
}

inline int16_t dm_cdeg_from_rad(fp32 rad)
{
  return static_cast<int16_t>(Omnix_RadToCdeg(rad));
}

inline fp32 clamp_fp32(fp32 value, fp32 min_value, fp32 max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

inline fp32 wrap_symmetric_period(fp32 value, fp32 period)
{
  if (period <= 0.0f) {
    return value;
  }
  fp32 wrapped = std::fmod(value + 0.5f * period, period);
  if (wrapped < 0.0f) {
    wrapped += period;
  }
  return wrapped - 0.5f * period;
}

inline fp32 nearest_equivalent(fp32 value, fp32 reference, fp32 period)
{
  if (period <= 0.0f) {
    return value;
  }
  return value + period * std::round((reference - value) / period);
}

inline const char* dm4310_mode_name(DM4310Mode mode)
{
  switch (mode) {
    case DM4310Mode::MIT:      return "MIT";
    case DM4310Mode::PosVel:   return "PosVel";
    case DM4310Mode::Velocity: return "Velocity";
    default:                   return "Unknown";
  }
}
} // namespace

extern "C" bool Gimbal_ReadDmJ4J7Snapshot(uint16_t raw_u16[4], int16_t angle_cdeg[4], uint8_t online[4])
{
  if (raw_u16 == nullptr || angle_cdeg == nullptr || online == nullptr) {
    return false;
  }

  uint32_t seq_a = 0u;
  uint32_t seq_b = 0u;
  DmJ4J7Snapshot local{};
  do {
    seq_a = g_dm_j4j7_guard;
    __DMB();
    local = g_dm_j4j7_snap;
    __DMB();
    seq_b = g_dm_j4j7_guard;
  } while (seq_a != seq_b);

  for (int i = 0; i < 4; ++i) {
    raw_u16[i] = local.raw_u16[i];
    angle_cdeg[i] = local.angle_cdeg[i];
    online[i] = local.online[i];
  }
  return true;
}

// 鍗曡酱灏佽锛氭瘡涓疄渚嬪彧绠＄悊鈥滀竴涓€?DM4310锛屽弬鏁扮敱 AxisPidCfg 娉ㄥ叆锛岀‘淇?yaw/pitch 瀹屽叏鐙珛
class GimbalAxis {
public:
  GimbalAxis(uint8_t id, uint8_t bus, uint16_t fb_sid,
             fp32 zero_ofst_rad, fp32 min_rel, fp32 max_rel,
             GimbalMotorPosRefMode pos_ref_mode,
             fp32 abs_window_min_rel, fp32 abs_window_max_rel,
             const AxisPidCfg& cfg)
  : motor_(id, fb_sid),
    bus_(bus),
    zero_ofst_(zero_ofst_rad),
    min_rel_(min_rel),
    max_rel_(max_rel),
    pos_ref_mode_(pos_ref_mode),
    abs_window_min_rel_(abs_window_min_rel),
    abs_window_max_rel_(abs_window_max_rel)
  {
    motor_.set_offline_timeout(100);
    (void)canrx_register_dm4310(bus, &motor_);
    (void)cantx_register_dm4310(bus, &motor_, 3);
    osDelay(1);

    PID_init(&pid_ang_, algo::PID_POSITION, cfg.ang, cfg.ang_max_out, cfg.ang_max_iout);
    mit_kp_ = cfg.mit_kp;
    mit_kd_ = cfg.mit_kd;
    last_tx_mode_ = desired_tx_mode();
    motor_.set_mode(last_tx_mode_);
  }

  enum class CtrlMode : uint8_t {
    Angle,  // 瑙掑害澶栫幆 + 閫熷害鍐呯幆锛堢幇鍦?pitch 鐢級
    Rate    // 绾€熷害鎺у埗锛堢幇鍦?yaw 鐢級
  };

  void set_ctrl_mode(CtrlMode m) { ctrl_mode_ = m; }
  void set_debug(volatile GimbalPidDebug* dbg) { dbg_ = dbg; }


  // Rate 妯″紡涓嬶紝鐢变笂灞傦紙琛屼负浠诲姟锛夌洿鎺ョ粰鈥滅洰鏍囪閫熷害鈥?
  void set_rate_cmd(fp32 w) { w_rate_cmd_ = w; }
  void set_torque_ff_nm(fp32 t_ff) { torque_ff_nm_ = t_ff; }
  void set_use_mit(bool enable) { use_mit_ = enable; }
  void set_direct_abs_target_enable(bool enable) { direct_abs_target_enable_ = enable; }
  void set_direct_abs_target_single_turn_shortest_wrap(bool enable) {
    direct_abs_single_turn_shortest_wrap_ = enable;
  }
  void set_direct_abs_target_rad(fp32 abs_rad) { direct_abs_target_rad_ = normalize_direct_abs_target_for_mode(abs_rad); }
  DM4310Mode tx_mode() const { return last_tx_mode_; }
  fp32 mit_kp() const { return mit_kp_; }
  fp32 mit_kd() const { return mit_kd_; }
  bool direct_abs_target_enable() const { return direct_abs_target_enable_; }
  fp32 motor_abs_rad() const { return motor_abs_rad_; }
  fp32 direct_abs_target_rad() const { return direct_abs_target_rad_; }
  fp32 target_angle() const { return theta_set_; }

  // Export one 8-byte TX frame according to current command state
  bool export_tx_frame(uint16_t* sid, uint8_t out[8]) const { return motor_.exportTxRaw8(sid, out); }

  // 浠ュ悗闇€瑕侊細鏆撮湶褰撳墠缂栫爜鍣ㄨ搴?/ 瑙掗€熷害
  fp32 theta() const { return theta_; }  // 鍗曞湀锛岀浉瀵?zero_ofst_
  fp32 omega() const { return omega_; }  // rad/s
  fp32 raw() const {return raw_; }  // ecd
  uint16_t raw_u16() const { return dm4310_raw_u16_from_pos_raw(static_cast<int16_t>(raw_)); }
  int16_t torque_raw() const { return torque_raw_; }
  uint8_t error_code() const { return error_code_; }

  /**
   * @brief 璁剧疆褰撳墠瑙掑害涓洪浂鐐癸紝鐢ㄤ簬鐩稿瑙掓帶鍒?
   */
  void rezero(fp32 new_zero_ofst_rad) {
    zero_ofst_ += new_zero_ofst_rad;
    theta_ = 0.0f;
    theta_set_ = 0.0f;
    abs_ref_valid_ = false;
    algo::PID_clear(&pid_ang_);
    algo::PID_clear(&pid_vel_);
  }

  // Set current encoder position as new zero (useful at startup)
  void rezero_here() {
    const auto s = motor_.state();
    zero_ofst_ = s.pos_rad_total;
    theta_ = 0.0f;
    theta_set_ = 0.0f;
    abs_ref_valid_ = false;
    algo::PID_clear(&pid_ang_);
    algo::PID_clear(&pid_vel_);
  }

  // Shift zero without snapping the current theta to 0.
  void shift_zero(fp32 delta_rad) {
    zero_ofst_ += delta_rad;
    theta_ -= delta_rad;
    theta_set_ -= delta_rad;
    abs_ref_valid_ = false;
    algo::PID_clear(&pid_ang_);
    algo::PID_clear(&pid_vel_);
  }

  void update_fb()
  {
    // 椹卞姩绂荤嚎鍒ゅ畾鎵撳嬀锛氶渶瑕佸懆鏈?tick() 鎵嶈兘鎶?online 浠?true 鎷変綆
    motor_.tick(HAL_GetTick());
    const auto s = motor_.state();
    online_ = s.online;
    error_code_ = s.error_code;
    theta_  = s.pos_rad_total - zero_ofst_;//璧嬪€肩數鏈鸿搴︾姸鎬?
    raw_ = s.pos_raw;//ecd
    motor_abs_rad_ = s.pos_rad;
    if (!online_) {
      abs_ref_valid_ = false;
    } else {
      switch (pos_ref_mode_) {
        case GimbalMotorPosRefMode::AbsoluteSingleTurn:
          theta_ = wrap_symmetric_period(s.pos_rad - zero_ofst_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
          break;
        case GimbalMotorPosRefMode::AbsoluteFiniteTurn: {
          const fp32 rel_single =
              wrap_symmetric_period(s.pos_rad - zero_ofst_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
          if (!abs_ref_valid_) {
            theta_ = clamp_fp32(rel_single, abs_window_min_rel_, abs_window_max_rel_);
            abs_ref_valid_ = true;
          } else {
            theta_ = nearest_equivalent(rel_single, theta_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
            theta_ = clamp_fp32(theta_, abs_window_min_rel_, abs_window_max_rel_);
          }
          break;
        }
        case GimbalMotorPosRefMode::RelativeMultiTurn:
        default:
          break;
      }
    }
    // 浼樺厛浣跨敤 rad/s锛涜嫢浣犵殑鍗忚鎻愪緵鐨勬槸 rpm锛屽彲鏀圭敤 s.speed_rpm
    // 浣跨敤鍙嶉閫熷害锛氳嫢璁惧缁欑殑鏄?rpm锛岃繖閲屽彲閫夌敤 rpm锛涙寜闇€瑕佽皟鍙?
    omega_  = static_cast<fp32>(s.speed_rad_s);
    torque_raw_ = s.torque_raw;
  }
  void set_target_angle(fp32 rad)
  {
    theta_set_ = normalize_target_for_mode(rad);
  }
  void recenter()
  {
    theta_set_ = 0.0f;
    return;
    theta_set_ = zero_ofst_; // 鎴栬€呮寜闇€璁句负 0
  }

  void add_set_by_inc(fp32 inc_rad)
  {
    theta_set_ = normalize_target_for_mode(theta_set_ + inc_rad);
  }

  void hold_here(){ theta_set_ = normalize_target_for_mode(theta_); }

  // 绔嬪嵆鍙戦€佸紑鍏冲姏鐭╁抚锛?
  // enable=true  -> FF FF FF FF FF FF FF FC
  // enable=false -> FF FF FF FF FF FF FF FD
  void send_enable_disable(bool enable) {
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, static_cast<uint8_t>(enable ? 0xFC : 0xFD)};
    // DM4310 special enable/disable frames always use 0x000 + id.
    const uint16_t sid = static_cast<uint16_t>(0x000u + motor_.canId());
    (void)cantx_send_now(bus_, sid, data, 8, 200);
  }

  void run(bool zero_force)
  {
    const DM4310Mode tx_mode = apply_tx_mode();
    // 妫€娴嬬绾?鍦ㄧ嚎杈规部
    const bool came_online  = (online_ && !was_online_);
    const bool went_offline = (!online_ && was_online_);

    if (went_offline) {
      // 涓嬬數/绂荤嚎锛氱珛鍗虫竻鐘舵€侊紝绛夊緟閲嶆柊涓婄嚎
      send_zero_cmd(tx_mode);
      algo::PID_clear(&pid_ang_);
      algo::PID_clear(&pid_vel_);
      w_cmd_ = 0.0f;
      abs_ref_valid_ = false;
      warmup_ticks_ = 0;
    }

    // 涓婄數/鍥炲埌鍦ㄧ嚎锛氬榻愭湡鏈涖€佽蒋鍚姩锛屽苟鎶婄數鏈哄姏鐭╃姸鎬佷笌褰撳墠 zero_force 涓€鑷?
    if (came_online) {
      //hold_here();               // 鎶?setpoint 瀵归綈鍒板綋鍓嶄綅缃紝閬垮厤鍙傝€冭烦鍙?
      algo::PID_clear(&pid_ang_);
      algo::PID_clear(&pid_vel_);
      w_cmd_ = 0.0f;
      warmup_ticks_ = 100;       // ~200ms 杞惎鍔ㄧ獥鍙ｏ紙2ms tick锛?
      send_enable_disable(!zero_force); // enable/disable only; periodic resend handled by task logic
    }

    if (zero_force || !online_) {
      send_zero_cmd(tx_mode);
      algo::PID_clear(&pid_ang_);
      if (dbg_) {
      dbg_->online = online_ ? 1u : 0u;
      dbg_->pos_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
      dbg_->pos_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
      dbg_->vel_target = w_cmd_;
      dbg_->vel_actual = omega_;
      dbg_->torque_target = torque_ff_nm_;
      dbg_->torque_actual = static_cast<fp32>(torque_raw_);
      if (ctrl_mode_ == CtrlMode::Angle) {
        dbg_->loop_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
        dbg_->loop_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
        dbg_->p_out = pid_ang_.Pout;
        dbg_->i_out = pid_ang_.Iout;
        dbg_->d_out = pid_ang_.Dout;
        dbg_->pid_out = pid_ang_.out;
      } else {
        dbg_->loop_target = w_rate_cmd_;
        dbg_->loop_actual = omega_;
        dbg_->p_out = 0.0f;
        dbg_->i_out = 0.0f;
        dbg_->d_out = 0.0f;
        dbg_->pid_out = 0.0f;
      }
      dbg_->mode = static_cast<uint8_t>(ctrl_mode_);
    }

    was_online_ = online_;
      return;
    }

    // 涓婄嚎鍒濇湡鐨勮蒋鍚姩锛氱淮鎸?0 杈撳嚭涓€灏忔鏃堕棿锛岀瓑椹卞姩鑷瀹屾垚
    if (warmup_ticks_ > 0) {
      send_zero_cmd(tx_mode);
      warmup_ticks_--;
      if (dbg_) {
      dbg_->online = online_ ? 1u : 0u;
      dbg_->pos_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
      dbg_->pos_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
      dbg_->vel_target = w_cmd_;
      dbg_->vel_actual = omega_;
      dbg_->torque_target = torque_ff_nm_;
      dbg_->torque_actual = static_cast<fp32>(torque_raw_);
      if (ctrl_mode_ == CtrlMode::Angle) {
        dbg_->loop_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
        dbg_->loop_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
        dbg_->p_out = pid_ang_.Pout;
        dbg_->i_out = pid_ang_.Iout;
        dbg_->d_out = pid_ang_.Dout;
        dbg_->pid_out = pid_ang_.out;
      } else {
        dbg_->loop_target = w_rate_cmd_;
        dbg_->loop_actual = omega_;
        dbg_->p_out = 0.0f;
        dbg_->i_out = 0.0f;
        dbg_->d_out = 0.0f;
        dbg_->pid_out = 0.0f;
      }
      dbg_->mode = static_cast<uint8_t>(ctrl_mode_);
    }

    was_online_ = online_;
      return;
    }

    // 杩欓噷鎶娾€滅洰鏍囪閫熷害鈥濈粺涓€鍙?w_target锛屽悗闈㈡枩鍧?闄愬箙澶嶇敤
    fp32 w_target = 0.0f;

    if (ctrl_mode_ == CtrlMode::Angle) {
      // 瑙掑害妯″紡锛氬拰鐜板湪涓€鏍凤紝wrap 瑙掑害璇樊锛岃窇 PID
      const fp32 err = angle_error_for_pid();

      // 杩戦浂姝诲尯閭ｆ鍙互淇濈暀锛堝噺灏戞姈锛?
      constexpr fp32 ANG_DB = 0.01f; // 绾?0.57掳
      constexpr fp32 VEL_DB = 5.0f * (2.0f*3.14159265f/60.0f); // 5 rpm
      if (fabsf(err) < ANG_DB && fabsf(omega_) < VEL_DB) {
        algo::PID_clear(&pid_ang_);
      }

      w_target = algo::PID_calc(&pid_ang_, /*get*/position_feedback_for_pid(), /*set*/theta_set_);

    } else { // CtrlMode::Rate
      // 閫熷害妯″紡锛氬畬鍏ㄥ拷鐣ヨ搴?PID锛岀洿鎺ョ敤涓婂眰缁欑殑瑙掗€熷害
      w_target = w_rate_cmd_;
    }
    // --- 杈撳嚭闄愬箙 + 鏂滃潯锛堝彲閫夛紝寤鸿淇濈暀锛?---
    // 鎶婅搴︾幆鐨勮緭鍑哄綋鈥滄湡鏈涜閫熷害鈥濓紙rad/s锛夈€傚洜姝ゆ妸瑙掑害 PID 鐨勬渶澶ц緭鍑鸿涓轰綘鍏佽鐨勬渶澶ф満姊拌閫熷害銆?
    // Increase maximum allowed angular velocity to allow stronger/faster motor effort
    constexpr fp32 W_MAX  = 10.0f;   // rad/s, raised from 5.0
    constexpr fp32 W_SLEW = 80.0f;  // rad/s^2 鏂滅巼闄愬埗 (raised from 40.0)
    const fp32 w_clip = std::max(-W_MAX, std::min(W_MAX, w_target));
    // 2ms tick
    const fp32 dt = 0.002f;
    const fp32 dw_lim = W_SLEW * dt;
    w_cmd_ += std::max(-dw_lim, std::min(dw_lim, w_clip - w_cmd_));

    // --- 涓嬪彂閫熷害璁惧畾 ---
    // w_cmd_ 鍗曚綅锛歳ad/s
    fp32 w_send = w_cmd_;

    // 鍐嶄繚闄╀竴涓嬶紝杩欓噷鍙互鍐嶅す涓€灞傜粷瀵规渶澶ц閫熷害
    constexpr fp32 W_ABS_MAX = 15.0f;  // raised absolute cap from 8.0 to 15.0
    if (w_send >  W_ABS_MAX) w_send =  W_ABS_MAX;
    if (w_send < -W_ABS_MAX) w_send = -W_ABS_MAX;

    // 閲忓寲鎴?int16_t锛岃繖閲屽厛鐢?1 rad/s 鐨勬杩涳紝绠€鍗曠洿鎺?
    if (tx_mode == DM4310Mode::MIT) {
      fp32 p_des = 0.0f;
      if (mit_kp_ > 0.0f) {
        p_des = direct_abs_target_enable_ ? direct_abs_target_rad_ : motor_target_pos_rad(theta_set_);
      }
      motor_.set_mit_cmd(p_des, w_send, mit_kp_, mit_kd_, torque_ff_nm_);
    } else if (tx_mode == DM4310Mode::PosVel) {
      motor_.set_pos_rad(direct_abs_target_enable_ ? direct_abs_target_rad_ : motor_target_pos_rad(theta_set_));
      motor_.set_vel_lim(fabsf(w_send));
    } else {
      motor_.set_vel_lim(w_send);
    }


    if (dbg_) {
      dbg_->online = online_ ? 1u : 0u;
      dbg_->pos_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
      dbg_->pos_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
      dbg_->vel_target = w_cmd_;
      dbg_->vel_actual = omega_;
      dbg_->torque_target = torque_ff_nm_;
      dbg_->torque_actual = static_cast<fp32>(torque_raw_);
      if (ctrl_mode_ == CtrlMode::Angle) {
        dbg_->loop_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
        dbg_->loop_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
        dbg_->p_out = pid_ang_.Pout;
        dbg_->i_out = pid_ang_.Iout;
        dbg_->d_out = pid_ang_.Dout;
        dbg_->pid_out = pid_ang_.out;
      } else {
        dbg_->loop_target = w_rate_cmd_;
        dbg_->loop_actual = omega_;
        dbg_->p_out = 0.0f;
        dbg_->i_out = 0.0f;
        dbg_->d_out = 0.0f;
        dbg_->pid_out = 0.0f;
      }
      dbg_->mode = static_cast<uint8_t>(ctrl_mode_);
    }

    was_online_ = online_;
  }


  bool online() const { return online_; }

  void reset_pid() { algo::PID_clear(&pid_ang_); algo::PID_clear(&pid_vel_); }

private:
  DM4310Mode desired_tx_mode() const
  {
    if (use_mit_) {
      return DM4310Mode::MIT;
    }
    return (ctrl_mode_ == CtrlMode::Rate) ? DM4310Mode::Velocity : DM4310Mode::PosVel;
  }

  DM4310Mode apply_tx_mode()
  {
    const DM4310Mode m = desired_tx_mode();
    if (m != last_tx_mode_) {
      motor_.set_mode(m);
      last_tx_mode_ = m;
    }
    return m;
  }

  void send_zero_cmd(DM4310Mode mode)
  {
    if (mode == DM4310Mode::MIT) {
      motor_.set_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    } else if (mode == DM4310Mode::PosVel) {
      motor_.set_pos_rad(direct_abs_target_enable_ ? motor_abs_rad_ : motor_target_pos_rad(theta_));
      motor_.set_vel_lim(0.0f);
    } else {
      motor_.set_vel_lim(0.0f);
    }
  }

  fp32 clamp_target_limits(fp32 rad) const
  {
    fp32 limited = clamp_fp32(rad, min_rel_, max_rel_);
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteFiniteTurn) {
      limited = clamp_fp32(limited, abs_window_min_rel_, abs_window_max_rel_);
    }
    return limited;
  }

  fp32 normalize_target_for_mode(fp32 rad) const
  {
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      return clamp_target_limits(wrap_symmetric_period(rad, GIMBAL_DM_SINGLE_TURN_SPAN_RAD));
    }
    return clamp_target_limits(rad);
  }

  fp32 motor_target_pos_rad(fp32 rel_rad) const
  {
    fp32 motor_pos = rel_rad + zero_ofst_;
    if (pos_ref_mode_ != GimbalMotorPosRefMode::RelativeMultiTurn) {
      motor_pos = wrap_symmetric_period(motor_pos, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
    }
    return motor_pos;
  }

  fp32 angle_error_for_pid() const
  {
    if (direct_abs_target_enable_) {
      if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
        if (!direct_abs_single_turn_shortest_wrap_) {
          return direct_abs_target_rad_ - motor_abs_rad_;
        }
        return wrap_symmetric_period(direct_abs_target_rad_ - motor_abs_rad_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      }
      return direct_abs_target_rad_ - motor_abs_rad_;
    }
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      return wrap_symmetric_period(theta_set_ - theta_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
    }
    return theta_set_ - theta_;
  }

  fp32 position_feedback_for_pid() const
  {
    if (direct_abs_target_enable_) {
      if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
        if (!direct_abs_single_turn_shortest_wrap_) {
          return motor_abs_rad_;
        }
        return direct_abs_target_rad_ - angle_error_for_pid();
      }
      return motor_abs_rad_;
    }
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      return theta_set_ - angle_error_for_pid();
    }
    return theta_;
  }

  fp32 normalize_motor_abs_for_mode(fp32 rad) const
  {
    if (pos_ref_mode_ != GimbalMotorPosRefMode::RelativeMultiTurn) {
      return wrap_symmetric_period(rad, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
    }
    return rad;
  }

  fp32 normalize_direct_abs_target_for_mode(fp32 rad) const
  {
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn &&
        !direct_abs_single_turn_shortest_wrap_) {
      return clamp_fp32(rad, DM4310_POS_RAD_MIN, DM4310_POS_RAD_MAX);
    }
    return normalize_motor_abs_for_mode(rad);
  }

  DM4310 motor_;
  uint8_t bus_;
  bool  online_{false};
  bool  was_online_{false};
  fp32  zero_ofst_{0.f};
  fp32  theta_{0.f}, theta_set_{0.f};
  fp32  raw_{0.f};
  fp32  omega_{0.f};
  int16_t torque_raw_{0};
  uint8_t error_code_{0};
  fp32  min_rel_{-3.14f}, max_rel_{3.14f};
  GimbalMotorPosRefMode pos_ref_mode_{GimbalMotorPosRefMode::RelativeMultiTurn};
  fp32 abs_window_min_rel_{0.0f};
  fp32 abs_window_max_rel_{0.0f};
  bool abs_ref_valid_{false};
  algo::pid_type_def pid_ang_{}, pid_vel_{};
  int   warmup_ticks_{0};
  fp32  w_cmd_{0.0f};
  CtrlMode ctrl_mode_ = CtrlMode::Angle;
  fp32 w_rate_cmd_ = 0.0f;
  fp32 mit_kp_{0.0f};
  fp32 mit_kd_{0.0f};
  fp32 torque_ff_nm_{0.0f};
  bool use_mit_{GIMBAL_DM4310_USE_MIT};
  bool direct_abs_target_enable_{false};
  bool direct_abs_single_turn_shortest_wrap_{true};
  fp32 direct_abs_target_rad_{0.0f};
  fp32 motor_abs_rad_{0.0f};
  DM4310Mode last_tx_mode_{DM4310Mode::MIT};
  volatile GimbalPidDebug* dbg_{nullptr};
};

class GimbalAxisDm4340MitJ4 {
public:
  GimbalAxisDm4340MitJ4(uint8_t id, uint8_t bus, uint16_t fb_sid,
                        fp32 zero_ofst_rad, fp32 min_rel, fp32 max_rel,
                        GimbalMotorPosRefMode pos_ref_mode,
                        fp32 abs_window_min_rel, fp32 abs_window_max_rel,
                        const AxisPidCfg& cfg,
                        fp32 mit_pos_abs_max_rad,
                        fp32 mit_vel_abs_max_rad_s,
                        fp32 mit_torque_abs_max_nm)
      : motor_(id, fb_sid),
        bus_(bus),
        zero_ofst_(zero_ofst_rad),
        min_rel_(min_rel),
        max_rel_(max_rel),
        pos_ref_mode_(pos_ref_mode),
        abs_window_min_rel_(abs_window_min_rel),
        abs_window_max_rel_(abs_window_max_rel),
        mit_pos_abs_max_rad_(mit_pos_abs_max_rad),
        mit_vel_abs_max_rad_s_(mit_vel_abs_max_rad_s),
        mit_torque_abs_max_nm_(mit_torque_abs_max_nm)
  {
    motor_.set_offline_timeout(100);
    motor_.set_mode(DM4340Mode::MIT);
    motor_.set_mit_limits(mit_pos_abs_max_rad_, mit_vel_abs_max_rad_s_, mit_torque_abs_max_nm_);
    (void)canrx_register_dm4340(bus, &motor_);
    (void)cantx_register_dm4340(bus, &motor_, 3);
    osDelay(1);

    PID_init(&pid_ang_, algo::PID_POSITION, cfg.ang, cfg.ang_max_out, cfg.ang_max_iout);
    mit_kp_ = cfg.mit_kp;
    mit_kd_ = cfg.mit_kd;
  }

  enum class CtrlMode : uint8_t {
    Angle,
    Rate
  };

  void set_ctrl_mode(CtrlMode m) { ctrl_mode_ = m; }
  void set_debug(volatile GimbalPidDebug* dbg) { dbg_ = dbg; }
  void set_rate_cmd(fp32 w) { w_rate_cmd_ = w; }
  void set_torque_ff_nm(fp32 t_ff) { torque_ff_nm_ = t_ff; }
  void set_direct_abs_target_enable(bool enable) { direct_abs_target_enable_ = enable; }
  void set_direct_abs_target_single_turn_shortest_wrap(bool enable) {
    direct_abs_single_turn_shortest_wrap_ = enable;
  }
  void set_direct_abs_target_rad(fp32 abs_rad) { direct_abs_target_rad_ = normalize_direct_abs_target_for_mode(abs_rad); }
  bool direct_abs_target_enable() const { return direct_abs_target_enable_; }
  fp32 motor_abs_rad() const { return motor_abs_rad_; }
  fp32 direct_abs_target_rad() const { return direct_abs_target_rad_; }
  fp32 target_angle() const { return theta_set_; }
  void set_limit_window_flipped(bool flipped) { limit_window_flipped_ = flipped; }
  void set_limit_clamp_enabled(bool enable) { limit_clamp_enabled_ = enable; }
  fp32 clamp_target_to_active_window(fp32 rad) const { return normalize_target_for_mode(rad); }
  bool export_tx_frame(uint16_t* sid, uint8_t out[8]) const { return motor_.exportTxRaw8(sid, out); }

  fp32 theta() const { return theta_; }
  fp32 omega() const { return omega_; }
  fp32 raw() const { return raw_; }
  uint16_t raw_u16() const { return raw_u16_; }
  int16_t torque_raw() const { return torque_raw_; }
  uint8_t error_code() const { return error_code_; }

  void rezero(fp32 new_zero_ofst_rad) {
    zero_ofst_ += new_zero_ofst_rad;
    theta_ = 0.0f;
    theta_set_ = 0.0f;
    abs_ref_valid_ = false;
    algo::PID_clear(&pid_ang_);
    algo::PID_clear(&pid_vel_);
  }

  void rezero_here() {
    const auto s = motor_.state();
    zero_ofst_ = static_cast<fp32>(s.pos_rad_total);
    theta_ = 0.0f;
    theta_set_ = 0.0f;
    abs_ref_valid_ = false;
    algo::PID_clear(&pid_ang_);
    algo::PID_clear(&pid_vel_);
  }

  void shift_zero(fp32 delta_rad) {
    zero_ofst_ += delta_rad;
    theta_ -= delta_rad;
    theta_set_ -= delta_rad;
    abs_ref_valid_ = false;
    algo::PID_clear(&pid_ang_);
    algo::PID_clear(&pid_vel_);
  }

  void update_fb()
  {
    motor_.tick(HAL_GetTick());
    const auto s = motor_.state();
    online_ = s.online;
    error_code_ = s.error_code;
    raw_u16_ = dm_pos_rad_to_u16(static_cast<fp32>(s.pos_rad), mit_pos_abs_max_rad_);
    raw_ = static_cast<fp32>(raw_u16_);
    motor_abs_rad_ = static_cast<fp32>(s.pos_rad);
    if (!online_) {
      abs_ref_valid_ = false;
      abs_unwrap_valid_ = false;
    } else {
      const fp32 rel_single = wrap_symmetric_period(motor_abs_rad_ - zero_ofst_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      switch (pos_ref_mode_) {
        case GimbalMotorPosRefMode::AbsoluteSingleTurn:
          theta_ = rel_single;
          break;
        case GimbalMotorPosRefMode::AbsoluteFiniteTurn:
          if (!abs_ref_valid_) {
            theta_ = clamp_fp32(rel_single, abs_window_min_rel_, abs_window_max_rel_);
            abs_ref_valid_ = true;
          } else {
            theta_ = nearest_equivalent(rel_single, theta_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
            theta_ = clamp_fp32(theta_, abs_window_min_rel_, abs_window_max_rel_);
          }
          break;
        case GimbalMotorPosRefMode::RelativeMultiTurn:
        default:
          if (!abs_unwrap_valid_) {
            abs_unwrap_rad_ = motor_abs_rad_;
            abs_unwrap_valid_ = true;
          } else {
            abs_unwrap_rad_ = nearest_equivalent(motor_abs_rad_, abs_unwrap_rad_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
          }
          theta_ = abs_unwrap_rad_ - zero_ofst_;
          break;
      }
    }
    omega_ = static_cast<fp32>(s.speed_rad_s);
    torque_raw_ = s.torque_raw;
  }

  void set_target_angle(fp32 rad)
  {
    theta_set_ = normalize_target_for_mode(rad);
  }

  void recenter()
  {
    theta_set_ = 0.0f;
  }

  void add_set_by_inc(fp32 inc_rad)
  {
    theta_set_ = normalize_target_for_mode(theta_set_ + inc_rad);
  }

  void hold_here(){ theta_set_ = normalize_target_for_mode(theta_); }

  void send_enable_disable(bool enable) {
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, static_cast<uint8_t>(enable ? 0xFC : 0xFD)};
    const uint16_t sid = static_cast<uint16_t>(0x000u + motor_.canId());
    (void)cantx_send_now(bus_, sid, data, 8, 200);
  }

  void run(bool zero_force)
  {
    const bool came_online  = (online_ && !was_online_);
    const bool went_offline = (!online_ && was_online_);

    if (went_offline) {
      send_zero_cmd();
      algo::PID_clear(&pid_ang_);
      algo::PID_clear(&pid_vel_);
      w_cmd_ = 0.0f;
      abs_ref_valid_ = false;
      warmup_ticks_ = 0;
    }

    if (came_online) {
      algo::PID_clear(&pid_ang_);
      algo::PID_clear(&pid_vel_);
      w_cmd_ = 0.0f;
      warmup_ticks_ = 100;
      send_enable_disable(!zero_force);
    }

    if (zero_force || !online_) {
      send_zero_cmd();
      algo::PID_clear(&pid_ang_);
      if (dbg_) {
        dbg_->online = online_ ? 1u : 0u;
        dbg_->pos_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
        dbg_->pos_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
        dbg_->vel_target = w_cmd_;
        dbg_->vel_actual = omega_;
        dbg_->torque_target = torque_ff_nm_;
        dbg_->torque_actual = static_cast<fp32>(torque_raw_);
        if (ctrl_mode_ == CtrlMode::Angle) {
          dbg_->loop_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
          dbg_->loop_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
          dbg_->p_out = pid_ang_.Pout;
          dbg_->i_out = pid_ang_.Iout;
          dbg_->d_out = pid_ang_.Dout;
          dbg_->pid_out = pid_ang_.out;
        } else {
          dbg_->loop_target = w_rate_cmd_;
          dbg_->loop_actual = omega_;
          dbg_->p_out = 0.0f;
          dbg_->i_out = 0.0f;
          dbg_->d_out = 0.0f;
          dbg_->pid_out = 0.0f;
        }
        dbg_->mode = static_cast<uint8_t>(ctrl_mode_);
      }

      was_online_ = online_;
      return;
    }

    if (warmup_ticks_ > 0) {
      send_zero_cmd();
      warmup_ticks_--;
      if (dbg_) {
        dbg_->online = online_ ? 1u : 0u;
        dbg_->pos_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
        dbg_->pos_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
        dbg_->vel_target = w_cmd_;
        dbg_->vel_actual = omega_;
        dbg_->torque_target = torque_ff_nm_;
        dbg_->torque_actual = static_cast<fp32>(torque_raw_);
        if (ctrl_mode_ == CtrlMode::Angle) {
          dbg_->loop_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
          dbg_->loop_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
          dbg_->p_out = pid_ang_.Pout;
          dbg_->i_out = pid_ang_.Iout;
          dbg_->d_out = pid_ang_.Dout;
          dbg_->pid_out = pid_ang_.out;
        } else {
          dbg_->loop_target = w_rate_cmd_;
          dbg_->loop_actual = omega_;
          dbg_->p_out = 0.0f;
          dbg_->i_out = 0.0f;
          dbg_->d_out = 0.0f;
          dbg_->pid_out = 0.0f;
        }
        dbg_->mode = static_cast<uint8_t>(ctrl_mode_);
      }

      was_online_ = online_;
      return;
    }

    fp32 w_target = 0.0f;
    if (ctrl_mode_ == CtrlMode::Angle) {
      const fp32 err = angle_error_for_pid();
      constexpr fp32 ANG_DB = 0.01f;
      constexpr fp32 VEL_DB = 5.0f * (2.0f*3.14159265f/60.0f);
      if (fabsf(err) < ANG_DB && fabsf(omega_) < VEL_DB) {
        algo::PID_clear(&pid_ang_);
      }
      w_target = algo::PID_calc(&pid_ang_, position_feedback_for_pid(), theta_set_);
    } else {
      w_target = w_rate_cmd_;
    }

    constexpr fp32 W_MAX  = 10.0f;
    constexpr fp32 W_SLEW = 80.0f;
    const fp32 w_clip = std::max(-W_MAX, std::min(W_MAX, w_target));
    const fp32 dt = 0.002f;
    const fp32 dw_lim = W_SLEW * dt;
    w_cmd_ += std::max(-dw_lim, std::min(dw_lim, w_clip - w_cmd_));

    fp32 w_send = w_cmd_;
    constexpr fp32 W_ABS_MAX = 15.0f;
    if (w_send >  W_ABS_MAX) w_send =  W_ABS_MAX;
    if (w_send < -W_ABS_MAX) w_send = -W_ABS_MAX;

    fp32 p_des = 0.0f;
    if (mit_kp_ > 0.0f) {
      p_des = direct_abs_target_enable_ ? direct_abs_target_rad_ : motor_target_pos_rad(theta_set_);
    }
    motor_.set_mit_cmd(p_des, w_send, mit_kp_, mit_kd_, torque_ff_nm_);

    if (dbg_) {
      dbg_->online = online_ ? 1u : 0u;
      dbg_->pos_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
      dbg_->pos_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
      dbg_->vel_target = w_cmd_;
      dbg_->vel_actual = omega_;
      dbg_->torque_target = torque_ff_nm_;
      dbg_->torque_actual = static_cast<fp32>(torque_raw_);
      if (ctrl_mode_ == CtrlMode::Angle) {
        dbg_->loop_target = direct_abs_target_enable_ ? direct_abs_target_rad_ : theta_set_;
        dbg_->loop_actual = direct_abs_target_enable_ ? motor_abs_rad_ : theta_;
        dbg_->p_out = pid_ang_.Pout;
        dbg_->i_out = pid_ang_.Iout;
        dbg_->d_out = pid_ang_.Dout;
        dbg_->pid_out = pid_ang_.out;
      } else {
        dbg_->loop_target = w_rate_cmd_;
        dbg_->loop_actual = omega_;
        dbg_->p_out = 0.0f;
        dbg_->i_out = 0.0f;
        dbg_->d_out = 0.0f;
        dbg_->pid_out = 0.0f;
      }
      dbg_->mode = static_cast<uint8_t>(ctrl_mode_);
    }

    was_online_ = online_;
  }

  bool online() const { return online_; }
  void reset_pid() { algo::PID_clear(&pid_ang_); algo::PID_clear(&pid_vel_); }

private:
  void send_zero_cmd()
  {
    motor_.set_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  fp32 clamp_target_limits(fp32 rad) const
  {
    fp32 limited = rad;
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      if (!limit_clamp_enabled_) {
        limited = wrap_symmetric_period(rad, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      } else {
        const fp32 base_center = 0.5f * (min_rel_ + max_rel_);
        const fp32 center = base_center + (limit_window_flipped_ ? GIMBAL_PI_RAD : 0.0f);
        const fp32 half_span = std::fabs(0.5f * (max_rel_ - min_rel_));
        const fp32 unwrapped = nearest_equivalent(wrap_symmetric_period(rad, GIMBAL_DM_SINGLE_TURN_SPAN_RAD),
                                                  center,
                                                  GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
        const fp32 clamped = clamp_fp32(unwrapped, center - half_span, center + half_span);
        limited = wrap_symmetric_period(clamped, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      }
    } else {
      limited = clamp_fp32(rad, min_rel_, max_rel_);
    }
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteFiniteTurn) {
      limited = clamp_fp32(limited, abs_window_min_rel_, abs_window_max_rel_);
    }
    return limited;
  }

  fp32 normalize_target_for_mode(fp32 rad) const
  {
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      const fp32 wrapped = wrap_symmetric_period(rad, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      if (!limit_clamp_enabled_) {
        return wrapped;
      }
      return clamp_target_limits(wrapped);
    }
    return clamp_target_limits(rad);
  }

  fp32 motor_target_pos_rad(fp32 rel_rad) const
  {
    fp32 motor_pos = rel_rad + zero_ofst_;
    if (pos_ref_mode_ != GimbalMotorPosRefMode::RelativeMultiTurn) {
      motor_pos = wrap_symmetric_period(motor_pos, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
    }
    return motor_pos;
  }

  fp32 angle_error_for_pid() const
  {
    if (direct_abs_target_enable_) {
      if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
        if (!direct_abs_single_turn_shortest_wrap_) {
          return direct_abs_target_rad_ - motor_abs_rad_;
        }
        return wrap_symmetric_period(direct_abs_target_rad_ - motor_abs_rad_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      }
      return direct_abs_target_rad_ - motor_abs_rad_;
    }
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      return wrap_symmetric_period(theta_set_ - theta_, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
    }
    return theta_set_ - theta_;
  }

  fp32 position_feedback_for_pid() const
  {
    if (direct_abs_target_enable_) {
      if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
        if (!direct_abs_single_turn_shortest_wrap_) {
          return motor_abs_rad_;
        }
        return direct_abs_target_rad_ - angle_error_for_pid();
      }
      return motor_abs_rad_;
    }
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn) {
      return theta_set_ - angle_error_for_pid();
    }
    return theta_;
  }

  fp32 normalize_motor_abs_for_mode(fp32 rad) const
  {
    if (pos_ref_mode_ != GimbalMotorPosRefMode::RelativeMultiTurn) {
      return wrap_symmetric_period(rad, GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
    }
    return rad;
  }

  fp32 normalize_direct_abs_target_for_mode(fp32 rad) const
  {
    if (pos_ref_mode_ == GimbalMotorPosRefMode::AbsoluteSingleTurn &&
        !direct_abs_single_turn_shortest_wrap_) {
      return clamp_fp32(rad, -mit_pos_abs_max_rad_, mit_pos_abs_max_rad_);
    }
    return normalize_motor_abs_for_mode(rad);
  }

  DM4340 motor_;
  uint8_t bus_;
  bool online_{false};
  bool was_online_{false};
  fp32 zero_ofst_{0.f};
  fp32 theta_{0.f}, theta_set_{0.f};
  fp32 raw_{0.f};
  uint16_t raw_u16_{0u};
  fp32 omega_{0.f};
  int16_t torque_raw_{0};
  uint8_t error_code_{0};
  fp32 min_rel_{-3.14f}, max_rel_{3.14f};
  GimbalMotorPosRefMode pos_ref_mode_{GimbalMotorPosRefMode::RelativeMultiTurn};
  fp32 abs_window_min_rel_{0.0f};
  fp32 abs_window_max_rel_{0.0f};
  bool abs_ref_valid_{false};
  bool abs_unwrap_valid_{false};
  fp32 abs_unwrap_rad_{0.0f};
  algo::pid_type_def pid_ang_{}, pid_vel_{};
  int warmup_ticks_{0};
  fp32 w_cmd_{0.0f};
  CtrlMode ctrl_mode_ = CtrlMode::Angle;
  fp32 w_rate_cmd_ = 0.0f;
  fp32 mit_kp_{0.0f};
  fp32 mit_kd_{0.0f};
  fp32 torque_ff_nm_{0.0f};
  bool direct_abs_target_enable_{false};
  bool direct_abs_single_turn_shortest_wrap_{true};
  bool limit_window_flipped_{false};
  bool limit_clamp_enabled_{true};
  fp32 direct_abs_target_rad_{0.0f};
  fp32 motor_abs_rad_{0.0f};
  fp32 mit_pos_abs_max_rad_{3.2f};
  fp32 mit_vel_abs_max_rad_s_{45.0f};
  fp32 mit_torque_abs_max_nm_{18.0f};
  volatile GimbalPidDebug* dbg_{nullptr};
};


// ------------------- 涓讳换鍔?-------------------
namespace {
inline fp32 dm_gcomp_nm(fp32 theta_rad, fp32 amp_nm, fp32 phase_deg)
{
  if (amp_nm == 0.0f) {
    return 0.0f;
  }
  constexpr fp32 DEG2RAD = 0.017453292519943295f;
  return amp_nm * std::sin(theta_rad + phase_deg * DEG2RAD);
}

inline fp32 dm_gcomp_clamp_nm(fp32 t_nm)
{
  if (GIMBAL_DM_GCOMP_MAX_NM <= 0.0f) {
    return t_nm;
  }
  if (t_nm >  GIMBAL_DM_GCOMP_MAX_NM) return GIMBAL_DM_GCOMP_MAX_NM;
  if (t_nm < -GIMBAL_DM_GCOMP_MAX_NM) return -GIMBAL_DM_GCOMP_MAX_NM;
  return t_nm;
}
} // namespace

extern "C" void Start_Gimbal_Task(void *argument) {
  (void)argument;

  // 鈥斺€?涓烘瘡杞存瀯閫犵嫭绔?PID 閰嶇疆锛堜娇鐢ㄤ綘鍦?CONF 涓尯鍒?yaw/pitch 鐨勫父閲忥級鈥斺€?
  const AxisPidCfg cfg_dm_j5 = {
    /* ang */ { GIMBAL_DM_J5_ANG_KP, GIMBAL_DM_J5_ANG_KI, GIMBAL_DM_J5_ANG_KD },
    /* ang lim */ GIMBAL_DM_J5_ANG_MAX_OUT, GIMBAL_DM_J5_ANG_MAX_IOUT,
    /* mit */ GIMBAL_DM_MIT_KP, GIMBAL_DM_MIT_KD,
  };
  osDelay(1);
  const AxisPidCfg cfg_dm_j6 = {
    /* ang */ { GIMBAL_DM_J6_ANG_KP, GIMBAL_DM_J6_ANG_KI, GIMBAL_DM_J6_ANG_KD },
    /* ang lim */ GIMBAL_DM_J6_ANG_MAX_OUT, GIMBAL_DM_J6_ANG_MAX_IOUT,
    /* mit */ GIMBAL_DM_MIT_KP, GIMBAL_DM_MIT_KD,
  };
  osDelay(1);
  const AxisPidCfg cfg_dm_j7 = {
    /* ang */ { GIMBAL_DM_J7_ANG_KP, GIMBAL_DM_J7_ANG_KI, GIMBAL_DM_J7_ANG_KD },
    /* ang lim */ GIMBAL_DM_J7_ANG_MAX_OUT, GIMBAL_DM_J7_ANG_MAX_IOUT,
    /* mit */ GIMBAL_J7_DIRECT_MIT_KP, GIMBAL_J7_DIRECT_MIT_KD,
  };
  osDelay(1);
  const AxisPidCfg cfg_dm_j4 = {
    /* ang */ { GIMBAL_DM_J4_ANG_KP, GIMBAL_DM_J4_ANG_KI, GIMBAL_DM_J4_ANG_KD },
    /* ang lim */ GIMBAL_DM_J4_ANG_MAX_OUT, GIMBAL_DM_J4_ANG_MAX_IOUT,
    /* mit */ GIMBAL_DM_J4_MIT_KP, GIMBAL_DM_J4_MIT_KD,
  };

  static GimbalAxis dm_j5(GIMBAL_DM_J5_ID, GIMBAL_DM_J5_BUS, GIMBAL_DM4310_FB_SID,
                                GIMBAL_DM_J5_ZERO_OFFSET_RAD, GIMBAL_DM_J5_MIN_REL_RAD, GIMBAL_DM_J5_MAX_REL_RAD,
                                GIMBAL_DM_J5_POS_REF_MODE, GIMBAL_DM_J5_ABS_MIN_REL_RAD, GIMBAL_DM_J5_ABS_MAX_REL_RAD,
                                cfg_dm_j5);
  static GimbalAxis dm_j6(GIMBAL_DM_J6_ID, GIMBAL_DM_J6_BUS, GIMBAL_DM4310_FB_SID,
                               GIMBAL_DM_J6_ZERO_OFFSET_RAD, GIMBAL_DM_J6_MIN_REL_RAD, GIMBAL_DM_J6_MAX_REL_RAD,
                               GIMBAL_DM_J6_POS_REF_MODE, 0.0f, 0.0f,
                               cfg_dm_j6);
  static GimbalAxis dm_j7(GIMBAL_DM_J7_ID, GIMBAL_DM_J7_BUS, GIMBAL_DM4310_FB_SID,
                        GIMBAL_DM_J7_ZERO_OFFSET_RAD, GIMBAL_DM_J7_MIN_REL_RAD, GIMBAL_DM_J7_MAX_REL_RAD,
                        GIMBAL_DM_J7_POS_REF_MODE, 0.0f, 0.0f,
                        cfg_dm_j7);
  static GimbalAxisDm4340MitJ4 dm_j4(GIMBAL_DM_J4_ID, GIMBAL_DM_J4_BUS, GIMBAL_DM_J4_FB_SID,
                                     GIMBAL_DM_J4_ZERO_OFFSET_RAD, GIMBAL_DM_J4_MIN_REL_RAD, GIMBAL_DM_J4_MAX_REL_RAD,
                                     GIMBAL_DM_J4_POS_REF_MODE, 0.0f, 0.0f,
                                     cfg_dm_j4,
                                     GIMBAL_DM_J4_PMAX_RAD,
                                     GIMBAL_DM_J4_VMAX_RAD_S,
                                     GIMBAL_DM_J4_TMAX_NM);

  const TickType_t period = pdMS_TO_TICKS(2);

  dm_j5.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
  dm_j6.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
  dm_j6.set_direct_abs_target_single_turn_shortest_wrap(GIMBAL_J6_DIRECT_SINGLE_TURN_SHORTEST_WRAP_ENABLE);
  dm_j4.set_direct_abs_target_single_turn_shortest_wrap(GIMBAL_J4_DIRECT_SINGLE_TURN_SHORTEST_WRAP_ENABLE);
  dm_j7.set_direct_abs_target_single_turn_shortest_wrap(GIMBAL_J7_DIRECT_SINGLE_TURN_SHORTEST_WRAP_ENABLE);
  dm_j7.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
  dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
  dm_j5.set_debug(&gimbal_dbg_dm_j5);
  dm_j6.set_debug(&gimbal_dbg_dm_j6);
  dm_j7.set_debug(&gimbal_dbg_dm_j7);
  dm_j4.set_debug(&gimbal_dbg_dm_j4);

  bool dm_j5_poweron_home_pending = GIMBAL_DM_J5_POWERON_HOME_ENABLE;
  bool dm_j6_poweron_home_pending = GIMBAL_DM_J6_POWERON_HOME_ENABLE;
  bool dm_j7_poweron_home_pending = GIMBAL_DM_J7_POWERON_HOME_ENABLE;
  bool dm_j4_poweron_home_pending = GIMBAL_DM_J4_POWERON_HOME_ENABLE;
  const auto pause_target_rad = [](uint8_t idx) -> fp32 {
    return static_cast<fp32>(Omnix_CdegToRad(GIMBAL_PAUSE_TARGET_CDEG[idx]));
  };
  const auto prime_dm_poweron_target = [&](auto& axis, bool enable_poweron_home, uint8_t pause_idx) {
    if (enable_poweron_home) {
      axis.set_target_angle(pause_target_rad(pause_idx));
    } else {
      axis.hold_here();
    }
  };
  const auto apply_dm_enable_target =
      [&](auto& axis, bool* poweron_home_pending, bool ext_valid, uint8_t pause_idx) {
    if (ext_valid) {
      return;
    }
    if (*poweron_home_pending) {
      axis.set_target_angle(pause_target_rad(pause_idx));
      *poweron_home_pending = false;
    } else {
      axis.hold_here();
    }
  };

  TickType_t last = xTaskGetTickCount();

  // 鍒濆鍙嶉鏇存柊
  dm_j5.update_fb();
  dm_j6.update_fb();
  dm_j7.update_fb();
  dm_j4.update_fb();

  // No zeroing on power-up; just lock current position.

  dm_j5.update_fb();
  dm_j6.update_fb();
  dm_j7.update_fb();
  dm_j4.update_fb();

  prime_dm_poweron_target(dm_j5, dm_j5_poweron_home_pending, 4u);
  prime_dm_poweron_target(dm_j6, dm_j6_poweron_home_pending, 5u);
  prime_dm_poweron_target(dm_j7, dm_j7_poweron_home_pending, 6u);
  prime_dm_poweron_target(dm_j4, dm_j4_poweron_home_pending, 3u);

  // 鍒濆绂佽兘
  dm_j5.send_enable_disable(false);
  dm_j6.send_enable_disable(false);
  dm_j7.send_enable_disable(false);
  dm_j4.send_enable_disable(false);

  RC_State rc_init; RC_GetSnapshot(&rc_init);
  RC_Status st_init = RC_GetStatus();
  const RcControlMode ctl_init = VT03_Gimbal_ResolveControlMode(rc_init, st_init);
  int16_t roll_ch1_trim = static_cast<int16_t>(
      rc_init.rc.ch[GIMBAL_DM_J4_RC_CH] * static_cast<int16_t>(GIMBAL_DM_J4_RC_SIGN));
  const auto rc_switch_match = [](uint8_t v, GimbalRcSwitchPos pos) -> bool {
    switch (pos) {
      case GimbalRcSwitchPos::Up:   return switch_is_up(v);
      case GimbalRcSwitchPos::Mid:  return switch_is_mid(v);
      case GimbalRcSwitchPos::Down: return switch_is_down(v);
      default: return false;
    }
  };
  bool prev_s0_down = (ctl_init.main_mode == RcMainMode::ZeroForce);
  if (!prev_s0_down) {
    dm_j5.send_enable_disable(true);
    dm_j6.send_enable_disable(true);
    dm_j7.send_enable_disable(true);
    dm_j4.send_enable_disable(true);
    dm_j5_poweron_home_pending = false;
    dm_j6_poweron_home_pending = false;
    dm_j7_poweron_home_pending = false;
    dm_j4_poweron_home_pending = false;
  }

  for(;;){
    vTaskDelayUntil(&last, period);

    dm_j5.update_fb();
    dm_j6.update_fb();
    dm_j7.update_fb();
    dm_j4.update_fb();

    {
      DmJ4J7Snapshot local{};
      local.raw_u16[0] = dm_j4.raw_u16();
      local.raw_u16[1] = dm_j5.raw_u16();
      local.raw_u16[2] = dm_j6.raw_u16();
      local.raw_u16[3] = dm_j7.raw_u16();

      local.angle_cdeg[0] = dm_cdeg_from_rad(dm_j4.theta());
      local.angle_cdeg[1] = dm_cdeg_from_rad(dm_j5.theta());
      local.angle_cdeg[2] = dm_cdeg_from_rad(dm_j6.theta());
      local.angle_cdeg[3] = dm_cdeg_from_rad(dm_j7.theta());

      local.online[0] = dm_j4.online() ? 1u : 0u;
      local.online[1] = dm_j5.online() ? 1u : 0u;
      local.online[2] = dm_j6.online() ? 1u : 0u;
      local.online[3] = dm_j7.online() ? 1u : 0u;

      __DMB();
      g_dm_j4j7_guard++;
      __DMB();
      g_dm_j4j7_snap = local;
      __DMB();
      g_dm_j4j7_guard++;
      __DMB();
    }

    static bool dm_j5_cal_logged = false;
    static bool dm_j6_cal_logged = false;
    static bool dm_j7_cal_logged = false;
    static bool dm_j4_cal_logged = false;
    const auto log_dm_cal = [](const char* name, const auto& axis, bool* logged) {
      if (*logged || !axis.online()) {
        return;
      }
      const uint16_t raw = axis.raw_u16();
      LOGI("[CAL][DM] %s raw=%u rad=%.5f",
           name,
           static_cast<unsigned>(raw),
           static_cast<double>(axis.motor_abs_rad()));
      *logged = true;
    };
    log_dm_cal("J5", dm_j5, &dm_j5_cal_logged);
    log_dm_cal("J6", dm_j6, &dm_j6_cal_logged);
    log_dm_cal("J7", dm_j7, &dm_j7_cal_logged);
    log_dm_cal("J4", dm_j4, &dm_j4_cal_logged);

    if (GIMBAL_DM_GCOMP_ENABLE) {
      const fp32 scale = GIMBAL_DM_GCOMP_SCALE;
      dm_j5.set_torque_ff_nm(dm_gcomp_clamp_nm(scale * dm_gcomp_nm(dm_j5.theta(), GIMBAL_DM_J5_GCOMP_NM, GIMBAL_DM_J5_GCOMP_PHASE_DEG)));
      dm_j6.set_torque_ff_nm(dm_gcomp_clamp_nm(scale * dm_gcomp_nm(dm_j6.theta(), GIMBAL_DM_J6_GCOMP_NM, GIMBAL_DM_J6_GCOMP_PHASE_DEG)));
      dm_j7.set_torque_ff_nm(dm_gcomp_clamp_nm(scale * dm_gcomp_nm(dm_j7.theta(), GIMBAL_DM_J7_GCOMP_NM, GIMBAL_DM_J7_GCOMP_PHASE_DEG)));
      dm_j4.set_torque_ff_nm(dm_gcomp_clamp_nm(scale * dm_gcomp_nm(dm_j4.theta(), GIMBAL_DM_J4_GCOMP_NM, GIMBAL_DM_J4_GCOMP_PHASE_DEG)));
    } else {
      dm_j5.set_torque_ff_nm(0.0f);
      dm_j6.set_torque_ff_nm(0.0f);
      dm_j7.set_torque_ff_nm(0.0f);
      dm_j4.set_torque_ff_nm(0.0f);
    }

    gimbal_feedback.yaw_relative = dm_j5.theta();
    gimbal_feedback.yaw_omega    = dm_j5.omega();
    gimbal_feedback.jaw_relative = dm_j7.theta();
    gimbal_feedback.jaw_omega    = dm_j7.omega();
    // Jaw torque force-feedback is disabled: always report 0
    gimbal_feedback.jaw_torque_raw = 0;
    gimbal_feedback.jaw_online     = dm_j7.online() ? 1u : 0u;

    static uint32_t dbg_cnt = 0;
    if (++dbg_cnt >= 1) {
      dbg_cnt = 0;

      const uint8_t roll_bus = static_cast<uint8_t>(GIMBAL_DM_J4_BUS);
      const bsp_fdcan_diag_t d = bsp_fdcan_get_diag(roll_bus);

      gimbal_cmd_t cmd;
      (void)read_gimbal_cmd(&cmd);
      int16_t pause_cmd_cdeg[9] = {0};
      uint8_t pause_phase = GIMBAL_PAUSE_IDLE;
      const bool pause_ok = Gimbal_ReadPauseCmdCdeg(pause_cmd_cdeg, &pause_phase);
      const bool pause_active = pause_ok &&
                                (pause_phase == GIMBAL_PAUSE_RUN || pause_phase == GIMBAL_PAUSE_DONE_BEEP);
      const bool pause_abort = pause_ok && (pause_phase == GIMBAL_PAUSE_ABORT);
      RC_State rc_now; RC_GetSnapshot(&rc_now);
      RC_Status st_now = RC_GetStatus();
      const RcControlMode ctl = VT03_Gimbal_ResolveControlMode(rc_now, st_now);
      const bool gear_chassis = (ctl.main_mode == RcMainMode::Chassis);
      const bool s0_down = (ctl.main_mode == RcMainMode::ZeroForce);
      const bool s0_mid  = (ctl.main_mode == RcMainMode::Chassis);
      const bool s0_up   = (ctl.main_mode == RcMainMode::Gimbal);
      const bool s1_up   = (ctl.gimbal_mode == RcGimbalMode::DM);
      const bool s1_mid  = (ctl.gimbal_mode == RcGimbalMode::LK);
      const bool s1_down = (ctl.gimbal_mode == RcGimbalMode::Link);
      const bool remote_safe_force = s0_down;
      const bool gear_exit_hold = (!remote_safe_force && prev_s0_down);
      const bool global_zero_force = gimbal_zero_force;
      const bool zero_force = global_zero_force || pause_abort || remote_safe_force;
      const bool j5_enable = !zero_force &&
                             (pause_active || (!s0_mid && s0_up && (s1_up || s1_mid || s1_down)));
      const bool j6_enable = !zero_force &&
                             (pause_active || (!s0_mid && s0_up && (s1_up || s1_mid || s1_down)));
      const bool zero_log_enable =
          (ctl.active_src == RC_SRC_DR16) &&
          rc_switch_match(rc_now.rc.s[GIMBAL_RC_S0_INDEX], GIMBAL_RC_ZERO_LOG_S0) &&
          rc_switch_match(rc_now.rc.s[GIMBAL_RC_S1_INDEX], GIMBAL_RC_ZERO_LOG_S1);
      static uint32_t zero_log_last_ms = 0;
#if GIMBAL_LOG_ZERO_DM
      if (zero_log_enable) {
        const uint32_t now_ms = HAL_GetTick();
        if (now_ms - zero_log_last_ms >= GIMBAL_ZERO_LOG_PERIOD_MS) {
          zero_log_last_ms = now_ms;
          LOGI("Zero log DM: J5=%.3f J6=%.3f J7=%.3f J4=%.3f (rad)",
               static_cast<double>(dm_j5.theta()),
               static_cast<double>(dm_j6.theta()),
               static_cast<double>(dm_j7.theta()),
               static_cast<double>(dm_j4.theta()));
        }
      }
#endif
      referee_joint_raw_cmd_t ext_cmd{};
      (void)Referee_ReadJointRawCmd(&ext_cmd);
      const bool ext_cmd_fresh = ext_cmd.online && ext_cmd.fresh;
      const bool custom_dm_session_active = ext_cmd_fresh;
      const bool custom_gcomp_off = custom_dm_session_active;
      const bool ext_j4_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 4u);
      const bool ext_j5_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 5u);
      const bool ext_j6_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 6u);
      const bool ext_j7_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 7u);
      const bool dm_external_safe_block = pause_active || zero_force;
      static bool dm_prev_external_safe_block = false;
      static bool dm_wait_external_fresh_after_safe_exit = false;
      static uint16_t dm_external_rearm_seq = 0u;
      if (!dm_external_safe_block && dm_prev_external_safe_block) {
        dm_wait_external_fresh_after_safe_exit = true;
        dm_external_rearm_seq = ext_cmd.seq;
      }
      if (dm_wait_external_fresh_after_safe_exit &&
          ext_cmd_fresh &&
          (ext_cmd.seq != dm_external_rearm_seq)) {
        dm_wait_external_fresh_after_safe_exit = false;
      }
      dm_prev_external_safe_block = dm_external_safe_block;
      const bool ext_j4_apply_valid = ext_j4_valid && !dm_external_safe_block &&
                                      !dm_wait_external_fresh_after_safe_exit;
      const bool ext_j5_apply_valid = ext_j5_valid && !dm_external_safe_block &&
                                      !dm_wait_external_fresh_after_safe_exit;
      const bool ext_j6_apply_valid = ext_j6_valid && !dm_external_safe_block &&
                                      !dm_wait_external_fresh_after_safe_exit;
      const bool ext_j7_apply_valid = ext_j7_valid && !dm_external_safe_block &&
                                      !dm_wait_external_fresh_after_safe_exit;
      static uint32_t custom_dm_log_last_ms = 0u;
      const uint16_t ext_j4_protocol_raw = ext_cmd.raw_u16[3];
      const uint16_t ext_j5_protocol_raw = ext_cmd.raw_u16[4];
      const uint16_t ext_j6_protocol_raw = ext_cmd.raw_u16[5];
      const uint16_t ext_j7_protocol_raw = ext_cmd.raw_u16[6];
      const GimbalExternalBidirRelAngleMapResult ext_j4_map =
          Gimbal_MapExternalRawToBidirRelAngle(ext_j4_protocol_raw, GIMBAL_J4_EXT_BIDIR_REL_ANGLE_MAP_CFG);
      const GimbalExternalBidirRelAngleMapResult ext_j5_map =
          Gimbal_MapExternalRawToBidirRelAngle(ext_j5_protocol_raw, GIMBAL_J5_EXT_BIDIR_REL_ANGLE_MAP_CFG);
      const GimbalExternalBidirRelAngleMapResult ext_j6_map =
          Gimbal_MapExternalRawToBidirRelAngle(ext_j6_protocol_raw, GIMBAL_J6_EXT_BIDIR_REL_ANGLE_MAP_CFG);
      const GimbalExternalBidirRelAngleMapResult ext_j7_map =
          Gimbal_MapExternalRawToHalfRangeReverseRelAngle(
              ext_j7_protocol_raw,
              GIMBAL_J7_EXT_HALF_RANGE_REVERSE_MAP_CFG);
      const uint16_t cur_j4_dm_raw = dm_j4.raw_u16();
      const uint16_t cur_j5_dm_raw = dm_j5.raw_u16();
      const uint16_t cur_j6_dm_raw = dm_j6.raw_u16();
      const uint16_t cur_j7_dm_raw = dm_j7.raw_u16();
      const uint8_t cur_j7_err = dm_j7.error_code();
      const bool cur_j7_overcurrent = (cur_j7_err == 0x0Au);
      const fp32 pre_j7_target_rel_rad = dm_j7.target_angle();
      const uint32_t custom_diag_now_ms = HAL_GetTick();
      static bool prev_ext_j4_apply_valid = false;
      static bool prev_ext_j5_apply_valid = false;
      static bool prev_ext_j6_apply_valid = false;
      static bool prev_ext_j7_apply_valid = false;
      static bool j4_limit_window_flipped = false;
      static bool j4_trigger_prev_pressed = false;
      static bool j4_trigger_beep_active = false;
      static uint32_t j4_trigger_beep_end_ms = 0u;
      static fp32 ext_j4_hold_target_rel_rad = 0.0f;
      static fp32 ext_j5_hold_target_rel_rad = 0.0f;
      static fp32 ext_j6_hold_target_rel_rad = 0.0f;
      static fp32 ext_j7_hold_target_rel_rad = 0.0f;
      bool j7_evt_cache_enter = false;
      bool j7_evt_cache_update = false;
      bool j7_src_session_hold = false;
      bool j7_src_s0mid_hold = false;
      bool j7_src_invalid_hold = false;
      bool j7_src_vt03_wheel = false;
      bool j7_src_inc = false;
      bool j7_src_ext_apply = false;
      bool j7_src_pause = false;
      bool j7_src_enable_home = false;
      bool j7_src_enable_hold = false;
      const bool j7_src_fault_freeze = false;
      if (ext_j4_apply_valid && !prev_ext_j4_apply_valid) { ext_j4_hold_target_rel_rad = dm_j4.target_angle(); }
      if (ext_j5_apply_valid && !prev_ext_j5_apply_valid) { ext_j5_hold_target_rel_rad = dm_j5.target_angle(); }
      if (ext_j6_apply_valid && !prev_ext_j6_apply_valid) { ext_j6_hold_target_rel_rad = dm_j6.target_angle(); }
      if (ext_j7_apply_valid && !prev_ext_j7_apply_valid) {
        ext_j7_hold_target_rel_rad = dm_j7.target_angle();
        j7_evt_cache_enter = true;
      }
      if (!custom_dm_session_active) {
        j4_limit_window_flipped = false;
      }
      const bool j4_limit_clamp_enabled =
          !(custom_dm_session_active && !pause_active && !zero_force);
      dm_j4.set_limit_clamp_enabled(j4_limit_clamp_enabled);
      dm_j4.set_limit_window_flipped(j4_limit_window_flipped);
      const fp32 ext_j4_target_rel_rad_from_map =
          -ext_j4_map.target_rel_rad + (j4_limit_window_flipped ? GIMBAL_PI_RAD : 0.0f);
      fp32 ext_j4_target_rel_rad_guarded = ext_j4_target_rel_rad_from_map;
      if (j4_limit_window_flipped && !ext_j4_map.hold_last_target) {
        ext_j4_target_rel_rad_guarded =
            nearest_equivalent(ext_j4_target_rel_rad_guarded,
                               ext_j4_hold_target_rel_rad,
                               GIMBAL_DM_SINGLE_TURN_SPAN_RAD);
      }
      const bool j5_ext_inverted = j4_limit_window_flipped && GIMBAL_J5_EXT_INVERT_WHEN_J4_FLIPPED;
      const bool j6_ext_inverted = j4_limit_window_flipped && GIMBAL_J6_EXT_INVERT_WHEN_J4_FLIPPED;
      const fp32 ext_j5_target_rel_rad_from_map =
          j5_ext_inverted ? -ext_j5_map.target_rel_rad : ext_j5_map.target_rel_rad;
      const fp32 ext_j6_target_rel_rad_from_map =
          j6_ext_inverted ? -ext_j6_map.target_rel_rad : ext_j6_map.target_rel_rad;
      if (ext_j4_apply_valid && !ext_j4_map.hold_last_target) {
        ext_j4_hold_target_rel_rad = dm_j4.clamp_target_to_active_window(ext_j4_target_rel_rad_guarded);
      }
      if (ext_j5_apply_valid && !ext_j5_map.hold_last_target) { ext_j5_hold_target_rel_rad = ext_j5_target_rel_rad_from_map; }
      if (ext_j6_apply_valid && !ext_j6_map.hold_last_target) { ext_j6_hold_target_rel_rad = ext_j6_target_rel_rad_from_map; }
      if (ext_j7_apply_valid && !ext_j7_map.hold_last_target) {
        ext_j7_hold_target_rel_rad = ext_j7_map.target_rel_rad;
        j7_evt_cache_update = true;
      }
      const bool j4_trigger_pressed =
          (ctl.active_src == RC_SRC_VT03) &&
          ((rc_now.key.v_ext & KEY_EXT_VT03_TRIGGER) != 0u);
      const bool j4_trigger_rise = j4_trigger_pressed && !j4_trigger_prev_pressed;
      j4_trigger_prev_pressed = j4_trigger_pressed;
      const bool j4_trigger_allowed = j4_trigger_rise && ext_j4_apply_valid;
      if (j4_trigger_allowed) {
        const fp32 pre_flip_target_rel_rad = ext_j4_hold_target_rel_rad;
        j4_limit_window_flipped = !j4_limit_window_flipped;
        dm_j4.set_limit_window_flipped(j4_limit_window_flipped);
        const fp32 post_flip_target_rel_rad = pre_flip_target_rel_rad + GIMBAL_PI_RAD;
        ext_j4_hold_target_rel_rad = dm_j4.clamp_target_to_active_window(post_flip_target_rel_rad);
        Buzzer_Start(GIMBAL_J4_TRIGGER_BEEP_HZ);
        j4_trigger_beep_active = true;
        j4_trigger_beep_end_ms = custom_diag_now_ms + GIMBAL_J4_TRIGGER_BEEP_MS;
        Referee_RequestRemoteBuzzerPulses(1u);
        LOGI("[J4][TRIGGER] rise=1 flip=%u j5_inv=%u j6_inv=%u tgt=%.3f",
             j4_limit_window_flipped ? 1u : 0u,
             (j4_limit_window_flipped && GIMBAL_J5_EXT_INVERT_WHEN_J4_FLIPPED) ? 1u : 0u,
             (j4_limit_window_flipped && GIMBAL_J6_EXT_INVERT_WHEN_J4_FLIPPED) ? 1u : 0u,
             static_cast<double>(ext_j4_hold_target_rel_rad));
      }
      if (j4_trigger_beep_active &&
          ((int32_t)(custom_diag_now_ms - j4_trigger_beep_end_ms) >= 0)) {
        Buzzer_Stop();
        j4_trigger_beep_active = false;
      }
      const fp32 ext_j4_target_rel_rad = ext_j4_hold_target_rel_rad;
      const fp32 ext_j5_target_rel_rad = ext_j5_hold_target_rel_rad;
      const fp32 ext_j6_target_rel_rad = ext_j6_hold_target_rel_rad;
      const fp32 ext_j7_target_rel_rad = ext_j7_hold_target_rel_rad;
      dm_j4.set_direct_abs_target_enable(false);
      dm_j5.set_direct_abs_target_enable(false);
      dm_j6.set_direct_abs_target_enable(false);
      dm_j7.set_direct_abs_target_enable(false);
      dm_j7.set_use_mit(GIMBAL_DM4310_USE_MIT);
      if (custom_gcomp_off) {
        dm_j5.set_torque_ff_nm(0.0f);
        dm_j6.set_torque_ff_nm(0.0f);
        dm_j4.set_torque_ff_nm(0.0f);
        dm_j7.set_torque_ff_nm(0.0f);
      }
      const bool custom_dm_log_due = (custom_diag_now_ms - custom_dm_log_last_ms >= 1000u);

      // J5 soft-limit logging mode


      static bool prev_s0_mid = false;
      static bool prev_j5_enable = false;
      static bool prev_j6_enable = false;
      static bool roll_rate_active = false;
      static bool prev_roll_rc_enable = false;
      const auto apply_jaw_inc = [&](fp32 inc) {
        constexpr fp32 RAD_PER_DEG = 3.14159265358979323846f / 180.0f;
        constexpr fp32 DT = 0.002f;
        const fp32 step_per_tick = GIMBAL_DM_J7_VT03_WHEEL_MAX_RATE_DPS * RAD_PER_DEG * DT;
        const float TH = 1e-3f;
        if (fabsf(inc) > TH) {
          dm_j7.add_set_by_inc((inc > 0.0f) ? step_per_tick : -step_per_tick);
        }
      };
      const auto apply_jaw_vt03_wheel = [&](int16_t ch_raw) {
        constexpr fp32 CH_MAX = 660.0f;
        constexpr fp32 RAD_PER_DEG = 3.14159265358979323846f / 180.0f;
        constexpr fp32 DT = 0.002f;
        int ch = static_cast<int>(ch_raw) * static_cast<int>(GIMBAL_DM_J7_RC_SIGN);
        const int dead = static_cast<int>(GIMBAL_DM_J7_DEADBAND);
        if (ch > -dead && ch < dead) {
          return;
        }
        if (ch > 660) ch = 660;
        if (ch < -660) ch = -660;
        const fp32 delta_rad =
            (static_cast<fp32>(ch) / CH_MAX) * GIMBAL_DM_J7_VT03_WHEEL_MAX_RATE_DPS * RAD_PER_DEG * DT;
        if (fabsf(delta_rad) > 0.0f) {
          dm_j7.add_set_by_inc(delta_rad);
        }
      };

      if (!pause_active) {
      if (custom_dm_session_active) {
        dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
        dm_j5.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j6.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j7.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j4.set_rate_cmd(0.0f);
        dm_j5.set_rate_cmd(0.0f);
        dm_j6.set_rate_cmd(0.0f);
        dm_j7.set_rate_cmd(0.0f);
        if (!ext_j4_apply_valid) { dm_j4.hold_here(); }
        if (!ext_j5_apply_valid) { dm_j5.hold_here(); }
        if (!ext_j6_apply_valid) { dm_j6.hold_here(); }
        if (!ext_j7_apply_valid) {
          dm_j7.hold_here();
          j7_src_session_hold = true;
        }
        prev_s0_mid = false;
        prev_j5_enable = false;
        prev_j6_enable = false;
        roll_rate_active = false;
        prev_roll_rc_enable = false;
      } else {
      if (ext_j7_valid) {
        dm_j7.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j7.set_rate_cmd(0.0f);
      }
      if (s0_mid) {
        if (!prev_s0_mid) {
          if (!ext_j5_apply_valid) { dm_j5.hold_here(); }
          if (!ext_j6_apply_valid) { dm_j6.hold_here(); }
          if (!ext_j7_apply_valid) {
            dm_j7.hold_here();
            j7_src_s0mid_hold = true;
          }
          if (!ext_j4_apply_valid) { dm_j4.hold_here(); }
          roll_ch1_trim = static_cast<int16_t>(
              rc_now.rc.ch[GIMBAL_DM_J4_RC_CH] * static_cast<int16_t>(GIMBAL_DM_J4_RC_SIGN));
        }
        prev_s0_mid = true;
      } else {
        prev_s0_mid = false;
        if (!j5_enable) {
          (void)s1_mid;
          (void)s1_down;
          if (prev_j5_enable && !ext_j5_apply_valid) {
            dm_j5.hold_here();
          }
        }
        if (!j6_enable) {
          if (prev_j6_enable && !ext_j6_apply_valid) {
            dm_j6.hold_here();
          }
        }
        if ((j5_enable && !ext_j5_apply_valid) || (j6_enable && !ext_j6_apply_valid)) {
          if (cmd.behaviour == GIMBAL_RELATIVE_ANGLE) {
            constexpr fp32 RATE_DEG_PER_S = 30.0f;
            constexpr fp32 RAD_PER_DEG = 3.14159265358979323846f / 180.0f;
            constexpr fp32 DT = 0.002f;
            constexpr fp32 STEP_PER_TICK = RATE_DEG_PER_S * RAD_PER_DEG * DT;
            const bool vt03_dm_mouse = ctl.vt03_active && (ctl.main_mode == RcMainMode::Gimbal) &&
                                       (ctl.gimbal_mode == RcGimbalMode::DM);

            const fp32 cur_pitch_inc = cmd.yaw_set_rad; // cmd 涓搴旂殑鍙兘鏄棫鍛藉悕
            const fp32 cur_roll_inc = cmd.add_pitch;

            const float TH = 1e-3f;
            if (j5_enable && !ext_j5_apply_valid) {
              if (vt03_dm_mouse && (rc_now.mouse.y != 0)) {
                dm_j5.add_set_by_inc(-static_cast<fp32>(rc_now.mouse.y) * GIMBAL_DM_J5_MOUSE_SEN);
              } else if (fabsf(cur_pitch_inc) > TH) {
                dm_j5.add_set_by_inc((cur_pitch_inc > 0.0f) ? STEP_PER_TICK : -STEP_PER_TICK);
              }
            }
            if (j6_enable && !ext_j6_apply_valid) {
              if (vt03_dm_mouse && (rc_now.mouse.x != 0)) {
                dm_j6.add_set_by_inc(-static_cast<fp32>(rc_now.mouse.x) * GIMBAL_DM_J6_MOUSE_SEN);
              } else if (fabsf(cur_roll_inc) > TH) {
                dm_j6.add_set_by_inc((cur_roll_inc > 0.0f) ? STEP_PER_TICK : -STEP_PER_TICK);
              }
            }
          }
        }
      }
      if (ext_j5_valid && !ext_j5_apply_valid) {
        dm_j5.hold_here();
      }
      if (ext_j6_valid && !ext_j6_apply_valid) {
        dm_j6.hold_here();
      }
      if (ext_j7_valid && !ext_j7_apply_valid) {
        dm_j7.hold_here();
        j7_src_invalid_hold = true;
      }
      const bool vt03_j7_wheel_ctrl =
          ctl.vt03_active &&
          ((ctl.main_mode == RcMainMode::Gimbal) || (ctl.main_mode == RcMainMode::Chassis)) &&
          !zero_force &&
          (cmd.behaviour == GIMBAL_RELATIVE_ANGLE) &&
          !ext_j7_apply_valid;
      if (vt03_j7_wheel_ctrl) {
        apply_jaw_vt03_wheel(rc_now.rc.ch[GIMBAL_DM_J7_RC_CH]);
        j7_src_vt03_wheel = true;
      } else if ((s0_up || s0_mid) &&
                 !zero_force &&
                 cmd.behaviour == GIMBAL_RELATIVE_ANGLE &&
                 !ext_j7_apply_valid) {
        apply_jaw_inc(cmd.add_jaw);
        j7_src_inc = true;
      }
      prev_j5_enable = j5_enable;
      prev_j6_enable = j6_enable;
      }

      if (!zero_force) {
        if (ext_j4_apply_valid) {
          dm_j4.set_target_angle(ext_j4_target_rel_rad);
        }
        if (ext_j5_apply_valid) {
          dm_j5.set_target_angle(ext_j5_target_rel_rad);
        }
        if (ext_j6_apply_valid) {
          dm_j6.set_target_angle(ext_j6_target_rel_rad);
        }
        if (ext_j7_apply_valid) {
          dm_j7.set_target_angle(ext_j7_target_rel_rad);
          j7_src_ext_apply = true;
        }
      }
      prev_ext_j4_apply_valid = ext_j4_apply_valid;
      prev_ext_j5_apply_valid = ext_j5_apply_valid;
      prev_ext_j6_apply_valid = ext_j6_apply_valid;
      prev_ext_j7_apply_valid = ext_j7_apply_valid;
      } else {
        prev_s0_mid = false;
        prev_j5_enable = false;
        prev_j6_enable = false;
        roll_rate_active = false;
        prev_roll_rc_enable = false;

        dm_j5.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j6.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j7.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
        dm_j5.set_rate_cmd(0.0f);
        dm_j6.set_rate_cmd(0.0f);
        dm_j7.set_rate_cmd(0.0f);
        dm_j4.set_rate_cmd(0.0f);

        const fp32 j4_pause_rad = static_cast<fp32>(Omnix_CdegToRad(pause_cmd_cdeg[3]));
        const fp32 j5_pause_rad = static_cast<fp32>(Omnix_CdegToRad(pause_cmd_cdeg[4]));
        const fp32 j6_pause_rad = static_cast<fp32>(Omnix_CdegToRad(pause_cmd_cdeg[5]));
        // J7 in self-recovery (pause_active) does not follow pause target, hold position
        dm_j4.set_target_angle(j4_pause_rad);
        dm_j5.set_target_angle(j5_pause_rad);
        dm_j6.set_target_angle(j6_pause_rad);
        dm_j7.hold_here();
        j7_src_pause = true;
      }

      // Enable/disable is unified for all DM axes (edge-triggered only)
      static bool prev_pause_abort = false;
      const bool dm_enable = !pause_abort && !zero_force;
      if (pause_abort && !prev_pause_abort) {
        dm_j5.send_enable_disable(false);
        dm_j6.send_enable_disable(false);
        dm_j4.send_enable_disable(false);
        dm_j7.send_enable_disable(false);
      }
      prev_pause_abort = pause_abort;
      static bool prev_dm_enable = false;
      if (dm_enable) {
        if (!prev_dm_enable) {
          const bool j4_flip_before_reset = j4_limit_window_flipped;
          j4_limit_window_flipped = false;
          dm_j4.set_limit_window_flipped(false);
          j4_trigger_prev_pressed = false;
          LOGI("[J4][RESET] cause=dm_reenable flip_before=%u flip_after=%u home_target=%.3f",
               j4_flip_before_reset ? 1u : 0u,
               0u,
               static_cast<double>(pause_target_rad(3u)));
          apply_dm_enable_target(dm_j5, &dm_j5_poweron_home_pending, ext_j5_apply_valid, 4u);
          apply_dm_enable_target(dm_j6, &dm_j6_poweron_home_pending, ext_j6_apply_valid, 5u);
          apply_dm_enable_target(dm_j4, &dm_j4_poweron_home_pending, ext_j4_apply_valid, 3u);
          const bool j7_poweron_home_before = dm_j7_poweron_home_pending;
          apply_dm_enable_target(dm_j7, &dm_j7_poweron_home_pending, ext_j7_apply_valid, 6u);
          if (!ext_j7_apply_valid) {
            if (j7_poweron_home_before) {
              j7_src_enable_home = true;
            } else {
              j7_src_enable_hold = true;
            }
          }
          dm_j5.send_enable_disable(true);
          dm_j6.send_enable_disable(true);
          dm_j4.send_enable_disable(true);
          dm_j7.send_enable_disable(true);
        }
      } else {
        if (prev_dm_enable) {
          dm_j5.send_enable_disable(false);
          dm_j6.send_enable_disable(false);
          dm_j4.send_enable_disable(false);
          dm_j7.send_enable_disable(false);
        }
      }
      prev_dm_enable = dm_enable;

      if (custom_dm_log_due) {
        custom_dm_log_last_ms = custom_diag_now_ms;
        LOGI("[GIMBAL][CUSTOM][DM] seq=%u fresh=%u pause=%u zero=%u wait_fresh=%u session_active=%u gcomp_off=%u flip=%u j5_inv=%u j6_inv=%u j4_limit_clamp_en=%u j4=%u/%u hold=%u in=%u d=%ld z=%u ctrl_deg=%.3f j4_map_raw_tgt=%.3f j4_map_guard_tgt=%.3f tgt=%.3f cur=%.3f raw=%u j5=%u/%u hold=%u in=%u d=%ld z=%u ctrl_deg=%.3f tgt=%.3f cur=%.3f raw=%u j6=%u/%u hold=%u in=%u d=%ld z=%u ctrl_deg=%.3f tgt=%.3f cur=%.3f raw=%u",
             static_cast<unsigned>(ext_cmd.seq),
             ext_cmd_fresh ? 1u : 0u,
             pause_active ? 1u : 0u,
             zero_force ? 1u : 0u,
             dm_wait_external_fresh_after_safe_exit ? 1u : 0u,
             custom_dm_session_active ? 1u : 0u,
             custom_gcomp_off ? 1u : 0u,
             j4_limit_window_flipped ? 1u : 0u,
             (j4_limit_window_flipped && GIMBAL_J5_EXT_INVERT_WHEN_J4_FLIPPED) ? 1u : 0u,
             (j4_limit_window_flipped && GIMBAL_J6_EXT_INVERT_WHEN_J4_FLIPPED) ? 1u : 0u,
             j4_limit_clamp_enabled ? 1u : 0u,
             ext_j4_valid ? 1u : 0u,
             ext_j4_apply_valid ? 1u : 0u,
             ext_j4_map.hold_last_target ? 1u : 0u,
             static_cast<unsigned>(ext_j4_protocol_raw),
             static_cast<long>(ext_j4_map.delta_raw),
             static_cast<unsigned>(GIMBAL_J4_EXT_BIDIR_REL_ANGLE_MAP_CFG.controller_calib.raw_zero),
             static_cast<double>(ext_j4_map.controller_rel_deg),
             static_cast<double>(ext_j4_target_rel_rad_from_map),
             static_cast<double>(ext_j4_target_rel_rad_guarded),
             static_cast<double>(ext_j4_target_rel_rad),
             static_cast<double>(dm_j4.theta()),
             static_cast<unsigned>(cur_j4_dm_raw),
             ext_j5_valid ? 1u : 0u,
             ext_j5_apply_valid ? 1u : 0u,
             ext_j5_map.hold_last_target ? 1u : 0u,
             static_cast<unsigned>(ext_j5_protocol_raw),
             static_cast<long>(ext_j5_map.delta_raw),
             static_cast<unsigned>(GIMBAL_J5_EXT_BIDIR_REL_ANGLE_MAP_CFG.controller_calib.raw_zero),
             static_cast<double>(ext_j5_map.controller_rel_deg),
             static_cast<double>(ext_j5_target_rel_rad),
             static_cast<double>(dm_j5.theta()),
             static_cast<unsigned>(cur_j5_dm_raw),
             ext_j6_valid ? 1u : 0u,
             ext_j6_apply_valid ? 1u : 0u,
             ext_j6_map.hold_last_target ? 1u : 0u,
             static_cast<unsigned>(ext_j6_protocol_raw),
             static_cast<long>(ext_j6_map.delta_raw),
             static_cast<unsigned>(GIMBAL_J6_EXT_BIDIR_REL_ANGLE_MAP_CFG.controller_calib.raw_zero),
             static_cast<double>(ext_j6_map.controller_rel_deg),
             static_cast<double>(ext_j6_target_rel_rad),
             static_cast<double>(dm_j6.theta()),
             static_cast<unsigned>(cur_j6_dm_raw));
      }

      if (!pause_active) {
      // Big Roll manual control (J4):
      // - Only when s[0]=UP and s[1]=UP (per requirement).
      // - While stick is away from center: use Rate mode (low latency).
      // - When stick returns to center: hold current angle to stop cleanly.
      const bool roll_rc_enable = (!zero_force && s0_up && s1_up);

      if (prev_roll_rc_enable && !roll_rc_enable) {
        dm_j4.hold_here();
        dm_j4.set_rate_cmd(0.0f);
        dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
        roll_rate_active = false;
      }
      prev_roll_rc_enable = roll_rc_enable;

      if (ext_j4_apply_valid && !zero_force) {
        dm_j4.set_rate_cmd(0.0f);
        dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
        roll_rate_active = false;
      } else if (roll_rc_enable) {
        const int dead = static_cast<int>(GIMBAL_DM_J4_DEADBAND);
        constexpr float CH_MAX = 660.0f;
        constexpr fp32 W_MAX_ROLL = 12.0f;  // rad/s
        int ch1 = static_cast<int>(rc_now.rc.ch[GIMBAL_DM_J4_RC_CH]) *
                      static_cast<int>(GIMBAL_DM_J4_RC_SIGN) -
                  static_cast<int>(roll_ch1_trim);
        if (ch1 > 660) ch1 = 660;
        if (ch1 < -660) ch1 = -660;

        if (ch1 > -dead && ch1 < dead) {
          if (roll_rate_active) {
            dm_j4.hold_here();
            dm_j4.set_rate_cmd(0.0f);
            dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
            roll_rate_active = false;
          }
        } else {
          const fp32 w = (static_cast<fp32>(ch1) / CH_MAX) * W_MAX_ROLL;
          dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Rate);
          dm_j4.set_rate_cmd(w);
          roll_rate_active = true;
        }
      } else {
        roll_rate_active = false;
        dm_j4.set_rate_cmd(0.0f);
        dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
      }
      } else {
        roll_rate_active = false;
        prev_roll_rc_enable = false;
      }

      if (gear_exit_hold) {
        dm_j4.set_ctrl_mode(GimbalAxisDm4340MitJ4::CtrlMode::Angle);
        dm_j5.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j6.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j7.set_ctrl_mode(GimbalAxis::CtrlMode::Angle);
        dm_j4.set_rate_cmd(0.0f);
        dm_j5.set_rate_cmd(0.0f);
        dm_j6.set_rate_cmd(0.0f);
        dm_j7.set_rate_cmd(0.0f);
        dm_j4.hold_here();
        dm_j5.hold_here();
        dm_j6.hold_here();
        dm_j7.hold_here();
        ext_j4_hold_target_rel_rad = dm_j4.theta();
        ext_j5_hold_target_rel_rad = dm_j5.theta();
        ext_j6_hold_target_rel_rad = dm_j6.theta();
        ext_j7_hold_target_rel_rad = dm_j7.theta();
        prev_ext_j4_apply_valid = false;
        prev_ext_j5_apply_valid = false;
        prev_ext_j6_apply_valid = false;
        prev_ext_j7_apply_valid = false;
      }

      if (custom_dm_log_due) {
        const fp32 final_j7_target_rel_rad = dm_j7.target_angle();
        const bool j7_overwritten =
            ext_j7_apply_valid && (fabsf(final_j7_target_rel_rad - ext_j7_target_rel_rad) > 1e-3f);
        LOGI("[GIMBAL][CUSTOM][J7] seq=%u fresh=%u pause=%u zero=%u wait_fresh=%u session=%u apply=%u valid=%u hold=%u upd=%u frz=%u in=%u d=%ld min=%u max=%u ctrl_deg=%.3f map=%.3f cache=%.3f pre=%.3f final=%.3f cur=%.3f raw=%u err=0x%02X oc=%u ovr=%u src_ext=%u src_fault=%u src_wh=%u src_inc=%u src_pause=%u src_en_home=%u src_en_hold=%u src_sess_hold=%u src_s0_hold=%u src_inv_hold=%u evt_enter=%u evt_upd=%u",
             static_cast<unsigned>(ext_cmd.seq),
             ext_cmd_fresh ? 1u : 0u,
             pause_active ? 1u : 0u,
             zero_force ? 1u : 0u,
             dm_wait_external_fresh_after_safe_exit ? 1u : 0u,
             custom_dm_session_active ? 1u : 0u,
             ext_j7_apply_valid ? 1u : 0u,
             ext_j7_valid ? 1u : 0u,
             ext_j7_map.hold_last_target ? 1u : 0u,
             (!ext_j7_map.hold_last_target) ? 1u : 0u,
             0u,
             static_cast<unsigned>(ext_j7_protocol_raw),
             static_cast<long>(ext_j7_map.delta_raw),
             static_cast<unsigned>(GIMBAL_J7_HALF_RANGE_RAW_AT_MOTOR_MAX),
             static_cast<unsigned>(GIMBAL_J7_HALF_RANGE_RAW_AT_MOTOR_MIN),
             static_cast<double>(ext_j7_map.controller_rel_deg),
             static_cast<double>(ext_j7_map.target_rel_rad),
             static_cast<double>(ext_j7_target_rel_rad),
             static_cast<double>(pre_j7_target_rel_rad),
             static_cast<double>(final_j7_target_rel_rad),
             static_cast<double>(dm_j7.theta()),
             static_cast<unsigned>(cur_j7_dm_raw),
             static_cast<unsigned>(cur_j7_err),
             cur_j7_overcurrent ? 1u : 0u,
             j7_overwritten ? 1u : 0u,
             j7_src_ext_apply ? 1u : 0u,
             j7_src_fault_freeze ? 1u : 0u,
             j7_src_vt03_wheel ? 1u : 0u,
             j7_src_inc ? 1u : 0u,
             j7_src_pause ? 1u : 0u,
             j7_src_enable_home ? 1u : 0u,
             j7_src_enable_hold ? 1u : 0u,
             j7_src_session_hold ? 1u : 0u,
             j7_src_s0mid_hold ? 1u : 0u,
             j7_src_invalid_hold ? 1u : 0u,
             j7_evt_cache_enter ? 1u : 0u,
             j7_evt_cache_update ? 1u : 0u);
      }

      // Always run and always send frames (even in hold mode), just freeze setpoints above.
      dm_j5.run(zero_force);
      dm_j6.run(zero_force);
      dm_j7.run(zero_force);
      dm_j4.run(zero_force);

      // DM control frames are sent by the CAN TX router.

      gimbal_all_online = dm_j5.online() && dm_j6.online() && dm_j7.online() && dm_j4.online();
      prev_s0_down = s0_down;
    }
  }
}

