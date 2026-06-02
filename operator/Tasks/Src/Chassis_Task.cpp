// Tasks/Src/Chassis_Task.cpp
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Gimbal_behavior_Task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include "cmath"

#include "bsp_srn_log.h"
#include "NTFDCAN_Router.h"

#include "lib_adp_dji_c620.h"
#include "lib_adp_dji_gm6020.h"
#include "lib_remote_control.h"     // RC_GetSnapshot / RC_GetStatus / switch helpers
#include "RC_Task.h"
#include "lib_adp_navision.h"

#include "CONF_Chassis_Task.h"
#include "RC_Control_Mode.h"
#include "mathutil.h"               // NORM_ANGLE / NORM_ANGLE_RAD / ABS / MIN / MAX / COMPARE_ABS 锟?
#include "swerve_kinematics.h"
#include "pid.h"                    // algo::PID
#include "filters.h"                // algo::first_order_filter_init / cali

#if CHASSIS_DIAG_ENABLE
struct ChassisDiag {
    float theta_curr[4];
    float theta_set[4];
    float target_velocity[4];
    float velocity[4];
    float pre_current[4];
    uint8_t anti[4];
    int16_t give_current[4];
    uint8_t drive_saturated[4];
    uint32_t drive_saturated_count[4];
};

ChassisDiag chassis_diag{};

typedef struct {
    int16_t vx;
    int16_t vy;
    int16_t omega_z;
    uint8_t is_online;
} NaviDebug;

volatile NaviDebug g_navi_debug = {};
#endif

volatile bool chassis_motor_all_online = false;
volatile bool chassis_zero_force = true;
volatile bool rc_flag = false;
gimbal_cmd_t cmd;

/* ========================= 灏忓伐鍏凤細DWT 璁℃椂锛堟洿鍑嗙殑 dt锟?========================= */
static inline bool dwt_try_enable(){
#if (__CORTEX_M == 7) || (__CORTEX_M == 4)
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
#else
    return false;
#endif
}

/* ================================ 妯″紡瀹氫箟 ================================ */
enum class ChassisMode : uint8_t { NoForce = 0, Follow, Auto, Hold };

static inline ChassisMode decide_mode_from_control(const RcControlMode& ctl){
    switch (ctl.main_mode) {
        case RcMainMode::Chassis:
            return ChassisMode::Follow;
        case RcMainMode::Gimbal:
            return ctl.vt03_active ? ChassisMode::Follow : ChassisMode::Hold;
        case RcMainMode::ZeroForce:
        default:
            return ChassisMode::NoForce;
    }
}

/* ================================ 缁撴瀯锟?================================ */

struct WheelState {
    float velocity{0.f};          // 瀹炴祴 m/s
    float target_velocity{0.f};   // 鐩爣 m/s
    int16_t give_current{0};      // 杈撳嚭鐢垫祦
    uint8_t anti{0};              // 鏄惁鍙嶅悜锛堢敱鑸佃閫夋嫨 set_b 鍐冲畾锟?
    float pre_current{0.f};       // PID 杈撳嚭鐨勬诞鐐圭數娴侊紙鍐嶈鍓负 int16锟?
    uint8_t saturated{0};
};

struct ServoState {
    float omega{0.f};             // deg/s
    float theta{0.f};             // 瀹為檯鑸佃 deg锛堢粡闆朵綅涓庡綊涓€鍖栧悗锟?
    float theta_set{0.f};         // 鐩爣鑸佃 deg
    uint16_t ecd{0};              // 鍘熷缂栫爜锟?
    uint16_t last_ecd{0};
    float vx{0.f}, vy{0.f};       // 璇ヨ疆鍒嗚В鍚庣殑 x/y 绾块€熷害鍒嗛噺
    int16_t give_current{0};      // 杈撳嚭鐢垫祦
};

static inline int16_t clamp_i16(float x, float lo, float hi){
    return static_cast<int16_t>(algo::clampf(x, lo, hi));
}
inline float map_linear(float x,float in_min, float in_max,float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) /(in_max - in_min) + out_min;
}

/* ============================== 鎺у埗鍣ㄦ湰锟?============================== */
class ChassisController {
public:
    ChassisController()
    : steer_{
        DJI_GM6020(CHASSIS_6020_RB_ID, 0),
        DJI_GM6020(CHASSIS_6020_RF_ID, 0),
        DJI_GM6020(CHASSIS_6020_LF_ID, 0),
        DJI_GM6020(CHASSIS_6020_LB_ID, 0)
      },
      drive_{
        DJI_C620  (CHASSIS_C620_RB_ID, 0),
        DJI_C620  (CHASSIS_C620_RF_ID, 0),
        DJI_C620  (CHASSIS_C620_LF_ID, 0),
        DJI_C620  (CHASSIS_C620_LB_ID, 0)
      }
    {
        // 绂荤嚎鍒ゅ畾绐楀彛
        for (auto &m : steer_) m.set_offline_timeout(MOTOR_OFFLINE_MS);
        for (auto &m : drive_) m.set_offline_timeout(MOTOR_OFFLINE_MS);

        // 娉ㄥ唽 Router锛堟寜 bus + slot锟?
        (void)canrx_register_gm6020(CHASSIS_6020_RB_BUS, &steer_[0]);
        (void)canrx_register_gm6020(CHASSIS_6020_RF_BUS, &steer_[1]);
        (void)canrx_register_gm6020(CHASSIS_6020_LF_BUS, &steer_[2]);
        (void)canrx_register_gm6020(CHASSIS_6020_LB_BUS, &steer_[3]);
        (void)cantx_register_gm6020(CHASSIS_6020_RB_BUS, &steer_[0], 2);
        (void)cantx_register_gm6020(CHASSIS_6020_RF_BUS, &steer_[1], 2);
        (void)cantx_register_gm6020(CHASSIS_6020_LF_BUS, &steer_[2], 2);
        (void)cantx_register_gm6020(CHASSIS_6020_LB_BUS, &steer_[3], 2);

        (void)canrx_register_c620(CHASSIS_C620_RB_BUS, &drive_[0]);
        (void)canrx_register_c620(CHASSIS_C620_RF_BUS, &drive_[1]);
        (void)canrx_register_c620(CHASSIS_C620_LF_BUS, &drive_[2]);
        (void)canrx_register_c620(CHASSIS_C620_LB_BUS, &drive_[3]);
        (void)cantx_register_c620(CHASSIS_C620_RB_BUS, &drive_[0], 2);
        (void)cantx_register_c620(CHASSIS_C620_RF_BUS, &drive_[1], 2);
        (void)cantx_register_c620(CHASSIS_C620_LF_BUS, &drive_[2], 2);
        (void)cantx_register_c620(CHASSIS_C620_LB_BUS, &drive_[3], 2);

        (void)canrx_register_navi(CHASSIS_NAVISION_BUS, &nav_);

        // 鍒濆锟?PID
        const float sv[3] = { SERVO_VELOCITY_PID_KP, SERVO_VELOCITY_PID_KI, SERVO_VELOCITY_PID_KD };
        const float sa[3] = { SERVO_ANGLE_PID_KP,    SERVO_ANGLE_PID_KI,    SERVO_ANGLE_PID_KD    };
        const float rv[3] = { SERVO_ANGLE_PID_KP,    SERVO_ANGLE_PID_KI,    SERVO_ANGLE_PID_KD    };
        const float ra[3] = { SERVO_ANGLE_PID_KP,    SERVO_ANGLE_PID_KI,    SERVO_ANGLE_PID_KD    };
        for (int i = 0; i < 4; ++i) {
            const float wv[3] = { WHEEL_VELOCITY_PID_KP[i], WHEEL_VELOCITY_PID_KI[i], WHEEL_VELOCITY_PID_KD[i] };
            PID_init(&pid_wheel_vel_[i],  algo::PID_POSITION, wv, WHEEL_VELOCITY_PID_MAX_OUT[i], WHEEL_VELOCITY_PID_MAX_IOUT[i]);
            PID_init(&pid_servo_vel_[i],  algo::PID_POSITION, sv, SERVO_VELOCITY_PID_MAX_OUT, SERVO_VELOCITY_PID_MAX_IOUT);
            PID_init(&pid_servo_ang_[i],  algo::PID_POSITION, sa, SERVO_ANGLE_PID_MAX_OUT,    SERVO_ANGLE_PID_MAX_IOUT);
        }
        PID_init(&pid_rotate_angle_,  algo::PID_POSITION, rv, YAW_ANGLE_MAX_OUT,    YAW_ANGLE_MAX_IOUT);
        PID_init(&pid_rotate_vel_,  algo::PID_POSITION, ra, YAW_VEL_MAX_OUT,    YAW_VEL_MAX_IOUT);
        // 閫熷害骞虫粦婊ゆ尝锛堝彲閫夛級
        const float fx[1] = { CHASSIS_ACCEL_VX_NUM };
        const float fy[1] = { CHASSIS_ACCEL_VY_NUM };
        const float fz[1] = { CHASSIS_ACCEL_WZ_NUM };
        algo::first_order_filter_init(&filt_vx_, 0.001f, fx);
        algo::first_order_filter_init(&filt_vy_, 0.001f, fy);
        algo::first_order_filter_init(&filt_wz_, 0.001f, fz);
    }

    void tick_1kHz()
    {
        const uint32_t now_ms = HAL_GetTick();

        // 1) 蹇冭烦锛堢绾胯鏁帮級
        for (auto &m : steer_) m.tick(now_ms);
        for (auto &m : drive_) m.tick(now_ms);

        // 2) 妯″紡鐘舵€佹満
        update_mode();

        // 3) 璇诲彇鐢垫満鍙嶉骞舵洿锟?servo_/wheel_ 瀹炴祴锟?
        update_feedback();
        update_navi_watch();
        chassis_motor_all_online = compute_all_online();

        // 4) 瀹夊叏鍏滃簳
        RC_Status st = RC_GetStatus();
        const bool safe_hold = (mode_ == ChassisMode::NoForce) || (!st.online) || (st.blocked);
        if (safe_hold) {
            chassis_zero_force = true;
            zero_outputs_and_send();
            vx_ = vy_ = wz_ = 0.f;
            for (auto &w : wheel_) w.target_velocity = 0.f;
#if CHASSIS_DIAG_ENABLE
            update_diag();
#endif
            if (now_ms - dbg_echo_ > 1000) {
                dbg_echo_ = now_ms;
                RC_State rc_dbg{};
                RC_GetSnapshot(&rc_dbg);
                const RcControlMode ctl_dbg = Rc_ResolveControlMode(rc_dbg, st);
                LOGD("SAFE HOLD. ecd= %.1f %.1f %.1f %.1f",
                    (float)servo_[0].ecd, (float)servo_[1].ecd, (float)servo_[2].ecd, (float)servo_[3].ecd);
                LOGD("[SAFE HOLD][RC] mode=%u online=%u blocked=%u active_src=%u dr16=%u vt03=%u s0=%u main_mode=%u",
                    static_cast<unsigned>(mode_),
                    st.online ? 1u : 0u,
                    st.blocked ? 1u : 0u,
                    static_cast<unsigned>(st.active_src),
                    st.dr16_online ? 1u : 0u,
                    st.vt03_online ? 1u : 0u,
                    static_cast<unsigned>(ctl_dbg.s0_state),
                    static_cast<unsigned>(ctl_dbg.main_mode));
#if CHASSIS_DIAG_ENABLE && CHASSIS_SAFE_HOLD_LOG_GIMBAL_J1_J9
                gimbal_joint_snapshot_t gimbal_snap{};
                if (Gimbal_ReadJointSnapshot(&gimbal_snap)) {
                    LOGD("[SAFE HOLD][GIM] J1=raw:%u pos:%d(%u) J2=raw:%u pos:%d(%u) J3=raw:%u pos:%d(%u)",
                        static_cast<unsigned>(gimbal_snap.raw_u16[0]),
                        static_cast<int>(gimbal_snap.angle_cdeg[0]),
                        static_cast<unsigned>(gimbal_snap.online[0]),
                        static_cast<unsigned>(gimbal_snap.raw_u16[1]),
                        static_cast<int>(gimbal_snap.angle_cdeg[1]),
                        static_cast<unsigned>(gimbal_snap.online[1]),
                        static_cast<unsigned>(gimbal_snap.raw_u16[2]),
                        static_cast<int>(gimbal_snap.angle_cdeg[2]),
                        static_cast<unsigned>(gimbal_snap.online[2]));
                    LOGD("[SAFE HOLD][GIM] J4=raw:%u pos:%d(%u) J5=raw:%u pos:%d(%u) J6=raw:%u pos:%d(%u)",
                        static_cast<unsigned>(gimbal_snap.raw_u16[3]),
                        static_cast<int>(gimbal_snap.angle_cdeg[3]),
                        static_cast<unsigned>(gimbal_snap.online[3]),
                        static_cast<unsigned>(gimbal_snap.raw_u16[4]),
                        static_cast<int>(gimbal_snap.angle_cdeg[4]),
                        static_cast<unsigned>(gimbal_snap.online[4]),
                        static_cast<unsigned>(gimbal_snap.raw_u16[5]),
                        static_cast<int>(gimbal_snap.angle_cdeg[5]),
                        static_cast<unsigned>(gimbal_snap.online[5]));
                    LOGD("[SAFE HOLD][GIM] J7=raw:%u pos:%d(%u) J8=raw:%u pos:%d(%u) J9=raw:%u pos:%d(%u)",
                        static_cast<unsigned>(gimbal_snap.raw_u16[6]),
                        static_cast<int>(gimbal_snap.angle_cdeg[6]),
                        static_cast<unsigned>(gimbal_snap.online[6]),
                        static_cast<unsigned>(gimbal_snap.raw_u16[7]),
                        static_cast<int>(gimbal_snap.angle_cdeg[7]),
                        static_cast<unsigned>(gimbal_snap.online[7]),
                        static_cast<unsigned>(gimbal_snap.raw_u16[8]),
                        static_cast<int>(gimbal_snap.angle_cdeg[8]),
                        static_cast<unsigned>(gimbal_snap.online[8]));
                }
#endif
            }
            return;
        }
        // 闈炲畨鍏ㄦā寮忥細瀵瑰鏍囪涓衡€滈潪 Zero-Force锟?
        chassis_zero_force = false;

        // 5) RC 锟?閫熷害
        rc_to_velocity();

        // 6) 杩愬姩瀛﹀垎瑙ｏ紙璁＄畻 theta_set / target_velocity锟?
        kinematics();

        // 7) PID
        run_pid();

        // 8) 鍙戯拷?
        can_send();
#if CHASSIS_DIAG_ENABLE
        update_diag();
        log_drive_diag(now_ms);
#endif
    }

    /* 鍙€夛細鐢ㄤ簬鍙锟?璋冭瘯 */
    void get_rc_snapshot(RC_State* out) const { if (out){ *out = rc_snap_; } }
    ChassisMode mode() const { return mode_; }

private:
    /* ============== 鎴愬憳锛氱‖浠跺锟?============== */
    DJI_GM6020 steer_[4];  // 0:Rb 1:Rf 2:Lf 3:Lb
    DJI_C620   drive_[4];
    Navi       nav_{CHASSIS_NAVISION_STD_ID};

    /* ============== 鎴愬憳锛氭帶鍒剁姸锟?============== */
    WheelState wheel_[4];
    ServoState servo_[4];

    algo::pid_type_def pid_wheel_vel_[4]{};
    algo::pid_type_def pid_servo_vel_[4]{};
    algo::pid_type_def pid_servo_ang_[4]{};
    algo::pid_type_def pid_rotate_vel_;
    algo::pid_type_def pid_rotate_angle_;

    algo::first_order_filter_type_t filt_vx_{}, filt_vy_{}, filt_wz_{};

    // 鏈熸湜閫熷害锛堟満浣撳潗鏍囩郴锟?
    float vx_{0.f}, vy_{0.f}, wz_{0.f};
    // 鐗规畩濮匡拷?45掳 灏忛檧锟?
    uint8_t stop45_{0};

    float gimbal_yaw_{0.f};
    bool  gimbal_online_{false};
    uint32_t drive_sat_count_[4]{};
    uint32_t drive_sat_log_ms_{0};

    // 妯″紡鐘舵€佹満
    ChassisMode mode_{ChassisMode::NoForce};
    ChassisMode target_mode_{ChassisMode::NoForce};
    uint32_t mode_since_ms_{0};
    static constexpr uint16_t MODE_HOLD_MS_ = 120;

    // 璋冭瘯
    RC_State rc_snap_{};
    uint32_t dbg_echo_{0};

    bool compute_all_online() const {
        // 椤哄簭锟? RB, 1 RF, 2 LF, 3 LB
        for (int i = 0; i < 4; ++i) {
            const GM6020State s_steer = steer_[i].state();
            const C620State   s_drive = drive_[i].state();
            if (!s_steer.online || !s_drive.online) {
                return false;
            }
        }
        return true;
    }

    inline void rotate_by_gimbal_yaw(float& vx, float& vy)
    {
        // gimbal_feedback.yaw_relative锛氬崟锟?rad锛堝ぇ yaw 鐩稿搴曠洏瑙掑害锟?
        // 浣跨敤璐熷彿锛氬皢鈥滃ぇ yaw/涓栫晫鍧愭爣鈥濇寚浠よ浆鎹㈠埌搴曠洏鍧愭爣锛堟柟鍚戜慨姝ｏ級
        const float yaw = -NORM_ANGLE_RAD(gimbal_feedback.yaw_relative);

        const float c = cosf(yaw);
        const float s = sinf(yaw);

        // 浠庘€滃ぇ yaw/涓栫晫鍧愭爣鈥濇棆杞埌鈥滃簳鐩樺潗鏍囷拷?
        const float vx_new =  c * vx - s * vy;
        const float vy_new =  s * vx + c * vy;

        vx = vx_new;
        vy = vy_new;
    }

    /* ============== 姝ラ瀹炵幇 ============== */
    void update_mode()
    {
        RC_State rc; RC_GetSnapshot(&rc);
        RC_Status st = RC_GetStatus();
        const RcControlMode ctl = Rc_ResolveControlMode(rc, st);
        const uint32_t now = HAL_GetTick();

        // 绂荤嚎/闃诲 锟?绔嬪嵆 NoForce
        if (!st.online || st.blocked) {
            if (mode_ != ChassisMode::NoForce) {
                mode_ = ChassisMode::NoForce;
                target_mode_ = ChassisMode::NoForce;
                mode_since_ms_ = now;
                LOGW("Mode forced: NoForce (offline/blocked) L=%d R=%d", rc.rc.s[0], rc.rc.s[1]);
            }
            return;
        }


        // 鍦ㄧ嚎锛氭寜鎷ㄦ潌寰楀埌鏈熸湜妯″紡
        const ChassisMode desired = decide_mode_from_control(ctl);
        if (desired == ChassisMode::NoForce || desired == ChassisMode::Hold) {
            if (mode_ != desired || target_mode_ != desired) {
                mode_ = desired;
                target_mode_ = desired;
                mode_since_ms_ = now;
                LOGI("Mode -> %s (L=%d R=%d)",
                     (mode_==ChassisMode::NoForce? "NoForce":
                      mode_==ChassisMode::Follow? "Follow":
                      mode_==ChassisMode::Hold? "Hold":"AUTO"),
                     rc.rc.s[0], rc.rc.s[1]);
            }
            return;
        }
        if (desired != target_mode_) {
            target_mode_ = desired;
            mode_since_ms_ = now;
        }
        if ((now - mode_since_ms_) >= MODE_HOLD_MS_ && mode_ != target_mode_) {
            mode_ = target_mode_;
            LOGI("Mode -> %s (L=%d R=%d)",
                 (mode_==ChassisMode::NoForce? "NoForce":
                  mode_==ChassisMode::Follow? "Follow":
                  mode_==ChassisMode::Hold? "Hold":"AUTO"),
                 rc.rc.s[0], rc.rc.s[1]);
        }
    }

    void rc_to_velocity()
    {
        RC_GetSnapshot(&rc_snap_);
        RC_Status st = RC_GetStatus();
        const RcControlMode ctl = Rc_ResolveControlMode(rc_snap_, st);
        const bool vt03_gimbal_keyboard_only =
            (st.active_src == RC_SRC_VT03) &&
            (ctl.main_mode == RcMainMode::Gimbal) &&
            st.online &&
            !st.blocked;
        const bool vt03_keyboard_move_enabled =
            (st.active_src == RC_SRC_VT03) &&
            (ctl.main_mode != RcMainMode::ZeroForce) &&
            st.online &&
            !st.blocked;
        constexpr uint16_t KEY_MASK_VX = KEY_PRESSED_OFFSET_A | KEY_PRESSED_OFFSET_D;
        constexpr uint16_t KEY_MASK_VY = KEY_PRESSED_OFFSET_W | KEY_PRESSED_OFFSET_S;
        constexpr uint16_t KEY_MASK_WZ = KEY_PRESSED_OFFSET_Q | KEY_PRESSED_OFFSET_E;
        const auto key_axis = [&](uint16_t pos_key, uint16_t neg_key, float magnitude) -> float {
            float out = 0.0f;
            if (rc_snap_.key.v & pos_key) out += magnitude;
            if (rc_snap_.key.v & neg_key) out -= magnitude;
            return out;
        };
        const auto analog_axis = [&](int16_t stick, float analog_scale, int16_t deadband) -> float {
            if (Rc_AxisIdle(stick, deadband)) {
                return 0.0f;
            }
            return static_cast<float>(stick) * analog_scale;
        };
        struct AxisInputCmd {
            float value;
            bool from_key;
        };
        const auto select_axis = [&](int16_t stick, float analog_scale, float key_value,
                                     uint16_t key_mask, int16_t deadband) -> AxisInputCmd {
            const float analog_value = analog_axis(stick, analog_scale, deadband);
            const bool key_active = (rc_snap_.key.v & key_mask) != 0u;
            if (vt03_gimbal_keyboard_only) {
                return AxisInputCmd{key_active ? key_value : 0.0f, key_active};
            }
            if (vt03_keyboard_move_enabled) {
                return AxisInputCmd{key_active ? key_value : analog_value, key_active};
            }
            if (!Rc_AxisIdle(stick, deadband)) {
                return AxisInputCmd{analog_value, false};
            }
            return AxisInputCmd{key_value, key_active};
        };
        // Keyboard control is always fixed to the operator-facing convention:
        // W/S -> front/back, A/D -> left/right, Q/E -> +/-wz (reversed per user request).
        // In the current chassis kinematics, +vy is front and +vx is left.
        // frontFLAG and yaw compensation only affect the joystick path.
        const float key_vx = key_axis(KEY_PRESSED_OFFSET_A, KEY_PRESSED_OFFSET_D,
                                      MAX_VELOCITY_RIGHT * 12.0f);
        const float key_vy = key_axis(KEY_PRESSED_OFFSET_W, KEY_PRESSED_OFFSET_S,
                                      MAX_VELOCITY_FORWARD * 12.0f);
        const float key_wz = key_axis(KEY_PRESSED_OFFSET_Q, KEY_PRESSED_OFFSET_E,
                                      MAX_VELOCITY_ROTATE * 12.0f);

        switch (mode_)
        {
            case ChassisMode::NoForce:
                vx_ = vy_ = wz_ = 0.f;
                break;

            case ChassisMode::Hold:
                vx_ = vy_ = wz_ = 0.f;
                break;

            case ChassisMode::Follow:
                {
                    const int deadband = 20;
                    if (frontFLAG) {
                        const AxisInputCmd vx_cmd = select_axis(rc_snap_.rc.ch[2], RC_SEN_FORWARD * 12.0f,
                                                                key_vx, KEY_MASK_VX, deadband);
                        const AxisInputCmd vy_cmd = select_axis(rc_snap_.rc.ch[3], -RC_SEN_RIGHT * 12.0f,
                                                                key_vy, KEY_MASK_VY, deadband);
                        const AxisInputCmd wz_cmd = select_axis(rc_snap_.rc.ch[0], -RC_SEN_ROTATE * 12.0f,
                                                                key_wz, KEY_MASK_WZ, deadband);
                        vx_ = vx_cmd.value;
                        vy_ = vy_cmd.value;
                        wz_ = wz_cmd.value;
                    } else {
                        const AxisInputCmd vx_cmd = select_axis(rc_snap_.rc.ch[3], -RC_SEN_FORWARD * 12.0f,
                                                                key_vx, KEY_MASK_VX, deadband);
                        const AxisInputCmd vy_cmd = select_axis(rc_snap_.rc.ch[2], RC_SEN_RIGHT * 12.0f,
                                                                key_vy, KEY_MASK_VY, deadband);
                        const AxisInputCmd wz_cmd = select_axis(rc_snap_.rc.ch[0], -RC_SEN_ROTATE * 12.0f,
                                                                key_wz, KEY_MASK_WZ, deadband);
                        vx_ = vx_cmd.value;
                        vy_ = vy_cmd.value;
                        wz_ = wz_cmd.value;
                    }
                }
                break;

            case ChassisMode::Auto:
                {
                    const int deadband = 20;
                    AxisInputCmd vx_cmd = select_axis(rc_snap_.rc.ch[0], -RC_SEN_FORWARD * 12.0f,
                                                      key_vx, KEY_MASK_VX, deadband);
                    AxisInputCmd vy_cmd = select_axis(rc_snap_.rc.ch[1], RC_SEN_RIGHT * 12.0f,
                                                      key_vy, KEY_MASK_VY, deadband);

                    if (vt03_keyboard_move_enabled && ((rc_snap_.key.v & KEY_MASK_WZ) != 0u)) {
                        wz_ = key_wz;
                    } else {
                        wz_ = 0.f;
                    }

                    // Keep joystick field-oriented behavior, but never rotate keyboard WASD.
                    float analog_vx = vx_cmd.from_key ? 0.0f : vx_cmd.value;
                    float analog_vy = vy_cmd.from_key ? 0.0f : vy_cmd.value;
                    if (gimbal_all_online) {
                        rotate_by_gimbal_yaw(analog_vx, analog_vy);
                    }
                    if (!vx_cmd.from_key) {
                        vx_cmd.value = analog_vx;
                    }
                    if (!vy_cmd.from_key) {
                        vy_cmd.value = analog_vy;
                    }

                    vx_ = vx_cmd.value;
                    vy_ = vy_cmd.value;
                }
                break;
        }

        const bool x_lock_request =
            (mode_ != ChassisMode::NoForce) &&
            (::fabsf(vx_) <= CHASSIS_XLOCK_IDLE_VXY_EPS) &&
            (::fabsf(vy_) <= CHASSIS_XLOCK_IDLE_VXY_EPS) &&
            (::fabsf(wz_) <= CHASSIS_XLOCK_IDLE_WZ_EPS);
        stop45_ = x_lock_request ? 1u : 0u;


        // 鍙€夛細涓€鍒嗛挓闃舵护娉紱鑻ヤ笉闇€瑕佸彲娉ㄩ噴锟?
        // first_order_filter_cali(&filt_vx_, vx_);
        // first_order_filter_cali(&filt_vy_, vy_);
        // first_order_filter_cali(&filt_wz_, wz_);
        // vx_ = filt_vx_.output; vy_ = filt_vy_.output; wz_ = filt_wz_.output;
    }

    void update_feedback()
    {
        // 椤哄簭锛歊B, RF, LF, LB锛堜笌锟?椹卞姩瀵硅薄鍒涘缓椤哄簭涓€鑷达級
        const C620State   c620[4] = { drive_[0].state(), drive_[1].state(), drive_[2].state(), drive_[3].state() };
        const GM6020State gm20[4] = { steer_[0].state(), steer_[1].state(), steer_[2].state(), steer_[3].state() };

        const float r = WHEEL_DIAMETER * 0.5f;
        for (int i = 0; i < 4; ++i) {
            // 杞瓙瑙掗€熷害锛歞eg/s -> rad/s -> 绾块€熷害 m/s
            const float wheel_deg_s = static_cast<float>(c620[i].speed_raw) * 6.0f; // rpm -> deg/s
            const float omega_rad   = wheel_deg_s * (PI_F / 180.f);
            wheel_[i].velocity = WHEEL_DIR[i] * (omega_rad * r);

            // 鑸垫満锛氳閫熷害/瑙掑害
            servo_[i].omega    = static_cast<float>(gm20[i].speed_raw) * (360.f / 60.f);   // rpm -> deg/s
            servo_[i].last_ecd = servo_[i].ecd;
            servo_[i].ecd      = gm20[i].mech_angle_raw;

            const float mech_deg =
                ((int32_t)servo_[i].ecd - (int32_t)SERVO_INIT[i]) * (360.f / ECD_PPR_6020);
            // 闆朵綅淇濇寔浣跨敤 SERVO_INIT锛屼絾鍏佽瀵规煇浜涜疆鈥滈浂浣嶇炕锟?80掳鈥濓紙鎴栦换鎰忚搴﹀亸缃級
            servo_[i].theta = NORM_ANGLE(mech_deg + SERVO_THETA_OFFSET_DEG[i]);
        }
    }

    void update_navi_watch()
    {
        const NaviState s = nav_.state();
        g_navi_debug.vx = s.vx;
        g_navi_debug.vy = s.vy;
        g_navi_debug.omega_z = s.omega_z;
        g_navi_debug.is_online = s.is_navi_online ? 1u : 0u;
    }

    void kinematics()
    {
        float curr_theta_deg[4];
        uint8_t last_anti[4];
        for (int i = 0; i < 4; ++i) {
            curr_theta_deg[i] = servo_[i].theta;
            last_anti[i] = wheel_[i].anti;
        }

        algo::SwerveConfig cfg{ CHASSIS_ANTI_HYST_DEG, CHASSIS_SPEED_EPS };
        algo::SwerveModuleCmd cmd[4];
        algo::solve_swerve_4wheel(vx_, vy_, wz_,
                                  WHEEL_POS_X, WHEEL_POS_Y,
                                  curr_theta_deg, last_anti,
                                  cfg, cmd);

        if (stop45_ == 1) {
            bool all_idle = true;
            for (int i = 0; i < 4; ++i) {
                if (::fabsf(cmd[i].wheel_mps) >= CHASSIS_SPEED_EPS) {
                    all_idle = false;
                    break;
                }
            }
            if (all_idle) {
                const float alpha_deg = ::atan2f(CHASSIS_HALF_L, CHASSIS_HALF_W) * (180.0f / PI_F);
                const float theta_set[4] = {
                    -alpha_deg,
                    +alpha_deg,
                    180.f - alpha_deg,
                    -(180.f - alpha_deg)
                };
                for (int i = 0; i < 4; ++i) {
                    cmd[i].theta_set_deg = NORM_ANGLE(theta_set[i] + CHASSIS_XLOCK_THETA_SHIFT_DEG);
                    cmd[i].wheel_mps = 0.f;
                    cmd[i].anti = 0;
                }
            }
        }

        for (int i = 0; i < 4; ++i) {
            servo_[i].theta_set = cmd[i].theta_set_deg;
            wheel_[i].target_velocity = cmd[i].wheel_mps;
            wheel_[i].anti = cmd[i].anti;
        }
    }

    void run_pid()
    {
        // 鑸垫満锛氳搴︾幆鈫掗€熷害鐜覆锟?
        for (int i = 0; i < 4; ++i) {
            const float delta_angle = NORM_ANGLE(servo_[i].theta - servo_[i].theta_set);
            const float angle_out   = PID_calc(&pid_servo_ang_[i], delta_angle, 0.f);
            const float vel_out     = PID_calc(&pid_servo_vel_[i], servo_[i].omega, angle_out);
            servo_[i].give_current  = clamp_i16(vel_out, -SERVO_VELOCITY_PID_MAX_OUT, SERVO_VELOCITY_PID_MAX_OUT);
        }

        // 杞瓙锛氶€熷害锟?
        const int16_t drive_limit = static_cast<int16_t>(CHASSIS_DRIVE_GIVE_CMD_MAX);
        for (int i = 0; i < 4; ++i) {
            float out = PID_calc(&pid_wheel_vel_[i], wheel_[i].velocity, wheel_[i].target_velocity);
            wheel_[i].pre_current = out;
            wheel_[i].give_current = clamp_i16(out, -WHEEL_VELOCITY_PID_MAX_OUT[i], WHEEL_VELOCITY_PID_MAX_OUT[i]);
            wheel_[i].saturated = (wheel_[i].give_current >= drive_limit || wheel_[i].give_current <= -drive_limit) ? 1u : 0u;
            if (wheel_[i].saturated) {
                ++drive_sat_count_[i];
            }
        }
    }

    void can_send()
    {
        // 6020锛堣埖锛夛細鎸夋瘡杞柟鍚戠郴鏁颁笅鍙戯紙榛樿 +1锛涜嫢绾垮簭锟?鍙嶈鍒欑疆 -1锟?
        steer_[0].set_give_cmd(SERVO_DIR[0] * servo_[0].give_current);
        steer_[1].set_give_cmd(SERVO_DIR[1] * servo_[1].give_current);
        steer_[2].set_give_cmd(SERVO_DIR[2] * servo_[2].give_current);
        steer_[3].set_give_cmd(SERVO_DIR[3] * servo_[3].give_current);

        // C620锛堥┍鍔級
        drive_[0].set_give_cmd(WHEEL_DIR[0] * wheel_[0].give_current);
        drive_[1].set_give_cmd(WHEEL_DIR[1] * wheel_[1].give_current);
        drive_[2].set_give_cmd(WHEEL_DIR[2] * wheel_[2].give_current);
        drive_[3].set_give_cmd(WHEEL_DIR[3] * wheel_[3].give_current);
    }

#if CHASSIS_DIAG_ENABLE
    void update_diag()
    {
        for (int i = 0; i < 4; ++i) {
            chassis_diag.theta_curr[i] = servo_[i].theta;
            chassis_diag.theta_set[i] = servo_[i].theta_set;
            chassis_diag.target_velocity[i] = wheel_[i].target_velocity;
            chassis_diag.velocity[i] = wheel_[i].velocity;
            chassis_diag.pre_current[i] = wheel_[i].pre_current;
            chassis_diag.anti[i] = wheel_[i].anti;
            chassis_diag.give_current[i] = wheel_[i].give_current;
            chassis_diag.drive_saturated[i] = wheel_[i].saturated;
            chassis_diag.drive_saturated_count[i] = drive_sat_count_[i];
        }
    }
    void log_drive_diag(uint32_t now_ms)
    {
        if ((now_ms - drive_sat_log_ms_) < CHASSIS_DRIVE_SAT_LOG_PERIOD_MS) {
            return;
        }
        drive_sat_log_ms_ = now_ms;
        LOGD("[CHASSIS][DRV][PRE] RB=%.1f RF=%.1f LF=%.1f LB=%.1f",
             static_cast<double>(wheel_[0].pre_current),
             static_cast<double>(wheel_[1].pre_current),
             static_cast<double>(wheel_[2].pre_current),
             static_cast<double>(wheel_[3].pre_current));
        LOGD("[CHASSIS][DRV][SAT] RB=%d(s%u/%lu) RF=%d(s%u/%lu) LF=%d(s%u/%lu) LB=%d(s%u/%lu)",
             static_cast<int>(wheel_[0].give_current),
             static_cast<unsigned>(wheel_[0].saturated),
             static_cast<unsigned long>(drive_sat_count_[0]),
             static_cast<int>(wheel_[1].give_current),
             static_cast<unsigned>(wheel_[1].saturated),
             static_cast<unsigned long>(drive_sat_count_[1]),
             static_cast<int>(wheel_[2].give_current),
             static_cast<unsigned>(wheel_[2].saturated),
             static_cast<unsigned long>(drive_sat_count_[2]),
             static_cast<int>(wheel_[3].give_current),
             static_cast<unsigned>(wheel_[3].saturated),
             static_cast<unsigned long>(drive_sat_count_[3]));
    }
#endif

    void zero_outputs_and_send()
    {
        for (int i = 0; i < 4; ++i) {
            servo_[i].give_current = 0;
            wheel_[i].give_current = 0;
            wheel_[i].saturated = 0u;
        }
        for (auto &m : steer_) m.set_give_cmd(0);
        for (auto &m : drive_) m.set_give_cmd(0);
    }
};

/* =============================== 浠诲姟涓讳綋 =============================== */
extern "C" void Start_Chassis_Task(void *argument)
{
    (void)argument;

    static ChassisController chassis;

    const TickType_t period   = pdMS_TO_TICKS(2);
    TickType_t last_wake      = xTaskGetTickCount();

    const bool dwt_ok         = dwt_try_enable();
    const uint32_t cpu_hz     = HAL_RCC_GetHCLKFreq();
    uint32_t last_cyccnt      = dwt_ok ? DWT->CYCCNT : 0;
    uint32_t last_ms          = HAL_GetTick();

    LOGI("Chassis task start. dwt=%d cpu=%luHz", dwt_ok?1:0, (unsigned long)cpu_hz);

    //const gimbal_feedback_t* gimbal;


    for (;;)
    {
        vTaskDelayUntil(&last_wake, period);

        chassis.tick_1kHz();

        // 鑻ヤ綘鎯崇敤 dt 鍋氬墠锟?鑷€傚簲锛屽彲鍦ㄦ璁＄畻 dt锛堝綋鍓嶉€昏緫鏈樉寮忎娇鐢級
        float dt = 0.001f;
        if (dwt_ok){
            uint32_t cy   = DWT->CYCCNT;
            uint32_t diff = cy - last_cyccnt;
            last_cyccnt   = cy;
            dt = (float)diff / (float)cpu_hz;
            if (dt < 0.0005f || dt > 0.003f) dt = 0.001f;
        } else {
            uint32_t ms_now = HAL_GetTick();
            dt = (ms_now == last_ms) ? 0.001f : (float)(ms_now - last_ms) * 0.001f;
            if (dt < 0.0005f || dt > 0.010f) dt = 0.001f;
            last_ms = ms_now;
        }

        (void)dt; // 鐩墠鏈敤锛屽彲鐢ㄤ簬鏈潵鐨勫墠锟?闄愬箙
    }
}
