#include "../Inc/Gimbal_behavior_Task.h"
#include "bsp_srn_log.h"
#include "Gimbal_Task.h"
#include "CONF_Gimbal_Task.h"
#include "Gimbal_Debug.h"
#include "RC_Control_Mode.h"
#include "Referee_Task.h"
#include "VT03_Gimbal_Mode.h"
#include "LED_Task.h"
#include "bsp_buzzer.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lib_remote_control.h"
#include "lib_adp_lk_mg8016e.h"
#include "lib_adp_dm_4310.h"
#include "lib_adp_dm_4340.h"
#include "NTFDCAN_Router.h"
#include "stm32h7xx_hal.h"
#include <cmath>

#ifndef GIMBAL_TEST_LOG_ONLY_J2J3_TX
#define GIMBAL_TEST_LOG_ONLY_J2J3_TX 1
#endif

#ifndef GIMBAL_TEST_J3_A8_TX_ONESHOT
#define GIMBAL_TEST_J3_A8_TX_ONESHOT 0
#endif

#ifndef GIMBAL_TEST_J23_POS_TX_THROTTLE_ENABLE
#define GIMBAL_TEST_J23_POS_TX_THROTTLE_ENABLE 0
#endif

#ifndef GIMBAL_TEST_J23_POS_TX_PERIOD_MS
#define GIMBAL_TEST_J23_POS_TX_PERIOD_MS 1000u
#endif

#ifndef GIMBAL_TEST_J23_DROP_A1_TX
#define GIMBAL_TEST_J23_DROP_A1_TX 1
#endif

#if GIMBAL_TEST_LOG_ONLY_J2J3_TX
#undef LOGI
#undef LOGW
#define LOGI(...) ((void)0)
#define LOGW(...) ((void)0)
#define GIMBAL_J23_TX_LOGI(...) ((void)0)
#else
#define GIMBAL_J23_TX_LOGI(fmt, ...) LOGI(fmt, ##__VA_ARGS__)
#endif

// Note: `gimbal_zero_force` is declared extern in the header and kept here
// to provide a single writable definition for other modules that may read
// this unified disable flag. Currently it may be unused in some modules,
// but it's intentionally left as an externally visible symbol.
volatile bool gimbal_zero_force = false;
// `frontFLAG` is referenced by other tasks (e.g., Chassis_Task/Gimbal_Task)
volatile bool frontFLAG = true;
// `last_v_ext_` appears unused in this file; commented out to avoid warnings
// static uint16_t last_v_ext_ = 0;
static gimbal_cmd_t g_cmd = {GIMBAL_RELATIVE_ANGLE, 0.0f, 0.0f, 0u};
RC_State rc_snap_{};

// Three LK8016E instances (J3/J2/J1)
static LK8016E lk_j3(GIMBAL_LK_J3_ID);
static LK8016E lk_j2(GIMBAL_LK_J2_ID);
static LK8016E lk_j1(GIMBAL_LK_J1_ID);

// Torque-compensation DM motors (J8/J9)
static DM4340 dm_j8(GIMBAL_DM_J8_ID, GIMBAL_DM_J8_FB_SID);
static DM4310 dm_j9(GIMBAL_DM_J9_ID, GIMBAL_DM4310_FB_SID);

extern "C" bool Gimbal_ReadDmJ4J7Snapshot(uint16_t raw_u16[4], int16_t angle_cdeg[4], uint8_t online[4]);

namespace {
volatile uint32_t g_joint_snap_guard = 0u;
gimbal_joint_snapshot_t g_joint_snap{};

struct PauseSnapshot {
    int16_t cmd_cdeg[9];
    uint8_t phase;
};
volatile uint32_t g_pause_guard = 0u;
PauseSnapshot g_pause_snap{};
volatile bool g_mapping_reset_request_pending = false;
volatile bool g_mapping_reset_local_done = false;
constexpr uint32_t GIMBAL_MAPPING_LOG_PERIOD_MS = 500u;

#if GIMBAL_LK_CAN_RAW_LOG_ENABLE
struct LkCanRawFrameDiag {
    uint16_t sid = 0u;
    uint8_t dlc = 0u;
    uint8_t cmd = 0u;
    uint8_t data[8] = {0};
    uint32_t tick_ms = 0u;
    uint32_t count = 0u;
    uint8_t valid = 0u;
};

struct LkCanRawTapState {
    volatile uint32_t guard = 0u;
    LkCanRawFrameDiag rx{};
    LkCanRawFrameDiag tx{};
};

struct LkCanRawTapSnapshot {
    LkCanRawFrameDiag rx{};
    LkCanRawFrameDiag tx{};
};

LkCanRawTapState g_lk_can_raw_j3{};
LkCanRawTapState g_lk_can_raw_j2{};

template <typename Fn>
inline void lk_can_raw_tap_mutate(LkCanRawTapState* state, Fn&& fn)
{
    if (state == nullptr) {
        return;
    }
    __DMB();
    state->guard++;
    __DMB();
    fn(*state);
    __DMB();
    state->guard++;
    __DMB();
}

inline bool lk_can_raw_tap_copy(const LkCanRawTapState* state, LkCanRawTapSnapshot* out)
{
    if (state == nullptr || out == nullptr) {
        return false;
    }
    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    do {
        seq_a = state->guard;
        __DMB();
        out->rx = state->rx;
        out->tx = state->tx;
        __DMB();
        seq_b = state->guard;
    } while (seq_a != seq_b);
    return true;
}

inline void lk_can_raw_frame_update(LkCanRawFrameDiag* frame,
                                    uint16_t sid,
                                    const uint8_t* data,
                                    uint8_t dlc,
                                    uint32_t now_ms)
{
    if (frame == nullptr || data == nullptr) {
        return;
    }
    const uint8_t copy_dlc = (dlc > 8u) ? 8u : dlc;
    frame->sid = sid;
    frame->dlc = copy_dlc;
    frame->cmd = (copy_dlc > 0u) ? data[0] : 0u;
    frame->tick_ms = now_ms;
    frame->valid = 1u;
    for (uint8_t i = 0u; i < 8u; ++i) {
        frame->data[i] = (i < copy_dlc) ? data[i] : 0u;
    }
    frame->count++;
}

inline void lk_can_raw_rx_tap_j3(uint16_t sid, const uint8_t* data, uint8_t dlc, void* user)
{
    (void)user;
    if (data == nullptr) {
        return;
    }
    lk_can_raw_tap_mutate(&g_lk_can_raw_j3, [&](LkCanRawTapState& state) {
        lk_can_raw_frame_update(&state.rx, sid, data, dlc, HAL_GetTick());
    });
}

inline void lk_can_raw_rx_tap_j2(uint16_t sid, const uint8_t* data, uint8_t dlc, void* user)
{
    (void)user;
    if (data == nullptr) {
        return;
    }
    lk_can_raw_tap_mutate(&g_lk_can_raw_j2, [&](LkCanRawTapState& state) {
        lk_can_raw_frame_update(&state.rx, sid, data, dlc, HAL_GetTick());
    });
}

inline void lk_can_raw_tx_tap(LkCanRawTapState* state,
                              uint16_t sid,
                              const uint8_t* data,
                              uint8_t dlc,
                              uint32_t now_ms)
{
    if (state == nullptr || data == nullptr) {
        return;
    }
    lk_can_raw_tap_mutate(state, [&](LkCanRawTapState& s) {
        lk_can_raw_frame_update(&s.tx, sid, data, dlc, now_ms);
    });
}

inline void lk_can_raw_log_axis(const char* axis,
                                uint8_t bus,
                                const LkCanRawTapSnapshot& snap,
                                uint32_t now_ms,
                                uint32_t* io_prev_rx_count)
{
    if (axis == nullptr || io_prev_rx_count == nullptr) {
        return;
    }

    const uint32_t rx_count = snap.rx.count;
    const uint32_t rx_delta = rx_count - *io_prev_rx_count;
    *io_prev_rx_count = rx_count;

    const uint32_t rx_age_ms = snap.rx.valid ? (now_ms - snap.rx.tick_ms) : 0xFFFFFFFFu;
    const uint32_t tx_age_ms = snap.tx.valid ? (now_ms - snap.tx.tick_ms) : 0xFFFFFFFFu;

    LOGI("[GIMBAL][LK][CANRAW][%s] bus=%u RX:sid=0x%03X dlc=%u cmd=0x%02X age=%lu cnt=%lu dcnt=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X | TX:sid=0x%03X dlc=%u cmd=0x%02X age=%lu cnt=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X",
         axis,
         static_cast<unsigned>(bus),
         static_cast<unsigned>(snap.rx.sid),
         static_cast<unsigned>(snap.rx.dlc),
         static_cast<unsigned>(snap.rx.cmd),
         static_cast<unsigned long>(rx_age_ms),
         static_cast<unsigned long>(snap.rx.count),
         static_cast<unsigned long>(rx_delta),
         static_cast<unsigned>(snap.rx.data[0]),
         static_cast<unsigned>(snap.rx.data[1]),
         static_cast<unsigned>(snap.rx.data[2]),
         static_cast<unsigned>(snap.rx.data[3]),
         static_cast<unsigned>(snap.rx.data[4]),
         static_cast<unsigned>(snap.rx.data[5]),
         static_cast<unsigned>(snap.rx.data[6]),
         static_cast<unsigned>(snap.rx.data[7]),
         static_cast<unsigned>(snap.tx.sid),
         static_cast<unsigned>(snap.tx.dlc),
         static_cast<unsigned>(snap.tx.cmd),
         static_cast<unsigned long>(tx_age_ms),
         static_cast<unsigned long>(snap.tx.count),
         static_cast<unsigned>(snap.tx.data[0]),
         static_cast<unsigned>(snap.tx.data[1]),
         static_cast<unsigned>(snap.tx.data[2]),
         static_cast<unsigned>(snap.tx.data[3]),
         static_cast<unsigned>(snap.tx.data[4]),
         static_cast<unsigned>(snap.tx.data[5]),
         static_cast<unsigned>(snap.tx.data[6]),
         static_cast<unsigned>(snap.tx.data[7]));
}
#endif

inline uint16_t dm4340_pos_raw_to_u16(int16_t raw)
{
    return static_cast<uint16_t>(static_cast<int32_t>(raw) + 32768);
}

inline void gimbal_send_dm_enable(uint8_t bus, uint8_t id, bool enable)
{
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, static_cast<uint8_t>(enable ? 0xFC : 0xFD)};
    const uint16_t sid = static_cast<uint16_t>(0x000u + id);
    (void)cantx_send_now(bus, sid, data, 8, 200);
}

inline bool lk_get_cross_zero_high_raw_to_zero_threshold(const GimbalLkPosSingleTurnCfg& cfg,
                                                          uint16_t* out_threshold_raw)
{
    if (out_threshold_raw == nullptr) {
        return false;
    }
    if (&cfg == &GIMBAL_LK_J2_POS_CFG) {
        *out_threshold_raw = GIMBAL_LK_J2_CROSS_ZERO_HIGH_RAW_TO_ZERO_THRESHOLD;
        return true;
    }
    if (&cfg == &GIMBAL_LK_J3_POS_CFG) {
        *out_threshold_raw = GIMBAL_LK_J3_CROSS_ZERO_HIGH_RAW_TO_ZERO_THRESHOLD;
        return true;
    }
    return false;
}

inline uint16_t lk_normalize_encoder_raw_cross_zero_high_to_zero(uint16_t encoder_raw,
                                                                  const GimbalLkPosSingleTurnCfg& cfg)
{
    uint16_t threshold_raw = 0u;
    if (!lk_get_cross_zero_high_raw_to_zero_threshold(cfg, &threshold_raw)) {
        return encoder_raw;
    }
    if (!cfg.forbid_zero_crossing || cfg.abs_limit_min_raw != 0u) {
        return encoder_raw;
    }
    return (encoder_raw >= threshold_raw) ? 0u : encoder_raw;
}

enum class LkTargetSource : uint8_t {
    None = 0u,
    Rc,
    Link,
    External,
    System,
};

struct LkTargetRequest {
    bool active = false;
    bool hold_target = false;
    bool use_absolute = false;
    fp32 absolute_rel_deg = 0.0f;
    fp32 delta_rel_deg = 0.0f;
    LkTargetSource source = LkTargetSource::None;
};

struct LkTargetState {
    bool initialized = false;
    fp32 target_rel_deg = 0.0f;
    bool source_active = false;
    bool hold_target = false;
    LkTargetSource source = LkTargetSource::None;
};

inline const char* lk_target_source_name(LkTargetSource source)
{
    switch (source) {
        case LkTargetSource::Rc:       return "rc";
        case LkTargetSource::Link:     return "link";
        case LkTargetSource::External: return "ext";
        case LkTargetSource::System:   return "sys";
        default:                       return "none";
    }
}

enum class LkExecMode : uint8_t {
    None = 0u,
    A4,
    A6,
    A8,
};

struct LkExecDiag {
    LkExecMode mode = LkExecMode::None;
    fp32 target_rel_deg = 0.0f;
    fp32 target_abs_deg = 0.0f;
    fp32 proto_target_deg = 0.0f;
    fp32 delta_rel_deg = 0.0f;
    fp32 current_rel_deg = 0.0f;
    fp32 current_abs_deg = 0.0f;
    fp32 cw_delta_deg = 0.0f;
    fp32 ccw_delta_deg = 0.0f;
    fp32 err_deg = 0.0f;
    fp32 speed_dps = 0.0f;
    fp32 max_dps = 0.0f;
    fp32 feedback_iq_A = 0.0f;
    uint16_t current_raw = 0u;
    uint16_t target_raw = 0u;
    uint16_t raw_limit_min = 0u;
    uint16_t raw_limit_max = 0u;
    uint8_t feedback_fresh = 0u;
    uint8_t raw_increase_is_ccw = 0u;
    uint8_t desired_dir = 0u;
    uint8_t dir = 0u;
    uint8_t dir_invert = 0u;
    uint8_t a6_hold = 0u;
    uint8_t cross_raw0_blocked = 0u;
    int32_t raw_cmd = 0;
};

struct LkA6HoldState {
    bool initialized = false;
    bool dir_latched = false;
    bool hold_latched = false;
    fp32 hold_target_abs_deg = 0.0f;
};

struct LkA6DirectionDecision {
    bool valid = false;
    bool ccw = false;
    fp32 signed_delta_deg = 0.0f;
    fp32 cw_delta_deg = 0.0f;
    fp32 ccw_delta_deg = 0.0f;
    fp32 solved_current_abs_deg = 0.0f;
    fp32 solved_target_abs_deg = 0.0f;
    fp32 cmd_target_deg = 0.0f;
    uint16_t solved_current_raw = 0u;
    uint16_t solved_target_raw = 0u;
    uint8_t cross_raw0_blocked = 0u;
};

inline const char* lk_exec_mode_name(LkExecMode mode)
{
    switch (mode) {
        case LkExecMode::A4: return "A4";
        case LkExecMode::A6: return "A6";
        case LkExecMode::A8: return "A8";
        default:             return "safe";
    }
}
} // namespace

extern "C" bool Gimbal_ReadJointSnapshot(gimbal_joint_snapshot_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    gimbal_joint_snapshot_t local{};
    do {
        seq_a = g_joint_snap_guard;
        __DMB();
        local = g_joint_snap;
        __DMB();
        seq_b = g_joint_snap_guard;
    } while (seq_a != seq_b);

    *out = local;
    return true;
}

extern "C" bool Gimbal_ReadPauseCmdCdeg(int16_t out_cdeg[9], uint8_t* phase)
{
    if (out_cdeg == nullptr || phase == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    PauseSnapshot local{};
    do {
        seq_a = g_pause_guard;
        __DMB();
        local = g_pause_snap;
        __DMB();
        seq_b = g_pause_guard;
    } while (seq_a != seq_b);

    for (int i = 0; i < 9; ++i) {
        out_cdeg[i] = local.cmd_cdeg[i];
    }
    *phase = local.phase;
    return true;
}

extern "C" void Gimbal_RequestMappingReset(void)
{
    g_mapping_reset_request_pending = true;
    g_mapping_reset_local_done = false;
}

extern "C" bool Gimbal_IsMappingResetLocalDone(void)
{
    return g_mapping_reset_local_done;
}


extern "C" void Start_Gimbal_behave(void *argument)
{
    (void)argument;

    // Register LK motors to CAN Rx Router
    canrx_register_lk8016e(GIMBAL_LK_J3_BUS, &lk_j3);
    canrx_register_lk8016e(GIMBAL_LK_J2_BUS, &lk_j2);
    canrx_register_lk8016e(GIMBAL_LK_J1_BUS, &lk_j1);
#if GIMBAL_LK_CAN_RAW_LOG_ENABLE
    const int canraw_sub_j3_ret = canrx_subscribe_raw(GIMBAL_LK_J3_BUS, lk_j3.rxId(), lk_can_raw_rx_tap_j3, nullptr);
    const int canraw_sub_j2_ret = canrx_subscribe_raw(GIMBAL_LK_J2_BUS, lk_j2.rxId(), lk_can_raw_rx_tap_j2, nullptr);
    if (canraw_sub_j3_ret != 0 || canraw_sub_j2_ret != 0) {
        LOGW("[GIMBAL][LK][CANRAW] subscribe failed j3=%d j2=%d sid_j3=0x%03X sid_j2=0x%03X",
             canraw_sub_j3_ret,
             canraw_sub_j2_ret,
             static_cast<unsigned>(lk_j3.rxId()),
             static_cast<unsigned>(lk_j2.rxId()));
    }
#endif
    lk_j3.set_torque_current_max_A(GIMBAL_LK_TORQUE_CURRENT_MAX_A);
    lk_j2.set_torque_current_max_A(GIMBAL_LK_TORQUE_CURRENT_MAX_A);
    lk_j1.set_torque_current_max_A(GIMBAL_LK_TORQUE_CURRENT_MAX_A);
    canrx_register_dm4340(GIMBAL_DM_J8_BUS, &dm_j8);
    canrx_register_dm4310(GIMBAL_DM_J9_BUS, &dm_j9);
    dm_j8.set_mode(DM4340Mode::MIT);
    dm_j9.set_mode(DM4310Mode::MIT);
    dm_j8.set_mit_limits(GIMBAL_DM_J8_PMAX_RAD, GIMBAL_DM_J8_VMAX_RAD_S, GIMBAL_DM_J8_TMAX_NM);

    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last = xTaskGetTickCount();
    for(;;){
        vTaskDelayUntil(&last, period);

        RC_State rc; RC_GetSnapshot(&rc);
        RC_Status st = RC_GetStatus();
        const RcControlMode ctl = VT03_Gimbal_ResolveControlMode(rc, st);
        const uint32_t now_ms = HAL_GetTick();

        static gimbal_joint_snapshot_t last_joint_snap{};
        static gimbal_pause_phase_e pause_phase = GIMBAL_PAUSE_IDLE;
        static int16_t pause_start_cdeg[9] = {0};
        static int16_t pause_cmd_cdeg[9] = {0};
        static bool pause_btn_prev = false;
        static bool pause_beep_on = false;
        static uint8_t pause_beep_count = 0;
        static uint32_t pause_beep_next_ms = 0;
        static uint32_t pause_run_start_ms = 0;
        static bool pause_exit_beep_active = false;
        static uint32_t pause_exit_beep_end_ms = 0u;
        static bool pause_abort_sent = false;
        static bool pause_led_flash_on = false;
        static bool mapping_reset_run = false;
        static uint32_t mapping_reset_start_ms = 0u;
        static uint32_t mapping_log_last_ms = 0u;
        bool mapping_hold = Referee_IsMappingResetActive();
        const bool pause_btn_pressed =
            (st.active_src == RC_SRC_VT03) &&
            ((rc.key.v_ext & KEY_EXT_VT03_PAUSE) != 0u);
        static bool pause_exit_wait_release = false;
        const bool pause_btn_edge = pause_btn_pressed && !pause_btn_prev;
        const bool pause_request_allowed =
            !mapping_hold &&
            !mapping_reset_run &&
            !g_mapping_reset_request_pending &&
            (pause_phase == GIMBAL_PAUSE_IDLE);
        const bool pause_enter_edge = pause_btn_edge && pause_request_allowed;
        const bool pause_exit_press_edge =
            pause_btn_edge &&
            !pause_enter_edge &&
            ((pause_phase != GIMBAL_PAUSE_IDLE) ||
             mapping_reset_run ||
             mapping_hold ||
             g_mapping_reset_request_pending);
        pause_btn_prev = pause_btn_pressed;
        if (pause_enter_edge) {
            pause_exit_wait_release = false;
        }
        if (pause_exit_press_edge && !pause_exit_wait_release) {
            pause_exit_wait_release = true;
            LOGI("[PAUSE][EXIT] second_press_detected wait_release=1 phase=%u hold=%u run=%u pending=%u local_done=%u",
                 static_cast<unsigned>(pause_phase),
                 mapping_hold ? 1u : 0u,
                 mapping_reset_run ? 1u : 0u,
                 g_mapping_reset_request_pending ? 1u : 0u,
                 g_mapping_reset_local_done ? 1u : 0u);
        }
        const bool pause_exit_release_edge = pause_exit_wait_release && !pause_btn_pressed;

        const auto lerp_cdeg = [](int16_t a, int16_t b, fp32 alpha) -> int16_t {
            fp32 v = static_cast<fp32>(a) + (static_cast<fp32>(b - a) * alpha);
            return static_cast<int16_t>(std::lround(v));
        };
        const auto pause_seq_cdeg = [&](uint32_t elapsed_ms, int16_t out_cdeg[9]) {
            for (int i = 0; i < 9; ++i) {
                out_cdeg[i] = pause_start_cdeg[i];
            }
            if (GIMBAL_PAUSE_RUN_MS == 0u) {
                for (int i = 0; i < 9; ++i) {
                    out_cdeg[i] = GIMBAL_PAUSE_TARGET_CDEG[i];
                }
                return;
            }
            const uint32_t span_ms = GIMBAL_PAUSE_RUN_MS;
            const fp32 alpha = static_cast<fp32>(elapsed_ms) / static_cast<fp32>(span_ms);
            for (int idx = 0; idx < 9; ++idx) {
                out_cdeg[idx] = lerp_cdeg(pause_start_cdeg[idx], GIMBAL_PAUSE_TARGET_CDEG[idx], alpha);
            }
        };
        const auto pause_lk_axes_near_zero = [&](const gimbal_joint_snapshot_t& snap) -> bool {
            constexpr int16_t LK_ZERO_ENTRY_DB_CDEG = 100;
            if (snap.online[0] == 0u || snap.online[1] == 0u || snap.online[2] == 0u) {
                return false;
            }
            return (std::abs(static_cast<int>(snap.angle_cdeg[0])) <= LK_ZERO_ENTRY_DB_CDEG) &&
                   (std::abs(static_cast<int>(snap.angle_cdeg[1])) <= LK_ZERO_ENTRY_DB_CDEG) &&
                   (std::abs(static_cast<int>(snap.angle_cdeg[2])) <= LK_ZERO_ENTRY_DB_CDEG);
        };
        const auto pause_exit_cleanup = [&]() {
            pause_phase = GIMBAL_PAUSE_IDLE;
            pause_beep_on = false;
            pause_beep_count = 0u;
            pause_beep_next_ms = 0u;
            pause_run_start_ms = 0u;
            pause_exit_beep_active = false;
            pause_exit_beep_end_ms = 0u;
            pause_abort_sent = false;
            mapping_reset_run = false;
            mapping_reset_start_ms = 0u;
            g_mapping_reset_request_pending = false;
            g_mapping_reset_local_done = false;
            gimbal_zero_force = false;
            pause_exit_wait_release = false;
            Buzzer_Stop();
            LED_SetPauseFlash(0u, GIMBAL_PAUSE_LED_FLASH_PERIOD_MS);
            pause_led_flash_on = false;
            Referee_ClearMappingReset();
            mapping_hold = Referee_IsMappingResetActive();
        };
        if (pause_exit_release_edge) {
            const bool hold_before = mapping_hold;
            const bool pending_before = g_mapping_reset_request_pending;
            const bool local_done_before = g_mapping_reset_local_done;
            const uint8_t phase_before = static_cast<uint8_t>(pause_phase);
            pause_exit_cleanup();
            Referee_RequestRemoteBuzzerPulses(1u);
            Buzzer_Start(GIMBAL_PAUSE_BEEP_HZ);
            pause_exit_beep_active = true;
            pause_exit_beep_end_ms = now_ms + GIMBAL_PAUSE_DONE_BEEP_MS;
            LOGI("[PAUSE][EXIT] release_exit=1 phase=%u->%u hold=%u->%u pending=%u->%u local_done=%u->%u",
                 static_cast<unsigned>(phase_before),
                 static_cast<unsigned>(pause_phase),
                 hold_before ? 1u : 0u,
                 mapping_hold ? 1u : 0u,
                 pending_before ? 1u : 0u,
                 g_mapping_reset_request_pending ? 1u : 0u,
                 local_done_before ? 1u : 0u,
                 g_mapping_reset_local_done ? 1u : 0u);
        }

        if (pause_phase == GIMBAL_PAUSE_IDLE) {
            pause_abort_sent = false;
            if (pause_enter_edge) {
                for (int i = 0; i < 9; ++i) {
                    pause_start_cdeg[i] = last_joint_snap.angle_cdeg[i];
                    pause_cmd_cdeg[i] = pause_start_cdeg[i];
                }
                gimbal_zero_force = false;
                pause_phase = GIMBAL_PAUSE_PRE_BEEP;
                pause_beep_on = true;
                pause_beep_count = 0u;
                pause_beep_next_ms = now_ms + GIMBAL_PAUSE_BEEP_ON_MS;
                pause_run_start_ms = now_ms;
                pause_exit_beep_active = false;
                pause_exit_beep_end_ms = 0u;
                mapping_reset_run = false;
                mapping_reset_start_ms = 0u;
                g_mapping_reset_request_pending = false;
                g_mapping_reset_local_done = false;
                pause_exit_wait_release = false;
                Buzzer_Start(GIMBAL_PAUSE_BEEP_HZ);
                Referee_RequestMappingReset();
                Referee_RequestRemoteBuzzerPulses(3u);
                LOGI("[PAUSE][ENTER] phase=pre_beep buzzer_local=3 remote_pulses=3");
            }
        } else if (pause_phase == GIMBAL_PAUSE_PRE_BEEP) {
            for (int i = 0; i < 9; ++i) {
                pause_cmd_cdeg[i] = pause_start_cdeg[i];
            }
            if ((int32_t)(now_ms - pause_beep_next_ms) >= 0) {
                if (pause_beep_on) {
                    Buzzer_Stop();
                    pause_beep_on = false;
                    pause_beep_count++;
                    if (pause_beep_count >= GIMBAL_PAUSE_BEEP_COUNT) {
                        pause_phase = GIMBAL_PAUSE_RUN;
                        pause_run_start_ms = now_ms;
                    } else {
                        pause_beep_next_ms = now_ms + GIMBAL_PAUSE_BEEP_OFF_MS;
                    }
                } else {
                    Buzzer_Start(GIMBAL_PAUSE_BEEP_HZ);
                    pause_beep_on = true;
                    pause_beep_next_ms = now_ms + GIMBAL_PAUSE_BEEP_ON_MS;
                }
            }
        } else if (pause_phase == GIMBAL_PAUSE_RUN) {
            uint32_t elapsed = now_ms - pause_run_start_ms;
            if (elapsed >= GIMBAL_PAUSE_RUN_MS) {
                elapsed = GIMBAL_PAUSE_RUN_MS;
            }
            pause_seq_cdeg(elapsed, pause_cmd_cdeg);
            if (elapsed >= GIMBAL_PAUSE_RUN_MS) {
                for (int i = 0; i < 9; ++i) {
                    pause_cmd_cdeg[i] = GIMBAL_PAUSE_TARGET_CDEG[i];
                }
                if (pause_lk_axes_near_zero(last_joint_snap)) {
                    pause_phase = GIMBAL_PAUSE_DONE_BEEP;
                    g_mapping_reset_local_done = true;
                    LOGI("[PAUSE][HOLD] reached_target=1 mapping_local_done=1");
                }
            }
        } else if (pause_phase == GIMBAL_PAUSE_DONE_BEEP) {
            for (int i = 0; i < 9; ++i) {
                pause_cmd_cdeg[i] = GIMBAL_PAUSE_TARGET_CDEG[i];
            }
        } else if (pause_phase == GIMBAL_PAUSE_ABORT) {
            if (!pause_abort_sent) {
                pause_abort_sent = true;
                gimbal_zero_force = true;
                Buzzer_Stop();
                lk_j3.request_disable();
                lk_j2.request_disable();
                lk_j1.request_disable();
                gimbal_send_dm_enable(GIMBAL_DM_J8_BUS, static_cast<uint8_t>(GIMBAL_DM_J8_ID), false);
                gimbal_send_dm_enable(GIMBAL_DM_J9_BUS, static_cast<uint8_t>(GIMBAL_DM_J9_ID), false);
            }
            if (!pause_btn_pressed) {
                pause_phase = GIMBAL_PAUSE_IDLE;
                gimbal_zero_force = false;
                pause_abort_sent = false;
                pause_exit_wait_release = false;
            }
        }

        if (pause_exit_beep_active &&
            (pause_phase == GIMBAL_PAUSE_IDLE) &&
            ((int32_t)(now_ms - pause_exit_beep_end_ms) >= 0)) {
            Buzzer_Stop();
            pause_exit_beep_active = false;
            pause_exit_beep_end_ms = 0u;
        }

        if (!mapping_hold && !mapping_reset_run && g_mapping_reset_local_done) {
            g_mapping_reset_local_done = false;
        }
        if (!mapping_reset_run && g_mapping_reset_request_pending && (pause_phase == GIMBAL_PAUSE_IDLE)) {
            for (int i = 0; i < 9; ++i) {
                pause_start_cdeg[i] = last_joint_snap.angle_cdeg[i];
                pause_cmd_cdeg[i] = pause_start_cdeg[i];
            }
            gimbal_zero_force = false;
            mapping_reset_start_ms = now_ms;
            mapping_reset_run = true;
            g_mapping_reset_request_pending = false;
            g_mapping_reset_local_done = false;
            LOGI("[MAP][RESET][LOCAL] start");
        }
        if (mapping_reset_run) {
            uint32_t elapsed = now_ms - mapping_reset_start_ms;
            if (elapsed >= GIMBAL_PAUSE_RUN_MS) {
                elapsed = GIMBAL_PAUSE_RUN_MS;
                for (int i = 0; i < 9; ++i) {
                    pause_cmd_cdeg[i] = GIMBAL_PAUSE_TARGET_CDEG[i];
                }
                if (pause_lk_axes_near_zero(last_joint_snap)) {
                    mapping_reset_run = false;
                    g_mapping_reset_local_done = true;
                    LOGI("[MAP][RESET][LOCAL] done");
                }
            } else {
                pause_seq_cdeg(elapsed, pause_cmd_cdeg);
            }
        } else if (mapping_hold && g_mapping_reset_local_done) {
            for (int i = 0; i < 9; ++i) {
                pause_cmd_cdeg[i] = GIMBAL_PAUSE_TARGET_CDEG[i];
            }
        }

        const bool pause_active =
            (pause_phase == GIMBAL_PAUSE_PRE_BEEP ||
             pause_phase == GIMBAL_PAUSE_RUN ||
             pause_phase == GIMBAL_PAUSE_DONE_BEEP) ||
            mapping_reset_run ||
            (mapping_hold && g_mapping_reset_local_done);
        if (pause_active && !pause_led_flash_on) {
            LED_SetPauseFlash(1u, GIMBAL_PAUSE_LED_FLASH_PERIOD_MS);
            pause_led_flash_on = true;
        } else if (!pause_active && pause_led_flash_on) {
            LED_SetPauseFlash(0u, GIMBAL_PAUSE_LED_FLASH_PERIOD_MS);
            pause_led_flash_on = false;
        }
        const bool pause_abort = (pause_phase == GIMBAL_PAUSE_ABORT);
        referee_joint_raw_cmd_t ext_cmd{};
        (void)Referee_ReadJointRawCmd(&ext_cmd);
        const bool ext_cmd_fresh = ext_cmd.online && ext_cmd.fresh;
        const bool ext_j1_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 1u);
        const bool ext_j2_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 2u);
        const bool ext_j3_valid = ext_cmd_fresh && Omnix_JointRawValid(ext_cmd.valid_mask, 3u);
        const bool remote_safe_force = (ctl.main_mode == RcMainMode::ZeroForce);
        static bool prev_remote_safe_force = false;
        const bool gear_exit_hold = (!remote_safe_force && prev_remote_safe_force);
        const bool lk_force_disable = remote_safe_force || pause_abort || gimbal_zero_force;
        const bool lk_external_safe_block = pause_active || lk_force_disable;
        static bool lk_prev_external_safe_block = false;
        static bool lk_wait_external_fresh_after_safe_exit = false;
        static uint16_t lk_external_rearm_seq = 0u;
        if (!lk_external_safe_block && lk_prev_external_safe_block) {
            lk_wait_external_fresh_after_safe_exit = true;
            lk_external_rearm_seq = ext_cmd.seq;
        }
        if (lk_wait_external_fresh_after_safe_exit &&
            ext_cmd_fresh &&
            (ext_cmd.seq != lk_external_rearm_seq)) {
            lk_wait_external_fresh_after_safe_exit = false;
        }
        lk_prev_external_safe_block = lk_external_safe_block;
        const bool ext_j1_apply_valid =
            ext_j1_valid && !lk_external_safe_block && !lk_wait_external_fresh_after_safe_exit;
        const bool ext_j2_apply_valid =
            ext_j2_valid && !lk_external_safe_block && !lk_wait_external_fresh_after_safe_exit;
        const bool ext_j3_apply_valid =
            ext_j3_valid && !lk_external_safe_block && !lk_wait_external_fresh_after_safe_exit;
        const bool ext_lk_apply_any = ext_j1_apply_valid || ext_j2_apply_valid || ext_j3_apply_valid;
        const uint16_t ext_j1_protocol_raw = ext_cmd.raw_u16[0];
        const uint16_t ext_j2_protocol_raw = ext_cmd.raw_u16[1];
        const uint16_t ext_j3_protocol_raw = ext_cmd.raw_u16[2];
        const auto log_ext_calib_warn = [](const char* axis,
                                           const GimbalExternalBidirRelAngleMapCfg& cfg,
                                           bool* logged) {
            if (logged == nullptr || *logged) {
                return;
            }
            const int32_t neg_span_raw = Omnix_ControllerNegativeSpanRaw(&cfg.controller_calib);
            const int32_t pos_span_raw = Omnix_ControllerPositiveSpanRaw(&cfg.controller_calib);
            if (neg_span_raw > static_cast<int32_t>(OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW) &&
                pos_span_raw > static_cast<int32_t>(OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW)) {
                return;
            }
            LOGW("[GIMBAL][LK][EXTCAL][%s] degenerate=1 min=%u zero=%u max=%u neg=%ld pos=%ld warn=%u",
                 axis,
                 static_cast<unsigned>(cfg.controller_calib.raw_min),
                 static_cast<unsigned>(cfg.controller_calib.raw_zero),
                 static_cast<unsigned>(cfg.controller_calib.raw_max),
                 static_cast<long>(neg_span_raw),
                 static_cast<long>(pos_span_raw),
                 static_cast<unsigned>(OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW));
            *logged = true;
        };
        static bool ext_j1_calib_warn_logged = false;
        static bool ext_j2_calib_warn_logged = false;
        static bool ext_j3_calib_warn_logged = false;
        log_ext_calib_warn("J1", GIMBAL_J1_EXT_BIDIR_REL_ANGLE_MAP_CFG, &ext_j1_calib_warn_logged);
        log_ext_calib_warn("J2", GIMBAL_J2_EXT_BIDIR_REL_ANGLE_MAP_CFG, &ext_j2_calib_warn_logged);
        log_ext_calib_warn("J3", GIMBAL_J3_EXT_BIDIR_REL_ANGLE_MAP_CFG, &ext_j3_calib_warn_logged);
        const GimbalExternalBidirRelAngleMapResult ext_j1_map =
            Gimbal_MapExternalRawToBidirRelAngle(ext_j1_protocol_raw, GIMBAL_J1_EXT_BIDIR_REL_ANGLE_MAP_CFG);
        const GimbalExternalBidirRelAngleMapResult ext_j2_map =
            Gimbal_MapExternalRawToHalfRangeReverseRelAngle(
                ext_j2_protocol_raw,
                GIMBAL_J2_EXT_HALF_RANGE_REVERSE_MAP_CFG);
        const GimbalExternalBidirRelAngleMapResult ext_j3_map =
            Gimbal_MapExternalRawToHalfRangeReverseRelAngle(
                ext_j3_protocol_raw,
                GIMBAL_J3_EXT_HALF_RANGE_REVERSE_MAP_CFG);
        const fp32 ext_j1_target_rel_deg =
            Gimbal_LkClampRelDeg(
                (ext_j1_map.hold_last_target ? 0.0f : (ext_j1_map.target_rel_rad * 57.2957795130823208768f)),
                GIMBAL_LK_J1_POS_CFG);
        const fp32 ext_j2_target_rel_deg =
            Gimbal_LkClampRelDeg(
                ext_j2_map.target_rel_rad * 57.2957795130823208768f,
                GIMBAL_LK_J2_POS_CFG);
        const fp32 ext_j3_target_rel_deg =
            Gimbal_LkClampRelDeg(
                ext_j3_map.target_rel_rad * 57.2957795130823208768f,
                GIMBAL_LK_J3_POS_CFG);
        const bool ext_j1_update_target = ext_j1_apply_valid && !ext_j1_map.hold_last_target;
        const bool ext_j2_update_target = ext_j2_apply_valid && !ext_j2_map.hold_last_target;
        const bool ext_j3_update_target = ext_j3_apply_valid && !ext_j3_map.hold_last_target;
        const auto rc_switch_match = [](uint8_t v, GimbalRcSwitchPos pos) -> bool {
            switch (pos) {
                case GimbalRcSwitchPos::Up:   return switch_is_up(v);
                case GimbalRcSwitchPos::Mid:  return switch_is_mid(v);
                case GimbalRcSwitchPos::Down: return switch_is_down(v);
                default: return false;
            }
        };
        const bool s0_up = (ctl.main_mode == RcMainMode::Gimbal);
        const bool s0_down = (ctl.main_mode == RcMainMode::ZeroForce);
        const bool s0_mid = (ctl.main_mode == RcMainMode::Chassis);
        const bool s1_up = (ctl.gimbal_mode == RcGimbalMode::DM);
        const bool s1_mid = (ctl.gimbal_mode == RcGimbalMode::LK);
        const bool s1_down = (ctl.gimbal_mode == RcGimbalMode::Link);
        const bool dm_j5j6_enable = (s0_up && (s1_up || s1_mid));
        const bool lk_ctrl_link = (s0_up && s1_down);
        const bool zero_log_enable =
            (ctl.active_src == RC_SRC_DR16) &&
            rc_switch_match(rc.rc.s[GIMBAL_RC_S0_INDEX], GIMBAL_RC_ZERO_LOG_S0) &&
            rc_switch_match(rc.rc.s[GIMBAL_RC_S1_INDEX], GIMBAL_RC_ZERO_LOG_S1);

        // Keep LK driver online state updated by ticking each instance
        lk_j3.tick(now_ms);
        lk_j2.tick(now_ms);
        lk_j1.tick(now_ms);
        dm_j8.tick(now_ms);
        dm_j9.tick(now_ms);

        static bool lk_j3_cal_logged = false;
        static bool lk_j2_cal_logged = false;
        static bool lk_j1_cal_logged = false;
        const auto log_lk_cal = [](const char* name, const LK8016EState& s, bool* logged) {
            if (*logged || !s.online) {
                return;
            }
            LOGI("[CAL][LK] %s raw=%u deg=%.2f",
                 name,
                 static_cast<unsigned>(s.encoder_raw),
                 static_cast<double>(LK8016E_EncoderRawToDeg(s.encoder_raw)));
            *logged = true;
        };
        {
            const auto s1 = lk_j3.state();
            const auto s2 = lk_j2.state();
            const auto s3 = lk_j1.state();
            log_lk_cal("J3", s1, &lk_j3_cal_logged);
            log_lk_cal("J2", s2, &lk_j2_cal_logged);
            log_lk_cal("J1", s3, &lk_j1_cal_logged);
        }

        // Periodic debug: every 100ms print LK status (online, last_cmd, rx_count)
        static int lk_dbg_cnt = 0;
        if (++lk_dbg_cnt >= 50) { // 2ms * 50 = 100ms
            lk_dbg_cnt = 0;
            {
                auto s1 = lk_j3.state();
                auto s2 = lk_j2.state();
                auto s3 = lk_j1.state();

            }
        }
        if (zero_log_enable) {
            static uint32_t zero_log_last_ms = 0;
            const uint32_t now_ms = HAL_GetTick();
            if (now_ms - zero_log_last_ms >= GIMBAL_ZERO_LOG_PERIOD_MS) {
                zero_log_last_ms = now_ms;
                const auto s1 = lk_j3.state();
                const auto s2 = lk_j2.state();
                const auto s3 = lk_j1.state();
                const fp32 j3_deg = static_cast<fp32>(s1.encoder_raw) * (360.0f / 65536.0f);
                const fp32 j2_deg = static_cast<fp32>(s2.encoder_raw) * (360.0f / 65536.0f);
                const fp32 j1_deg = static_cast<fp32>(s3.encoder_raw) * (360.0f / 65536.0f);
                LOGI("Zero log LK: J3=%.2f J2=%.2f J1=%.2f (deg)", 
                     static_cast<double>(j3_deg),
                     static_cast<double>(j2_deg),
                     static_cast<double>(j1_deg));
            }
        }

        // Always operate in relative-angle (angle control) mode.
        gimbal_cmd_t local;
        local.behaviour = GIMBAL_RELATIVE_ANGLE;
        local.reset_zero_axis = 0;

        // Map channels: ch3 -> DM J5, ch2 -> DM J6, ch0 -> DM J7
        // Note: When user switches to s1_mid (manual angle mode), a different
        // mapping is used (ch0->J3, ch1->J2, ch2->J1). Keep this mapping
        // distinction in mind when operating the remote.
        // ch range assumed similar to elsewhere (~-660..660). Scale to +/-90deg.
        constexpr float SCALE = (3.14159265358979323846f / 2.0f) / 660.0f; // rad per ch unit
        int ch_yaw   = static_cast<int>(rc.rc.ch[GIMBAL_DM_J5_RC_CH])  * static_cast<int>(GIMBAL_DM_J5_RC_SIGN);
        int ch_pitch = static_cast<int>(rc.rc.ch[GIMBAL_DM_J6_RC_CH])  * static_cast<int>(GIMBAL_DM_J6_RC_SIGN);
        int ch_jaw   = static_cast<int>(rc.rc.ch[GIMBAL_DM_J7_RC_CH])  * static_cast<int>(GIMBAL_DM_J7_RC_SIGN);
        int ch_roll  = static_cast<int>(rc.rc.ch[GIMBAL_DM_J4_RC_CH]) * static_cast<int>(GIMBAL_DM_J4_RC_SIGN);
        int ch_j5_link = static_cast<int>(rc.rc.ch[GIMBAL_LINK_J5_RC_CH]) * static_cast<int>(GIMBAL_LINK_J5_RC_SIGN);

        // Apply deadband for channels (keep RC deadband; per-axis configured in CONF)
        const int dead_yaw  = static_cast<int>(GIMBAL_DM_J5_DEADBAND);
        const int dead_pit  = static_cast<int>(GIMBAL_DM_J6_DEADBAND);
        const int dead_jaw  = static_cast<int>(GIMBAL_DM_J7_DEADBAND);
        const int dead_roll = static_cast<int>(GIMBAL_DM_J4_DEADBAND);
        if (ch_yaw > -dead_yaw && ch_yaw < dead_yaw) ch_yaw = 0;
        if (ch_pitch > -dead_pit && ch_pitch < dead_pit) ch_pitch = 0;
        if (ch_jaw > -dead_jaw && ch_jaw < dead_jaw) ch_jaw = 0;
        if (ch_roll > -dead_roll && ch_roll < dead_roll) ch_roll = 0;
        if (ch_j5_link > -dead_yaw && ch_j5_link < dead_yaw) ch_j5_link = 0;

        // 鏉╂劘顢戦柅鏄忕帆閿涘牓鍣哥€规矮绠熼敍?
        // - s[0]=UP 娑?s[1]=UP閿涙碍甯堕崚鎯版彧婵℃瑧鏁搁張鐚寸礄DM 娴滄垵褰撮敍?
        // - s[1]=MID閿涙碍甯堕崚?LK 閻㈠灚婧€閿涘牊婧€濮婃媽鍣﹂敍?
        // - s[1]=DOWN閿涙岸鍏樻稉宥嗗付閸掕绱欐穱婵囧瘮閸樼喍缍呯純顕嗙礆
        if (dm_j5j6_enable) {
            local.yaw_set_rad = static_cast<fp32>(ch_yaw) * SCALE;
            local.add_pitch   = static_cast<fp32>(ch_pitch) * SCALE;
        } else if (s0_up && s1_down) {
            local.yaw_set_rad = static_cast<fp32>(ch_j5_link) * SCALE;
            local.add_pitch   = 0.0f;
        } else {
            (void)s1_down;
            local.yaw_set_rad = 0.0f;
            local.add_pitch   = 0.0f;
        }
        local.add_jaw = (!s0_down) ? static_cast<fp32>(ch_jaw) * SCALE : 0.0f;
        // local.add_roll    = static_cast<fp32>(ch_roll) * SCALE; // 婵″倹鐏?gimbal_cmd_t 閺€顖涘瘮

        // expose snapshot safely: mark in-progress (seq++), write, then mark done (seq++)
        __DMB();
        g_cmd.seq++; __DMB();
        g_cmd = local; __DMB();
        g_cmd.seq++; __DMB();

        // --- LK motors: RC-driven enable/angle and CAN3 send ---

        // Enable/disable only on mode edge; send a short 3-frame burst on transition.
        static bool prev_lk_enable = false;
        static bool prev_j1_enable = false;
        static uint8_t lk_enable_burst_left = 0u;
        static uint8_t lk_disable_burst_left = 0u;
        static uint8_t j1_enable_burst_left = 0u;
        static uint8_t j1_disable_burst_left = 0u;
        const bool lk_enable = !lk_force_disable;
        const bool j1_enable = !lk_force_disable;
        if (lk_enable != prev_lk_enable) {
            if (lk_enable) {
                lk_enable_burst_left = 3u;
                lk_disable_burst_left = 0u;
            } else {
                lk_disable_burst_left = 3u;
                lk_enable_burst_left = 0u;
            }
        }
        if (lk_enable_burst_left > 0u) {
            lk_j3.request_enable();
            lk_j2.request_enable();
            --lk_enable_burst_left;
        }
        if (lk_disable_burst_left > 0u) {
            lk_j3.request_disable();
            lk_j2.request_disable();
            --lk_disable_burst_left;
        }

        if (j1_enable != prev_j1_enable) {
            if (j1_enable) {
                j1_enable_burst_left = 3u;
                j1_disable_burst_left = 0u;
            } else {
                j1_disable_burst_left = 3u;
                j1_enable_burst_left = 0u;
            }
        }
        if (j1_enable_burst_left > 0u) {
            lk_j1.request_enable();
            --j1_enable_burst_left;
        }
        if (j1_disable_burst_left > 0u) {
            lk_j1.request_disable();
            --j1_disable_burst_left;
        }
        prev_lk_enable = lk_enable;
        prev_j1_enable = j1_enable;

        // LK target layer: all remote/control sources resolve into per-axis targets first,
        // then the existing J1/J2/J3 executors consume those targets.
        static LkTargetState lk_j3_target_state{};
        static LkTargetState lk_j2_target_state{};
        static LkTargetState lk_j1_target_state{};
        static LkExecMode lk_j3_exec_mode = LkExecMode::None;
        static LkExecMode lk_j2_exec_mode = LkExecMode::None;
        static LkExecMode lk_j1_exec_mode = LkExecMode::None;
        static LkA6HoldState lk_j3_a6_hold_state{};
        static LkA6HoldState lk_j2_a6_hold_state{};
        static LkA6HoldState lk_j1_a6_hold_state{};
        static uint32_t lk_target_log_last_ms = 0u;
        static uint32_t lk_exec_log_last_ms = 0u;
        static fp32 lk_j3_cmd_A = 0.0f;
        static fp32 lk_j2_cmd_A = 0.0f;
        static fp32 lk_j1_cmd_A = 0.0f;
        static bool lk_j1_ext_filter_active = false;
        static fp32 lk_j1_ext_filter_deg = 0.0f;
        static fp32 lk_j1_ext_slew_deg = 0.0f;
        static uint32_t lk_j1_ext_filter_log_last_ms = 0u;
        static bool lk_wait_rearm_after_safe_exit = false;
        static uint32_t lk_j3_rearm_feedback_base = 0u;
        static uint32_t lk_j2_rearm_feedback_base = 0u;
        static uint32_t lk_j1_rearm_feedback_base = 0u;
        static uint32_t lk_j3_next_read_ms = 0u;
        static uint32_t lk_j2_next_read_ms = 0u;
        static uint32_t lk_j1_next_read_ms = 0u;
        constexpr uint32_t LK_FEEDBACK_QUERY_PERIOD_MS = 20u;

        const bool lk_hold = s0_mid && !ext_lk_apply_any;
        const bool j1_local_session = (s0_up && s1_mid) || lk_ctrl_link || lk_hold;
        const bool j2_local_session = (s0_up && s1_mid) || lk_ctrl_link || lk_hold;
        const bool j3_local_session = (s0_up && s1_up) || lk_ctrl_link || lk_hold;
        const bool lk_ctrl_j2 = j2_local_session || ext_j2_apply_valid;
        const bool lk_ctrl_j3 = j3_local_session || ext_j3_apply_valid;
        const bool lk_ctrl_any = lk_ctrl_j2 || lk_ctrl_j3 || lk_ctrl_link || ext_lk_apply_any;

        const auto s1 = lk_j3.state();
        const auto s2 = lk_j2.state();
        const auto s3 = lk_j1.state();
        const bool lk_j3_can_query_feedback = (lk_enable_burst_left == 0u) && (lk_disable_burst_left == 0u);
        const bool lk_j2_can_query_feedback = (lk_enable_burst_left == 0u) && (lk_disable_burst_left == 0u);
        const bool lk_j1_can_query_feedback = (j1_enable_burst_left == 0u) && (j1_disable_burst_left == 0u);
        const bool lk_j3_feedback_query_enable = lk_j3_can_query_feedback;
        const bool lk_j2_feedback_query_enable = lk_j2_can_query_feedback;
        const bool lk_j1_feedback_query_enable = lk_j1_can_query_feedback;
        if (lk_j3_feedback_query_enable && (now_ms >= lk_j3_next_read_ms)) {
            lk_j3.request_read_multi_turn_angle();
            lk_j3_next_read_ms = now_ms + LK_FEEDBACK_QUERY_PERIOD_MS;
        }
        if (lk_j2_feedback_query_enable && (now_ms >= lk_j2_next_read_ms)) {
            lk_j2.request_read_multi_turn_angle();
            lk_j2_next_read_ms = now_ms + LK_FEEDBACK_QUERY_PERIOD_MS;
        }
        if (lk_j1_feedback_query_enable && (now_ms >= lk_j1_next_read_ms)) {
            lk_j1.request_read_multi_turn_angle();
            lk_j1_next_read_ms = now_ms + LK_FEEDBACK_QUERY_PERIOD_MS;
        }

        const auto enc_to_deg = [](uint16_t enc) -> fp32 {
            return static_cast<fp32>(enc) * (360.0f / 65536.0f);
        };
        const auto deg_to_raw = [](fp32 deg) -> uint16_t {
            fp32 wrapped = std::fmod(deg, 360.0f);
            if (wrapped < 0.0f) {
                wrapped += 360.0f;
            }
            const fp32 scaled = wrapped * (65536.0f / 360.0f);
            uint32_t raw = static_cast<uint32_t>(scaled + 0.5f);
            raw &= 0xFFFFu;
            return static_cast<uint16_t>(raw);
        };
        const auto wrap_0_360 = [](fp32 deg) -> fp32 {
            fp32 v = std::fmod(deg, 360.0f);
            if (v < 0.0f) v += 360.0f;
            return v;
        };
        const auto wrap_err = [](fp32 deg) -> fp32 {
            while (deg > 180.0f) deg -= 360.0f;
            while (deg < -180.0f) deg += 360.0f;
            return deg;
        };
        const auto lk_window_wrapped = [](const GimbalLkPosSingleTurnCfg& cfg) -> bool {
            return cfg.forbid_zero_crossing && (cfg.abs_limit_max_raw < cfg.abs_limit_min_raw);
        };
        const auto lk_abs_raw_in_window = [&lk_window_wrapped](uint16_t abs_raw,
                                                               const GimbalLkPosSingleTurnCfg& cfg) -> bool {
            if (!cfg.forbid_zero_crossing) {
                return true;
            }
            if (lk_window_wrapped(cfg)) {
                return (abs_raw >= cfg.abs_limit_min_raw) || (abs_raw <= cfg.abs_limit_max_raw);
            }
            return (abs_raw >= cfg.abs_limit_min_raw) && (abs_raw <= cfg.abs_limit_max_raw);
        };
        const auto lk_raw_ring_distance = [](uint16_t from_raw, uint16_t to_raw) -> uint16_t {
            const int32_t diff = static_cast<int32_t>(to_raw) - static_cast<int32_t>(from_raw);
            const uint32_t cw = static_cast<uint32_t>((diff + 65536) % 65536);
            const uint32_t ccw = static_cast<uint32_t>((-diff + 65536) % 65536);
            return static_cast<uint16_t>((cw <= ccw) ? cw : ccw);
        };
        const auto lk_clamp_abs_raw_to_window = [&](uint16_t abs_raw,
                                                    const GimbalLkPosSingleTurnCfg& cfg) -> uint16_t {
            if (!cfg.forbid_zero_crossing) {
                return abs_raw;
            }

            if (lk_abs_raw_in_window(abs_raw, cfg)) {
                return abs_raw;
            }

            const uint16_t dist_to_min = lk_raw_ring_distance(abs_raw, cfg.abs_limit_min_raw);
            const uint16_t dist_to_max = lk_raw_ring_distance(abs_raw, cfg.abs_limit_max_raw);
            return (dist_to_min <= dist_to_max) ? cfg.abs_limit_min_raw : cfg.abs_limit_max_raw;
        };
        const auto lk_clamp_abs_deg_to_window = [&](fp32 abs_deg,
                                                    const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            const uint16_t raw = deg_to_raw(abs_deg);
            return enc_to_deg(lk_clamp_abs_raw_to_window(raw, cfg));
        };
        const auto lk_mech_wrap_0_360 = [](fp32 deg) -> fp32 {
            fp32 wrapped = std::fmod(deg, 360.0f);
            if (wrapped < 0.0f) {
                wrapped += 360.0f;
            }
            return wrapped;
        };
        const auto lk_mech_abs_deg_from_raw = [&](uint16_t raw,
                                                  const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            return enc_to_deg(lk_clamp_abs_raw_to_window(raw, cfg)) * cfg.mech_gear_ratio;
        };
        const auto lk_mech_abs_deg_from_state = [&](const LK8016EState& s,
                                                    const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            const uint16_t current_raw =
                lk_normalize_encoder_raw_cross_zero_high_to_zero(s.encoder_raw, cfg);
            if (current_raw == s.encoder_raw) {
                return s.multi_turn_angle_deg;
            }

            const fp32 cycle_span_deg = 360.0f * cfg.mech_gear_ratio;
            if (cycle_span_deg <= 1.0e-6f) {
                return lk_mech_abs_deg_from_raw(current_raw, cfg);
            }
            const fp32 cycle_index = std::floor(s.multi_turn_angle_deg / cycle_span_deg);
            return cycle_index * cycle_span_deg + lk_mech_abs_deg_from_raw(current_raw, cfg);
        };
        const auto lk_raw_from_mech_abs_deg = [&](fp32 mech_abs_deg,
                                                  const GimbalLkPosSingleTurnCfg& cfg) -> uint16_t {
            const fp32 clamped_abs_deg =
                Gimbal_MaxFp32(cfg.abs_limit_min_deg,
                               Gimbal_MinFp32(cfg.abs_limit_max_deg, mech_abs_deg));
            const fp32 single_turn_deg = clamped_abs_deg / cfg.mech_gear_ratio;
            return lk_clamp_abs_raw_to_window(deg_to_raw(single_turn_deg), cfg);
        };
        const auto lk_solve_j1_a6_direction = [&](fp32 target_mech_abs_deg,
                                                  uint16_t current_raw,
                                                  const LkA6HoldState* hold_state) -> LkA6DirectionDecision {
            LkA6DirectionDecision out{};
            const uint16_t target_raw = lk_raw_from_mech_abs_deg(target_mech_abs_deg, GIMBAL_LK_J1_POS_CFG);
            const uint16_t current_clamped_raw = lk_clamp_abs_raw_to_window(current_raw, GIMBAL_LK_J1_POS_CFG);
            const fp32 current_mech_abs_deg = lk_mech_abs_deg_from_raw(current_clamped_raw, GIMBAL_LK_J1_POS_CFG);
            const fp32 target_mech_clamped_deg = lk_mech_abs_deg_from_raw(target_raw, GIMBAL_LK_J1_POS_CFG);
            const int32_t raw_err =
                static_cast<int32_t>(target_raw) - static_cast<int32_t>(current_clamped_raw);
            const uint32_t cw_ring =
                static_cast<uint32_t>((raw_err + 65536) % 65536);
            const uint32_t ccw_ring =
                static_cast<uint32_t>((-raw_err + 65536) % 65536);
            const bool raw_increase = (raw_err > 0);

            out.valid = true;
            out.solved_current_raw = current_clamped_raw;
            out.solved_target_raw = target_raw;
            out.solved_current_abs_deg = current_mech_abs_deg;
            out.solved_target_abs_deg = target_mech_clamped_deg;
            out.cmd_target_deg = lk_mech_wrap_0_360(target_mech_clamped_deg);
            out.cross_raw0_blocked =
                (raw_err != 0) &&
                ((raw_increase && (cw_ring > ccw_ring)) ||
                 (!raw_increase && (ccw_ring > cw_ring))) ? 1u : 0u;

            if (raw_err == 0) {
                out.ccw = (hold_state != nullptr && hold_state->initialized) ? hold_state->dir_latched : false;
                out.cw_delta_deg = 0.0f;
                out.ccw_delta_deg = 0.0f;
                out.signed_delta_deg = 0.0f;
                return out;
            }

            const fp32 abs_delta_deg = std::fabs(target_mech_clamped_deg - current_mech_abs_deg);
            out.ccw = raw_increase;
            if (out.ccw) {
                out.ccw_delta_deg = abs_delta_deg;
                out.signed_delta_deg = -abs_delta_deg;
            } else {
                out.cw_delta_deg = abs_delta_deg;
                out.signed_delta_deg = abs_delta_deg;
            }
            return out;
        };
        const auto lk_solve_a6_direction = [&](fp32 target_abs_deg,
                                               uint16_t current_raw,
                                               const GimbalLkPosSingleTurnCfg& cfg,
                                               const LkA6HoldState* hold_state) -> LkA6DirectionDecision {
            constexpr fp32 INF = 1.0e9f;
            constexpr fp32 TIE_EPS = 1.0e-4f;

            LkA6DirectionDecision out{};
            const uint16_t target_raw = lk_clamp_abs_raw_to_window(deg_to_raw(target_abs_deg), cfg);
            const uint16_t current_clamped_raw = lk_clamp_abs_raw_to_window(current_raw, cfg);
            out.solved_current_raw = current_clamped_raw;
            out.solved_target_raw = target_raw;
            out.solved_current_abs_deg = enc_to_deg(current_clamped_raw);
            out.solved_target_abs_deg = enc_to_deg(target_raw);
            out.cmd_target_deg = out.solved_target_abs_deg;

            const int32_t raw_err = static_cast<int32_t>(target_raw) - static_cast<int32_t>(current_clamped_raw);
            const fp32 abs_delta_deg = static_cast<fp32>(std::abs(raw_err)) * LK8016E_ENCODER_RAW_TO_DEG;

            out.valid = true;
            out.cw_delta_deg = INF;
            out.ccw_delta_deg = INF;

            if (raw_err == 0) {
                out.ccw = (hold_state != nullptr && hold_state->initialized) ? hold_state->dir_latched : cfg.a6_raw_increase_cmd_ccw;
                out.cw_delta_deg = 0.0f;
                out.ccw_delta_deg = 0.0f;
                out.signed_delta_deg = 0.0f;
                return out;
            }

            const bool raw_increase = (raw_err > 0);
            const bool cmd_ccw = raw_increase ? cfg.a6_raw_increase_cmd_ccw : !cfg.a6_raw_increase_cmd_ccw;
            out.ccw = cmd_ccw;
            if (cmd_ccw) {
                out.ccw_delta_deg = abs_delta_deg;
                out.signed_delta_deg = -abs_delta_deg;
            } else {
                out.cw_delta_deg = abs_delta_deg;
                out.signed_delta_deg = abs_delta_deg;
            }
            return out;
        };
        const auto lk_cur_rel_deg_from_cfg_zero = [enc_to_deg, wrap_err](const LK8016EState& s, const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            const uint16_t current_raw =
                lk_normalize_encoder_raw_cross_zero_high_to_zero(s.encoder_raw, cfg);
            const fp32 rel_deg = wrap_err(enc_to_deg(current_raw) - cfg.cfg_zero_deg);
            return Gimbal_LkClampRelDeg(rel_deg, cfg);
        };
        const auto lk_cur_rel_deg_from_mech = [&](const LK8016EState& s,
                                                  const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            const fp32 rel_deg = lk_mech_abs_deg_from_state(s, cfg) - cfg.cfg_zero_deg;
            return Gimbal_LkClampRelDeg(rel_deg, cfg);
        };
        const auto lk_target_mech_abs_deg_from_rel = [&](fp32 target_rel_deg,
                                                         const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            return cfg.cfg_zero_deg + Gimbal_LkClampRelDeg(target_rel_deg, cfg);
        };
        const auto lk_mech_cycle_span_deg = [](const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            return 360.0f * cfg.mech_gear_ratio;
        };
        const auto lk_resolve_mech_target_same_cycle = [&](fp32 target_mech_abs_deg,
                                                           fp32 current_mech_abs_deg,
                                                           const GimbalLkPosSingleTurnCfg& cfg,
                                                           bool* out_cross_blocked) -> fp32 {
            if (out_cross_blocked != nullptr) {
                *out_cross_blocked = false;
            }
            if (!cfg.forbid_zero_crossing) {
                return target_mech_abs_deg;
            }

            const fp32 cycle_span_deg = lk_mech_cycle_span_deg(cfg);
            if (cycle_span_deg <= 1.0e-6f) {
                return target_mech_abs_deg;
            }

            const fp32 cycle_index = std::floor(current_mech_abs_deg / cycle_span_deg);
            const fp32 resolved_target = target_mech_abs_deg + cycle_index * cycle_span_deg;
            if (out_cross_blocked != nullptr) {
                *out_cross_blocked = (std::fabs(resolved_target - target_mech_abs_deg) > 1.0e-3f);
            }
            return resolved_target;
        };
        const auto lk_target_abs_deg_from_rel = [&](fp32 target_rel_deg, const GimbalLkPosSingleTurnCfg& cfg) -> fp32 {
            return lk_clamp_abs_deg_to_window(
                cfg.cfg_zero_deg + Gimbal_LkClampRelDeg(target_rel_deg, cfg),
                cfg);
        };
        const auto lk_j1_target_abs_deg_from_rel = [&](fp32 target_rel_deg) -> fp32 {
            return lk_target_mech_abs_deg_from_rel(target_rel_deg, GIMBAL_LK_J1_POS_CFG);
        };
        constexpr fp32 LK_DT = 0.002f;
        const fp32 j3_cur_rel = (s1.online && s1.multi_turn_valid) ? lk_cur_rel_deg_from_mech(s1, GIMBAL_LK_J3_POS_CFG) : 0.0f;
        const fp32 j2_cur_rel = (s2.online && s2.multi_turn_valid) ? lk_cur_rel_deg_from_mech(s2, GIMBAL_LK_J2_POS_CFG) : 0.0f;
        const fp32 j1_cur_rel = (s3.online && s3.multi_turn_valid) ? lk_cur_rel_deg_from_mech(s3, GIMBAL_LK_J1_POS_CFG) : 0.0f;
        if (gear_exit_hold) {
            lk_wait_rearm_after_safe_exit = true;
            lk_j3_rearm_feedback_base = s1.multi_turn_rx_count;
            lk_j2_rearm_feedback_base = s2.multi_turn_rx_count;
            lk_j1_rearm_feedback_base = s3.multi_turn_rx_count;
            lk_j3_next_read_ms = now_ms;
            lk_j2_next_read_ms = now_ms;
            lk_j1_next_read_ms = now_ms;
            lk_j3_target_state = {};
            lk_j2_target_state = {};
            lk_j1_target_state = {};
            lk_j3_exec_mode = LkExecMode::None;
            lk_j2_exec_mode = LkExecMode::None;
            lk_j1_exec_mode = LkExecMode::None;
            lk_j3_a6_hold_state = {};
            lk_j2_a6_hold_state = {};
            lk_j1_a6_hold_state = {};
        }
        if (pause_active || lk_force_disable) {
            lk_wait_rearm_after_safe_exit = false;
        }
        const bool lk_j3_feedback_fresh = (s1.multi_turn_rx_count > lk_j3_rearm_feedback_base);
        const bool lk_j2_feedback_fresh = (s2.multi_turn_rx_count > lk_j2_rearm_feedback_base);
        const bool lk_j1_feedback_fresh = (s3.multi_turn_rx_count > lk_j1_rearm_feedback_base);
        const auto lk_make_hold_request = [](LkTargetSource source) -> LkTargetRequest {
            LkTargetRequest req{};
            req.active = true;
            req.hold_target = true;
            req.source = source;
            return req;
        };
        const auto lk_make_delta_request = [](fp32 delta_rel_deg, LkTargetSource source) -> LkTargetRequest {
            LkTargetRequest req{};
            req.active = true;
            req.delta_rel_deg = delta_rel_deg;
            req.source = source;
            return req;
        };
        const auto lk_make_absolute_request = [](fp32 target_rel_deg, LkTargetSource source) -> LkTargetRequest {
            LkTargetRequest req{};
            req.active = true;
            req.use_absolute = true;
            req.absolute_rel_deg = target_rel_deg;
            req.source = source;
            return req;
        };
        const auto lk_ensure_target_initialized = [](LkTargetState* state, const GimbalLkPosSingleTurnCfg& cfg) {
            if (state == nullptr || state->initialized) {
                return;
            }
            state->target_rel_deg = Gimbal_LkClampRelDeg(0.0f, cfg);
            state->initialized = true;
        };
        const auto lk_resolve_target = [&](LkTargetState* state,
                                           const LkTargetRequest& request,
                                           const GimbalLkPosSingleTurnCfg& cfg) {
            if (state == nullptr) {
                return;
            }
            lk_ensure_target_initialized(state, cfg);
            if (!request.active) {
                state->source_active = false;
                state->hold_target = false;
                state->source = LkTargetSource::None;
                return;
            }

            if (request.use_absolute) {
                state->target_rel_deg = request.absolute_rel_deg;
            } else if (!request.hold_target) {
                state->target_rel_deg += request.delta_rel_deg;
            }

            state->target_rel_deg = Gimbal_LkClampRelDeg(state->target_rel_deg, cfg);
            state->source_active = true;
            state->hold_target = request.hold_target;
            state->source = request.source;
        };
        const auto lk_apply_override_target = [&](LkTargetState* state,
                                                  fp32 target_rel_deg,
                                                  const GimbalLkPosSingleTurnCfg& cfg) {
            if (state == nullptr) {
                return;
            }
            state->initialized = true;
            state->target_rel_deg = Gimbal_LkClampRelDeg(target_rel_deg, cfg);
            state->source_active = true;
            state->hold_target = false;
            state->source = LkTargetSource::System;
        };
        const auto lk_seed_external_takeover = [&](LkTargetState* state,
                                                   const LkTargetRequest& request,
                                                   LkExecMode last_mode,
                                                   fp32 current_rel_deg,
                                                   const GimbalLkPosSingleTurnCfg& cfg) {
            (void)last_mode;
            if (state == nullptr || !request.active || request.source != LkTargetSource::External) {
                return;
            }
            const bool was_external = (state->source == LkTargetSource::External);
            if (was_external) {
                return;
            }
            state->initialized = true;
            state->target_rel_deg = Gimbal_LkClampRelDeg(current_rel_deg, cfg);
        };
        const auto lk_seed_local_takeover = [&](LkTargetState* state,
                                                const LkTargetRequest& request,
                                                LkExecMode last_mode,
                                                fp32 current_rel_deg,
                                                const GimbalLkPosSingleTurnCfg& cfg) {
            (void)last_mode;
            if (state == nullptr || !request.active) {
                return;
            }
            if (request.source != LkTargetSource::Rc && request.source != LkTargetSource::Link) {
                return;
            }
            const bool was_local = (state->source == LkTargetSource::Rc || state->source == LkTargetSource::Link);
            if (was_local) {
                return;
            }
            state->initialized = true;
            state->target_rel_deg = Gimbal_LkClampRelDeg(current_rel_deg, cfg);
        };

        constexpr fp32 LK_CH_MAX = 660.0f;
        const auto apply_axis_deadband = [](int raw, uint16_t deadband) -> int {
            const int db = static_cast<int>(deadband);
            return (raw > -db && raw < db) ? 0 : raw;
        };

        int j3_in_raw = 0;
        int j2_in_raw = 0;
        int j1_in_raw = 0;
        if (lk_ctrl_link) {
            j3_in_raw = static_cast<int>(rc.rc.ch[GIMBAL_LINK_J3_RC_CH]) * static_cast<int>(GIMBAL_LINK_J3_RC_SIGN);
            j2_in_raw = static_cast<int>(rc.rc.ch[GIMBAL_LINK_J2_RC_CH]) * static_cast<int>(GIMBAL_LINK_J2_RC_SIGN);
            j1_in_raw = static_cast<int>(rc.rc.ch[GIMBAL_LINK_J1_RC_CH]) * static_cast<int>(GIMBAL_LINK_J1_RC_SIGN);
        } else {
            if (s0_up && s1_up) {
                j3_in_raw = static_cast<int>(rc.rc.ch[GIMBAL_LK_J3_RC_CH]) * static_cast<int>(GIMBAL_LK_J3_RC_SIGN);
            }
            if (s0_up && s1_mid) {
                j2_in_raw = static_cast<int>(rc.rc.ch[GIMBAL_LK_J2_RC_CH]) * static_cast<int>(GIMBAL_LK_J2_RC_SIGN);
                j1_in_raw = static_cast<int>(rc.rc.ch[GIMBAL_LK_J1_RC_CH]) * static_cast<int>(GIMBAL_LK_J1_RC_SIGN);
            }
        }
        if (lk_hold) {
            j3_in_raw = 0;
            j2_in_raw = 0;
            j1_in_raw = 0;
        }

        j3_in_raw = apply_axis_deadband(j3_in_raw, lk_ctrl_link ? GIMBAL_LK_J3_POS_CFG.link_deadband_raw : GIMBAL_LK_J3_POS_CFG.rc_deadband_raw);
        j2_in_raw = apply_axis_deadband(j2_in_raw, lk_ctrl_link ? GIMBAL_LK_J2_POS_CFG.link_deadband_raw : GIMBAL_LK_J2_POS_CFG.rc_deadband_raw);
        j1_in_raw = apply_axis_deadband(j1_in_raw, lk_ctrl_link ? GIMBAL_LK_J1_POS_CFG.link_deadband_raw : GIMBAL_LK_J1_POS_CFG.rc_deadband_raw);

        const fp32 j3_local_rate_scale = j3_local_session ?
            static_cast<fp32>(GIMBAL_LK_J3_A8_LOCAL_RATE_SCALE) : 1.0f;
        const fp32 j2_local_rate_scale = j2_local_session ?
            static_cast<fp32>(GIMBAL_LK_J2_A8_LOCAL_RATE_SCALE) : 1.0f;
        const fp32 j1_local_rate_scale = j1_local_session ?
            static_cast<fp32>(GIMBAL_LK_J1_A8_LOCAL_RATE_SCALE) : 1.0f;
        const fp32 j3_inc_deg = (static_cast<fp32>(j3_in_raw) / LK_CH_MAX) *
                                GIMBAL_LK_J3_POS_CFG.manual_rate_dps *
                                j3_local_rate_scale * LK_DT;
        const fp32 j2_inc_deg = (static_cast<fp32>(j2_in_raw) / LK_CH_MAX) *
                                GIMBAL_LK_J2_POS_CFG.manual_rate_dps *
                                j2_local_rate_scale * LK_DT;
        const fp32 j1_inc_deg = (static_cast<fp32>(j1_in_raw) / LK_CH_MAX) *
                                GIMBAL_LK_J1_POS_CFG.manual_rate_dps *
                                j1_local_rate_scale * LK_DT;
        const LkTargetSource lk_local_source = lk_ctrl_link ? LkTargetSource::Link : LkTargetSource::Rc;

        LkTargetRequest j3_request{};
        if (ext_j3_apply_valid) {
            j3_request = ext_j3_update_target ?
                lk_make_absolute_request(ext_j3_target_rel_deg, LkTargetSource::External) :
                lk_make_hold_request(LkTargetSource::External);
        } else if (j3_local_session) {
            j3_request = (std::fabs(j3_inc_deg) >= GIMBAL_LK_J3_POS_CFG.manual_min_step_deg) ?
                lk_make_delta_request(j3_inc_deg, lk_local_source) :
                lk_make_hold_request(lk_local_source);
        }

        LkTargetRequest j2_request{};
        if (ext_j2_apply_valid) {
            j2_request = ext_j2_update_target ?
                lk_make_absolute_request(ext_j2_target_rel_deg, LkTargetSource::External) :
                lk_make_hold_request(LkTargetSource::External);
        } else if (j2_local_session) {
            j2_request = (std::fabs(j2_inc_deg) >= GIMBAL_LK_J2_POS_CFG.manual_min_step_deg) ?
                lk_make_delta_request(j2_inc_deg, lk_local_source) :
                lk_make_hold_request(lk_local_source);
        }

        fp32 ext_j1_target_rel_deg_stable = ext_j1_target_rel_deg;
        if (!ext_j1_apply_valid) {
            lk_j1_ext_filter_active = false;
        } else {
            if (!lk_j1_ext_filter_active) {
                const fp32 seed_rel_deg =
                    Gimbal_LkClampRelDeg(lk_j1_target_state.target_rel_deg, GIMBAL_LK_J1_POS_CFG);
                lk_j1_ext_filter_deg = seed_rel_deg;
                lk_j1_ext_slew_deg = seed_rel_deg;
                lk_j1_ext_filter_active = true;
            }

            if (ext_j1_update_target) {
                if (GIMBAL_LK_J1_EXT_FILTER_ENABLE) {
                    const fp32 tau_s = static_cast<fp32>(GIMBAL_LK_J1_EXT_FILTER_TAU_S);
                    const fp32 alpha = (tau_s > 0.0f) ? (LK_DT / (tau_s + LK_DT)) : 1.0f;
                    lk_j1_ext_filter_deg += alpha * (ext_j1_target_rel_deg - lk_j1_ext_filter_deg);
                } else {
                    lk_j1_ext_filter_deg = ext_j1_target_rel_deg;
                }

                const fp32 max_step_deg = static_cast<fp32>(GIMBAL_LK_J1_EXT_MAX_DPS) * LK_DT;
                if (max_step_deg > 0.0f) {
                    fp32 delta_deg = lk_j1_ext_filter_deg - lk_j1_ext_slew_deg;
                    if (delta_deg > max_step_deg) {
                        delta_deg = max_step_deg;
                    } else if (delta_deg < -max_step_deg) {
                        delta_deg = -max_step_deg;
                    }
                    lk_j1_ext_slew_deg += delta_deg;
                } else {
                    lk_j1_ext_slew_deg = lk_j1_ext_filter_deg;
                }
            }

            ext_j1_target_rel_deg_stable =
                Gimbal_LkClampRelDeg(lk_j1_ext_slew_deg, GIMBAL_LK_J1_POS_CFG);
        }

        if (ext_j1_apply_valid && ((now_ms - lk_j1_ext_filter_log_last_ms) >= 500u)) {
            lk_j1_ext_filter_log_last_ms = now_ms;
            LOGI("[GIMBAL][LK][J1][EXTSTAB] raw_in=%u map_target=%.3f filt_target=%.3f slew_target=%.3f hold=%u apply=%u",
                 static_cast<unsigned>(ext_j1_protocol_raw),
                 static_cast<double>(ext_j1_target_rel_deg),
                 static_cast<double>(lk_j1_ext_filter_deg),
                 static_cast<double>(ext_j1_target_rel_deg_stable),
                 ext_j1_map.hold_last_target ? 1u : 0u,
                 ext_j1_apply_valid ? 1u : 0u);
        }

        LkTargetRequest j1_request{};
        if (ext_j1_apply_valid) {
            j1_request = ext_j1_update_target ?
                lk_make_absolute_request(ext_j1_target_rel_deg_stable, LkTargetSource::External) :
                lk_make_hold_request(LkTargetSource::External);
        } else if (j1_local_session) {
            j1_request = (std::fabs(j1_inc_deg) >= GIMBAL_LK_J1_POS_CFG.manual_min_step_deg) ?
                lk_make_delta_request(j1_inc_deg, lk_local_source) :
                lk_make_hold_request(lk_local_source);
        }

        const bool lk_rearm_takeover =
            ext_j1_apply_valid || ext_j2_apply_valid || ext_j3_apply_valid ||
            (std::fabs(j1_inc_deg) >= GIMBAL_LK_J1_POS_CFG.manual_min_step_deg) ||
            (std::fabs(j2_inc_deg) >= GIMBAL_LK_J2_POS_CFG.manual_min_step_deg) ||
            (std::fabs(j3_inc_deg) >= GIMBAL_LK_J3_POS_CFG.manual_min_step_deg);
        const bool lk_rearm_feedback_ready =
            lk_j3_feedback_fresh && lk_j2_feedback_fresh && lk_j1_feedback_fresh;
        if (lk_wait_rearm_after_safe_exit && lk_rearm_takeover && lk_rearm_feedback_ready) {
            lk_wait_rearm_after_safe_exit = false;
        }
        if (lk_wait_rearm_after_safe_exit) {
            j3_request = {};
            j2_request = {};
            j1_request = {};
        }

        lk_seed_external_takeover(&lk_j3_target_state, j3_request, lk_j3_exec_mode, j3_cur_rel, GIMBAL_LK_J3_POS_CFG);
        lk_seed_external_takeover(&lk_j2_target_state, j2_request, lk_j2_exec_mode, j2_cur_rel, GIMBAL_LK_J2_POS_CFG);
        lk_seed_external_takeover(&lk_j1_target_state, j1_request, lk_j1_exec_mode, j1_cur_rel, GIMBAL_LK_J1_POS_CFG);
        lk_seed_local_takeover(&lk_j3_target_state, j3_request, lk_j3_exec_mode, j3_cur_rel, GIMBAL_LK_J3_POS_CFG);
        lk_seed_local_takeover(&lk_j2_target_state, j2_request, lk_j2_exec_mode, j2_cur_rel, GIMBAL_LK_J2_POS_CFG);
        lk_seed_local_takeover(&lk_j1_target_state, j1_request, lk_j1_exec_mode, j1_cur_rel, GIMBAL_LK_J1_POS_CFG);

        lk_resolve_target(&lk_j3_target_state, j3_request, GIMBAL_LK_J3_POS_CFG);
        lk_resolve_target(&lk_j2_target_state, j2_request, GIMBAL_LK_J2_POS_CFG);
        lk_resolve_target(&lk_j1_target_state, j1_request, GIMBAL_LK_J1_POS_CFG);

        lk_j2_cmd_A = 0.0f;
        lk_j3_cmd_A = 0.0f;
        lk_j1_cmd_A = 0.0f;
        const bool lk_exec_log_due = (GIMBAL_LK_EXEC_DIAG_ENABLE != 0) &&
                                     ((now_ms - lk_exec_log_last_ms) >= GIMBAL_LK_EXEC_DIAG_PERIOD_MS);
        LkExecDiag lk_j3_exec_diag{};
        LkExecDiag lk_j2_exec_diag{};
        LkExecDiag lk_j1_exec_diag{};
        const bool lk_system_absolute_mode = pause_active;
        const bool lk_exec_enable = !lk_force_disable && (pause_active || lk_ctrl_any);
        const bool j1_exec_enable = j1_enable && (pause_active || lk_ctrl_any);
        fp32 lk_j2_ff_A = 0.0f;
        fp32 lk_j3_ff_A = 0.0f;

        if (pause_active) {
            lk_apply_override_target(
                &lk_j3_target_state,
                static_cast<fp32>(Omnix_CdegToDeg(pause_cmd_cdeg[2])),
                GIMBAL_LK_J3_POS_CFG);
            lk_apply_override_target(
                &lk_j2_target_state,
                static_cast<fp32>(Omnix_CdegToDeg(pause_cmd_cdeg[1])),
                GIMBAL_LK_J2_POS_CFG);
            lk_apply_override_target(
                &lk_j1_target_state,
                static_cast<fp32>(Omnix_CdegToDeg(pause_cmd_cdeg[0])),
                GIMBAL_LK_J1_POS_CFG);
        }
        lk_j3_target_state.target_rel_deg = Gimbal_LkClampRelDeg(lk_j3_target_state.target_rel_deg, GIMBAL_LK_J3_POS_CFG);
        lk_j2_target_state.target_rel_deg = Gimbal_LkClampRelDeg(lk_j2_target_state.target_rel_deg, GIMBAL_LK_J2_POS_CFG);
        lk_j1_target_state.target_rel_deg = Gimbal_LkClampRelDeg(lk_j1_target_state.target_rel_deg, GIMBAL_LK_J1_POS_CFG);

        const auto lk_fill_position_dbg = [&](volatile GimbalPidDebug* dbg,
                                              const LK8016EState& state,
                                              const LkExecDiag& diag) {
            if (dbg == nullptr) {
                return;
            }
            dbg->online = state.online ? 1u : 0u;
            dbg->loop_target = diag.target_rel_deg;
            dbg->loop_actual = diag.current_rel_deg;
            dbg->pos_target = diag.target_abs_deg;
            dbg->pos_actual = diag.current_abs_deg;
            dbg->vel_target = 0.0f;
            dbg->vel_actual = static_cast<fp32>(state.speed_dps);
            dbg->torque_target = 0.0f;
            dbg->torque_actual = diag.feedback_iq_A;
            dbg->p_out = 0.0f;
            dbg->i_out = 0.0f;
            dbg->d_out = 0.0f;
            dbg->pid_out = 0.0f;
            dbg->mode = static_cast<uint8_t>(diag.mode == LkExecMode::A4 ? 4u :
                                             (diag.mode == LkExecMode::A6 ? 6u :
                                             (diag.mode == LkExecMode::A8 ? 8u : 0u)));
        };
        const auto lk_execute_axis = [&](LK8016E& motor,
                                         const LK8016EState& state,
                                         LkTargetState& target_state,
                                         const LkTargetRequest& request,
                                         const GimbalLkPosSingleTurnCfg& cfg,
                                         bool use_mech_calibration,
                                         bool feedback_fresh,
                                         bool axis_enable,
                                         bool local_hold_a6_enable,
                                         fp32 local_a8_max_dps_scale,
                                         bool dir_invert,
                                         LkExecMode* io_exec_mode,
                                         LkA6HoldState* io_a6_hold_state,
                                         LkExecDiag* out_diag,
                                         volatile GimbalPidDebug* dbg) {
            (void)dir_invert;
            if (io_exec_mode == nullptr || io_a6_hold_state == nullptr || out_diag == nullptr) {
                return;
            }

            const uint16_t current_raw =
                lk_normalize_encoder_raw_cross_zero_high_to_zero(state.encoder_raw, cfg);
            const fp32 current_single_abs_deg = enc_to_deg(current_raw);
            const fp32 current_mech_abs_deg = lk_mech_abs_deg_from_state(state, cfg);
            const bool axis_is_j2j3 =
                (&cfg == &GIMBAL_LK_J2_POS_CFG) || (&cfg == &GIMBAL_LK_J3_POS_CFG);
            const bool external_source = (target_state.source == LkTargetSource::External);
            const bool external_cross_zero_high_normalized =
                axis_is_j2j3 &&
                external_source &&
                (state.encoder_raw != current_raw) &&
                (current_raw == 0u);
            uint16_t effective_current_raw = current_raw;
            fp32 effective_current_single_abs_deg = current_single_abs_deg;
            fp32 effective_current_mech_abs_deg = current_mech_abs_deg;
            if (external_cross_zero_high_normalized) {
                // For J2/J3 external control, treat wrapped-high crossing as zero to keep direction consistent.
                effective_current_raw = 0u;
                effective_current_single_abs_deg = enc_to_deg(0u);
                effective_current_mech_abs_deg = lk_mech_abs_deg_from_raw(0u, cfg);
            }

            LkExecDiag diag{};
            diag.target_rel_deg = Gimbal_LkClampRelDeg(target_state.target_rel_deg, cfg);
            diag.target_abs_deg = use_mech_calibration ?
                lk_target_mech_abs_deg_from_rel(diag.target_rel_deg, cfg) :
                lk_target_abs_deg_from_rel(diag.target_rel_deg, cfg);
            diag.proto_target_deg = use_mech_calibration ?
                lk_mech_wrap_0_360(diag.target_abs_deg) :
                diag.target_abs_deg;
            diag.current_abs_deg = use_mech_calibration ?
                effective_current_mech_abs_deg :
                effective_current_single_abs_deg;
            if (state.online) {
                if (external_cross_zero_high_normalized) {
                    const fp32 current_rel_deg = (use_mech_calibration ?
                        effective_current_mech_abs_deg :
                        effective_current_single_abs_deg) - cfg.cfg_zero_deg;
                    diag.current_rel_deg = Gimbal_LkClampRelDeg(current_rel_deg, cfg);
                } else {
                    diag.current_rel_deg = use_mech_calibration ?
                        lk_cur_rel_deg_from_mech(state, cfg) :
                        lk_cur_rel_deg_from_cfg_zero(state, cfg);
                }
            } else {
                diag.current_rel_deg = 0.0f;
            }
            diag.speed_dps = static_cast<fp32>(state.speed_dps);
            diag.max_dps = cfg.max_dps;
            diag.feedback_iq_A = state.online ? state.iq_A : 0.0f;
            diag.current_raw = effective_current_raw;
            diag.target_raw = use_mech_calibration ?
                lk_raw_from_mech_abs_deg(diag.target_abs_deg, cfg) :
                deg_to_raw(diag.target_abs_deg);
            diag.raw_limit_min = cfg.abs_limit_min_raw;
            diag.raw_limit_max = cfg.abs_limit_max_raw;
            diag.feedback_fresh = feedback_fresh ? 1u : 0u;
            diag.raw_increase_is_ccw = use_mech_calibration ?
                1u :
                (cfg.a6_raw_increase_cmd_ccw ? 1u : 0u);
            if (request.active &&
                (request.source == LkTargetSource::Rc || request.source == LkTargetSource::Link) &&
                !request.use_absolute &&
                !request.hold_target) {
                diag.delta_rel_deg = request.delta_rel_deg;
            }

            if (!axis_enable || !state.online) {
                motor.set_iq_A(0.0f);
                *io_exec_mode = LkExecMode::None;
                io_a6_hold_state->initialized = false;
                io_a6_hold_state->hold_latched = false;
                lk_fill_position_dbg(dbg, state, diag);
                *out_diag = diag;
                return;
            }

            const bool local_hold_a6 = local_hold_a6_enable &&
                                       target_state.hold_target &&
                                       (target_state.source == LkTargetSource::Rc ||
                                        target_state.source == LkTargetSource::Link);
            const bool pause_system_control = pause_active &&
                                              (target_state.source == LkTargetSource::System);
            LkExecMode desired_mode = LkExecMode::None;
            if (lk_system_absolute_mode || target_state.source == LkTargetSource::System || target_state.source == LkTargetSource::External) {
                desired_mode = LkExecMode::A6;
            } else if (target_state.source == LkTargetSource::Rc || target_state.source == LkTargetSource::Link) {
                desired_mode = local_hold_a6 ? LkExecMode::A6 : LkExecMode::A8;
            } else {
                // No active source means no one should keep replaying the last LK command.
                desired_mode = LkExecMode::None;
            }

            if (use_mech_calibration && desired_mode == LkExecMode::A6) {
                if (!state.multi_turn_valid) {
                    motor.set_iq_A(0.0f);
                    *io_exec_mode = LkExecMode::None;
                    diag.mode = LkExecMode::None;
                    lk_fill_position_dbg(dbg, state, diag);
                    *out_diag = diag;
                    return;
                }
                bool cross_zero_cut_blocked = false;
                const fp32 resolved_target_abs_deg =
                    lk_resolve_mech_target_same_cycle(diag.target_abs_deg,
                                                      diag.current_abs_deg,
                                                      cfg,
                                                      &cross_zero_cut_blocked);
                const fp32 err_deg = resolved_target_abs_deg - diag.current_abs_deg;
                diag.mode = LkExecMode::A4;
                diag.target_abs_deg = resolved_target_abs_deg;
                diag.proto_target_deg = lk_mech_wrap_0_360(resolved_target_abs_deg);
                diag.err_deg = err_deg;
                diag.cw_delta_deg = (err_deg >= 0.0f) ? err_deg : 0.0f;
                diag.ccw_delta_deg = (err_deg < 0.0f) ? -err_deg : 0.0f;
                diag.desired_dir = (err_deg < 0.0f) ? 1u : 0u;
                diag.dir = diag.desired_dir;
                diag.dir_invert = 0u;
                diag.cross_raw0_blocked = cross_zero_cut_blocked ? 1u : 0u;
                diag.raw_cmd = static_cast<int32_t>(resolved_target_abs_deg * 100.0f +
                                                    ((resolved_target_abs_deg >= 0.0f) ? 0.5f : -0.5f));
                const fp32 a4_cmd_max_dps =
                    (&cfg == &GIMBAL_LK_J1_POS_CFG) ? static_cast<fp32>(GIMBAL_LK_J1_A4_MAX_DPS) : diag.max_dps;
                diag.max_dps = a4_cmd_max_dps;
                motor.set_angle_deg_max_speed(resolved_target_abs_deg, a4_cmd_max_dps);
                *io_exec_mode = LkExecMode::A4;
                io_a6_hold_state->initialized = false;
                io_a6_hold_state->hold_latched = false;
            } else if (desired_mode == LkExecMode::A6) {
                LkA6DirectionDecision active_decision{};
                fp32 cmd_max_dps = diag.max_dps;

                if (pause_system_control) {
                    io_a6_hold_state->initialized = false;
                    io_a6_hold_state->dir_latched = false;
                    io_a6_hold_state->hold_latched = false;
                    io_a6_hold_state->hold_target_abs_deg = 0.0f;
                    active_decision = use_mech_calibration ?
                        lk_solve_j1_a6_direction(diag.target_abs_deg, effective_current_raw, nullptr) :
                        lk_solve_a6_direction(diag.target_abs_deg, effective_current_raw, cfg, nullptr);
                } else {
                    const LkA6DirectionDecision live_decision = use_mech_calibration ?
                        lk_solve_j1_a6_direction(diag.target_abs_deg, effective_current_raw, io_a6_hold_state) :
                        lk_solve_a6_direction(diag.target_abs_deg, effective_current_raw, cfg, io_a6_hold_state);
                    const fp32 live_abs_err_deg = std::fabs(live_decision.signed_delta_deg);
                    const fp32 abs_speed_dps = std::fabs(diag.speed_dps);
                    const fp32 hold_err_db_deg = GIMBAL_LK_A6_HOLD_ERR_DB_DEG;
                    const fp32 hold_spd_db_dps = GIMBAL_LK_A6_HOLD_SPD_DB_DPS;
                    const fp32 hold_exit_err_deg = hold_err_db_deg + GIMBAL_LK_A6_DIR_HYST_DEG;

                    if (!io_a6_hold_state->initialized) {
                        io_a6_hold_state->initialized = true;
                        io_a6_hold_state->dir_latched = live_decision.ccw;
                        io_a6_hold_state->hold_latched = false;
                        io_a6_hold_state->hold_target_abs_deg = live_decision.solved_target_abs_deg;
                    }

                    if (!io_a6_hold_state->hold_latched) {
                        if (live_abs_err_deg <= hold_err_db_deg && abs_speed_dps <= hold_spd_db_dps) {
                            io_a6_hold_state->hold_latched = true;
                            io_a6_hold_state->hold_target_abs_deg = live_decision.solved_target_abs_deg;
                        }
                    } else if (live_abs_err_deg > hold_exit_err_deg) {
                        io_a6_hold_state->hold_latched = false;
                    }

                    const fp32 active_target_abs_deg = io_a6_hold_state->hold_latched ?
                        io_a6_hold_state->hold_target_abs_deg :
                        live_decision.solved_target_abs_deg;
                    active_decision = use_mech_calibration ?
                        lk_solve_j1_a6_direction(active_target_abs_deg, effective_current_raw, io_a6_hold_state) :
                        lk_solve_a6_direction(active_target_abs_deg, effective_current_raw, cfg, io_a6_hold_state);
                    cmd_max_dps = local_hold_a6 ?
                        std::fmin(diag.max_dps, static_cast<fp32>(GIMBAL_LK_A6_LOCAL_HOLD_MAX_DPS)) :
                        diag.max_dps;
                    io_a6_hold_state->dir_latched = active_decision.ccw;
                }

                if (!active_decision.valid) {
                    motor.set_iq_A(0.0f);
                    *io_exec_mode = LkExecMode::None;
                    diag.mode = LkExecMode::None;
                    diag.a6_hold = 0u;
                    lk_fill_position_dbg(dbg, state, diag);
                    *out_diag = diag;
                    return;
                }

                const bool desired_ccw = active_decision.ccw;
                const bool cmd_ccw = desired_ccw;
                const fp32 cmd_target_abs_deg = active_decision.cmd_target_deg;
                diag.mode = LkExecMode::A6;
                diag.desired_dir = desired_ccw ? 1u : 0u;
                diag.dir = cmd_ccw ? 1u : 0u;
                diag.dir_invert = 0u;
                diag.a6_hold = (pause_system_control || !io_a6_hold_state->hold_latched) ? 0u : 1u;
                diag.target_abs_deg = active_decision.solved_target_abs_deg;
                diag.proto_target_deg = active_decision.cmd_target_deg;
                diag.current_abs_deg = active_decision.solved_current_abs_deg;
                diag.target_raw = active_decision.solved_target_raw;
                diag.current_raw = active_decision.solved_current_raw;
                diag.cw_delta_deg = active_decision.cw_delta_deg;
                diag.ccw_delta_deg = active_decision.ccw_delta_deg;
                diag.err_deg = active_decision.signed_delta_deg;
                diag.cross_raw0_blocked = active_decision.cross_raw0_blocked;
                diag.raw_cmd = static_cast<int32_t>(cmd_target_abs_deg * 100.0f + 0.5f);
                diag.max_dps = cmd_max_dps;
                motor.set_circle_angle_deg_max_speed(cmd_target_abs_deg, cmd_ccw, cmd_max_dps);
                *io_exec_mode = LkExecMode::A6;
            } else if (desired_mode == LkExecMode::A8) {
                const bool local_a8 = (target_state.source == LkTargetSource::Rc ||
                                       target_state.source == LkTargetSource::Link);
                const fp32 a8_scale = (local_a8 && local_a8_max_dps_scale > 0.0f) ?
                    local_a8_max_dps_scale : 1.0f;
                const fp32 cmd_max_dps = diag.max_dps * a8_scale;
                diag.mode = LkExecMode::A8;
                diag.raw_cmd = static_cast<int32_t>(diag.delta_rel_deg * 100.0f);
                diag.max_dps = cmd_max_dps;
                motor.set_inc_angle_deg_max_speed(diag.delta_rel_deg, cmd_max_dps);
                *io_exec_mode = LkExecMode::A8;
                io_a6_hold_state->initialized = false;
                io_a6_hold_state->hold_latched = false;
            } else {
                motor.set_iq_A(0.0f);
                *io_exec_mode = LkExecMode::None;
                io_a6_hold_state->initialized = false;
                io_a6_hold_state->hold_latched = false;
            }

            lk_fill_position_dbg(dbg, state, diag);
            *out_diag = diag;
        };

        lk_execute_axis(
            lk_j3,
            s1,
            lk_j3_target_state,
            j3_request,
            GIMBAL_LK_J3_POS_CFG,
            true,
            lk_j3_feedback_fresh,
            lk_exec_enable,
            (GIMBAL_LK_LOCAL_HOLD_USE_A6 != 0),
            static_cast<fp32>(GIMBAL_LK_J3_A8_LOCAL_MAX_DPS_SCALE),
            GIMBAL_LK_J3_POS_DIR_INVERT,
            &lk_j3_exec_mode,
            &lk_j3_a6_hold_state,
            &lk_j3_exec_diag,
            &gimbal_dbg_lk_j3);
        lk_execute_axis(
            lk_j2,
            s2,
            lk_j2_target_state,
            j2_request,
            GIMBAL_LK_J2_POS_CFG,
            true,
            lk_j2_feedback_fresh,
            lk_exec_enable,
            (GIMBAL_LK_LOCAL_HOLD_USE_A6 != 0),
            static_cast<fp32>(GIMBAL_LK_J2_A8_LOCAL_MAX_DPS_SCALE),
            GIMBAL_LK_J2_POS_DIR_INVERT,
            &lk_j2_exec_mode,
            &lk_j2_a6_hold_state,
            &lk_j2_exec_diag,
            &gimbal_dbg_lk_j2);
        lk_execute_axis(
            lk_j1,
            s3,
            lk_j1_target_state,
            j1_request,
            GIMBAL_LK_J1_POS_CFG,
            true,
            lk_j1_feedback_fresh,
            j1_exec_enable,
            false,
            static_cast<fp32>(GIMBAL_LK_J1_A8_LOCAL_MAX_DPS_SCALE),
            GIMBAL_LK_J1_POS_DIR_INVERT,
            &lk_j1_exec_mode,
            &lk_j1_a6_hold_state,
            &lk_j1_exec_diag,
            &gimbal_dbg_lk_j1);

        lk_j3_cmd_A = (lk_j3_exec_diag.mode != LkExecMode::None) ? lk_j3_exec_diag.feedback_iq_A : 0.0f;
        lk_j2_cmd_A = (lk_j2_exec_diag.mode != LkExecMode::None) ? lk_j2_exec_diag.feedback_iq_A : 0.0f;
        lk_j1_cmd_A = (lk_j1_exec_diag.mode != LkExecMode::None) ? lk_j1_exec_diag.feedback_iq_A : 0.0f;

        if (lk_exec_log_due) {
            const auto lk_log_exec_short = [&](const char* axis,
                                               const LkTargetState& target_state,
                                               const LkExecDiag& diag,
                                               const LK8016EState& state) {
                LOGI("[GIMBAL][LK][EXEC1][%s] src=%s mode=%s hold=%u a6=%u last=0x%02X raw=%ld",
                     axis,
                     lk_target_source_name(target_state.source),
                     lk_exec_mode_name(diag.mode),
                     target_state.hold_target ? 1u : 0u,
                     static_cast<unsigned>(diag.a6_hold),
                     static_cast<unsigned>(state.last_cmd),
                     static_cast<long>(diag.raw_cmd));
                LOGI("[GIMBAL][LK][EXEC2][%s] err=%.3f spd=%.1f drel=%.3f trg=%.3f cur=%.3f",
                     axis,
                     static_cast<double>(diag.err_deg),
                     static_cast<double>(diag.speed_dps),
                     static_cast<double>(diag.delta_rel_deg),
                     static_cast<double>(diag.target_rel_deg),
                     static_cast<double>(diag.current_rel_deg));
                LOGI("[GIMBAL][LK][EXEC3][%s] abs_cur=%.3f abs_tgt=%.3f cw=%.3f ccw=%.3f desired_dir=%s cmd_dir=%s dir_invert=%u pause=%u",
                     axis,
                     static_cast<double>(diag.current_abs_deg),
                     static_cast<double>(diag.target_abs_deg),
                     static_cast<double>(diag.cw_delta_deg),
                     static_cast<double>(diag.ccw_delta_deg),
                     (diag.desired_dir != 0u) ? "CCW" : "CW",
                     (diag.dir != 0u) ? "CCW" : "CW",
                     static_cast<unsigned>(diag.dir_invert),
                     pause_active ? 1u : 0u);
                LOGI("[GIMBAL][LK][EXEC4][%s] raw_cur=%u raw_tgt=%u raw_min=%u raw_max=%u proto_tgt=%.3f feedback_fresh=%u inc_is_ccw=%u cross_raw0=%u",
                     axis,
                     static_cast<unsigned>(diag.current_raw),
                     static_cast<unsigned>(diag.target_raw),
                     static_cast<unsigned>(diag.raw_limit_min),
                     static_cast<unsigned>(diag.raw_limit_max),
                     static_cast<double>(diag.proto_target_deg),
                     static_cast<unsigned>(diag.feedback_fresh),
                     static_cast<unsigned>(diag.raw_increase_is_ccw),
                     static_cast<unsigned>(diag.cross_raw0_blocked));
            };
            lk_log_exec_short("J3", lk_j3_target_state, lk_j3_exec_diag, s1);
            lk_log_exec_short("J2", lk_j2_target_state, lk_j2_exec_diag, s2);
            lk_log_exec_short("J1", lk_j1_target_state, lk_j1_exec_diag, s3);
        }
        if (lk_exec_log_due) {
            lk_exec_log_last_ms = now_ms;
        }
        if ((now_ms - lk_target_log_last_ms) >= 1000u) {
            lk_target_log_last_ms = now_ms;
            LOGI("[GIMBAL][LK][EXT] seq=%u fresh=%u valid_j1=%u valid_j2=%u valid_j3=%u apply_j1=%u apply_j2=%u apply_j3=%u pause=%u abort=%u zero=%u force_disable=%u wait_fresh=%u wait_fb=%u fb_j1=%u fb_j2=%u fb_j3=%u",
                 static_cast<unsigned>(ext_cmd.seq),
                 ext_cmd_fresh ? 1u : 0u,
                 ext_j1_valid ? 1u : 0u,
                 ext_j2_valid ? 1u : 0u,
                 ext_j3_valid ? 1u : 0u,
                 ext_j1_apply_valid ? 1u : 0u,
                 ext_j2_apply_valid ? 1u : 0u,
                 ext_j3_apply_valid ? 1u : 0u,
                 pause_active ? 1u : 0u,
                 pause_abort ? 1u : 0u,
                 gimbal_zero_force ? 1u : 0u,
                 lk_force_disable ? 1u : 0u,
                 lk_wait_external_fresh_after_safe_exit ? 1u : 0u,
                 lk_wait_rearm_after_safe_exit ? 1u : 0u,
                 lk_j1_feedback_fresh ? 1u : 0u,
                 lk_j2_feedback_fresh ? 1u : 0u,
                 lk_j3_feedback_fresh ? 1u : 0u);
            LOGI("[GIMBAL][LK][TARGET][J1] src=%s active=%u hold=%u target=%.3f current=%.3f ext=%u local=%u",
                 lk_target_source_name(lk_j1_target_state.source),
                 lk_j1_target_state.source_active ? 1u : 0u,
                 lk_j1_target_state.hold_target ? 1u : 0u,
                 static_cast<double>(lk_j1_target_state.target_rel_deg),
                 static_cast<double>(j1_cur_rel),
                 ext_j1_apply_valid ? 1u : 0u,
                 j1_local_session ? 1u : 0u);
            LOGI("[GIMBAL][LK][TARGET][J2] src=%s active=%u hold=%u target=%.3f current=%.3f ext=%u local=%u",
                 lk_target_source_name(lk_j2_target_state.source),
                 lk_j2_target_state.source_active ? 1u : 0u,
                 lk_j2_target_state.hold_target ? 1u : 0u,
                 static_cast<double>(lk_j2_target_state.target_rel_deg),
                 static_cast<double>(j2_cur_rel),
                 ext_j2_apply_valid ? 1u : 0u,
                 j2_local_session ? 1u : 0u);
            LOGI("[GIMBAL][LK][TARGET][J3] src=%s active=%u hold=%u target=%.3f current=%.3f ext=%u local=%u",
                 lk_target_source_name(lk_j3_target_state.source),
                 lk_j3_target_state.source_active ? 1u : 0u,
                 lk_j3_target_state.hold_target ? 1u : 0u,
                 static_cast<double>(lk_j3_target_state.target_rel_deg),
                 static_cast<double>(j3_cur_rel),
                 ext_j3_apply_valid ? 1u : 0u,
                 j3_local_session ? 1u : 0u);
        }

        // --- DM compensation motors J8/J9 (MIT torque) ---
        const bool j8_enable = !lk_force_disable;
        const bool j9_enable = !lk_force_disable;
        static bool prev_j8_enable = false;
        static bool prev_j9_enable = false;
        if (j8_enable) {
            if (!prev_j8_enable) {
                gimbal_send_dm_enable(GIMBAL_DM_J8_BUS, static_cast<uint8_t>(GIMBAL_DM_J8_ID), true);
            }
        } else {
            if (prev_j8_enable) {
                gimbal_send_dm_enable(GIMBAL_DM_J8_BUS, static_cast<uint8_t>(GIMBAL_DM_J8_ID), false);
            }
        }
        if (j9_enable) {
            if (!prev_j9_enable) {
                gimbal_send_dm_enable(GIMBAL_DM_J9_BUS, static_cast<uint8_t>(GIMBAL_DM_J9_ID), true);
            }
        } else {
            if (prev_j9_enable) {
                gimbal_send_dm_enable(GIMBAL_DM_J9_BUS, static_cast<uint8_t>(GIMBAL_DM_J9_ID), false);
            }
        }
        prev_j8_enable = j8_enable;
        prev_j9_enable = j9_enable;

        const fp32 dt = 0.002f;
        const auto clamp_nm = [](fp32 v, fp32 max_nm) -> fp32 {
            if (max_nm <= 0.0f) return v;
            if (v >  max_nm) return  max_nm;
            if (v < -max_nm) return -max_nm;
            return v;
        };
        const auto slew_nm = [dt](fp32 cur, fp32 target, fp32 slew_nm_s) -> fp32 {
            if (slew_nm_s <= 0.0f) return target;
            const fp32 lim = slew_nm_s * dt;
            const fp32 delta = target - cur;
            if (delta >  lim) return cur + lim;
            if (delta < -lim) return cur - lim;
            return target;
        };
        const auto lpf_alpha = [dt](fp32 tau_s) -> fp32 {
            if (tau_s <= 0.0f) {
                return 1.0f;
            }
            return dt / (tau_s + dt);
        };
        struct DmFollowCompRuntime {
            fp32 torque_prev = 0.0f;
            fp32 cmd_lpf_A = 0.0f;
            bool external_follow_active = false;
        };
        static DmFollowCompRuntime j8_runtime{};
        static DmFollowCompRuntime j9_runtime{};
        static bool dm_follow_cfg_logged = false;
        if (!dm_follow_cfg_logged) {
            dm_follow_cfg_logged = true;
            LOGI("[GIMBAL][DM][CFG][J8] db=%.2f ext_db=%.2f tau=%.3f enter=%.2f exit=%.2f share=%.2f max_nm=%.2f slew=%.1f sign=%d nm_per_a=%.2f",
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.follow_cmd_db_A),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.external_cmd_db_A),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.external_lpf_tau_s),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.external_follow_db_enter_A),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.external_follow_db_exit_A),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.torque_share),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.torque_max_nm),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.torque_slew_nm_s),
                 static_cast<int>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.torque_sign),
                 static_cast<double>(GIMBAL_DM_J8_FOLLOW_COMP_CFG.torque_nm_per_A));
            LOGI("[GIMBAL][DM][CFG][J9] db=%.2f ext_db=%.2f tau=%.3f enter=%.2f exit=%.2f share=%.2f max_nm=%.2f slew=%.1f sign=%d nm_per_a=%.2f",
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.follow_cmd_db_A),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.external_cmd_db_A),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.external_lpf_tau_s),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.external_follow_db_enter_A),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.external_follow_db_exit_A),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.torque_share),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.torque_max_nm),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.torque_slew_nm_s),
                 static_cast<int>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.torque_sign),
                 static_cast<double>(GIMBAL_DM_J9_FOLLOW_COMP_CFG.torque_nm_per_A));
        }
        fp32 j8_torque_nm = 0.0f;
        fp32 j9_torque_nm = 0.0f;
        fp32 j8_target_torque_nm = 0.0f;
        fp32 j9_target_torque_nm = 0.0f;
        fp32 j8_cmd_in_A = lk_j2_cmd_A;
        fp32 j9_cmd_in_A = lk_j3_cmd_A;
        fp32 j8_cmd_lpf_A = 0.0f;
        fp32 j9_cmd_lpf_A = 0.0f;
        fp32 j8_follow_cmd_A = 0.0f;
        fp32 j9_follow_cmd_A = 0.0f;
        bool j8_follow_deadband_hit = false;
        bool j9_follow_deadband_hit = false;
        bool j8_follow_active = false;
        bool j9_follow_active = false;
        const bool j8_external_source =
            lk_j2_target_state.source_active &&
            (lk_j2_target_state.source == LkTargetSource::External);
        const bool j9_external_source =
            lk_j3_target_state.source_active &&
            (lk_j3_target_state.source == LkTargetSource::External);
        const auto run_dm_follow_comp = [&](const GimbalDmFollowCompCfg& cfg,
                                            bool axis_enable,
                                            bool external_source,
                                            fp32 cmd_in_A,
                                            DmFollowCompRuntime* rt,
                                            fp32* out_follow_cmd_A,
                                            fp32* out_cmd_lpf_A,
                                            bool* out_follow_active,
                                            bool* out_deadband_hit,
                                            fp32* out_target_torque_nm,
                                            fp32* out_applied_torque_nm) {
            if (rt == nullptr || out_follow_cmd_A == nullptr || out_cmd_lpf_A == nullptr ||
                out_follow_active == nullptr || out_deadband_hit == nullptr ||
                out_target_torque_nm == nullptr || out_applied_torque_nm == nullptr) {
                return;
            }

            if (!axis_enable) {
                rt->torque_prev = 0.0f;
                rt->cmd_lpf_A = 0.0f;
                rt->external_follow_active = false;
                *out_follow_cmd_A = 0.0f;
                *out_cmd_lpf_A = 0.0f;
                *out_follow_active = false;
                *out_deadband_hit = false;
                *out_target_torque_nm = 0.0f;
                *out_applied_torque_nm = 0.0f;
                return;
            }

            fp32 cmd_after_filter_A = cmd_in_A;
            if (GIMBAL_DM_J89_EXTERNAL_FILTER_ENABLE && external_source) {
                if ((cfg.external_cmd_db_A > 0.0f) &&
                    (std::fabs(cmd_after_filter_A) < cfg.external_cmd_db_A)) {
                    cmd_after_filter_A = 0.0f;
                }
                rt->cmd_lpf_A += lpf_alpha(cfg.external_lpf_tau_s) *
                                 (cmd_after_filter_A - rt->cmd_lpf_A);
                cmd_after_filter_A = rt->cmd_lpf_A;

                const fp32 abs_cmd = std::fabs(cmd_after_filter_A);
                if (rt->external_follow_active) {
                    if (abs_cmd <= cfg.external_follow_db_exit_A) {
                        rt->external_follow_active = false;
                    }
                } else if (abs_cmd >= cfg.external_follow_db_enter_A) {
                    rt->external_follow_active = true;
                }
            } else {
                rt->cmd_lpf_A = cmd_after_filter_A;
                rt->external_follow_active = false;
            }

            *out_cmd_lpf_A = rt->cmd_lpf_A;
            bool deadband_hit = false;
            if (GIMBAL_DM_J89_EXTERNAL_FILTER_ENABLE && external_source && !rt->external_follow_active) {
                cmd_after_filter_A = 0.0f;
                deadband_hit = true;
            }
            if (std::fabs(cmd_after_filter_A) < cfg.follow_cmd_db_A) {
                cmd_after_filter_A = 0.0f;
                deadband_hit = true;
            }

            *out_follow_cmd_A = cmd_after_filter_A;
            *out_deadband_hit = deadband_hit;
            *out_follow_active =
                (GIMBAL_DM_J89_EXTERNAL_FILTER_ENABLE && external_source) ? rt->external_follow_active : true;
            *out_target_torque_nm =
                static_cast<fp32>(cfg.torque_sign) *
                cfg.torque_nm_per_A * cfg.torque_share * cmd_after_filter_A;
            fp32 applied_nm = clamp_nm(*out_target_torque_nm, cfg.torque_max_nm);
            applied_nm = slew_nm(rt->torque_prev, applied_nm, cfg.torque_slew_nm_s);
            rt->torque_prev = applied_nm;
            *out_applied_torque_nm = applied_nm;
        };
        run_dm_follow_comp(GIMBAL_DM_J8_FOLLOW_COMP_CFG,
                           j8_enable,
                           j8_external_source,
                           j8_cmd_in_A,
                           &j8_runtime,
                           &j8_follow_cmd_A,
                           &j8_cmd_lpf_A,
                           &j8_follow_active,
                           &j8_follow_deadband_hit,
                           &j8_target_torque_nm,
                           &j8_torque_nm);
        run_dm_follow_comp(GIMBAL_DM_J9_FOLLOW_COMP_CFG,
                           j9_enable,
                           j9_external_source,
                           j9_cmd_in_A,
                           &j9_runtime,
                           &j9_follow_cmd_A,
                           &j9_cmd_lpf_A,
                           &j9_follow_active,
                           &j9_follow_deadband_hit,
                           &j9_target_torque_nm,
                           &j9_torque_nm);
        lk_j2_ff_A = j8_follow_cmd_A;
        lk_j3_ff_A = j9_follow_cmd_A;
        dm_j8.set_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, j8_torque_nm);
        dm_j9.set_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, j9_torque_nm);
        {
            const auto s_j8 = dm_j8.state();
            const auto s_j9 = dm_j9.state();
            gimbal_dbg_dm_j8.online = s_j8.online ? 1u : 0u;
            gimbal_dbg_dm_j8.loop_target = j8_torque_nm;
            gimbal_dbg_dm_j8.loop_actual = s_j8.torque_nm;
            gimbal_dbg_dm_j8.pos_target = 0.0f;
            gimbal_dbg_dm_j8.pos_actual = s_j8.pos_rad;
            gimbal_dbg_dm_j8.vel_target = 0.0f;
            gimbal_dbg_dm_j8.vel_actual = s_j8.speed_rad_s;
            gimbal_dbg_dm_j8.torque_target = j8_torque_nm;
            gimbal_dbg_dm_j8.torque_actual = s_j8.torque_nm;
            gimbal_dbg_dm_j8.p_out = 0.0f;
            gimbal_dbg_dm_j8.i_out = 0.0f;
            gimbal_dbg_dm_j8.d_out = 0.0f;
            gimbal_dbg_dm_j8.pid_out = 0.0f;
            gimbal_dbg_dm_j8.mode = 2;

            gimbal_dbg_dm_j9.online = s_j9.online ? 1u : 0u;
            gimbal_dbg_dm_j9.loop_target = j9_torque_nm;
            gimbal_dbg_dm_j9.loop_actual = static_cast<fp32>(s_j9.torque_raw);
            gimbal_dbg_dm_j9.pos_target = 0.0f;
            gimbal_dbg_dm_j9.pos_actual = s_j9.pos_rad_total;
            gimbal_dbg_dm_j9.vel_target = 0.0f;
            gimbal_dbg_dm_j9.vel_actual = s_j9.speed_rad_s;
            gimbal_dbg_dm_j9.torque_target = j9_torque_nm;
            gimbal_dbg_dm_j9.torque_actual = static_cast<fp32>(s_j9.torque_raw);
            gimbal_dbg_dm_j9.p_out = 0.0f;
            gimbal_dbg_dm_j9.i_out = 0.0f;
            gimbal_dbg_dm_j9.d_out = 0.0f;
            gimbal_dbg_dm_j9.pid_out = 0.0f;
            gimbal_dbg_dm_j9.mode = 2;

            if (lk_exec_log_due) {
                LOGI("[GIMBAL][DM][EXEC][J8] cmd_in_A=%.3f cmd_lpf_A=%.3f follow_active=%u db_hit=%u target_torque_nm=%.3f applied_torque=%.3f ext=%u",
                     static_cast<double>(j8_cmd_in_A),
                     static_cast<double>(j8_cmd_lpf_A),
                     j8_follow_active ? 1u : 0u,
                     j8_follow_deadband_hit ? 1u : 0u,
                     static_cast<double>(j8_target_torque_nm),
                     static_cast<double>(j8_torque_nm),
                     j8_external_source ? 1u : 0u);
                LOGI("[GIMBAL][DM][EXEC][J9] cmd_in_A=%.3f cmd_lpf_A=%.3f follow_active=%u db_hit=%u target_torque_nm=%.3f applied_torque=%.3f ext=%u",
                     static_cast<double>(j9_cmd_in_A),
                     static_cast<double>(j9_cmd_lpf_A),
                     j9_follow_active ? 1u : 0u,
                     j9_follow_deadband_hit ? 1u : 0u,
                     static_cast<double>(j9_target_torque_nm),
                     static_cast<double>(j9_torque_nm),
                     j9_external_source ? 1u : 0u);
            }
        }

        {
            uint16_t sid; uint8_t buf[8];
            if (dm_j8.exportTxRaw8(&sid, buf)) { (void)cantx_send_now(GIMBAL_DM_J8_BUS, sid, buf, 8, 0); }
            if (dm_j9.exportTxRaw8(&sid, buf)) { (void)cantx_send_now(GIMBAL_DM_J9_BUS, sid, buf, 8, 0); }
        }

        const auto s_j3 = lk_j3.state();
        const auto s_j2 = lk_j2.state();
        const auto s_j1 = lk_j1.state();
        const auto s_j8 = dm_j8.state();
        const auto s_j9 = dm_j9.state();
        const uint16_t s_j3_norm_raw =
            lk_normalize_encoder_raw_cross_zero_high_to_zero(s_j3.encoder_raw, GIMBAL_LK_J3_POS_CFG);
        const uint16_t s_j2_norm_raw =
            lk_normalize_encoder_raw_cross_zero_high_to_zero(s_j2.encoder_raw, GIMBAL_LK_J2_POS_CFG);

        uint16_t dm_raw_u16[4] = {0u, 0u, 0u, 0u};
        int16_t dm_angle_cdeg[4] = {0, 0, 0, 0};
        uint8_t dm_online[4] = {0u, 0u, 0u, 0u};
        const bool dm_snap_ok = Gimbal_ReadDmJ4J7Snapshot(dm_raw_u16, dm_angle_cdeg, dm_online);

        gimbal_joint_snapshot_t joint_snap{};
        joint_snap.raw_u16[0] = s_j1.encoder_raw;
        joint_snap.raw_u16[1] = s_j2_norm_raw;
        joint_snap.raw_u16[2] = s_j3_norm_raw;
        joint_snap.raw_u16[3] = dm_snap_ok ? dm_raw_u16[0] : 0u;
        joint_snap.raw_u16[4] = dm_snap_ok ? dm_raw_u16[1] : 0u;
        joint_snap.raw_u16[5] = dm_snap_ok ? dm_raw_u16[2] : 0u;
        joint_snap.raw_u16[6] = dm_snap_ok ? dm_raw_u16[3] : 0u;
        joint_snap.raw_u16[7] = dm4340_pos_raw_to_u16(s_j8.pos_raw);
        joint_snap.raw_u16[8] = DM4310_PosRawToU16(s_j9.pos_raw);

        const auto lk_rel_cdeg_from_zero = [enc_to_deg, wrap_err](uint16_t encoder_raw, fp32 zero_deg) -> int16_t {
            return static_cast<int16_t>(Omnix_DegToCdeg(wrap_err(enc_to_deg(encoder_raw) - zero_deg)));
        };
        joint_snap.angle_cdeg[0] = lk_rel_cdeg_from_zero(s_j1.encoder_raw, GIMBAL_LK_J1_ZERO_DEG);
        joint_snap.angle_cdeg[1] = lk_rel_cdeg_from_zero(s_j2_norm_raw, GIMBAL_LK_J2_ZERO_DEG);
        joint_snap.angle_cdeg[2] = lk_rel_cdeg_from_zero(s_j3_norm_raw, GIMBAL_LK_J3_ZERO_DEG);
        joint_snap.angle_cdeg[3] = dm_snap_ok ? dm_angle_cdeg[0] : 0;
        joint_snap.angle_cdeg[4] = dm_snap_ok ? dm_angle_cdeg[1] : 0;
        joint_snap.angle_cdeg[5] = dm_snap_ok ? dm_angle_cdeg[2] : 0;
        joint_snap.angle_cdeg[6] = dm_snap_ok ? dm_angle_cdeg[3] : 0;
        joint_snap.angle_cdeg[7] = static_cast<int16_t>(Omnix_RadToCdeg(s_j8.pos_rad));
        joint_snap.angle_cdeg[8] = static_cast<int16_t>(Omnix_RadToCdeg(s_j9.pos_rad));

        joint_snap.online[0] = s_j1.online ? 1u : 0u;
        joint_snap.online[1] = s_j2.online ? 1u : 0u;
        joint_snap.online[2] = s_j3.online ? 1u : 0u;
        joint_snap.online[3] = dm_snap_ok ? dm_online[0] : 0u;
        joint_snap.online[4] = dm_snap_ok ? dm_online[1] : 0u;
        joint_snap.online[5] = dm_snap_ok ? dm_online[2] : 0u;
        joint_snap.online[6] = dm_snap_ok ? dm_online[3] : 0u;
        joint_snap.online[7] = s_j8.online ? 1u : 0u;
        joint_snap.online[8] = s_j9.online ? 1u : 0u;

        __DMB();
        g_joint_snap_guard++;
        __DMB();
        g_joint_snap = joint_snap;
        __DMB();
        g_joint_snap_guard++;
        __DMB();

        last_joint_snap = joint_snap;
        prev_remote_safe_force = remote_safe_force;

        __DMB();
        g_pause_guard++;
        __DMB();
        if (mapping_reset_run) {
            g_pause_snap.phase = static_cast<uint8_t>(GIMBAL_PAUSE_RUN);
        } else if (mapping_hold && g_mapping_reset_local_done) {
            g_pause_snap.phase = static_cast<uint8_t>(GIMBAL_PAUSE_DONE_BEEP);
        } else {
            g_pause_snap.phase = static_cast<uint8_t>(pause_phase);
        }
        for (int i = 0; i < 9; ++i) {
            g_pause_snap.cmd_cdeg[i] = pause_cmd_cdeg[i];
        }
        __DMB();
        g_pause_guard++;
        __DMB();

        if (mapping_hold && ((now_ms - mapping_log_last_ms) >= GIMBAL_MAPPING_LOG_PERIOD_MS)) {
            mapping_log_last_ms = now_ms;
            LOGI("[MAP][MOTOR][RX][LK] J1 raw=%u cdeg=%d J2 raw=%u cdeg=%d J3 raw=%u cdeg=%d",
                 static_cast<unsigned>(joint_snap.raw_u16[0]),
                 static_cast<int>(joint_snap.angle_cdeg[0]),
                 static_cast<unsigned>(joint_snap.raw_u16[1]),
                 static_cast<int>(joint_snap.angle_cdeg[1]),
                 static_cast<unsigned>(joint_snap.raw_u16[2]),
                 static_cast<int>(joint_snap.angle_cdeg[2]));
            LOGI("[MAP][MOTOR][RX][DM] J4 raw=%u cdeg=%d J5 raw=%u cdeg=%d J6 raw=%u cdeg=%d J7 raw=%u cdeg=%d",
                 static_cast<unsigned>(joint_snap.raw_u16[3]),
                 static_cast<int>(joint_snap.angle_cdeg[3]),
                 static_cast<unsigned>(joint_snap.raw_u16[4]),
                 static_cast<int>(joint_snap.angle_cdeg[4]),
                 static_cast<unsigned>(joint_snap.raw_u16[5]),
                 static_cast<int>(joint_snap.angle_cdeg[5]),
                 static_cast<unsigned>(joint_snap.raw_u16[6]),
                 static_cast<int>(joint_snap.angle_cdeg[6]));
            LOGI("[MAP][MOTOR][RX][DM] J8 raw=%u cdeg=%d J9 raw=%u cdeg=%d",
                 static_cast<unsigned>(joint_snap.raw_u16[7]),
                 static_cast<int>(joint_snap.angle_cdeg[7]),
                 static_cast<unsigned>(joint_snap.raw_u16[8]),
                 static_cast<int>(joint_snap.angle_cdeg[8]));
            LOGI("[MAP][MOTOR][TX] local_done=%u hold=%u J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d J7=%d J8=%d J9=%d (cdeg)",
                 g_mapping_reset_local_done ? 1u : 0u,
                 mapping_hold ? 1u : 0u,
                 static_cast<int>(pause_cmd_cdeg[0]),
                 static_cast<int>(pause_cmd_cdeg[1]),
                 static_cast<int>(pause_cmd_cdeg[2]),
                 static_cast<int>(pause_cmd_cdeg[3]),
                 static_cast<int>(pause_cmd_cdeg[4]),
                 static_cast<int>(pause_cmd_cdeg[5]),
                 static_cast<int>(pause_cmd_cdeg[6]),
                 static_cast<int>(pause_cmd_cdeg[7]),
                 static_cast<int>(pause_cmd_cdeg[8]));
        }

#if GIMBAL_LOG_J1_J9_POS
        static uint32_t j1_j9_log_last_ms = 0;
        const uint32_t j1_j9_now_ms = HAL_GetTick();
        if (j1_j9_now_ms - j1_j9_log_last_ms >= GIMBAL_LOG_J1_J9_POS_PERIOD_MS) {
            j1_j9_log_last_ms = j1_j9_now_ms;
            const int j1_cdeg = static_cast<int>(joint_snap.angle_cdeg[0]);
            const int j2_cdeg = static_cast<int>(joint_snap.angle_cdeg[1]);
            const int j3_cdeg = static_cast<int>(joint_snap.angle_cdeg[2]);
            const int j4_cdeg = static_cast<int>(joint_snap.angle_cdeg[3]);
            const int j5_cdeg = static_cast<int>(joint_snap.angle_cdeg[4]);
            const int j6_cdeg = static_cast<int>(joint_snap.angle_cdeg[5]);
            const int j7_cdeg = static_cast<int>(joint_snap.angle_cdeg[6]);
            const int j8_cdeg = static_cast<int>(joint_snap.angle_cdeg[7]);
            const int j9_cdeg = static_cast<int>(joint_snap.angle_cdeg[8]);

            const int j1_on = joint_snap.online[0] ? 1 : 0;
            const int j2_on = joint_snap.online[1] ? 1 : 0;
            const int j3_on = joint_snap.online[2] ? 1 : 0;
            const int j4_on = joint_snap.online[3] ? 1 : 0;
            const int j5_on = joint_snap.online[4] ? 1 : 0;
            const int j6_on = joint_snap.online[5] ? 1 : 0;
            const int j7_on = joint_snap.online[6] ? 1 : 0;
            const int j8_on = joint_snap.online[7] ? 1 : 0;
            const int j9_on = joint_snap.online[8] ? 1 : 0;

            LOGI("[POS][cdeg] J1=%d(%d) J2=%d(%d) J3=%d(%d)",
                 j1_cdeg, j1_on,
                 j2_cdeg, j2_on,
                 j3_cdeg, j3_on);
            LOGI("[POS][cdeg] J4=%d(%d) J5=%d(%d) J6=%d(%d)",
                 j4_cdeg, j4_on,
                 j5_cdeg, j5_on,
                 j6_cdeg, j6_on);
            LOGI("[POS][cdeg] J7=%d(%d) J8=%d(%d) J9=%d(%d)",
                 j7_cdeg, j7_on,
                 j8_cdeg, j8_on,
                 j9_cdeg, j9_on);
        }
#endif

        // Export & enqueue frames to TX Router for CAN1 (bus=1).
        // Use cantx_send_raw to let the TX router send periodically.
        {
            uint16_t sid; uint8_t buf[8];
            static uint32_t j3_next_pos_tx_ms = 0u;
            static uint32_t j2_next_pos_tx_ms = 0u;
#if GIMBAL_TEST_J3_A8_TX_ONESHOT
            static bool j3_a8_sent_once = false;
#endif
            if (lk_j3.exportTxRaw8(&sid, buf)) {
                bool allow_j3_tx = true;
#if GIMBAL_TEST_J23_DROP_A1_TX
                if (buf[0] == 0xA1u) {
                    allow_j3_tx = false;
                }
#endif
#if GIMBAL_TEST_J23_POS_TX_THROTTLE_ENABLE
                if (allow_j3_tx && (buf[0] == 0xA6u || buf[0] == 0xA8u)) {
                    if (now_ms < j3_next_pos_tx_ms) {
                        allow_j3_tx = false;
                    } else {
                        j3_next_pos_tx_ms = now_ms + static_cast<uint32_t>(GIMBAL_TEST_J23_POS_TX_PERIOD_MS);
                    }
                }
#endif
#if GIMBAL_TEST_J3_A8_TX_ONESHOT
                if (buf[0] == 0xA8u) {
                    if (j3_a8_sent_once) {
                        allow_j3_tx = false;
                    } else {
                        j3_a8_sent_once = true;
                    }
                }
#endif
                if (allow_j3_tx) {
#if GIMBAL_LK_CAN_RAW_LOG_ENABLE
                    lk_can_raw_tx_tap(&g_lk_can_raw_j3, sid, buf, 8u, now_ms);
#endif
                    GIMBAL_J23_TX_LOGI("[TEST][LK_TX][J3] bus=%u sid=0x%03X data=%02X %02X %02X %02X %02X %02X %02X %02X",
                                       static_cast<unsigned>(GIMBAL_LK_J3_BUS),
                                       static_cast<unsigned>(sid),
                                       static_cast<unsigned>(buf[0]),
                                       static_cast<unsigned>(buf[1]),
                                       static_cast<unsigned>(buf[2]),
                                       static_cast<unsigned>(buf[3]),
                                       static_cast<unsigned>(buf[4]),
                                       static_cast<unsigned>(buf[5]),
                                       static_cast<unsigned>(buf[6]),
                                       static_cast<unsigned>(buf[7]));
                    (void)cantx_send_now(GIMBAL_LK_J3_BUS, sid, buf, 8, 0);
                }
            }
            if (lk_j2.exportTxRaw8(&sid, buf)) {
                bool allow_j2_tx = true;
#if GIMBAL_TEST_J23_DROP_A1_TX
                if (buf[0] == 0xA1u) {
                    allow_j2_tx = false;
                }
#endif
#if GIMBAL_TEST_J23_POS_TX_THROTTLE_ENABLE
                if (allow_j2_tx && (buf[0] == 0xA6u || buf[0] == 0xA8u)) {
                    if (now_ms < j2_next_pos_tx_ms) {
                        allow_j2_tx = false;
                    } else {
                        j2_next_pos_tx_ms = now_ms + static_cast<uint32_t>(GIMBAL_TEST_J23_POS_TX_PERIOD_MS);
                    }
                }
#endif
                if (allow_j2_tx) {
#if GIMBAL_LK_CAN_RAW_LOG_ENABLE
                    lk_can_raw_tx_tap(&g_lk_can_raw_j2, sid, buf, 8u, now_ms);
#endif
                    GIMBAL_J23_TX_LOGI("[TEST][LK_TX][J2] bus=%u sid=0x%03X data=%02X %02X %02X %02X %02X %02X %02X %02X",
                                       static_cast<unsigned>(GIMBAL_LK_J2_BUS),
                                       static_cast<unsigned>(sid),
                                       static_cast<unsigned>(buf[0]),
                                       static_cast<unsigned>(buf[1]),
                                       static_cast<unsigned>(buf[2]),
                                       static_cast<unsigned>(buf[3]),
                                       static_cast<unsigned>(buf[4]),
                                       static_cast<unsigned>(buf[5]),
                                       static_cast<unsigned>(buf[6]),
                                       static_cast<unsigned>(buf[7]));
                    (void)cantx_send_now(GIMBAL_LK_J2_BUS, sid, buf, 8, 0);
                }
            }
            if (lk_j1.exportTxRaw8(&sid, buf)) { (void)cantx_send_now(GIMBAL_LK_J1_BUS, sid, buf, 8, 0); }
        }
#if GIMBAL_LK_CAN_RAW_LOG_ENABLE
        static uint32_t lk_can_raw_log_last_ms = 0u;
        static uint32_t lk_j3_prev_rx_count = 0u;
        static uint32_t lk_j2_prev_rx_count = 0u;
        if ((now_ms - lk_can_raw_log_last_ms) >= GIMBAL_LK_CAN_RAW_LOG_PERIOD_MS) {
            lk_can_raw_log_last_ms = now_ms;
            LkCanRawTapSnapshot j3_snap{};
            LkCanRawTapSnapshot j2_snap{};
            (void)lk_can_raw_tap_copy(&g_lk_can_raw_j3, &j3_snap);
            (void)lk_can_raw_tap_copy(&g_lk_can_raw_j2, &j2_snap);
            lk_can_raw_log_axis("J3", static_cast<uint8_t>(GIMBAL_LK_J3_BUS), j3_snap, now_ms, &lk_j3_prev_rx_count);
            lk_can_raw_log_axis("J2", static_cast<uint8_t>(GIMBAL_LK_J2_BUS), j2_snap, now_ms, &lk_j2_prev_rx_count);
        }
#endif
    }
  }

// 閹笛嗩攽娴犺濮熸笟褑顕伴崣鏍ф彥閻撗嶇礄閹绘劒绶垫径鏍劥閸欘垵顫嗙粭锕€褰块敍灞肩返 Gimbal 閹笛嗩攽娴犺濮熺拫鍐暏閿?
bool read_gimbal_cmd(gimbal_cmd_t* out){
    uint32_t s1, s2;
    do{
        s1 = g_cmd.seq; __DMB();
        *out = g_cmd;   __DMB();
        s2 = g_cmd.seq;
    }while(s1 != s2);
    return true;
    }
