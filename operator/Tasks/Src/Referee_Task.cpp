#include "../Inc/Referee_Task.h"

#include "bsp_srn_log.h"
#include "bsp_vt03.h"
#include "cmsis_os2.h"
#include "CONF_Gimbal_Task.h"
#include "Gimbal_behavior_Task.h"
#include "RC_Control_Mode.h"
#include "lib_remote_control.h"
#include "stm32h7xx_hal.h"
#include "usart.h"

#include <cstring>

extern "C" {
volatile referee_debug_probe_t g_referee_debug_probe __attribute__((used)) = {};
}

#ifdef OMNIX_FEATURE_MPU_DCACHE
  #if __has_include("mem_sections.h")
    #include "mem_sections.h"
    #define REFEREE_DMA_SEC SEC_DMA_NC_BUF
  #else
    #define REFEREE_DMA_SEC
  #endif
#else
  #define REFEREE_DMA_SEC
#endif

#ifndef REFEREE_RX_FRAME_HEADER_TRACE_ENABLE
#define REFEREE_RX_FRAME_HEADER_TRACE_ENABLE 0
#endif

#ifndef REF_RX_DBG_ENABLE
#define REF_RX_DBG_ENABLE 0
#endif

/* Minimal custom-controller logs:
 * keep only [CTRL][0302][RX] and [REF][0309][TX] by default.
 */
#ifndef REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
#define REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309 1
#endif

#ifndef REF_RX_DBG_SUMMARY_MS
#define REF_RX_DBG_SUMMARY_MS 1000u
#endif

#ifndef REF_RX_DBG_BURST_MS
#define REF_RX_DBG_BURST_MS 3000u
#endif

#ifndef REF_RX_DBG_RESYNC_WARN_BYTES
#define REF_RX_DBG_RESYNC_WARN_BYTES 64u
#endif

#ifndef REF_RX_DBG_SILENCE_WARN_MS
#define REF_RX_DBG_SILENCE_WARN_MS 1500u
#endif

#define REF_RX_DBG_TRIG_OVERFLOW 0x01u
#define REF_RX_DBG_TRIG_CRC      0x02u
#define REF_RX_DBG_TRIG_UART     0x04u

#ifndef REF_RX_DBG_BURST_TRIGGER_MASK
#define REF_RX_DBG_BURST_TRIGGER_MASK (REF_RX_DBG_TRIG_OVERFLOW | REF_RX_DBG_TRIG_CRC | REF_RX_DBG_TRIG_UART)
#endif

namespace {

constexpr uint8_t REFEREE_SOF = 0xA5u;
constexpr uint16_t REFEREE_CMD_ID_CUSTOM_CONTROLLER_TO_ROBOT = 0x0302u;
constexpr uint16_t REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER = 0x0309u;
constexpr uint16_t REFEREE_CUSTOM_DATA_LEN = OMNIX_REFEREE_CUSTOM_DATA_LEN;
constexpr uint16_t REFEREE_HEADER_LEN = 5u;
constexpr uint16_t REFEREE_CMD_ID_LEN = 2u;
constexpr uint16_t REFEREE_TAIL_LEN = 2u;
constexpr uint16_t REFEREE_MIN_FRAME_LEN = REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_TAIL_LEN;
constexpr uint16_t REFEREE_MAX_FRAME_LEN = 256u;
constexpr uint16_t REFEREE_TX_FRAME_LEN =
    REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_CUSTOM_DATA_LEN + REFEREE_TAIL_LEN;
constexpr uint32_t REFEREE_TX_INTERVAL_ACTIVE_MS = 100u;
constexpr uint32_t REFEREE_TX_INTERVAL_IDLE_MS = 200u;
constexpr uint16_t REFEREE_RX_FIFO_SIZE = 1024u;
constexpr uint16_t REFEREE_RX_PARSE_BUF_SIZE = 512u;
constexpr uint32_t REFEREE_SOURCE_SWITCH_SILENCE_MS = 500u;
constexpr uint8_t REFEREE_SOURCE_SWITCH_GOOD_FRAMES = 3u;

uint16_t referee_joint_rel_cdeg_to_controller_raw(uint8_t joint_index, int16_t angle_cdeg)
{
    const fp32 angle_rad = Omnix_CdegToRad(angle_cdeg);

    switch (joint_index) {
        case 0u:
            return Gimbal_MapRelAngleToExternalRaw(angle_rad, GIMBAL_J1_EXT_BIDIR_REL_ANGLE_MAP_CFG);
        case 1u:
            return Gimbal_MapHalfRangeReverseRelAngleToExternalRaw(
                angle_rad,
                GIMBAL_J2_EXT_HALF_RANGE_REVERSE_MAP_CFG);
        case 2u:
            return Gimbal_MapHalfRangeReverseRelAngleToExternalRaw(
                angle_rad,
                GIMBAL_J3_EXT_HALF_RANGE_REVERSE_MAP_CFG);
        case 3u:
            return Gimbal_MapRelAngleToExternalRaw(-angle_rad, GIMBAL_J4_EXT_BIDIR_REL_ANGLE_MAP_CFG);
        case 4u:
            return Gimbal_MapRelAngleToExternalRaw(angle_rad, GIMBAL_J5_EXT_BIDIR_REL_ANGLE_MAP_CFG);
        case 5u:
            return Gimbal_MapRelAngleToExternalRaw(angle_rad, GIMBAL_J6_EXT_BIDIR_REL_ANGLE_MAP_CFG);
        case 6u:
            return Gimbal_MapRelAngleToExternalRaw(angle_rad, GIMBAL_J7_EXT_BIDIR_REL_ANGLE_MAP_CFG);
        default:
            return 0u;
    }
}
constexpr const char* REFEREE_LINK_KIND = "vtx";
constexpr const char* REFEREE_UART_NAME = "uart9";
constexpr uint32_t REFEREE_LINK_BAUDRATE = 921600u;
constexpr const char* REFEREE_RX_SOURCE = "vtx_uart9";
constexpr const char* REFEREE_TX_SOURCE = "vtx_uart9";
constexpr uint8_t REFEREE_LATENCY_PROBE_ENABLE_MASK = 0x01u;
constexpr size_t REFEREE_LATENCY_WINDOW_SIZE = 128u;
constexpr uint8_t REFEREE_GATE_REASON_VT03_ONLINE = 0x01u;
constexpr uint8_t REFEREE_GATE_REASON_ACTIVE_VT03 = 0x02u;
constexpr uint8_t REFEREE_GATE_REASON_GEAR_MAIN = 0x04u;
constexpr uint8_t REFEREE_GATE_REASON_CMD_FRESH = 0x08u;
constexpr uint8_t REFEREE_GATE_REASON_ZERO_FORCE = 0x10u;
static_assert((REFEREE_RX_FIFO_SIZE & (REFEREE_RX_FIFO_SIZE - 1u)) == 0u, "referee fifo size must be power of two");
static_assert(REFEREE_RX_PARSE_BUF_SIZE >= REFEREE_MAX_FRAME_LEN, "referee parse buffer must hold max frame");
static_assert(OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE <= REFEREE_CUSTOM_DATA_LEN, "joint raw payload exceeds 0x0302 data space");
static_assert(OMNIX_ROBOT_JOINT_FEEDBACK_PAYLOAD_SIZE == REFEREE_CUSTOM_DATA_LEN, "0x0309 payload must be 30 bytes");

volatile uint32_t g_joint_rx_guard = 0u;
referee_joint_raw_cmd_t g_joint_rx_cmd = {
    false,
    false,
    0u,
    0u,
    {0u, 0u, 0u, 0u, 0u, 0u, 0u},
    0u,
    0u
};

volatile uint32_t g_stats_guard = 0u;
referee_rx_stats_t g_rx_stats = {};

volatile uint32_t g_raw_guard = 0u;
bool g_raw_0302_valid = false;
uint8_t g_raw_0302[REFEREE_CUSTOM_DATA_LEN] = {0};

volatile uint32_t g_joint_feedback_tx_guard = 0u;
omnix_robot_joint_feedback_payload_t g_joint_feedback_tx_payload = {
    OMNIX_ROBOT_JOINT_FEEDBACK_VERSION,
    0u,
    0u,
    0u,
    0u,
    {0, 0, 0, 0, 0, 0, 0},
    0u,
    0u,
    {0, 0, 0, 0, 0, 0, 0, 0}
};
volatile uint32_t g_joint_feedback_tx_tick_ms = 0u;
uint8_t g_referee_tx_seq = 0u;
uint8_t g_referee_tx_frame[REFEREE_TX_FRAME_LEN] = {0};

volatile bool g_external_control_enabled = false;
volatile uint8_t g_remote_buzzer_pulses_pending = 0u;
volatile uint8_t g_remote_buzzer_tx_high_phase = 0u;
volatile bool g_mapping_reset_active = false;
bool g_mapping_reset_controller_done = false;

volatile uint8_t g_referee_rx_fifo[REFEREE_RX_FIFO_SIZE] = {0};
volatile uint16_t g_referee_rx_head = 0u;
volatile uint16_t g_referee_rx_tail = 0u;
uint16_t g_referee_rx_parse_len = 0u;
uint8_t g_referee_rx_parse_buf[REFEREE_RX_PARSE_BUF_SIZE] = {0};

struct referee_uart_diag_t {
    uint32_t rx_event_count;
    uint32_t rx_event_bytes;
    uint32_t rx_event_invalid_size;
    uint32_t rx_referee_feed_bytes;
    uint32_t rx_vt03_feed_bytes;
    uint32_t rx_vt03_feed_events;
    uint32_t rx_arm_ok;
    uint32_t rx_arm_busy;
    uint32_t rx_arm_err;
    uint32_t rx_arm_last_ret;
    uint32_t rx_armed;
    uint32_t rx_hw_error_count;
    uint32_t rx_hw_error_last_code;
    uint32_t rx_hw_error_ore_count;
    uint32_t rx_hw_error_fe_count;
    uint32_t rx_hw_error_ne_count;
};

struct referee_rx_dbg_diag_t {
    uint16_t fifo_used_last;
    uint16_t fifo_used_max;
    uint16_t parse_buf_used_max;
    uint16_t rx_evt_size_min;
    uint16_t rx_evt_size_max;
    uint32_t fifo_overflow_burst;
    uint32_t parse_buf_slide_count;
    uint32_t valid_0302_interval_last_ms;
    uint32_t valid_0302_interval_max_ms;
    uint32_t valid_0302_silence_ms;
    uint32_t rx_evt_zero_or_oversize_count;
    uint32_t resync_drop_run_max;
    uint32_t resync_drop_run_cur;
    uint32_t last_valid_0302_tick_ms;
};

struct referee_rx_dbg_runtime_t {
    uint32_t task_start_ms;
    uint32_t last_summary_ms;
    uint32_t last_silence_warn_ms;
    uint32_t last_resync_warn_ms;
    uint8_t burst_active;
    uint8_t burst_trigger_mask;
    uint32_t burst_end_ms;
    uint32_t burst_frame_count;
    uint32_t burst_crc_fail_count;
    uint32_t burst_overflow_count;
    uint32_t burst_resync_drop_run_max;
    uint32_t last_uart_err_seen;
    referee_rx_stats_t prev_stats;
    referee_uart_diag_t prev_uart_diag;
};

volatile uint32_t g_uart_diag_guard = 0u;
referee_uart_diag_t g_uart_diag = {};
referee_rx_dbg_diag_t g_referee_rx_dbg_diag = {
    0u, 0u, 0u, 0xFFFFu, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u
};
referee_rx_dbg_runtime_t g_referee_rx_dbg_rt = {};
volatile uint32_t g_referee_rx_dbg_pending_trigger_mask = 0u;
volatile uint32_t g_referee_rx_dbg_pending_overflow_count = 0u;
uint8_t g_referee_task_buf[128] = {0};
referee_joint_raw_cmd_t g_referee_task_cmd = {};
referee_rx_stats_t g_referee_task_stats = {};
referee_uart_diag_t g_referee_task_uart_diag = {};
omnix_robot_joint_feedback_payload_t g_referee_task_tx_feedback = {};

struct referee_latency_hist_t {
    uint16_t samples[REFEREE_LATENCY_WINDOW_SIZE];
    uint16_t count;
    uint16_t head;
    uint16_t min_ms;
    uint16_t max_ms;
    uint16_t last_ms;
    uint32_t total_samples;
    uint8_t has_sample;
};

struct referee_latency_state_t {
    referee_latency_hist_t ingress_age_ms;
    referee_latency_hist_t board_rx_to_tx_ms;
    uint32_t last_controller_tick_ms;
    uint32_t last_board_rx_tick_ms;
    uint8_t probe_enabled;
};

struct referee_latency_snapshot_t {
    uint8_t has_sample;
    uint32_t ingress_total_samples;
    uint16_t ingress_last_ms;
    uint16_t ingress_min_ms;
    uint16_t ingress_p50_ms;
    uint16_t ingress_p95_ms;
    uint16_t ingress_max_ms;
    uint32_t board_total_samples;
    uint16_t board_last_ms;
    uint16_t board_min_ms;
    uint16_t board_p50_ms;
    uint16_t board_p95_ms;
    uint16_t board_max_ms;
};

struct referee_custom_rx_runtime_t {
    referee_custom_rx_path_diag_t paths[3];
    RefCustomCtrlRxSource active_source;
    uint32_t active_source_since_ms;
};

referee_latency_state_t g_referee_latency = {};
volatile uint32_t g_custom_rx_guard = 0u;
referee_custom_rx_runtime_t g_custom_rx_runtime = {};

const char* referee_custom_rx_source_name(RefCustomCtrlRxSource source)
{
    switch (source) {
        case REF_CUSTOM_CTRL_RX_SOURCE_VTX_RAW:
            return "vtx_raw";
        case REF_CUSTOM_CTRL_RX_SOURCE_SERVER_BRIDGE:
            return "server_bridge";
        case REF_CUSTOM_CTRL_RX_SOURCE_NONE:
        default:
            return "none";
    }
}

size_t referee_custom_rx_source_index(RefCustomCtrlRxSource source)
{
    switch (source) {
        case REF_CUSTOM_CTRL_RX_SOURCE_VTX_RAW:
            return 1u;
        case REF_CUSTOM_CTRL_RX_SOURCE_SERVER_BRIDGE:
            return 2u;
        case REF_CUSTOM_CTRL_RX_SOURCE_NONE:
        default:
            return 0u;
    }
}

template <typename Fn>
void referee_custom_rx_mutate(Fn&& fn)
{
    __DMB();
    g_custom_rx_guard++;
    __DMB();
    fn(g_custom_rx_runtime);
    __DMB();
    g_custom_rx_guard++;
    __DMB();
}

bool referee_custom_rx_copy(referee_custom_rx_runtime_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    referee_custom_rx_runtime_t local{};

    do {
        seq_a = g_custom_rx_guard;
        __DMB();
        local = g_custom_rx_runtime;
        __DMB();
        seq_b = g_custom_rx_guard;
    } while (seq_a != seq_b);

    *out = local;
    return true;
}

referee_custom_rx_path_diag_t* referee_custom_rx_path_mut(referee_custom_rx_runtime_t& runtime,
                                                          RefCustomCtrlRxSource source)
{
    return &runtime.paths[referee_custom_rx_source_index(source)];
}

const referee_custom_rx_path_diag_t* referee_custom_rx_path(const referee_custom_rx_runtime_t& runtime,
                                                            RefCustomCtrlRxSource source)
{
    return &runtime.paths[referee_custom_rx_source_index(source)];
}

void referee_custom_rx_note_ingress(RefCustomCtrlRxSource source, uint32_t bytes, uint32_t events)
{
    referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
        referee_custom_rx_path_diag_t* path = referee_custom_rx_path_mut(runtime, source);
        path->bytes += bytes;
        path->events += events;
    });
}

bool referee_custom_rx_select_active_locked(referee_custom_rx_runtime_t& runtime,
                                            RefCustomCtrlRxSource candidate,
                                            uint32_t now_ms)
{
    referee_custom_rx_path_diag_t* candidate_path = referee_custom_rx_path_mut(runtime, candidate);
    if (runtime.active_source == REF_CUSTOM_CTRL_RX_SOURCE_NONE) {
        runtime.active_source = candidate;
        runtime.active_source_since_ms = now_ms;
        return true;
    }

    if (runtime.active_source == candidate) {
        return true;
    }

    const referee_custom_rx_path_diag_t* active_path = referee_custom_rx_path(runtime, runtime.active_source);
    const bool active_stale =
        (active_path->last_good_tick_ms == 0u) ||
        ((now_ms - active_path->last_good_tick_ms) > REFEREE_SOURCE_SWITCH_SILENCE_MS);
    if (active_stale &&
        (candidate_path->consecutive_good_frames >= REFEREE_SOURCE_SWITCH_GOOD_FRAMES)) {
        runtime.active_source = candidate;
        runtime.active_source_since_ms = now_ms;
        return true;
    }

    return false;
}

bool referee_custom_rx_note_valid_frame(RefCustomCtrlRxSource source,
                                        uint16_t seq,
                                        uint32_t now_ms)
{
    bool selected = false;
    referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
        referee_custom_rx_path_diag_t* path = referee_custom_rx_path_mut(runtime, source);
        path->cmd_0302_good++;
        if (path->last_seq_valid != 0u) {
            const uint16_t expected_seq = static_cast<uint16_t>(path->last_seq + 1u);
            if (seq != expected_seq) {
                path->seq_gap_count++;
                path->consecutive_good_frames = 1u;
            } else if (path->consecutive_good_frames < 0xFFu) {
                path->consecutive_good_frames++;
            }
        } else {
            path->consecutive_good_frames = 1u;
        }
        path->last_seq = seq;
        path->last_seq_valid = 1u;
        path->last_good_tick_ms = now_ms;
        selected = referee_custom_rx_select_active_locked(runtime, source, now_ms);
    });
    return selected;
}


static inline uint16_t referee_read_u16_le(const uint8_t* src)
{
    if (src == nullptr) {
        return 0u;
    }
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8u));
}

static inline void referee_write_u16_le(uint8_t* dst, uint16_t value)
{
    if (dst == nullptr) {
        return;
    }
    dst[0] = (uint8_t)(value & 0x00FFu);
    dst[1] = (uint8_t)((value >> 8u) & 0x00FFu);
}

static inline uint32_t referee_read_u32_le(const uint8_t* src)
{
    if (src == nullptr) {
        return 0u;
    }
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8u) |
           ((uint32_t)src[2] << 16u) |
           ((uint32_t)src[3] << 24u);
}

static inline void referee_write_u32_le(uint8_t* dst, uint32_t value)
{
    if (dst == nullptr) {
        return;
    }
    dst[0] = (uint8_t)(value & 0x000000FFu);
    dst[1] = (uint8_t)((value >> 8u) & 0x000000FFu);
    dst[2] = (uint8_t)((value >> 16u) & 0x000000FFu);
    dst[3] = (uint8_t)((value >> 24u) & 0x000000FFu);
}

void referee_latency_hist_add(referee_latency_hist_t* hist, uint16_t value_ms)
{
    if (hist == nullptr) {
        return;
    }

    hist->samples[hist->head] = value_ms;
    hist->head = (uint16_t)((hist->head + 1u) % REFEREE_LATENCY_WINDOW_SIZE);
    if (hist->count < REFEREE_LATENCY_WINDOW_SIZE) {
        hist->count++;
    }
    hist->total_samples++;
    hist->last_ms = value_ms;
    if (hist->has_sample == 0u || value_ms < hist->min_ms) {
        hist->min_ms = value_ms;
    }
    if (hist->has_sample == 0u || value_ms > hist->max_ms) {
        hist->max_ms = value_ms;
    }
    hist->has_sample = 1u;
}

uint16_t referee_latency_hist_percentile(const referee_latency_hist_t* hist, uint8_t percentile)
{
    if (hist == nullptr || hist->count == 0u) {
        return 0u;
    }

    uint16_t sorted[REFEREE_LATENCY_WINDOW_SIZE] = {0};
    for (uint16_t i = 0u; i < hist->count; ++i) {
        sorted[i] = hist->samples[i];
    }

    for (uint16_t i = 1u; i < hist->count; ++i) {
        const uint16_t key = sorted[i];
        uint16_t j = i;
        while (j > 0u && sorted[j - 1u] > key) {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = key;
    }

    const size_t rank = ((size_t)(hist->count - 1u) * (size_t)percentile + 99u) / 100u;
    return sorted[(rank < hist->count) ? rank : (hist->count - 1u)];
}

void referee_latency_on_rx_0302(const omnix_custom_controller_raw_payload_t& payload, uint32_t now_ms)
{
    const bool probe_enabled = ((payload.reserved[4] & REFEREE_LATENCY_PROBE_ENABLE_MASK) != 0u);
    const uint32_t controller_tick_ms = referee_read_u32_le(&payload.reserved[0]);

    g_referee_latency.probe_enabled = probe_enabled ? 1u : 0u;
    if (!probe_enabled || controller_tick_ms == 0u) {
        return;
    }

    // Controller and robot ticks are from different local clocks.
    // Only preserve the echoed controller tick here and measure board-local
    // rx->tx delay when composing 0x0309.
    g_referee_latency.last_controller_tick_ms = controller_tick_ms;
    g_referee_latency.last_board_rx_tick_ms = now_ms;
}

void referee_latency_fill_0309_reserved(omnix_robot_joint_feedback_payload_t* payload, uint32_t now_ms)
{
    if (payload == nullptr) {
        return;
    }

    std::memset(payload->reserved, 0, sizeof(payload->reserved));
    if (g_referee_latency.probe_enabled == 0u ||
        g_referee_latency.last_controller_tick_ms == 0u ||
        g_referee_latency.last_board_rx_tick_ms == 0u) {
        return;
    }

    const uint32_t board_delay_u32 = now_ms - g_referee_latency.last_board_rx_tick_ms;
    const uint16_t board_delay_ms = (board_delay_u32 > 0xFFFFu) ? 0xFFFFu : (uint16_t)board_delay_u32;

    referee_write_u32_le(&payload->reserved[0], g_referee_latency.last_controller_tick_ms);
    referee_write_u16_le(&payload->reserved[4], board_delay_ms);
    referee_latency_hist_add(&g_referee_latency.board_rx_to_tx_ms, board_delay_ms);
}

bool referee_latency_snapshot(referee_latency_snapshot_t* out)
{
    if (out == nullptr || g_referee_latency.ingress_age_ms.has_sample == 0u || g_referee_latency.board_rx_to_tx_ms.has_sample == 0u) {
        return false;
    }

    out->has_sample = 1u;
    out->ingress_total_samples = g_referee_latency.ingress_age_ms.total_samples;
    out->ingress_last_ms = g_referee_latency.ingress_age_ms.last_ms;
    out->ingress_min_ms = g_referee_latency.ingress_age_ms.min_ms;
    out->ingress_p50_ms = referee_latency_hist_percentile(&g_referee_latency.ingress_age_ms, 50u);
    out->ingress_p95_ms = referee_latency_hist_percentile(&g_referee_latency.ingress_age_ms, 95u);
    out->ingress_max_ms = g_referee_latency.ingress_age_ms.max_ms;
    out->board_total_samples = g_referee_latency.board_rx_to_tx_ms.total_samples;
    out->board_last_ms = g_referee_latency.board_rx_to_tx_ms.last_ms;
    out->board_min_ms = g_referee_latency.board_rx_to_tx_ms.min_ms;
    out->board_p50_ms = referee_latency_hist_percentile(&g_referee_latency.board_rx_to_tx_ms, 50u);
    out->board_p95_ms = referee_latency_hist_percentile(&g_referee_latency.board_rx_to_tx_ms, 95u);
    out->board_max_ms = g_referee_latency.board_rx_to_tx_ms.max_ms;
    return true;
}

uint8_t referee_crc8(const uint8_t* data, uint16_t len)
{
    uint8_t crc = 0xFFu;

    if (data == nullptr) {
        return crc;
    }

    while (len-- > 0u) {
        crc ^= *data++;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x01u) != 0u) {
                crc = (uint8_t)((crc >> 1u) ^ 0x8Cu);
            } else {
                crc >>= 1u;
            }
        }
    }

    return crc;
}

uint16_t referee_crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;

    if (data == nullptr) {
        return crc;
    }

    while (len-- > 0u) {
        crc ^= *data++;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1u) ^ 0x8408u);
            } else {
                crc >>= 1u;
            }
        }
    }

    return crc;
}

#if REFEREE_RX_FRAME_HEADER_TRACE_ENABLE
void referee_trace_rx_frame_header(const uint8_t* frame,
                                   uint16_t frame_len,
                                   uint16_t data_len,
                                   uint8_t frame_seq,
                                   uint16_t cmd_id,
                                   uint8_t frame_crc8,
                                   uint8_t calc_crc8,
                                   uint16_t frame_crc16,
                                   uint16_t calc_crc16)
{
    const uint8_t sof = (frame != nullptr && frame_len > 0u) ? frame[0] : 0u;
    const uint8_t crc8_ok = (frame_crc8 == calc_crc8) ? 1u : 0u;
    const uint8_t crc16_ok = (frame_crc16 == calc_crc16) ? 1u : 0u;
    const uint8_t is_0302 = (cmd_id == REFEREE_CMD_ID_CUSTOM_CONTROLLER_TO_ROBOT) ? 1u : 0u;

    LOGI("[REF][HDR][RX][%s] sof=0x%02X len=%u seq=%u cmd=0x%04X frame_len=%u crc8=0x%02X/0x%02X ok=%u crc16=0x%04X/0x%04X ok=%u is_0302=%u",
         REFEREE_RX_SOURCE,
         static_cast<unsigned>(sof),
         static_cast<unsigned>(data_len),
         static_cast<unsigned>(frame_seq),
         static_cast<unsigned>(cmd_id),
         static_cast<unsigned>(frame_len),
         static_cast<unsigned>(frame_crc8),
         static_cast<unsigned>(calc_crc8),
         static_cast<unsigned>(crc8_ok),
         static_cast<unsigned>(frame_crc16),
         static_cast<unsigned>(calc_crc16),
         static_cast<unsigned>(crc16_ok),
         static_cast<unsigned>(is_0302));
}
#endif

template <typename Fn>
void referee_stats_mutate(Fn&& fn)
{
    __DMB();
    g_stats_guard++;
    __DMB();
    fn(g_rx_stats);
    __DMB();
    g_stats_guard++;
    __DMB();
}

bool referee_stats_copy(referee_rx_stats_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    referee_rx_stats_t local{};

    do {
        seq_a = g_stats_guard;
        __DMB();
        local = g_rx_stats;
        __DMB();
        seq_b = g_stats_guard;
    } while (seq_a != seq_b);

    *out = local;
    return true;
}

template <typename Fn>
void referee_uart_diag_mutate(Fn&& fn)
{
    __DMB();
    g_uart_diag_guard++;
    __DMB();
    fn(g_uart_diag);
    __DMB();
    g_uart_diag_guard++;
    __DMB();
}

bool referee_uart_diag_copy(referee_uart_diag_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    referee_uart_diag_t local{};

    do {
        seq_a = g_uart_diag_guard;
        __DMB();
        local = g_uart_diag;
        __DMB();
        seq_b = g_uart_diag_guard;
    } while (seq_a != seq_b);

    *out = local;
    return true;
}

static inline uint16_t referee_fifo_used_nolock(void)
{
    return (uint16_t)(((uint16_t)(g_referee_rx_head - g_referee_rx_tail)) & (REFEREE_RX_FIFO_SIZE - 1u));
}

void referee_dbg_note_fifo_used(uint16_t used)
{
#if REF_RX_DBG_ENABLE
    g_referee_rx_dbg_diag.fifo_used_last = used;
    if (used > g_referee_rx_dbg_diag.fifo_used_max) {
        g_referee_rx_dbg_diag.fifo_used_max = used;
    }
#else
    (void)used;
#endif
}

void referee_dbg_note_parse_used(uint16_t used)
{
#if REF_RX_DBG_ENABLE
    if (used > g_referee_rx_dbg_diag.parse_buf_used_max) {
        g_referee_rx_dbg_diag.parse_buf_used_max = used;
    }
#else
    (void)used;
#endif
}

void referee_dbg_note_parse_slide(void)
{
#if REF_RX_DBG_ENABLE
    g_referee_rx_dbg_diag.parse_buf_slide_count++;
#endif
}

void referee_dbg_note_valid_0302(uint32_t now_ms)
{
#if REF_RX_DBG_ENABLE
    if (g_referee_rx_dbg_diag.last_valid_0302_tick_ms != 0u) {
        const uint32_t gap_ms = now_ms - g_referee_rx_dbg_diag.last_valid_0302_tick_ms;
        g_referee_rx_dbg_diag.valid_0302_interval_last_ms = gap_ms;
        if (gap_ms > g_referee_rx_dbg_diag.valid_0302_interval_max_ms) {
            g_referee_rx_dbg_diag.valid_0302_interval_max_ms = gap_ms;
        }
    }
    g_referee_rx_dbg_diag.last_valid_0302_tick_ms = now_ms;
#else
    (void)now_ms;
#endif
}

void referee_dbg_note_resync_drop(void)
{
#if REF_RX_DBG_ENABLE
    g_referee_rx_dbg_diag.resync_drop_run_cur++;
    if (g_referee_rx_dbg_diag.resync_drop_run_cur > g_referee_rx_dbg_diag.resync_drop_run_max) {
        g_referee_rx_dbg_diag.resync_drop_run_max = g_referee_rx_dbg_diag.resync_drop_run_cur;
    }
#endif
}

void referee_dbg_reset_resync_drop(void)
{
#if REF_RX_DBG_ENABLE
    g_referee_rx_dbg_diag.resync_drop_run_cur = 0u;
#endif
}

void referee_dbg_trigger_burst(uint8_t trigger, uint32_t now_ms, const char* reason)
{
#if REF_RX_DBG_ENABLE
    if ((REF_RX_DBG_BURST_TRIGGER_MASK & trigger) == 0u) {
        return;
    }

    if (g_referee_rx_dbg_rt.burst_active == 0u) {
        g_referee_rx_dbg_rt.burst_active = 1u;
        g_referee_rx_dbg_rt.burst_trigger_mask = 0u;
        g_referee_rx_dbg_rt.burst_frame_count = 0u;
        g_referee_rx_dbg_rt.burst_crc_fail_count = 0u;
        g_referee_rx_dbg_rt.burst_overflow_count = 0u;
        g_referee_rx_dbg_rt.burst_resync_drop_run_max = 0u;
    }

    g_referee_rx_dbg_rt.burst_trigger_mask = (uint8_t)(g_referee_rx_dbg_rt.burst_trigger_mask | trigger);
    g_referee_rx_dbg_rt.burst_end_ms = now_ms + REF_RX_DBG_BURST_MS;
    LOGW("[REF][DBG][RX_ERR] kind=%s trig=0x%02X burst_ms=%lu",
         (reason != nullptr) ? reason : "unknown",
         static_cast<unsigned>(trigger),
         static_cast<unsigned long>(REF_RX_DBG_BURST_MS));
#else
    (void)trigger;
    (void)now_ms;
    (void)reason;
#endif
}

void referee_dbg_log_burst_frame(uint32_t now_ms, uint16_t cmd_id, uint16_t data_len, uint8_t frame_seq, uint8_t crc_ok, uint16_t parse_used)
{
#if REF_RX_DBG_ENABLE
    if (g_referee_rx_dbg_rt.burst_active == 0u) {
        return;
    }

    g_referee_rx_dbg_rt.burst_frame_count++;
    if (crc_ok == 0u) {
        g_referee_rx_dbg_rt.burst_crc_fail_count++;
    }
    if (g_referee_rx_dbg_diag.resync_drop_run_max > g_referee_rx_dbg_rt.burst_resync_drop_run_max) {
        g_referee_rx_dbg_rt.burst_resync_drop_run_max = g_referee_rx_dbg_diag.resync_drop_run_max;
    }

    LOGI("[REF][DBG][RX_BURST] cmd=0x%04X len=%u seq=%u crc_ok=%u parse_drop_run=%lu fifo_used=%u parse_used=%u t=%lu",
         static_cast<unsigned>(cmd_id),
         static_cast<unsigned>(data_len),
         static_cast<unsigned>(frame_seq),
         static_cast<unsigned>(crc_ok),
         static_cast<unsigned long>(g_referee_rx_dbg_diag.resync_drop_run_cur),
         static_cast<unsigned>(g_referee_rx_dbg_diag.fifo_used_last),
         static_cast<unsigned>(parse_used),
         static_cast<unsigned long>(now_ms));
#else
    (void)now_ms;
    (void)cmd_id;
    (void)data_len;
    (void)frame_seq;
    (void)crc_ok;
    (void)parse_used;
#endif
}

uint32_t referee_dbg_fetch_and_clear_u32(volatile uint32_t* target)
{
    uint32_t value = 0u;

    if (target == nullptr) {
        return 0u;
    }

    __disable_irq();
    value = *target;
    *target = 0u;
    __enable_irq();
    return value;
}

void referee_dbg_handle_pending_triggers(uint32_t now_ms)
{
#if REF_RX_DBG_ENABLE
    const uint32_t pending_mask = referee_dbg_fetch_and_clear_u32(&g_referee_rx_dbg_pending_trigger_mask);
    const uint32_t pending_overflow = referee_dbg_fetch_and_clear_u32(&g_referee_rx_dbg_pending_overflow_count);

    if ((pending_mask & REF_RX_DBG_TRIG_OVERFLOW) != 0u) {
        referee_dbg_trigger_burst(REF_RX_DBG_TRIG_OVERFLOW, now_ms, "fifo_overflow");
        g_referee_rx_dbg_rt.burst_overflow_count += pending_overflow;
    }
#else
    (void)now_ms;
#endif
}

void referee_dbg_service_burst_timeout(uint32_t now_ms)
{
#if REF_RX_DBG_ENABLE
    if (g_referee_rx_dbg_rt.burst_active == 0u) {
        return;
    }

    if ((int32_t)(now_ms - g_referee_rx_dbg_rt.burst_end_ms) < 0) {
        return;
    }

    LOGI("[REF][DBG][RX_BURST] done trig=0x%02X frames=%lu crc_fail=%lu ovf=%lu resync_drop_run_max=%lu",
         static_cast<unsigned>(g_referee_rx_dbg_rt.burst_trigger_mask),
         static_cast<unsigned long>(g_referee_rx_dbg_rt.burst_frame_count),
         static_cast<unsigned long>(g_referee_rx_dbg_rt.burst_crc_fail_count),
         static_cast<unsigned long>(g_referee_rx_dbg_rt.burst_overflow_count),
         static_cast<unsigned long>(g_referee_rx_dbg_rt.burst_resync_drop_run_max));
    g_referee_rx_dbg_rt.burst_active = 0u;
#else
    (void)now_ms;
#endif
}

void referee_dbg_log_summary(const referee_rx_stats_t& stats, const referee_uart_diag_t& uart_diag, uint32_t now_ms)
{
#if REF_RX_DBG_ENABLE
    if ((now_ms - g_referee_rx_dbg_rt.last_summary_ms) < REF_RX_DBG_SUMMARY_MS) {
        return;
    }
    g_referee_rx_dbg_rt.last_summary_ms = now_ms;

    const uint32_t evt_1s = uart_diag.rx_event_count - g_referee_rx_dbg_rt.prev_uart_diag.rx_event_count;
    const uint32_t bytes_1s = uart_diag.rx_event_bytes - g_referee_rx_dbg_rt.prev_uart_diag.rx_event_bytes;
    const uint32_t ovf_1s = stats.fifo_overflows - g_referee_rx_dbg_rt.prev_stats.fifo_overflows;
    const uint32_t frames_1s = stats.total_frames - g_referee_rx_dbg_rt.prev_stats.total_frames;
    const uint32_t good_1s = stats.good_frames - g_referee_rx_dbg_rt.prev_stats.good_frames;
    const uint32_t crc8_1s = stats.crc8_errors - g_referee_rx_dbg_rt.prev_stats.crc8_errors;
    const uint32_t crc16_1s = stats.crc16_errors - g_referee_rx_dbg_rt.prev_stats.crc16_errors;
    const uint32_t len_1s = stats.payload_len_errors - g_referee_rx_dbg_rt.prev_stats.payload_len_errors;
    const uint32_t ver_1s = stats.payload_version_errors - g_referee_rx_dbg_rt.prev_stats.payload_version_errors;
    const uint32_t unknown_1s = stats.unknown_cmd_frames - g_referee_rx_dbg_rt.prev_stats.unknown_cmd_frames;
    const uint32_t seq_gap_1s = stats.cmd_0302_seq_gap_count - g_referee_rx_dbg_rt.prev_stats.cmd_0302_seq_gap_count;
    const uint32_t uart_err_1s = uart_diag.rx_hw_error_count - g_referee_rx_dbg_rt.prev_uart_diag.rx_hw_error_count;
    const uint32_t ore_1s = uart_diag.rx_hw_error_ore_count - g_referee_rx_dbg_rt.prev_uart_diag.rx_hw_error_ore_count;
    const uint32_t fe_1s = uart_diag.rx_hw_error_fe_count - g_referee_rx_dbg_rt.prev_uart_diag.rx_hw_error_fe_count;
    const uint32_t ne_1s = uart_diag.rx_hw_error_ne_count - g_referee_rx_dbg_rt.prev_uart_diag.rx_hw_error_ne_count;
    const uint32_t silence_ms =
        (g_referee_rx_dbg_diag.last_valid_0302_tick_ms != 0u) ?
        (now_ms - g_referee_rx_dbg_diag.last_valid_0302_tick_ms) :
        (now_ms - g_referee_rx_dbg_rt.task_start_ms);
    const uint16_t evt_size_min = (g_referee_rx_dbg_diag.rx_evt_size_min == 0xFFFFu) ? 0u : g_referee_rx_dbg_diag.rx_evt_size_min;
    g_referee_rx_dbg_diag.valid_0302_silence_ms = silence_ms;

    LOGI("[REF][DBG][RX_SUM] evt_1s=%lu bytes_1s=%lu fifo_used_max=%u parse_used_max=%u ovf_1s=%lu frames_1s=%lu good_1s=%lu crc8_1s=%lu crc16_1s=%lu len_1s=%lu ver_1s=%lu unknown_1s=%lu seq_gap_1s=%lu valid0302_gap_max_ms=%lu silence_ms=%lu uart_err_1s=%lu ore_1s=%lu fe_1s=%lu ne_1s=%lu",
         static_cast<unsigned long>(evt_1s),
         static_cast<unsigned long>(bytes_1s),
         static_cast<unsigned>(g_referee_rx_dbg_diag.fifo_used_max),
         static_cast<unsigned>(g_referee_rx_dbg_diag.parse_buf_used_max),
         static_cast<unsigned long>(ovf_1s),
         static_cast<unsigned long>(frames_1s),
         static_cast<unsigned long>(good_1s),
         static_cast<unsigned long>(crc8_1s),
         static_cast<unsigned long>(crc16_1s),
         static_cast<unsigned long>(len_1s),
         static_cast<unsigned long>(ver_1s),
         static_cast<unsigned long>(unknown_1s),
         static_cast<unsigned long>(seq_gap_1s),
         static_cast<unsigned long>(g_referee_rx_dbg_diag.valid_0302_interval_max_ms),
         static_cast<unsigned long>(silence_ms),
         static_cast<unsigned long>(uart_err_1s),
         static_cast<unsigned long>(ore_1s),
         static_cast<unsigned long>(fe_1s),
         static_cast<unsigned long>(ne_1s));

    LOGI("[REF][DBG][RX_EVT] evt_size_min=%u evt_size_max=%u zero_or_oversize=%lu parse_slide_total=%lu resync_drop_run_max=%lu",
         static_cast<unsigned>(evt_size_min),
         static_cast<unsigned>(g_referee_rx_dbg_diag.rx_evt_size_max),
         static_cast<unsigned long>(g_referee_rx_dbg_diag.rx_evt_zero_or_oversize_count),
         static_cast<unsigned long>(g_referee_rx_dbg_diag.parse_buf_slide_count),
         static_cast<unsigned long>(g_referee_rx_dbg_diag.resync_drop_run_max));

    if (uart_err_1s > 0u) {
        referee_dbg_trigger_burst(REF_RX_DBG_TRIG_UART, now_ms, "uart_hw_error");
    }
    if ((silence_ms >= REF_RX_DBG_SILENCE_WARN_MS) &&
        ((now_ms - g_referee_rx_dbg_rt.last_silence_warn_ms) >= REF_RX_DBG_SUMMARY_MS)) {
        g_referee_rx_dbg_rt.last_silence_warn_ms = now_ms;
        LOGW("[REF][DBG][RX_ERR] kind=silence silence_ms=%lu last_seq=%u",
             static_cast<unsigned long>(silence_ms),
             (stats.last_0302_seq_valid != 0u) ? stats.last_0302_seq : 0u);
    }
    if ((g_referee_rx_dbg_diag.resync_drop_run_max >= REF_RX_DBG_RESYNC_WARN_BYTES) &&
        ((now_ms - g_referee_rx_dbg_rt.last_resync_warn_ms) >= REF_RX_DBG_SUMMARY_MS)) {
        g_referee_rx_dbg_rt.last_resync_warn_ms = now_ms;
        LOGW("[REF][DBG][RX_ERR] kind=resync drop_run_max=%lu threshold=%lu",
             static_cast<unsigned long>(g_referee_rx_dbg_diag.resync_drop_run_max),
             static_cast<unsigned long>(REF_RX_DBG_RESYNC_WARN_BYTES));
    }

    g_referee_rx_dbg_rt.prev_stats = stats;
    g_referee_rx_dbg_rt.prev_uart_diag = uart_diag;

    g_referee_rx_dbg_diag.fifo_used_max = g_referee_rx_dbg_diag.fifo_used_last;
    g_referee_rx_dbg_diag.parse_buf_used_max = g_referee_rx_parse_len;
    g_referee_rx_dbg_diag.rx_evt_size_min = 0xFFFFu;
    g_referee_rx_dbg_diag.rx_evt_size_max = 0u;
    g_referee_rx_dbg_diag.valid_0302_interval_max_ms = 0u;
#else
    (void)stats;
    (void)uart_diag;
    (void)now_ms;
#endif
}

template <typename Fn>
void referee_raw_mutate(Fn&& fn)
{
    __DMB();
    g_raw_guard++;
    __DMB();
    fn();
    __DMB();
    g_raw_guard++;
    __DMB();
}

bool referee_raw_copy(uint8_t* out_buf, uint16_t out_len)
{
    if (out_buf == nullptr || out_len < REFEREE_CUSTOM_DATA_LEN) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    bool valid = false;
    uint8_t local[REFEREE_CUSTOM_DATA_LEN] = {0};

    do {
        seq_a = g_raw_guard;
        __DMB();
        valid = g_raw_0302_valid;
        std::memcpy(local, g_raw_0302, sizeof(local));
        __DMB();
        seq_b = g_raw_guard;
    } while (seq_a != seq_b);

    if (!valid) {
        return false;
    }

    std::memcpy(out_buf, local, sizeof(local));
    return true;
}

bool referee_tx_feedback_copy(omnix_robot_joint_feedback_payload_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    uint32_t local_tick = 0u;
    omnix_robot_joint_feedback_payload_t local{};

    do {
        seq_a = g_joint_feedback_tx_guard;
        __DMB();
        local = g_joint_feedback_tx_payload;
        local_tick = g_joint_feedback_tx_tick_ms;
        __DMB();
        seq_b = g_joint_feedback_tx_guard;
    } while (seq_a != seq_b);

    if (local_tick == 0u) {
        return false;
    }

    *out = local;
    return true;
}

void referee_store_tx_feedback(const omnix_robot_joint_feedback_payload_t* payload)
{
    if (payload == nullptr) {
        return;
    }

    __DMB();
    g_joint_feedback_tx_guard++;
    __DMB();
    g_joint_feedback_tx_payload = *payload;
    g_joint_feedback_tx_tick_ms = HAL_GetTick();
    __DMB();
    g_joint_feedback_tx_guard++;
    __DMB();
}

bool referee_copy_rx(referee_joint_raw_cmd_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    referee_joint_raw_cmd_t local{};

    do {
        seq_a = g_joint_rx_guard;
        __DMB();
        local = g_joint_rx_cmd;
        __DMB();
        seq_b = g_joint_rx_guard;
    } while (seq_a != seq_b);

    local.fresh = local.online && ((HAL_GetTick() - local.rx_tick_ms) <= OMNIX_CUSTOM_CONTROLLER_RX_TIMEOUT_MS);
    *out = local;
    return true;
}

void referee_store_payload(const omnix_custom_controller_raw_payload_t* payload)
{
    uint8_t i = 0u;

    if (payload == nullptr) {
        return;
    }

    __DMB();
    g_joint_rx_guard++;
    __DMB();
    g_joint_rx_cmd.online = (payload->valid_mask != 0u);
    g_joint_rx_cmd.fresh = g_joint_rx_cmd.online;
    g_joint_rx_cmd.seq = payload->seq;
    g_joint_rx_cmd.valid_mask = payload->valid_mask;
    g_joint_rx_cmd.diag_flags = payload->diag_flags;
    for (i = 0u; i < OMNIX_JOINT_RAW_COUNT; ++i) {
        g_joint_rx_cmd.raw_u16[i] = payload->raw_u16[i];
    }
    g_joint_rx_cmd.rx_tick_ms = HAL_GetTick();
    __DMB();
    g_joint_rx_guard++;
    __DMB();

    if ((payload->diag_flags & OMNIX_DIAG_FLAG_MAPPING_RESET_DONE) != 0u) {
        g_mapping_reset_controller_done = true;
    }
}

void referee_store_raw0302(const uint8_t* data)
{
    if (data == nullptr) {
        return;
    }

    referee_raw_mutate([&]() {
        std::memcpy(g_raw_0302, data, REFEREE_CUSTOM_DATA_LEN);
        g_raw_0302_valid = true;
    });
}

void referee_set_external_control_enabled(bool enabled)
{
    g_external_control_enabled = enabled;
}

void referee_debug_probe_update(uint32_t now_ms,
                                size_t rx_chunk_bytes,
                                const referee_joint_raw_cmd_t& cmd,
                                const referee_rx_stats_t& stats,
                                const referee_uart_diag_t& uart_diag,
                                const referee_custom_rx_runtime_t& custom_rx)
{
    volatile referee_debug_probe_t* probe = &g_referee_debug_probe;
    probe->signature = 0x52444247u;  /* 'RDBG' */
    probe->version = 1u;

    probe->task_loop_count += 1u;
    probe->task_last_os_tick = (uint32_t)osKernelGetTickCount();
    probe->task_last_ms_tick = now_ms;

    probe->rx_chunk_last_bytes = (uint32_t)rx_chunk_bytes;
    if (rx_chunk_bytes > 0u) {
        probe->rx_chunk_nonzero_loops += 1u;
    }

    const uint32_t fifo_used = (uint32_t)referee_fifo_used_nolock();
    probe->rx_fifo_used = fifo_used;
    if (fifo_used > probe->rx_fifo_used_max_observed) {
        probe->rx_fifo_used_max_observed = fifo_used;
    }

    const uint32_t parse_len = (uint32_t)g_referee_rx_parse_len;
    probe->rx_parse_len = parse_len;
    if (parse_len > probe->rx_parse_len_max_observed) {
        probe->rx_parse_len_max_observed = parse_len;
    }

    probe->uart9_rx_event_count = uart_diag.rx_event_count;
    probe->uart9_rx_event_bytes = uart_diag.rx_event_bytes;
    probe->uart9_rx_event_invalid_size = uart_diag.rx_event_invalid_size;
    probe->uart9_rx_vt03_feed_events = uart_diag.rx_vt03_feed_events;
    probe->uart9_rx_vt03_feed_bytes = uart_diag.rx_vt03_feed_bytes;
    probe->uart9_rx_hw_error_count = uart_diag.rx_hw_error_count;
    probe->uart9_rx_hw_error_ore_count = uart_diag.rx_hw_error_ore_count;
    probe->uart9_rx_hw_error_fe_count = uart_diag.rx_hw_error_fe_count;
    probe->uart9_rx_hw_error_ne_count = uart_diag.rx_hw_error_ne_count;

    probe->referee_total_bytes = stats.total_bytes;
    probe->referee_total_frames = stats.total_frames;
    probe->referee_good_frames = stats.good_frames;
    probe->referee_cmd_0302_frames = stats.cmd_0302_frames;
    probe->referee_cmd_0302_good_frames = stats.cmd_0302_good_frames;
    probe->referee_crc8_errors = stats.crc8_errors;
    probe->referee_crc16_errors = stats.crc16_errors;
    probe->referee_payload_len_errors = stats.payload_len_errors;
    probe->referee_payload_version_errors = stats.payload_version_errors;
    probe->referee_unknown_cmd_frames = stats.unknown_cmd_frames;
    probe->referee_fifo_overflows = stats.fifo_overflows;

    const uint32_t cmd_age_ms = (cmd.rx_tick_ms != 0u) ? (now_ms - cmd.rx_tick_ms) : 0u;
    probe->cmd_online = cmd.online ? 1u : 0u;
    probe->cmd_fresh = cmd.fresh ? 1u : 0u;
    probe->cmd_age_ms = cmd_age_ms;
    probe->cmd_rx_tick_ms = cmd.rx_tick_ms;
    probe->cmd_seq = cmd.seq;
    probe->cmd_valid_mask = cmd.valid_mask;

    const referee_custom_rx_path_diag_t* active_path =
        referee_custom_rx_path(custom_rx, custom_rx.active_source);
    probe->active_source = (uint32_t)custom_rx.active_source;
    probe->active_source_since_ms = custom_rx.active_source_since_ms;
    probe->active_source_frames_good = active_path->frames_good;
    probe->active_source_cmd_0302_good = active_path->cmd_0302_good;
}

bool referee_build_frame(uint16_t cmd_id, const uint8_t* data, uint16_t data_len, uint8_t* out_frame, uint16_t out_len)
{
    uint16_t crc16 = 0u;

    if (data == nullptr || out_frame == nullptr || data_len != REFEREE_CUSTOM_DATA_LEN || out_len < REFEREE_TX_FRAME_LEN) {
        return false;
    }

    std::memset(out_frame, 0, REFEREE_TX_FRAME_LEN);
    out_frame[0] = REFEREE_SOF;
    referee_write_u16_le(&out_frame[1], data_len);
    out_frame[3] = g_referee_tx_seq++;
    out_frame[4] = referee_crc8(out_frame, 4u);
    referee_write_u16_le(&out_frame[5], cmd_id);
    std::memcpy(&out_frame[REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN], data, data_len);
    crc16 = referee_crc16(out_frame, (uint16_t)(REFEREE_TX_FRAME_LEN - REFEREE_TAIL_LEN));
    referee_write_u16_le(&out_frame[REFEREE_TX_FRAME_LEN - REFEREE_TAIL_LEN], crc16);
    return true;
}

bool referee_uart_tx_ready(void)
{
    return (huart9.gState == HAL_UART_STATE_READY) &&
           (huart9.hdmatx != nullptr) &&
           (huart9.hdmatx->State == HAL_DMA_STATE_READY);
}

uint32_t referee_tx_interval_ms(bool tx_active)
{
    return tx_active ? REFEREE_TX_INTERVAL_ACTIVE_MS : REFEREE_TX_INTERVAL_IDLE_MS;
}

void referee_remote_buzzer_append_pulses(uint8_t count)
{
    if (count == 0u) {
        return;
    }

    const uint8_t pending_before = g_remote_buzzer_pulses_pending;
    uint16_t pending_after = (uint16_t)pending_before + (uint16_t)count;
    if (pending_after > 0xFFu) {
        pending_after = 0xFFu;
    }
    g_remote_buzzer_pulses_pending = (uint8_t)pending_after;
    if (pending_before == 0u) {
        g_remote_buzzer_tx_high_phase = 1u;
    }
}

bool referee_remote_buzzer_tx_active(void)
{
    return g_remote_buzzer_pulses_pending > 0u;
}

bool referee_remote_buzzer_tx_should_set_flag(void)
{
    return (g_remote_buzzer_pulses_pending > 0u) && (g_remote_buzzer_tx_high_phase != 0u);
}

void referee_remote_buzzer_on_tx_success(bool sent_high)
{
    if (g_remote_buzzer_pulses_pending == 0u) {
        g_remote_buzzer_tx_high_phase = 0u;
        return;
    }

    if (sent_high) {
        g_remote_buzzer_tx_high_phase = 0u;
        return;
    }

    g_remote_buzzer_pulses_pending--;
    if (g_remote_buzzer_pulses_pending > 0u) {
        g_remote_buzzer_tx_high_phase = 1u;
    } else {
        g_remote_buzzer_tx_high_phase = 0u;
    }
}

bool referee_prepare_tx_feedback_payload(omnix_robot_joint_feedback_payload_t* out, uint16_t seq, uint32_t now_ms)
{
    gimbal_joint_snapshot_t joint_snap{};
    int16_t pause_cmd_cdeg[9] = {0};
    uint8_t pause_phase = 0u;
    omnix_robot_joint_feedback_payload_t payload = {
        OMNIX_ROBOT_JOINT_FEEDBACK_VERSION,
        0u,
        seq,
        0u,
        0u,
        {0u, 0u, 0u, 0u, 0u, 0u, 0u},
        0u,
        0u,
        {0, 0, 0, 0, 0, 0, 0, 0}
    };

    if (out == nullptr) {
        return false;
    }

    if (!Gimbal_ReadJointSnapshot(&joint_snap)) {
        return false;
    }

    (void)Gimbal_ReadPauseCmdCdeg(pause_cmd_cdeg, &pause_phase);

    for (uint8_t i = 0u; i < OMNIX_JOINT_RAW_COUNT; ++i) {
        if (joint_snap.online[i] != 0u) {
            payload.raw_u16[i] = referee_joint_rel_cdeg_to_controller_raw(i, joint_snap.angle_cdeg[i]);
            payload.valid_mask |= (uint8_t)(1u << i);
            payload.online_mask |= (uint8_t)(1u << i);
        } else {
            payload.raw_u16[i] = 0u;
        }
    }

    if (Referee_IsExternalControlEnabled()) {
        payload.flags |= OMNIX_JOINT_FEEDBACK_FLAG_EXTERNAL_CONTROL_ENABLED;
    }
    if (pause_phase == (uint8_t)GIMBAL_PAUSE_PRE_BEEP ||
        pause_phase == (uint8_t)GIMBAL_PAUSE_RUN ||
        pause_phase == (uint8_t)GIMBAL_PAUSE_DONE_BEEP) {
        payload.flags |= OMNIX_JOINT_FEEDBACK_FLAG_PAUSE_ACTIVE;
    }
    if (gimbal_zero_force) {
        payload.flags |= OMNIX_JOINT_FEEDBACK_FLAG_ZERO_FORCE;
    }
    if (gimbal_all_online) {
        payload.flags |= OMNIX_JOINT_FEEDBACK_FLAG_GIMBAL_ALL_ONLINE;
    }
    payload.pause_phase = pause_phase;
    if (referee_remote_buzzer_tx_should_set_flag()) {
        payload.diag_flags |= OMNIX_DIAG_FLAG_REMOTE_BUZZER_REQ;
    }
    if (g_mapping_reset_active) {
        payload.diag_flags |= OMNIX_DIAG_FLAG_MAPPING_RESET_REQ;
    }

    referee_latency_fill_0309_reserved(&payload, now_ms);
    *out = payload;
    return true;
}

static inline uint16_t referee_fifo_free_nolock(void)
{
    const uint16_t head = g_referee_rx_head;
    const uint16_t tail = g_referee_rx_tail;
    return (uint16_t)(REFEREE_RX_FIFO_SIZE - (((uint16_t)(head - tail)) & (REFEREE_RX_FIFO_SIZE - 1u)) - 1u);
}

void referee_fifo_push_bytes(const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0u) {
        return;
    }

    uint16_t free = referee_fifo_free_nolock();
    uint16_t write_len = (len <= free) ? len : free;

    for (uint16_t i = 0u; i < write_len; ++i) {
        uint16_t head = g_referee_rx_head;
        g_referee_rx_fifo[head] = data[i];
        g_referee_rx_head = (uint16_t)((head + 1u) & (REFEREE_RX_FIFO_SIZE - 1u));
    }

    referee_dbg_note_fifo_used(referee_fifo_used_nolock());

    if (write_len < len) {
        referee_stats_mutate([](referee_rx_stats_t& stats) {
            stats.fifo_overflows++;
        });
#if REF_RX_DBG_ENABLE
        g_referee_rx_dbg_diag.fifo_overflow_burst++;
        g_referee_rx_dbg_pending_trigger_mask |= REF_RX_DBG_TRIG_OVERFLOW;
        g_referee_rx_dbg_pending_overflow_count++;
#endif
    }
}

size_t referee_fifo_pop_bytes(uint8_t* out, size_t max_len)
{
    size_t n = 0u;

    if (out == nullptr || max_len == 0u) {
        return 0u;
    }

    while (n < max_len) {
        const uint16_t tail = g_referee_rx_tail;
        if (tail == g_referee_rx_head) {
            break;
        }
        out[n++] = g_referee_rx_fifo[tail];
        g_referee_rx_tail = (uint16_t)((tail + 1u) & (REFEREE_RX_FIFO_SIZE - 1u));
    }

    return n;
}

void referee_on_vt03_raw_tap(const uint8_t* data, uint16_t size)
{
#if REF_RX_DBG_ENABLE
    if ((size == 0u) || (size > VT03_DMA_BUF_SIZE)) {
        g_referee_rx_dbg_diag.rx_evt_zero_or_oversize_count++;
    }
#endif

    if (data == nullptr || size == 0u) {
        referee_uart_diag_mutate([&](referee_uart_diag_t& diag) {
            diag.rx_event_invalid_size++;
        });
        return;
    }
    referee_uart_diag_mutate([&](referee_uart_diag_t& diag) {
        diag.rx_event_count++;
        diag.rx_event_bytes += size;
        diag.rx_referee_feed_bytes += size;
        diag.rx_vt03_feed_bytes += size;
        diag.rx_vt03_feed_events++;
    });
    referee_custom_rx_note_ingress(REF_CUSTOM_CTRL_RX_SOURCE_VTX_RAW, size, 1u);

#if REF_RX_DBG_ENABLE
    if (size < g_referee_rx_dbg_diag.rx_evt_size_min) {
        g_referee_rx_dbg_diag.rx_evt_size_min = size;
    }
    if (size > g_referee_rx_dbg_diag.rx_evt_size_max) {
        g_referee_rx_dbg_diag.rx_evt_size_max = size;
    }
#endif
    referee_fifo_push_bytes(data, size);
}

void referee_parse_reset(void)
{
    g_referee_rx_parse_len = 0u;
    referee_dbg_note_parse_used(g_referee_rx_parse_len);
}

void referee_process_frame(const uint8_t* frame, uint16_t frame_len, RefCustomCtrlRxSource source)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint16_t data_len = referee_read_u16_le(&frame[1]);
    const uint8_t frame_seq = (frame_len > 3u) ? frame[3] : 0u;
    const uint16_t cmd_id = (frame_len >= (REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN)) ? referee_read_u16_le(&frame[5]) : 0u;
    const uint8_t frame_crc8 = (frame_len > 4u) ? frame[4] : 0u;
    const uint8_t calc_crc8 = referee_crc8(frame, 4u);
    const uint16_t frame_crc16 = referee_read_u16_le(&frame[frame_len - REFEREE_TAIL_LEN]);
    const uint16_t calc_crc16 = referee_crc16(frame, (uint16_t)(frame_len - REFEREE_TAIL_LEN));
    const uint8_t crc_ok = ((frame_crc8 == calc_crc8) && (frame_crc16 == calc_crc16)) ? 1u : 0u;

#if REFEREE_RX_FRAME_HEADER_TRACE_ENABLE
    referee_trace_rx_frame_header(frame,
                                  frame_len,
                                  data_len,
                                  frame_seq,
                                  cmd_id,
                                  frame_crc8,
                                  calc_crc8,
                                  frame_crc16,
                                  calc_crc16);
#endif

    referee_stats_mutate([&](referee_rx_stats_t& stats) {
        stats.total_frames++;
        stats.last_rx_tick_ms = now_ms;
        stats.last_cmd_id = cmd_id;
        stats.last_frame_len = frame_len;
    });
    referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
        referee_custom_rx_path_diag_t* path = referee_custom_rx_path_mut(runtime, source);
        path->frames_total++;
    });

    if (frame_crc8 != calc_crc8) {
        referee_stats_mutate([](referee_rx_stats_t& stats) {
            stats.crc8_errors++;
        });
        referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
            referee_custom_rx_path_mut(runtime, source)->crc8_errors++;
        });
        referee_dbg_trigger_burst(REF_RX_DBG_TRIG_CRC, now_ms, "crc8");
        referee_dbg_log_burst_frame(now_ms, cmd_id, data_len, frame_seq, crc_ok, frame_len);
        return;
    }

    if (calc_crc16 != frame_crc16) {
        referee_stats_mutate([](referee_rx_stats_t& stats) {
            stats.crc16_errors++;
        });
        referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
            referee_custom_rx_path_mut(runtime, source)->crc16_errors++;
        });
        referee_dbg_trigger_burst(REF_RX_DBG_TRIG_CRC, now_ms, "crc16");
        referee_dbg_log_burst_frame(now_ms, cmd_id, data_len, frame_seq, crc_ok, frame_len);
        return;
    }

    referee_stats_mutate([](referee_rx_stats_t& stats) {
        stats.good_frames++;
    });
    referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
        referee_custom_rx_path_mut(runtime, source)->frames_good++;
    });

    if (cmd_id != REFEREE_CMD_ID_CUSTOM_CONTROLLER_TO_ROBOT) {
        referee_stats_mutate([](referee_rx_stats_t& stats) {
            stats.unknown_cmd_frames++;
        });
        referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
            referee_custom_rx_path_mut(runtime, source)->unknown_cmd_frames++;
        });
        referee_dbg_log_burst_frame(now_ms, cmd_id, data_len, frame_seq, crc_ok, frame_len);
        return;
    }

    referee_stats_mutate([](referee_rx_stats_t& stats) {
        stats.cmd_0302_frames++;
    });

    if (data_len != REFEREE_CUSTOM_DATA_LEN) {
        referee_stats_mutate([](referee_rx_stats_t& stats) {
            stats.payload_len_errors++;
        });
        referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
            referee_custom_rx_path_mut(runtime, source)->payload_len_errors++;
        });
        referee_dbg_log_burst_frame(now_ms, cmd_id, data_len, frame_seq, crc_ok, frame_len);
        return;
    }

    omnix_custom_controller_raw_payload_t payload{};
    std::memcpy(&payload, &frame[REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN], sizeof(payload));

    if (payload.version != OMNIX_CUSTOM_CONTROLLER_PROTOCOL_VERSION) {
        referee_stats_mutate([](referee_rx_stats_t& stats) {
            stats.payload_version_errors++;
        });
        referee_custom_rx_mutate([&](referee_custom_rx_runtime_t& runtime) {
            referee_custom_rx_path_mut(runtime, source)->payload_version_errors++;
        });
        referee_dbg_log_burst_frame(now_ms, cmd_id, data_len, frame_seq, crc_ok, frame_len);
        return;
    }

    const bool source_selected = referee_custom_rx_note_valid_frame(source, payload.seq, now_ms);
    referee_custom_rx_runtime_t rx_runtime{};
    (void)referee_custom_rx_copy(&rx_runtime);
    const referee_custom_rx_path_diag_t* active_path =
        referee_custom_rx_path(rx_runtime, rx_runtime.active_source);

    referee_stats_mutate([&](referee_rx_stats_t& stats) {
        stats.cmd_0302_good_frames++;
        stats.cmd_0302_seq_gap_count = active_path->seq_gap_count;
        stats.last_0302_seq = active_path->last_seq;
        stats.last_0302_seq_valid = active_path->last_seq_valid;
    });

    if (source_selected &&
        Referee_IngestJointRawPayload(&frame[REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN], data_len)) {
        referee_dbg_note_valid_0302(now_ms);
        referee_latency_on_rx_0302(payload, now_ms);
        referee_store_raw0302(&frame[REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN]);
    }
    referee_dbg_log_burst_frame(now_ms, cmd_id, data_len, frame_seq, crc_ok, frame_len);
}

void referee_drop_parse_prefix(uint16_t drop_len)
{
    if (drop_len == 0u) {
        return;
    }

    if (drop_len >= g_referee_rx_parse_len) {
        g_referee_rx_parse_len = 0u;
        referee_dbg_note_parse_used(g_referee_rx_parse_len);
        return;
    }

    std::memmove(g_referee_rx_parse_buf,
                 &g_referee_rx_parse_buf[drop_len],
                 (size_t)(g_referee_rx_parse_len - drop_len));
    g_referee_rx_parse_len = (uint16_t)(g_referee_rx_parse_len - drop_len);
    referee_dbg_note_parse_used(g_referee_rx_parse_len);
}

void referee_try_parse_frames(uint32_t now_ms)
{
    while (g_referee_rx_parse_len > 0u) {
        if (g_referee_rx_parse_buf[0] != REFEREE_SOF) {
            referee_dbg_note_resync_drop();
            referee_dbg_note_parse_slide();
            referee_drop_parse_prefix(1u);
            continue;
        }

        if (g_referee_rx_parse_len < REFEREE_HEADER_LEN) {
            break;
        }

        if (referee_crc8(g_referee_rx_parse_buf, 4u) != g_referee_rx_parse_buf[4]) {
            referee_stats_mutate([](referee_rx_stats_t& stats) {
                stats.crc8_errors++;
            });
            referee_dbg_trigger_burst(REF_RX_DBG_TRIG_CRC, now_ms, "crc8");
            referee_dbg_note_resync_drop();
            referee_dbg_note_parse_slide();
            referee_drop_parse_prefix(1u);
            continue;
        }

        const uint16_t data_len = referee_read_u16_le(&g_referee_rx_parse_buf[1]);
        const uint16_t frame_len =
            (uint16_t)(REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_TAIL_LEN);

        if (frame_len < REFEREE_MIN_FRAME_LEN) {
            referee_stats_mutate([](referee_rx_stats_t& stats) {
                stats.short_frames++;
            });
            referee_dbg_note_resync_drop();
            referee_dbg_note_parse_slide();
            referee_drop_parse_prefix(1u);
            continue;
        }

        if ((frame_len > REFEREE_MAX_FRAME_LEN) || (frame_len > REFEREE_RX_PARSE_BUF_SIZE)) {
            referee_stats_mutate([](referee_rx_stats_t& stats) {
                stats.oversize_frames++;
            });
            referee_dbg_note_resync_drop();
            referee_dbg_note_parse_slide();
            referee_drop_parse_prefix(1u);
            continue;
        }

        if (g_referee_rx_parse_len < frame_len) {
            break;
        }

        const uint16_t frame_crc16 =
            referee_read_u16_le(&g_referee_rx_parse_buf[frame_len - REFEREE_TAIL_LEN]);
        const uint16_t calc_crc16 =
            referee_crc16(g_referee_rx_parse_buf, (uint16_t)(frame_len - REFEREE_TAIL_LEN));

        referee_process_frame(g_referee_rx_parse_buf, frame_len, REF_CUSTOM_CTRL_RX_SOURCE_VTX_RAW);

        if (calc_crc16 != frame_crc16) {
            referee_dbg_note_resync_drop();
            referee_dbg_note_parse_slide();
            referee_drop_parse_prefix(1u);
            continue;
        }

        referee_dbg_reset_resync_drop();
        referee_drop_parse_prefix(frame_len);
    }
}

void referee_framer_feed_bytes(const uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0u) {
        return;
    }

    for (size_t i = 0u; i < len; ++i) {
        if (g_referee_rx_parse_len < REFEREE_RX_PARSE_BUF_SIZE) {
            g_referee_rx_parse_buf[g_referee_rx_parse_len++] = data[i];
        } else {
            std::memmove(g_referee_rx_parse_buf,
                         &g_referee_rx_parse_buf[1],
                         (size_t)(REFEREE_RX_PARSE_BUF_SIZE - 1u));
            g_referee_rx_parse_buf[REFEREE_RX_PARSE_BUF_SIZE - 1u] = data[i];
            referee_dbg_note_parse_slide();
        }

        referee_dbg_note_parse_used(g_referee_rx_parse_len);
    }

    referee_try_parse_frames(HAL_GetTick());
}

void referee_update_external_control_gate(void)
{
    static bool last_logged_enabled = false;
    static bool last_logged_vt03_online = false;
    static bool last_logged_active_vt03 = false;
    static bool last_logged_gear_chassis = false;
    static bool last_logged_ext_cmd_fresh = false;
    static bool last_logged_zero_force = false;

    RC_State rc{};
    referee_joint_raw_cmd_t cmd{};
    RC_GetSnapshot(&rc);
    (void)referee_copy_rx(&cmd);
    const RC_Status status = RC_GetStatus();
    const RcControlMode mode = Rc_ResolveControlMode(rc, status);
    const bool vt03_online = status.vt03_online;
    const bool active_vt03 = (status.active_src == RC_SRC_VT03);
    const bool gear_chassis = (mode.main_mode == RcMainMode::Chassis);
    const bool ext_cmd_fresh = cmd.fresh;
    const bool diag_enabled = ext_cmd_fresh;
    uint8_t reason_bits = 0u;
    if (vt03_online) {
        reason_bits |= REFEREE_GATE_REASON_VT03_ONLINE;
    }
    if (active_vt03) {
        reason_bits |= REFEREE_GATE_REASON_ACTIVE_VT03;
    }
    if (gear_chassis) {
        reason_bits |= REFEREE_GATE_REASON_GEAR_MAIN;
    }
    if (ext_cmd_fresh) {
        reason_bits |= REFEREE_GATE_REASON_CMD_FRESH;
    }
    if (gimbal_zero_force) {
        reason_bits |= REFEREE_GATE_REASON_ZERO_FORCE;
    }

#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
    if ((diag_enabled != last_logged_enabled) ||
        (vt03_online != last_logged_vt03_online) ||
        (active_vt03 != last_logged_active_vt03) ||
        (gear_chassis != last_logged_gear_chassis) ||
        (ext_cmd_fresh != last_logged_ext_cmd_fresh) ||
        (gimbal_zero_force != last_logged_zero_force)) {
        last_logged_enabled = diag_enabled;
        last_logged_vt03_online = vt03_online;
        last_logged_active_vt03 = active_vt03;
        last_logged_gear_chassis = gear_chassis;
        last_logged_ext_cmd_fresh = ext_cmd_fresh;
        last_logged_zero_force = gimbal_zero_force;
        LOGI("[REF][GATE][DIAG] recv_only=1 data_enabled=%u reason=0x%02X vt03_online=%u active_vt03=%u gear_main=%u cmd_fresh=%u zero_force=%u",
             diag_enabled ? 1u : 0u,
             reason_bits,
             vt03_online ? 1u : 0u,
             active_vt03 ? 1u : 0u,
             gear_chassis ? 1u : 0u,
             ext_cmd_fresh ? 1u : 0u,
             gimbal_zero_force ? 1u : 0u);
    }
#else
    (void)last_logged_enabled;
    (void)last_logged_vt03_online;
    (void)last_logged_active_vt03;
    (void)last_logged_gear_chassis;
    (void)last_logged_ext_cmd_fresh;
    (void)last_logged_zero_force;
    (void)reason_bits;
#endif

    // Legacy flag is kept only as a diagnostic snapshot for 0309 feedback and
    // now indicates fresh custom-controller data reception rather than active motor takeover.
    referee_set_external_control_enabled(diag_enabled);
}

} // namespace

extern "C" bool Referee_IngestJointRawPayload(const uint8_t* data, uint16_t len)
{
    omnix_custom_controller_raw_payload_t payload{};

    if (data == nullptr || len < OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE) {
        return false;
    }

    std::memcpy(&payload, data, sizeof(payload));
    if (payload.version != OMNIX_CUSTOM_CONTROLLER_PROTOCOL_VERSION) {
        return false;
    }

    referee_store_payload(&payload);
    return true;
}

extern "C" bool Referee_IngestCustomCtrlFrameFromBridge(const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len < REFEREE_MIN_FRAME_LEN || len > REFEREE_MAX_FRAME_LEN) {
        return false;
    }

    referee_custom_rx_note_ingress(REF_CUSTOM_CTRL_RX_SOURCE_SERVER_BRIDGE, len, 1u);
    referee_process_frame(data, static_cast<uint16_t>(len), REF_CUSTOM_CTRL_RX_SOURCE_SERVER_BRIDGE);
    return true;
}

extern "C" bool Referee_ReadJointRawCmd(referee_joint_raw_cmd_t* out)
{
    return referee_copy_rx(out);
}

extern "C" bool Referee_GetJointRawU16(uint8_t joint_id, uint16_t* out_raw_u16)
{
    referee_joint_raw_cmd_t cmd{};

    if (out_raw_u16 == nullptr || !referee_copy_rx(&cmd) || !cmd.fresh ||
        !Omnix_JointRawValid(cmd.valid_mask, joint_id)) {
        return false;
    }

    *out_raw_u16 = cmd.raw_u16[joint_id - 1u];
    return true;
}

extern "C" bool Referee_ReadRxStats(referee_rx_stats_t* out)
{
    return referee_stats_copy(out);
}

extern "C" bool Referee_ReadRxRaw0302(uint8_t* out_buf, uint16_t out_len)
{
    return referee_raw_copy(out_buf, out_len);
}

extern "C" bool Referee_IsExternalControlEnabled(void)
{
    return g_external_control_enabled;
}

extern "C" bool Referee_IsMappingResetActive(void)
{
    return g_mapping_reset_active;
}

extern "C" void Referee_RequestMappingReset(void)
{
    g_mapping_reset_active = true;
    g_mapping_reset_controller_done = false;
}

extern "C" void Referee_ClearMappingReset(void)
{
    g_mapping_reset_active = false;
    g_mapping_reset_controller_done = false;
}

extern "C" void Referee_RequestRemoteBuzzerPulses(uint8_t count)
{
    const uint8_t pending_before = g_remote_buzzer_pulses_pending;
    referee_remote_buzzer_append_pulses(count);
    const uint8_t pending_after = g_remote_buzzer_pulses_pending;
    if (count > 0u) {
        LOGI("[REF][BUZZER] pulses_req=%u pending=%u->%u",
             static_cast<unsigned>(count),
             static_cast<unsigned>(pending_before),
             static_cast<unsigned>(pending_after));
    }
}

extern "C" bool Referee_ReadTxRobotJointFeedback(omnix_robot_joint_feedback_payload_t* out)
{
    return referee_tx_feedback_copy(out);
}

extern "C" void Start_Referee_Task(void *argument)
{
    (void)argument;
    referee_joint_raw_cmd_t& cmd = g_referee_task_cmd;
    referee_rx_stats_t& stats = g_referee_task_stats;
    referee_uart_diag_t& uart_diag = g_referee_task_uart_diag;
    omnix_robot_joint_feedback_payload_t& tx_feedback = g_referee_task_tx_feedback;

    VT03_RegisterRawTapCallback(referee_on_vt03_raw_tap);
    referee_parse_reset();
    referee_uart_diag_mutate([&](referee_uart_diag_t& diag) {
        diag.rx_armed = (huart9.hdmarx != nullptr) ? 1u : 0u;
        diag.rx_arm_last_ret = (uint32_t)HAL_OK;
    });
    referee_uart_diag_t init_uart_diag{};
    (void)referee_uart_diag_copy(&init_uart_diag);
#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
    LOGI("[REF][INIT][%s] link=%s uart=%s baud=%lu rx_cmd=0x%04lX tx_cmd=0x%04lX armed=%lu arm_ret=%lu",
         REFEREE_RX_SOURCE,
         REFEREE_LINK_KIND,
         REFEREE_UART_NAME,
         static_cast<unsigned long>(REFEREE_LINK_BAUDRATE),
         static_cast<unsigned long>(REFEREE_CMD_ID_CUSTOM_CONTROLLER_TO_ROBOT),
         static_cast<unsigned long>(REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER),
         static_cast<unsigned long>(init_uart_diag.rx_armed),
         static_cast<unsigned long>(init_uart_diag.rx_arm_last_ret));
    LOGI("[REF][TAP][INIT][%s] raw_tap_registered=1 owner=VT03 uart=%s",
         REFEREE_RX_SOURCE,
         REFEREE_UART_NAME);
#endif

    uint32_t last_log_ms = 0u;
    uint32_t last_map_log_ms = 0u;
    referee_uart_diag_t last_uart_diag = {};
    referee_rx_stats_t last_stats = {};
    referee_custom_rx_runtime_t last_custom_rx = {};

    uint32_t last_tx_log_ms = 0u;
    uint32_t last_tx_busy_log_ms = 0u;
    uint16_t last_tx_logged_seq = 0xFFFFu;
    uint32_t last_tx_send_ms = 0u;
    const uint32_t task_start_ms = HAL_GetTick();
    uint16_t next_tx_feedback_seq = 0u;
    uint16_t last_sent_tx_feedback_seq = 0u;
    bool tx_has_success = false;
    uint32_t last_tap_wait_warn_ms = 0u;
    bool tap_first_logged = false;
    tx_feedback.version = OMNIX_ROBOT_JOINT_FEEDBACK_VERSION;

#if REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
    (void)last_map_log_ms;
    (void)last_tx_busy_log_ms;
    (void)last_tap_wait_warn_ms;
    (void)tap_first_logged;
#endif
#if REF_RX_DBG_ENABLE
    g_referee_rx_dbg_diag = {0u, 0u, 0u, 0xFFFFu, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    g_referee_rx_dbg_rt = {};
    g_referee_rx_dbg_rt.task_start_ms = task_start_ms;
    g_referee_rx_dbg_rt.last_summary_ms = task_start_ms;
    g_referee_rx_dbg_rt.last_silence_warn_ms = task_start_ms;
    g_referee_rx_dbg_rt.last_resync_warn_ms = task_start_ms;
    g_referee_rx_dbg_rt.last_uart_err_seen = 0u;
#endif

    for (;;) {
        const size_t n = referee_fifo_pop_bytes(g_referee_task_buf, sizeof(g_referee_task_buf));

        if (n > 0u) {
            referee_stats_mutate([&](referee_rx_stats_t& mutable_stats) {
                mutable_stats.total_bytes += (uint32_t)n;
            });
            referee_framer_feed_bytes(g_referee_task_buf, n);
        }

        (void)referee_copy_rx(&cmd);
        referee_update_external_control_gate();
        (void)referee_stats_copy(&stats);
        (void)referee_uart_diag_copy(&uart_diag);
        referee_custom_rx_runtime_t custom_rx{};
        (void)referee_custom_rx_copy(&custom_rx);

        const uint32_t now_ms = HAL_GetTick();
        referee_debug_probe_update(now_ms, n, cmd, stats, uart_diag, custom_rx);
        if (g_mapping_reset_active &&
            Gimbal_IsMappingResetLocalDone() &&
            g_mapping_reset_controller_done) {
            g_mapping_reset_active = false;
            g_mapping_reset_controller_done = false;
            LOGI("[MAP][SYNC] complete local_done=1 ctrl_done=1");
        }
        const bool tx_active = cmd.fresh || g_mapping_reset_active || referee_remote_buzzer_tx_active();
        const uint32_t tx_interval_ms = referee_tx_interval_ms(tx_active);
#if REF_RX_DBG_ENABLE
        referee_dbg_handle_pending_triggers(now_ms);
        if (uart_diag.rx_hw_error_count != g_referee_rx_dbg_rt.last_uart_err_seen) {
            g_referee_rx_dbg_rt.last_uart_err_seen = uart_diag.rx_hw_error_count;
            referee_dbg_trigger_burst(REF_RX_DBG_TRIG_UART, now_ms, "uart_hw_error");
        }
        referee_dbg_log_summary(stats, uart_diag, now_ms);
        referee_dbg_service_burst_timeout(now_ms);
#endif
#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
        if (!tap_first_logged && uart_diag.rx_event_count > 0u) {
            tap_first_logged = true;
            LOGI("[REF][TAP][RX][%s] first_evt=%lu bytes=%lu",
                 REFEREE_RX_SOURCE,
                 static_cast<unsigned long>(uart_diag.rx_event_count),
                 static_cast<unsigned long>(uart_diag.rx_event_bytes));
        } else if (!tap_first_logged &&
                   (now_ms - task_start_ms >= 3000u) &&
                   (now_ms - last_tap_wait_warn_ms >= 3000u)) {
            last_tap_wait_warn_ms = now_ms;
            LOGW("[REF][TAP][RX][%s] waiting_raw_tap evt=%lu bytes=%lu",
                 REFEREE_RX_SOURCE,
                 static_cast<unsigned long>(uart_diag.rx_event_count),
                 static_cast<unsigned long>(uart_diag.rx_event_bytes));
        }
#endif

        if ((now_ms - last_log_ms) >= 1000u) {
            const uint32_t rx_event_count_delta = uart_diag.rx_event_count - last_uart_diag.rx_event_count;
            const uint32_t rx_event_bytes_delta = uart_diag.rx_event_bytes - last_uart_diag.rx_event_bytes;
            const uint32_t vt03_bytes_delta = uart_diag.rx_vt03_feed_bytes - last_uart_diag.rx_vt03_feed_bytes;
            const uint32_t tx_ok_delta = stats.good_tx_frames - last_stats.good_tx_frames;
            const referee_custom_rx_path_diag_t* active_path =
                referee_custom_rx_path(custom_rx, custom_rx.active_source);
            const referee_custom_rx_path_diag_t* last_active_path =
                referee_custom_rx_path(last_custom_rx, custom_rx.active_source);
            const uint32_t cmd_0302_good_delta = active_path->cmd_0302_good - last_active_path->cmd_0302_good;
            const uint32_t frame_good_delta = active_path->frames_good - last_active_path->frames_good;
            const uint32_t crc8_delta = active_path->crc8_errors - last_active_path->crc8_errors;
            const uint32_t crc16_delta = active_path->crc16_errors - last_active_path->crc16_errors;
            const uint32_t len_delta = active_path->payload_len_errors - last_active_path->payload_len_errors;
            const uint32_t ver_delta = active_path->payload_version_errors - last_active_path->payload_version_errors;
            const uint32_t unknown_delta = active_path->unknown_cmd_frames - last_active_path->unknown_cmd_frames;
            const uint16_t last_0302_seq = (stats.last_0302_seq_valid != 0u) ? stats.last_0302_seq : 0u;
            const uint32_t age_ms = (cmd.rx_tick_ms != 0u) ? (now_ms - cmd.rx_tick_ms) : 0u;
            const uint32_t valid_0302_silence_ms =
                (cmd.rx_tick_ms != 0u) ? (now_ms - cmd.rx_tick_ms) : (now_ms - task_start_ms);
            last_log_ms = now_ms;
            LOGI("[REF][HEALTH][%s] src=%s 0302_good_1s=%lu frame_good_1s=%lu 0302_silence_ms=%lu 0309_tx_ok_1s=%lu uart9_evt_1s=%lu vt03_bytes_1s=%lu crc8_1s=%lu crc16_1s=%lu len_1s=%lu ver_1s=%lu unknown_1s=%lu 0309_tx_int_ms=%lu",
                 REFEREE_RX_SOURCE,
                 referee_custom_rx_source_name(custom_rx.active_source),
                 static_cast<unsigned long>(cmd_0302_good_delta),
                 static_cast<unsigned long>(frame_good_delta),
                 static_cast<unsigned long>(valid_0302_silence_ms),
                 static_cast<unsigned long>(tx_ok_delta),
                 static_cast<unsigned long>(rx_event_count_delta),
                 static_cast<unsigned long>(vt03_bytes_delta),
                 static_cast<unsigned long>(crc8_delta),
                 static_cast<unsigned long>(crc16_delta),
                 static_cast<unsigned long>(len_delta),
                 static_cast<unsigned long>(ver_delta),
                 static_cast<unsigned long>(unknown_delta),
                 static_cast<unsigned long>(tx_interval_ms));
            if ((vt03_bytes_delta > 0u) && (cmd_0302_good_delta == 0u)) {
                LOGW("[REF][HEALTH][%s] bytes_present_but_no_valid_0302 src=%s bytes_1s=%lu frame_good_1s=%lu crc8_1s=%lu crc16_1s=%lu len_1s=%lu ver_1s=%lu unknown_1s=%lu",
                     REFEREE_RX_SOURCE,
                     referee_custom_rx_source_name(custom_rx.active_source),
                     static_cast<unsigned long>(vt03_bytes_delta),
                     static_cast<unsigned long>(frame_good_delta),
                     static_cast<unsigned long>(crc8_delta),
                     static_cast<unsigned long>(crc16_delta),
                     static_cast<unsigned long>(len_delta),
                     static_cast<unsigned long>(ver_delta),
                     static_cast<unsigned long>(unknown_delta));
            }
#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
            LOGI("[CTRL][RX][%s] fresh=%u seq=%u last_seq=%u seq_gap=%lu age=%lu frames=%lu good=%lu 0302=%lu 0302_good_1s=%lu evt_1s=%lu bytes_1s=%lu c8=%lu c16=%lu len=%lu ver=%lu rx_dma=%u tx_dma=%u hw_err=%lu",
                 REFEREE_RX_SOURCE,
                 cmd.fresh ? 1u : 0u,
                 cmd.seq,
                 last_0302_seq,
                 static_cast<unsigned long>(stats.cmd_0302_seq_gap_count),
                 static_cast<unsigned long>(age_ms),
                 static_cast<unsigned long>(stats.total_frames),
                 static_cast<unsigned long>(stats.good_frames),
                 static_cast<unsigned long>(stats.cmd_0302_frames),
                 static_cast<unsigned long>(cmd_0302_good_delta),
                 static_cast<unsigned long>(rx_event_count_delta),
                 static_cast<unsigned long>(rx_event_bytes_delta),
                 static_cast<unsigned long>(stats.crc8_errors),
                 static_cast<unsigned long>(stats.crc16_errors),
                 static_cast<unsigned long>(stats.payload_len_errors),
                 static_cast<unsigned long>(stats.payload_version_errors),
                 (huart9.hdmarx != nullptr) ? 1u : 0u,
                 (huart9.hdmatx != nullptr) ? 1u : 0u,
                 static_cast<unsigned long>(uart_diag.rx_hw_error_count));
#endif
            LOGI("[CTRL][0302][RX][%s] src=%s seq=%u mask=0x%02X diag=0x%02X age=%lu J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
                 REFEREE_RX_SOURCE,
                 referee_custom_rx_source_name(custom_rx.active_source),
                 cmd.seq,
                 cmd.valid_mask,
                 cmd.diag_flags,
                 static_cast<unsigned long>(age_ms),
                 static_cast<unsigned>(cmd.raw_u16[0]),
                 static_cast<unsigned>(cmd.raw_u16[1]),
                 static_cast<unsigned>(cmd.raw_u16[2]),
                 static_cast<unsigned>(cmd.raw_u16[3]),
                 static_cast<unsigned>(cmd.raw_u16[4]),
                 static_cast<unsigned>(cmd.raw_u16[5]),
                 static_cast<unsigned>(cmd.raw_u16[6]));

            gimbal_joint_snapshot_t gimbal_snap{};
            if (Gimbal_ReadJointSnapshot(&gimbal_snap)) {
                LOGI("[CTRL][GIM][ENC] J1=%u(%u) J2=%u(%u) J3=%u(%u) J4=%u(%u) J5=%u(%u) J6=%u(%u) J7=%u(%u) J8=%u(%u) J9=%u(%u)",
                     static_cast<unsigned>(gimbal_snap.raw_u16[0]), static_cast<unsigned>(gimbal_snap.online[0]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[1]), static_cast<unsigned>(gimbal_snap.online[1]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[2]), static_cast<unsigned>(gimbal_snap.online[2]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[3]), static_cast<unsigned>(gimbal_snap.online[3]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[4]), static_cast<unsigned>(gimbal_snap.online[4]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[5]), static_cast<unsigned>(gimbal_snap.online[5]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[6]), static_cast<unsigned>(gimbal_snap.online[6]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[7]), static_cast<unsigned>(gimbal_snap.online[7]),
                     static_cast<unsigned>(gimbal_snap.raw_u16[8]), static_cast<unsigned>(gimbal_snap.online[8]));
            } else {
                LOGW("[CTRL][GIM][ENC] snapshot_unavailable");
            }

#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
            referee_latency_snapshot_t latency{};
            if (referee_latency_snapshot(&latency)) {
                LOGI("[CTRL][LAT][0302->0309][%s] ingress_ms(n/last/min/p50/p95/max)=%lu/%u/%u/%u/%u/%u board_rx_to_tx_ms(n/last/min/p50/p95/max)=%lu/%u/%u/%u/%u/%u",
                     REFEREE_RX_SOURCE,
                     static_cast<unsigned long>(latency.ingress_total_samples),
                     latency.ingress_last_ms,
                     latency.ingress_min_ms,
                     latency.ingress_p50_ms,
                     latency.ingress_p95_ms,
                     latency.ingress_max_ms,
                     static_cast<unsigned long>(latency.board_total_samples),
                     latency.board_last_ms,
                     latency.board_min_ms,
                     latency.board_p50_ms,
                     latency.board_p95_ms,
                     latency.board_max_ms);
            } else {
                LOGI("[CTRL][LAT][0302->0309][%s] latency_sample_absent src=%s",
                     REFEREE_RX_SOURCE,
                     referee_custom_rx_source_name(custom_rx.active_source));
            }
#endif
            last_uart_diag = uart_diag;
            last_stats = stats;
            last_custom_rx = custom_rx;
        }

#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
        if (Referee_IsMappingResetActive() &&
            ((now_ms - last_map_log_ms) >= 500u)) {
            const uint32_t age_ms = (cmd.rx_tick_ms != 0u) ? (now_ms - cmd.rx_tick_ms) : 0u;
            last_map_log_ms = now_ms;
            LOGI("[MAP][0302][RX][%s] seq=%u done=%u age=%lu J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
                 REFEREE_RX_SOURCE,
                 cmd.seq,
                 ((cmd.diag_flags & OMNIX_DIAG_FLAG_MAPPING_RESET_DONE) != 0u) ? 1u : 0u,
                 static_cast<unsigned long>(age_ms),
                 static_cast<unsigned>(cmd.raw_u16[0]),
                 static_cast<unsigned>(cmd.raw_u16[1]),
                 static_cast<unsigned>(cmd.raw_u16[2]),
                 static_cast<unsigned>(cmd.raw_u16[3]),
                 static_cast<unsigned>(cmd.raw_u16[4]),
                 static_cast<unsigned>(cmd.raw_u16[5]),
                 static_cast<unsigned>(cmd.raw_u16[6]));
            LOGI("[MAP][SYNC] local_done=%u ctrl_done=%u",
                 Gimbal_IsMappingResetLocalDone() ? 1u : 0u,
                 g_mapping_reset_controller_done ? 1u : 0u);
        }
#endif

        bool tx_feedback_ready = false;
        tx_feedback_ready = referee_prepare_tx_feedback_payload(&tx_feedback, last_sent_tx_feedback_seq, now_ms);
        if (tx_feedback_ready) {
            referee_store_tx_feedback(&tx_feedback);
        }

        if (tx_has_success &&
            tx_feedback_ready &&
            (tx_feedback.seq != last_tx_logged_seq) &&
            ((now_ms - last_tx_log_ms) >= 1000u)) {
            last_tx_logged_seq = tx_feedback.seq;
            last_tx_log_ms = now_ms;
            const uint32_t echo_controller_tick_ms = referee_read_u32_le(&tx_feedback.reserved[0]);
            const uint16_t board_rx_to_tx_ms = referee_read_u16_le(&tx_feedback.reserved[4]);
            LOGI("[REF][0309][TX][%s] seq=%u echo=%lu board_rx_to_tx=%u flags=0x%02X diag=0x%02X valid=0x%02X online=0x%02X pause=%u J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
                 REFEREE_TX_SOURCE,
                 tx_feedback.seq,
                 static_cast<unsigned long>(echo_controller_tick_ms),
                 board_rx_to_tx_ms,
                 tx_feedback.flags,
                 tx_feedback.diag_flags,
                 tx_feedback.valid_mask,
                 tx_feedback.online_mask,
                 tx_feedback.pause_phase,
                 static_cast<unsigned>(tx_feedback.raw_u16[0]),
                 static_cast<unsigned>(tx_feedback.raw_u16[1]),
                 static_cast<unsigned>(tx_feedback.raw_u16[2]),
                 static_cast<unsigned>(tx_feedback.raw_u16[3]),
                 static_cast<unsigned>(tx_feedback.raw_u16[4]),
                 static_cast<unsigned>(tx_feedback.raw_u16[5]),
                 static_cast<unsigned>(tx_feedback.raw_u16[6]));
        }

        if (tx_feedback_ready &&
            ((last_tx_send_ms == 0u) || ((now_ms - last_tx_send_ms) >= tx_interval_ms))) {
            referee_stats_mutate([](referee_rx_stats_t& mutable_stats) {
                mutable_stats.total_tx_frames++;
            });

            if (!referee_uart_tx_ready()) {
                referee_stats_mutate([](referee_rx_stats_t& mutable_stats) {
                    mutable_stats.tx_busy++;
                });
#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
                if ((now_ms - last_tx_busy_log_ms) >= 1000u) {
                    last_tx_busy_log_ms = now_ms;
                    LOGW("[REF][TX][%s] uart9 busy gState=%lu dma_link=%u dma_state=%lu",
                         REFEREE_TX_SOURCE,
                         static_cast<unsigned long>(huart9.gState),
                         (huart9.hdmatx != nullptr) ? 1u : 0u,
                         (huart9.hdmatx != nullptr) ? static_cast<unsigned long>(huart9.hdmatx->State) : 0xFFFFFFFFul);
                }
#endif
            } else {
                omnix_robot_joint_feedback_payload_t tx_feedback_to_send = tx_feedback;
                tx_feedback_to_send.seq = next_tx_feedback_seq;
                if (referee_build_frame(REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER,
                                        reinterpret_cast<const uint8_t*>(&tx_feedback_to_send),
                                           OMNIX_ROBOT_JOINT_FEEDBACK_PAYLOAD_SIZE,
                                           g_referee_tx_frame,
                                        sizeof(g_referee_tx_frame))) {
                    const HAL_StatusTypeDef ret =
                        HAL_UART_Transmit_DMA(&huart9, g_referee_tx_frame, REFEREE_TX_FRAME_LEN);
                    if (ret == HAL_OK) {
                        last_tx_send_ms = now_ms;
                        tx_has_success = true;
                        last_sent_tx_feedback_seq = next_tx_feedback_seq;
                        next_tx_feedback_seq = (uint16_t)(next_tx_feedback_seq + 1u);
                        tx_feedback = tx_feedback_to_send;
                        referee_store_tx_feedback(&tx_feedback);
                        const bool remote_buzzer_flag_set =
                            ((tx_feedback.diag_flags & OMNIX_DIAG_FLAG_REMOTE_BUZZER_REQ) != 0u);
                        referee_remote_buzzer_on_tx_success(remote_buzzer_flag_set);
                        referee_stats_mutate([&](referee_rx_stats_t& mutable_stats) {
                            mutable_stats.good_tx_frames++;
                            mutable_stats.cmd_0309_tx_frames++;
                            mutable_stats.last_tx_tick_ms = now_ms;
                            mutable_stats.last_tx_cmd_id = REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER;
                        });
                    } else {
                        referee_stats_mutate([](referee_rx_stats_t& mutable_stats) {
                            mutable_stats.tx_errors++;
                        });
#if !REF_CUSTOM_CTRL_LOG_ONLY_RX0302_TX0309
                        if (ret != HAL_BUSY) {
                            LOGW("[REF][TX][%s] uart9 dma send failed ret=%d seq=%u",
                                 REFEREE_TX_SOURCE,
                                 (int)ret,
                                 tx_feedback_to_send.seq);
                        }
#endif
                    }
                } else {
                    referee_stats_mutate([](referee_rx_stats_t& mutable_stats) {
                        mutable_stats.tx_errors++;
                    });
                }
            }
        }

        osDelay(1u);
    }
}
