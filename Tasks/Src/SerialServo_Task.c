#include "SerialServo_Task.h"
#include "Referee_Task.h"

#include "lib_serial_servo_h12h.h"
#include "bsp_buzzer.h"
#include "bsp_srn_log.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

volatile uint8_t g_servo_online_count = 0u;
volatile serial_servo_boot_state_t g_servo_boot_state = SERVO_BOOT_WAIT_ONLINE;
volatile uint8_t g_servo_mapping_reset_active = 0u;

#define SERVO_RESTORE_RETRY_COUNT            3u
#define SERVO_RESTORE_MOVE_TIME_MS           1500u
#define SERVO_RESTORE_RETRY_DELAY_MS         30u
#define SERVO_RESTORE_INTER_ID_DELAY_MS      30u
#define SERVO_POST_RESTORE_HOLD_MS           3000u
#define SERVO_OFFLINE_FAST_BEEP_MS           3000u
#define SERVO_OFFLINE_FAST_BEEP_FREQ_HZ      2400u
#define SERVO_OFFLINE_FAST_BEEP_ON_MS        80u
#define SERVO_OFFLINE_FAST_BEEP_OFF_MS       80u
#define SERVO_SHORT_BEEP_FREQ_HZ             1800u
#define SERVO_SHORT_BEEP_MS                  120u
#define SERVO_BOOT_WAIT_POLL_MS              50u
#define SERVO_BUS_DIAG_LOG_PERIOD_MS         1000u
#define SERVO_READ_FAIL_HOLD_TIMEOUT_MS      200u

static const uint16_t g_servo_default_pos[SERVO_COUNT] = {
    SERVO_ZERO_RAW_BY_INDEX_INITIALIZER
};
static uint16_t g_servo_last_raw_u16[SERVO_COUNT] = {0u};
static uint32_t g_servo_last_ok_tick_ms[SERVO_COUNT] = {0u};
static uint8_t g_servo_has_last[SERVO_COUNT] = {0u};

typedef struct {
    uint8_t servo_id;
    uint8_t joint_id;
    uint16_t raw_min;
    uint16_t raw_zero;
    uint16_t raw_max;
} servo_joint_calib_t;

static const servo_joint_calib_t g_servo_joint_calib[SERVO_COUNT] = {
    {1u, 1u, OMNIX_CTRL_J1_RAW_MIN, OMNIX_CTRL_J1_RAW_ZERO, OMNIX_CTRL_J1_RAW_MAX},
    {2u, 2u, OMNIX_CTRL_J2_RAW_MIN, OMNIX_CTRL_J2_RAW_ZERO, OMNIX_CTRL_J2_RAW_MAX},
    {3u, 3u, OMNIX_CTRL_J3_RAW_MIN, OMNIX_CTRL_J3_RAW_ZERO, OMNIX_CTRL_J3_RAW_MAX},
    {4u, 4u, OMNIX_CTRL_J4_RAW_MIN, OMNIX_CTRL_J4_RAW_ZERO, OMNIX_CTRL_J4_RAW_MAX},
    {5u, 5u, OMNIX_CTRL_J5_RAW_MIN, OMNIX_CTRL_J5_RAW_ZERO, OMNIX_CTRL_J5_RAW_MAX},
    {6u, 6u, OMNIX_CTRL_J6_RAW_MIN, OMNIX_CTRL_J6_RAW_ZERO, OMNIX_CTRL_J6_RAW_MAX},
    {7u, 7u, OMNIX_CTRL_J7_RAW_MIN, OMNIX_CTRL_J7_RAW_ZERO, OMNIX_CTRL_J7_RAW_MAX},
};

typedef struct {
    uint32_t scan_calls;
    uint32_t scan_last_ms;
    uint32_t scan_max_ms;
    uint32_t scan_total_ms;
    uint32_t scan_overrun_count;
    uint32_t scan_last_online_count;
    uint32_t read_ok;
    uint32_t read_fail;
    uint32_t read_timeout;
    uint32_t hold_cache_used;
    uint32_t hold_cache_expired;
    uint32_t uart6_read_fail;
    uint32_t uart9_read_fail;
    uint32_t uart6_read_timeout;
    uint32_t uart9_read_timeout;
    uint32_t max_single_read_ms;
    uint8_t max_single_read_id;
    uint32_t last_single_read_ms;
    uint8_t last_single_read_id;
    uint32_t restore_runs;
    uint32_t restore_last_ms;
    uint32_t restore_max_ms;
    uint32_t restore_fail;
    uint32_t restore_max_op_ms;
    uint8_t restore_max_op_id;
    uint32_t loop_last_ms;
    uint32_t loop_max_ms;
    uint32_t loop_overrun_count;
} serial_servo_bus_diag_t;

static serial_servo_bus_diag_t g_servo_bus_diag = {0};

static uint8_t servo_id_to_index(uint8_t servo_id)
{
    return (uint8_t)(servo_id - SERVO_ID_MIN);
}

static omnix_controller_joint_raw_calib_t servo_joint_protocol_calib(const servo_joint_calib_t* calib)
{
    omnix_controller_joint_raw_calib_t protocol = {0u, 0u, 0u};

    if (calib == NULL) {
        return protocol;
    }

    protocol.raw_min = calib->raw_min;
    protocol.raw_zero = calib->raw_zero;
    protocol.raw_max = calib->raw_max;
    return protocol;
}

static uint16_t servo_joint_clamp_raw_no_log(const servo_joint_calib_t* calib, int16_t position_raw)
{
    const omnix_controller_joint_raw_calib_t protocol = servo_joint_protocol_calib(calib);
    const uint16_t raw = (position_raw <= 0) ? 0u : (uint16_t)position_raw;
    return Omnix_ControllerClampRaw(raw, &protocol);
}

static int16_t servo_joint_angle_cdeg_from_raw(const servo_joint_calib_t* calib, int16_t position_raw)
{
    const omnix_controller_joint_raw_calib_t protocol = servo_joint_protocol_calib(calib);
    const uint16_t raw = (position_raw <= 0) ? 0u : (uint16_t)position_raw;

    if (calib == NULL) {
        return 0;
    }
    return Omnix_DegToCdeg(Omnix_ControllerRawToRelDeg(raw, &protocol));
}

static float servo_joint_angle_deg_from_raw(const servo_joint_calib_t* calib, int16_t position_raw)
{
    const omnix_controller_joint_raw_calib_t protocol = servo_joint_protocol_calib(calib);
    const uint16_t raw = (position_raw <= 0) ? 0u : (uint16_t)position_raw;

    if (calib == NULL) {
        return 0.0f;
    }
    return Omnix_ControllerRawToRelDeg(raw, &protocol);
}

static uint16_t serial_servo_protocol_raw_for_publish(const servo_joint_calib_t* calib,
                                                      int16_t position_raw,
                                                      uint8_t cached_value)
{
    static uint32_t last_clamp_log_ms[SERVO_COUNT] = {0u};
    const omnix_controller_joint_raw_calib_t protocol = servo_joint_protocol_calib(calib);
    const uint16_t raw = (position_raw <= 0) ? 0u : (uint16_t)position_raw;
    const uint16_t clamped = Omnix_ControllerClampRaw(raw, &protocol);
    const uint8_t servo_index = (calib != NULL) ? servo_id_to_index(calib->servo_id) : 0u;
    const uint32_t now_ms = HAL_GetTick();

    if (calib == NULL) {
        return raw;
    }

    if ((clamped != raw) && ((now_ms - last_clamp_log_ms[servo_index]) >= 1000u)) {
        last_clamp_log_ms[servo_index] = now_ms;
        LOGW("[CTRL][0302][CLAMP] joint=%u id=%u raw=%u clamp=%u min=%u zero=%u max=%u cache=%u",
             calib->joint_id,
             calib->servo_id,
             raw,
             clamped,
             calib->raw_min,
             calib->raw_zero,
             calib->raw_max,
             cached_value);
    }

    return clamped;
}

static void serial_servo_log_mapping_targets(const char* stage)
{
    uint8_t i = 0u;

    if (stage == NULL) {
        stage = "map";
    }

    for (i = 0u; i < SERVO_COUNT; ++i) {
        const servo_joint_calib_t* calib = &g_servo_joint_calib[i];
        LOGI("[MAP][SERVO][TX][%s] joint=%u id=%u min=%u zero=%u max=%u zero_deg=%.2f zero_cdeg=%d",
             stage,
             calib->joint_id,
             calib->servo_id,
             calib->raw_min,
             calib->raw_zero,
             calib->raw_max,
             (double)servo_joint_angle_deg_from_raw(calib, (int16_t)calib->raw_zero),
             (int)servo_joint_angle_cdeg_from_raw(calib, (int16_t)calib->raw_zero));
    }
}

static void serial_servo_log_mapping_feedback(const int16_t* positions, const int* ret_pos)
{
    uint8_t i = 0u;

    if (positions == NULL || ret_pos == NULL) {
        return;
    }

    for (i = 0u; i < SERVO_COUNT; ++i) {
        const servo_joint_calib_t* calib = &g_servo_joint_calib[i];
        const uint8_t servo_index = servo_id_to_index(calib->servo_id);
        LOGI("[MAP][SERVO][RX] joint=%u id=%u raw=%d ok=%u rel_deg=%.2f rel_cdeg=%d",
             calib->joint_id,
             calib->servo_id,
             positions[servo_index],
             (ret_pos[servo_index] == SERIAL_SERVO_H12H_OK) ? 1u : 0u,
             (double)servo_joint_angle_deg_from_raw(calib, positions[servo_index]),
             (int)servo_joint_angle_cdeg_from_raw(calib, positions[servo_index]));
    }
}

static void serial_servo_log_session_status(const int16_t* positions, const int* ret_pos, uint8_t reset_done, uint8_t buzzer_req)
{
    const uint8_t idx_j5 = servo_id_to_index(5u);
    const uint8_t idx_j6 = servo_id_to_index(6u);
    const uint8_t idx_j7 = servo_id_to_index(7u);

    if (positions == NULL || ret_pos == NULL) {
        return;
    }

    LOGI("[MAP][SESSION][SERVO] reset_done=%u servos_unloaded=1 buzzer_req=%u "
         "J5=%d/%.2f/%d J6=%d/%.2f/%d J7=%d/%.2f/%d",
         reset_done,
         buzzer_req,
         positions[idx_j5],
         (double)servo_joint_angle_deg_from_raw(&g_servo_joint_calib[idx_j5], positions[idx_j5]),
         (int)servo_joint_angle_cdeg_from_raw(&g_servo_joint_calib[idx_j5], positions[idx_j5]),
         positions[idx_j6],
         (double)servo_joint_angle_deg_from_raw(&g_servo_joint_calib[idx_j6], positions[idx_j6]),
         (int)servo_joint_angle_cdeg_from_raw(&g_servo_joint_calib[idx_j6], positions[idx_j6]),
         positions[idx_j7],
         (double)servo_joint_angle_deg_from_raw(&g_servo_joint_calib[idx_j7], positions[idx_j7]),
         (int)servo_joint_angle_cdeg_from_raw(&g_servo_joint_calib[idx_j7], positions[idx_j7]));
}

static void serial_servo_log_j2j3_tx_raw(const int16_t* positions, const int* ret_pos,
                                         uint16_t j2_tx_raw_field, uint16_t j3_tx_raw_field)
{
    static uint32_t last_log_ms = 0u;
    static uint32_t sample_total = 0u;
    static uint32_t j2_ok_total = 0u;
    static uint32_t j3_ok_total = 0u;
    const uint8_t idx_j2 = servo_id_to_index(2u);
    const uint8_t idx_j3 = servo_id_to_index(3u);
    const uint32_t now_ms = HAL_GetTick();
    const uint8_t j2_valid = (ret_pos != NULL && ret_pos[idx_j2] == SERIAL_SERVO_H12H_OK) ? 1u : 0u;
    const uint8_t j3_valid = (ret_pos != NULL && ret_pos[idx_j3] == SERIAL_SERVO_H12H_OK) ? 1u : 0u;

    sample_total++;
    if (j2_valid != 0u) {
        j2_ok_total++;
    }
    if (j3_valid != 0u) {
        j3_ok_total++;
    }

    if ((now_ms - last_log_ms) < 1000u) {
        return;
    }
    last_log_ms = now_ms;

    LOGI("[CTRL][SERVO][J2J3] samples=%lu j2=%d/%u valid=%u ok=%lu j3=%d/%u valid=%u ok=%lu",
         (unsigned long)sample_total,
         (positions != NULL) ? positions[idx_j2] : 0,
         (unsigned)j2_tx_raw_field,
         (unsigned)j2_valid,
         (unsigned long)j2_ok_total,
         (positions != NULL) ? positions[idx_j3] : 0,
         (unsigned)j3_tx_raw_field,
         (unsigned)j3_valid,
         (unsigned long)j3_ok_total);

    sample_total = 0u;
    j2_ok_total = 0u;
    j3_ok_total = 0u;
}

static void serial_servo_publish_joint_payload(const int16_t* positions, const int* ret_pos)
{
    omnix_custom_controller_raw_payload_t payload = {
        OMNIX_CUSTOM_CONTROLLER_PROTOCOL_VERSION,
        0u,
        0u,
        0u,
        {0u, 0u, 0u, 0u, 0u, 0u, 0u},
        0u,
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    uint8_t i = 0u;
    const uint32_t now_ms = HAL_GetTick();

    for (i = 0u; i < SERVO_COUNT; ++i) {
        const servo_joint_calib_t* calib = &g_servo_joint_calib[i];
        const uint8_t servo_index = servo_id_to_index(calib->servo_id);
        const uint8_t joint_index = (uint8_t)(calib->joint_id - 1u);

        if ((ret_pos != NULL) && (ret_pos[servo_index] == SERIAL_SERVO_H12H_OK)) {
            payload.valid_mask |= (uint8_t)(1u << joint_index);
            payload.raw_u16[joint_index] =
                (positions != NULL) ? serial_servo_protocol_raw_for_publish(calib, positions[servo_index], 0u) : 0u;
            continue;
        }

        if ((ret_pos != NULL) && (g_servo_has_last[servo_index] != 0u)) {
            const uint32_t hold_age_ms = now_ms - g_servo_last_ok_tick_ms[servo_index];
            if (hold_age_ms <= SERVO_READ_FAIL_HOLD_TIMEOUT_MS) {
                payload.valid_mask |= (uint8_t)(1u << joint_index);
                payload.raw_u16[joint_index] =
                    serial_servo_protocol_raw_for_publish(calib, (int16_t)g_servo_last_raw_u16[servo_index], 1u);
                g_servo_bus_diag.hold_cache_used++;
            } else {
                g_servo_bus_diag.hold_cache_expired++;
            }
        }
    }

    Referee_PublishJointRawCmd(&payload);
    serial_servo_log_j2j3_tx_raw(positions, ret_pos, payload.raw_u16[1], payload.raw_u16[2]);
}

static const char* serial_servo_ret_str(int ret)
{
    switch (ret) {
        case SERIAL_SERVO_H12H_OK:
            return "ok";
        case SERIAL_SERVO_H12H_ERR_ARG:
            return "arg";
        case SERIAL_SERVO_H12H_ERR_LOCK:
            return "lock";
        case SERIAL_SERVO_H12H_ERR_TX:
            return "tx";
        case SERIAL_SERVO_H12H_ERR_RX_TIMEOUT:
            return "rx_timeout";
        case SERIAL_SERVO_H12H_ERR_CHECKSUM:
            return "checksum";
        case SERIAL_SERVO_H12H_ERR_LEN:
            return "len";
        case SERIAL_SERVO_H12H_ERR_ID:
            return "id";
        case SERIAL_SERVO_H12H_ERR_CMD:
            return "cmd";
        default:
            return "unknown";
    }
}

static const char* serial_servo_bus_str(uint8_t servo_id)
{
    if (servo_id >= SERVO_UART6_ID_MIN && servo_id <= SERVO_UART6_ID_MAX) {
        return "uart6";
    }
    if (servo_id >= SERVO_UART9_ID_MIN && servo_id <= SERVO_UART9_ID_MAX) {
        return "uart9";
    }
    return "invalid";
}

static serial_servo_h12h_t* serial_servo_bus_handle(uint8_t servo_id,
                                                     serial_servo_h12h_t* servo_uart6,
                                                     serial_servo_h12h_t* servo_uart9)
{
    if (servo_id >= SERVO_UART6_ID_MIN && servo_id <= SERVO_UART6_ID_MAX) {
        return servo_uart6;
    }
    if (servo_id >= SERVO_UART9_ID_MIN && servo_id <= SERVO_UART9_ID_MAX) {
        return servo_uart9;
    }
    return NULL;
}

static void serial_servo_diag_note_read(uint8_t servo_id, int ret, uint32_t elapsed_ms)
{
    g_servo_bus_diag.last_single_read_id = servo_id;
    g_servo_bus_diag.last_single_read_ms = elapsed_ms;
    if (elapsed_ms >= g_servo_bus_diag.max_single_read_ms) {
        g_servo_bus_diag.max_single_read_ms = elapsed_ms;
        g_servo_bus_diag.max_single_read_id = servo_id;
    }

    if (ret == SERIAL_SERVO_H12H_OK) {
        g_servo_bus_diag.read_ok++;
        return;
    }

    g_servo_bus_diag.read_fail++;
    if (servo_id >= SERVO_UART6_ID_MIN && servo_id <= SERVO_UART6_ID_MAX) {
        g_servo_bus_diag.uart6_read_fail++;
    } else if (servo_id >= SERVO_UART9_ID_MIN && servo_id <= SERVO_UART9_ID_MAX) {
        g_servo_bus_diag.uart9_read_fail++;
    }

    if (ret == SERIAL_SERVO_H12H_ERR_RX_TIMEOUT) {
        g_servo_bus_diag.read_timeout++;
        if (servo_id >= SERVO_UART6_ID_MIN && servo_id <= SERVO_UART6_ID_MAX) {
            g_servo_bus_diag.uart6_read_timeout++;
        } else if (servo_id >= SERVO_UART9_ID_MIN && servo_id <= SERVO_UART9_ID_MAX) {
            g_servo_bus_diag.uart9_read_timeout++;
        }
    }
}

static void serial_servo_diag_note_scan(uint32_t elapsed_ms, uint8_t online_count)
{
    g_servo_bus_diag.scan_calls++;
    g_servo_bus_diag.scan_last_ms = elapsed_ms;
    g_servo_bus_diag.scan_total_ms += elapsed_ms;
    g_servo_bus_diag.scan_last_online_count = online_count;
    if (elapsed_ms >= g_servo_bus_diag.scan_max_ms) {
        g_servo_bus_diag.scan_max_ms = elapsed_ms;
    }
    if (elapsed_ms > SERIAL_SERVO_POLL_MS) {
        g_servo_bus_diag.scan_overrun_count++;
    }
}

static void serial_servo_diag_note_loop(uint32_t elapsed_ms)
{
    g_servo_bus_diag.loop_last_ms = elapsed_ms;
    if (elapsed_ms >= g_servo_bus_diag.loop_max_ms) {
        g_servo_bus_diag.loop_max_ms = elapsed_ms;
    }
    if (elapsed_ms > SERIAL_SERVO_POLL_MS) {
        g_servo_bus_diag.loop_overrun_count++;
    }
}

static void serial_servo_diag_note_restore(uint8_t servo_id, int ret, uint32_t elapsed_ms)
{
    g_servo_bus_diag.restore_runs++;
    g_servo_bus_diag.restore_last_ms = elapsed_ms;
    if (elapsed_ms >= g_servo_bus_diag.restore_max_ms) {
        g_servo_bus_diag.restore_max_ms = elapsed_ms;
    }
    if (elapsed_ms >= g_servo_bus_diag.restore_max_op_ms) {
        g_servo_bus_diag.restore_max_op_ms = elapsed_ms;
        g_servo_bus_diag.restore_max_op_id = servo_id;
    }
    if (ret != SERIAL_SERVO_H12H_OK) {
        g_servo_bus_diag.restore_fail++;
    }
}

static void serial_servo_log_bus_transport(const char* bus_name, UART_HandleTypeDef* huart)
{
    const uint32_t halfduplex =
        (huart != NULL && huart->Instance != NULL && ((huart->Instance->CR3 & USART_CR3_HDSEL) != 0u)) ? 1u : 0u;

    LOGI("[SERVO][BUS][CFG] bus=%s baud=%lu halfduplex=%lu tx_path=dma hdmarx=%u hdmarx_state=%lu hdmatx=%u hdmatx_state=%lu gState=%lu rxState=%lu",
         (bus_name != NULL) ? bus_name : "unknown",
         (huart != NULL) ? (unsigned long)huart->Init.BaudRate : 0ul,
         (unsigned long)halfduplex,
         (huart != NULL && huart->hdmarx != NULL) ? 1u : 0u,
         (huart != NULL && huart->hdmarx != NULL) ? (unsigned long)huart->hdmarx->State : 0xFFFFFFFFul,
         (huart != NULL && huart->hdmatx != NULL) ? 1u : 0u,
         (huart != NULL && huart->hdmatx != NULL) ? (unsigned long)huart->hdmatx->State : 0xFFFFFFFFul,
         (huart != NULL) ? (unsigned long)huart->gState : 0xFFFFFFFFul,
         (huart != NULL) ? (unsigned long)huart->RxState : 0xFFFFFFFFul);
}

static void serial_servo_log_bus_diag_periodic(serial_servo_h12h_t* servo_uart6,
                                               serial_servo_h12h_t* servo_uart9)
{
    static uint32_t last_log_ms = 0u;
    static uint32_t last_hold_cache_used = 0u;
    static uint32_t last_hold_cache_expired = 0u;
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t hold_cache_used_1s = g_servo_bus_diag.hold_cache_used - last_hold_cache_used;
    const uint32_t hold_cache_expired_1s = g_servo_bus_diag.hold_cache_expired - last_hold_cache_expired;

    if ((now_ms - last_log_ms) < SERVO_BUS_DIAG_LOG_PERIOD_MS) {
        return;
    }
    last_log_ms = now_ms;
    last_hold_cache_used = g_servo_bus_diag.hold_cache_used;
    last_hold_cache_expired = g_servo_bus_diag.hold_cache_expired;

    LOGI("[SERVO][BUS][DIAG] scan_calls=%lu last_scan_ms=%lu max_scan_ms=%lu scan_ovr=%lu loop_last_ms=%lu loop_max_ms=%lu loop_ovr=%lu online=%lu/%u read_ok=%lu read_fail=%lu read_to=%lu hold_used=%lu hold_used_1s=%lu hold_exp=%lu hold_exp_1s=%lu hold_to_ms=%u u6_fail=%lu u6_to=%lu u9_fail=%lu u9_to=%lu max_id_ms=%lu max_id=%u last_id_ms=%lu last_id=%u restore_runs=%lu restore_last_ms=%lu restore_max_ms=%lu restore_fail=%lu restore_op_max_ms=%lu restore_op_id=%u u6_rx_dma=%u u6_tx_dma=%u u9_rx_dma=%u u9_tx_dma=%u tx_path=blocking",
         (unsigned long)g_servo_bus_diag.scan_calls,
         (unsigned long)g_servo_bus_diag.scan_last_ms,
         (unsigned long)g_servo_bus_diag.scan_max_ms,
         (unsigned long)g_servo_bus_diag.scan_overrun_count,
         (unsigned long)g_servo_bus_diag.loop_last_ms,
         (unsigned long)g_servo_bus_diag.loop_max_ms,
         (unsigned long)g_servo_bus_diag.loop_overrun_count,
         (unsigned long)g_servo_bus_diag.scan_last_online_count,
         SERVO_COUNT,
         (unsigned long)g_servo_bus_diag.read_ok,
         (unsigned long)g_servo_bus_diag.read_fail,
         (unsigned long)g_servo_bus_diag.read_timeout,
         (unsigned long)g_servo_bus_diag.hold_cache_used,
         (unsigned long)hold_cache_used_1s,
         (unsigned long)g_servo_bus_diag.hold_cache_expired,
         (unsigned long)hold_cache_expired_1s,
         (unsigned)SERVO_READ_FAIL_HOLD_TIMEOUT_MS,
         (unsigned long)g_servo_bus_diag.uart6_read_fail,
         (unsigned long)g_servo_bus_diag.uart6_read_timeout,
         (unsigned long)g_servo_bus_diag.uart9_read_fail,
         (unsigned long)g_servo_bus_diag.uart9_read_timeout,
         (unsigned long)g_servo_bus_diag.max_single_read_ms,
         g_servo_bus_diag.max_single_read_id,
         (unsigned long)g_servo_bus_diag.last_single_read_ms,
         g_servo_bus_diag.last_single_read_id,
         (unsigned long)g_servo_bus_diag.restore_runs,
         (unsigned long)g_servo_bus_diag.restore_last_ms,
         (unsigned long)g_servo_bus_diag.restore_max_ms,
         (unsigned long)g_servo_bus_diag.restore_fail,
         (unsigned long)g_servo_bus_diag.restore_max_op_ms,
         g_servo_bus_diag.restore_max_op_id,
         (servo_uart6 != NULL && servo_uart6->huart != NULL && servo_uart6->huart->hdmarx != NULL) ? 1u : 0u,
         (servo_uart6 != NULL && servo_uart6->huart != NULL && servo_uart6->huart->hdmatx != NULL) ? 1u : 0u,
         (servo_uart9 != NULL && servo_uart9->huart != NULL && servo_uart9->huart->hdmarx != NULL) ? 1u : 0u,
         (servo_uart9 != NULL && servo_uart9->huart != NULL && servo_uart9->huart->hdmatx != NULL) ? 1u : 0u);
}

static uint8_t serial_servo_scan_positions(serial_servo_h12h_t* servo_uart6,
                                           serial_servo_h12h_t* servo_uart9,
                                           int16_t* positions,
                                           int* ret_pos)
{
    uint8_t servo_id = 0u;
    uint8_t online_count = 0u;
    int16_t position = 0;
    const uint32_t scan_start_ms = HAL_GetTick();

    for (servo_id = SERVO_ID_MIN; servo_id <= SERVO_ID_MAX; ++servo_id) {
        uint8_t index = servo_id_to_index(servo_id);
        serial_servo_h12h_t* bus = serial_servo_bus_handle(servo_id, servo_uart6, servo_uart9);
        int ret = SERIAL_SERVO_H12H_ERR_ARG;
        const uint32_t read_start_ms = HAL_GetTick();

        if (bus != NULL) {
            ret = serial_servo_h12h_read_position(bus, servo_id, &position);
        }
        serial_servo_diag_note_read(servo_id, ret, HAL_GetTick() - read_start_ms);

        if (ret_pos != NULL) {
            ret_pos[index] = ret;
        }

        if (ret == SERIAL_SERVO_H12H_OK) {
            if (positions != NULL) {
                positions[index] = position;
            }
            g_servo_last_raw_u16[index] = servo_joint_clamp_raw_no_log(&g_servo_joint_calib[index], position);
            g_servo_last_ok_tick_ms[index] = HAL_GetTick();
            g_servo_has_last[index] = 1u;
            online_count++;
        } else if (positions != NULL) {
            positions[index] = 0;
        }
    }

    serial_servo_diag_note_scan(HAL_GetTick() - scan_start_ms, online_count);

    return online_count;
}

static void serial_servo_beep_short_once(void)
{
    Buzzer_Start(SERVO_SHORT_BEEP_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SHORT_BEEP_MS));
    Buzzer_Stop();
}

static void serial_servo_unload_all(serial_servo_h12h_t* servo_uart6, serial_servo_h12h_t* servo_uart9)
{
    uint8_t servo_id = 0u;

    for (servo_id = SERVO_ID_MIN; servo_id <= SERVO_ID_MAX; ++servo_id) {
        serial_servo_h12h_t* bus = serial_servo_bus_handle(servo_id, servo_uart6, servo_uart9);
        int ret = serial_servo_h12h_load_write(bus, servo_id, 0u);
        if (ret != SERIAL_SERVO_H12H_OK) {
            LOGW("[SERVO][RESTORE][%s] id=%u unload failed ret=%d(%s)",
                 serial_servo_bus_str(servo_id),
                 servo_id,
                 ret,
                 serial_servo_ret_str(ret));
        }
    }
}

static void serial_servo_log_zero_config(void)
{
    uint8_t i = 0u;

    for (i = 0u; i < SERVO_COUNT; ++i) {
        const servo_joint_calib_t* calib = &g_servo_joint_calib[i];
        const omnix_controller_joint_raw_calib_t protocol = servo_joint_protocol_calib(calib);
        const int32_t neg_span_raw = Omnix_ControllerNegativeSpanRaw(&protocol);
        const int32_t pos_span_raw = Omnix_ControllerPositiveSpanRaw(&protocol);
        LOGI("[SERVO][ZERO] joint=%u id=%u raw=%u",
             calib->joint_id,
             calib->servo_id,
             calib->raw_zero);
        LOGI("[SERVO][PROTO] joint=%u id=%u min=%u zero=%u max=%u neg_span=%ld pos_span=%ld neg_deg=%.2f pos_deg=%.2f",
             calib->joint_id,
             calib->servo_id,
             calib->raw_min,
             calib->raw_zero,
             calib->raw_max,
             (long)neg_span_raw,
             (long)pos_span_raw,
             (double)Omnix_ControllerRawToRelDeg(calib->raw_min, &protocol),
             (double)Omnix_ControllerRawToRelDeg(calib->raw_max, &protocol));
        if ((uint32_t)neg_span_raw <= OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW) {
            LOGW("[SERVO][PROTO][WARN] joint=%u id=%u negative span is degenerate span_raw=%ld threshold=%u",
                 calib->joint_id,
                 calib->servo_id,
                 (long)neg_span_raw,
                 (unsigned)OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW);
        }
        if ((uint32_t)pos_span_raw <= OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW) {
            LOGW("[SERVO][PROTO][WARN] joint=%u id=%u positive span is degenerate span_raw=%ld threshold=%u",
                 calib->joint_id,
                 calib->servo_id,
                 (long)pos_span_raw,
                 (unsigned)OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW);
        }
    }
}

static int serial_servo_restore_one(serial_servo_h12h_t* bus, uint8_t servo_id, uint16_t target_pos)
{
    uint8_t attempt = 0u;
    int last_ret = SERIAL_SERVO_H12H_ERR_ARG;
    const uint32_t restore_start_ms = HAL_GetTick();

    if (bus == NULL) {
        serial_servo_diag_note_restore(servo_id, SERIAL_SERVO_H12H_ERR_ARG, 0u);
        return SERIAL_SERVO_H12H_ERR_ARG;
    }

    for (attempt = 0u; attempt < SERVO_RESTORE_RETRY_COUNT; ++attempt) {
        uint32_t op_start_ms = HAL_GetTick();
        int ret = serial_servo_h12h_set_mode_position(bus, servo_id);
        uint32_t op_elapsed_ms = HAL_GetTick() - op_start_ms;
        if (ret != SERIAL_SERVO_H12H_OK) {
            last_ret = ret;
            LOGW("[SERVO][RESTORE][%s] id=%u op=set_mode elapsed=%lu ret=%d(%s) try=%u/%u",
                 serial_servo_bus_str(servo_id),
                 servo_id,
                 (unsigned long)op_elapsed_ms,
                 ret,
                 serial_servo_ret_str(ret),
                 (uint8_t)(attempt + 1u),
                 SERVO_RESTORE_RETRY_COUNT);
            goto retry_next;
        }

        op_start_ms = HAL_GetTick();
        ret = serial_servo_h12h_load_write(bus, servo_id, 1u);
        op_elapsed_ms = HAL_GetTick() - op_start_ms;
        if (ret != SERIAL_SERVO_H12H_OK) {
            last_ret = ret;
            LOGW("[SERVO][RESTORE][%s] id=%u op=load_on elapsed=%lu ret=%d(%s) try=%u/%u",
                 serial_servo_bus_str(servo_id),
                 servo_id,
                 (unsigned long)op_elapsed_ms,
                 ret,
                 serial_servo_ret_str(ret),
                 (uint8_t)(attempt + 1u),
                 SERVO_RESTORE_RETRY_COUNT);
            goto retry_next;
        }

        op_start_ms = HAL_GetTick();
        ret = serial_servo_h12h_set_position(bus, servo_id, target_pos, SERVO_RESTORE_MOVE_TIME_MS);
        op_elapsed_ms = HAL_GetTick() - op_start_ms;
        if (ret == SERIAL_SERVO_H12H_OK) {
            LOGI("[SERVO][RESTORE][%s] id=%u op=set_pos elapsed=%lu target=%u move=%u ok try=%u/%u",
                 serial_servo_bus_str(servo_id),
                 servo_id,
                 (unsigned long)op_elapsed_ms,
                 target_pos,
                 (uint16_t)SERVO_RESTORE_MOVE_TIME_MS,
                 (uint8_t)(attempt + 1u),
                 SERVO_RESTORE_RETRY_COUNT);
            serial_servo_diag_note_restore(servo_id, SERIAL_SERVO_H12H_OK, HAL_GetTick() - restore_start_ms);
            return SERIAL_SERVO_H12H_OK;
        }

        last_ret = ret;
        LOGW("[SERVO][RESTORE][%s] id=%u op=set_pos elapsed=%lu ret=%d(%s) try=%u/%u",
             serial_servo_bus_str(servo_id),
             servo_id,
             (unsigned long)op_elapsed_ms,
             ret,
             serial_servo_ret_str(ret),
             (uint8_t)(attempt + 1u),
             SERVO_RESTORE_RETRY_COUNT);

retry_next:
        if ((uint8_t)(attempt + 1u) < SERVO_RESTORE_RETRY_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(SERVO_RESTORE_RETRY_DELAY_MS));
        }
    }

    serial_servo_diag_note_restore(servo_id, last_ret, HAL_GetTick() - restore_start_ms);
    return last_ret;
}

static uint8_t serial_servo_restore_defaults(serial_servo_h12h_t* servo_uart6,
                                             serial_servo_h12h_t* servo_uart9,
                                             const char* stage,
                                             uint8_t log_mapping)
{
    uint8_t servo_id = 0u;
    uint8_t ok_count = 0u;

    LOGI("[SERVO][RESTORE][%s] start", (stage != NULL) ? stage : "generic");
    if (log_mapping != 0u) {
        serial_servo_log_mapping_targets(stage);
    }

    for (servo_id = SERVO_ID_MIN; servo_id <= SERVO_ID_MAX; ++servo_id) {
        uint8_t idx = servo_id_to_index(servo_id);
        serial_servo_h12h_t* bus = serial_servo_bus_handle(servo_id, servo_uart6, servo_uart9);
        int ret = serial_servo_restore_one(bus, servo_id, g_servo_default_pos[idx]);
        if (ret == SERIAL_SERVO_H12H_OK) {
            ok_count++;
        } else {
            LOGW("[SERVO][RESTORE][%s] id=%u target=%u failed ret=%d(%s)",
                 serial_servo_bus_str(servo_id),
                 servo_id,
                 g_servo_default_pos[idx],
                 ret,
                 serial_servo_ret_str(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_RESTORE_INTER_ID_DELAY_MS));
    }

    vTaskDelay(pdMS_TO_TICKS(SERVO_RESTORE_MOVE_TIME_MS));
    LOGI("[SERVO][RESTORE][%s] done ok=%u/%u", (stage != NULL) ? stage : "generic", ok_count, SERVO_COUNT);
    return ok_count;
}

static uint8_t serial_servo_run_restore_flow(serial_servo_h12h_t* servo_uart6,
                                             serial_servo_h12h_t* servo_uart9,
                                             const char* stage,
                                             uint8_t hold_and_unload,
                                             uint8_t beep_after,
                                             uint8_t log_mapping)
{
    uint8_t restore_ok_count = 0u;
    const uint32_t restore_flow_start_ms = HAL_GetTick();

    restore_ok_count = serial_servo_restore_defaults(servo_uart6, servo_uart9, stage, log_mapping);

    if (beep_after != 0u) {
        serial_servo_beep_short_once();
    }

    if (hold_and_unload != 0u) {
        LOGI("[SERVO][RESTORE][%s] hold load for %u ms",
             (stage != NULL) ? stage : "generic",
             (uint16_t)SERVO_POST_RESTORE_HOLD_MS);
        vTaskDelay(pdMS_TO_TICKS(SERVO_POST_RESTORE_HOLD_MS));
        serial_servo_unload_all(servo_uart6, servo_uart9);
        if (beep_after != 0u) {
            serial_servo_beep_short_once();
        }
    }

    g_servo_bus_diag.restore_last_ms = HAL_GetTick() - restore_flow_start_ms;
    if (g_servo_bus_diag.restore_last_ms >= g_servo_bus_diag.restore_max_ms) {
        g_servo_bus_diag.restore_max_ms = g_servo_bus_diag.restore_last_ms;
    }

    return restore_ok_count;
}

static void serial_servo_wait_all_online(serial_servo_h12h_t* servo_uart6, serial_servo_h12h_t* servo_uart9)
{
    uint8_t online_count = 0u;
    uint8_t prev_online_count = 0xFFu;
    uint8_t beep_started = 0u;
    uint8_t beep_on = 0u;
    TickType_t beep_start_tick = 0u;
    TickType_t beep_last_toggle_tick = 0u;

    g_servo_boot_state = SERVO_BOOT_WAIT_ONLINE;
    LOGI("[SERVO][BOOT] waiting all online...");

    for (;;) {
        TickType_t now_tick = xTaskGetTickCount();
        online_count = serial_servo_scan_positions(servo_uart6, servo_uart9, NULL, NULL);
        g_servo_online_count = online_count;

        if (online_count != prev_online_count) {
            prev_online_count = online_count;
            LOGI("[SERVO][BOOT] online=%u/%u", online_count, SERVO_COUNT);
        }

        if (online_count >= SERVO_COUNT) {
            Buzzer_Stop();
            LOGI("[SERVO][BOOT] all servo online");
            return;
        }

        if (beep_started == 0u) {
            beep_started = 1u;
            beep_on = 1u;
            beep_start_tick = now_tick;
            beep_last_toggle_tick = now_tick;
            Buzzer_Start(SERVO_OFFLINE_FAST_BEEP_FREQ_HZ);
            LOGW("[SERVO][BOOT] offline detected, fast beep for %u ms", (uint16_t)SERVO_OFFLINE_FAST_BEEP_MS);
        } else if ((now_tick - beep_start_tick) < pdMS_TO_TICKS(SERVO_OFFLINE_FAST_BEEP_MS)) {
            uint32_t phase_ms = (beep_on != 0u) ? SERVO_OFFLINE_FAST_BEEP_ON_MS : SERVO_OFFLINE_FAST_BEEP_OFF_MS;
            if ((now_tick - beep_last_toggle_tick) >= pdMS_TO_TICKS(phase_ms)) {
                beep_on = (uint8_t)(1u - beep_on);
                beep_last_toggle_tick = now_tick;
                if (beep_on != 0u) {
                    Buzzer_Start(SERVO_OFFLINE_FAST_BEEP_FREQ_HZ);
                } else {
                    Buzzer_Stop();
                }
            }
        } else if (beep_on != 0u) {
            beep_on = 0u;
            Buzzer_Stop();
        }

        vTaskDelay(pdMS_TO_TICKS(SERVO_BOOT_WAIT_POLL_MS));
    }
}

static void serial_servo_boot_restore_flow(serial_servo_h12h_t* servo_uart6, serial_servo_h12h_t* servo_uart9)
{
    uint8_t restore_ok_count = 0u;

    serial_servo_wait_all_online(servo_uart6, servo_uart9);
    serial_servo_log_zero_config();

    g_servo_boot_state = SERVO_BOOT_RESTORING;
    LOGI("[SERVO][BOOT] start restore");
    restore_ok_count = serial_servo_run_restore_flow(servo_uart6, servo_uart9, "boot", 1u, 1u, 0u);

    g_servo_boot_state = SERVO_BOOT_DONE;
    LOGI("[SERVO][BOOT] finished restore ok=%u/%u unloaded", restore_ok_count, SERVO_COUNT);
}

void Start_SerialServo_Task(void *argument)
{
    serial_servo_h12h_t servo_uart6;
    serial_servo_h12h_t servo_uart9;
    uint8_t prev_mapping_reset_request = 0u;
    uint32_t last_mapping_log_ms = 0u;
    TickType_t last_wake_tick = 0u;
    const TickType_t poll_ticks = (pdMS_TO_TICKS(SERIAL_SERVO_POLL_MS) > 0u) ? pdMS_TO_TICKS(SERIAL_SERVO_POLL_MS) : 1u;

    (void)argument;

    serial_servo_h12h_init(&servo_uart6, &huart6, 25u);
    serial_servo_h12h_init(&servo_uart9, &huart9, 25u);
    serial_servo_log_bus_transport("uart6", &huart6);
    serial_servo_log_bus_transport("uart9", &huart9);
    vTaskDelay(pdMS_TO_TICKS(300));

    serial_servo_boot_restore_flow(&servo_uart6, &servo_uart9);
    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        int16_t positions[SERVO_COUNT] = {0};
        int ret_pos[SERVO_COUNT] = {0};
        uint8_t online_count = 0u;
        uint8_t servo_id = 0u;
        const TickType_t loop_start_tick = xTaskGetTickCount();
        const uint32_t loop_start_ms = HAL_GetTick();
        const uint8_t mapping_reset_request = Referee_IsMappingResetRequested() ? 1u : 0u;

        if ((mapping_reset_request != 0u) && (prev_mapping_reset_request == 0u)) {
            uint8_t restore_ok_count = 0u;
            g_servo_mapping_reset_active = 1u;
            Referee_SetMappingResetDone(false);
            LOGI("[MAP][RESET][SERVO] start");
            restore_ok_count = serial_servo_run_restore_flow(&servo_uart6, &servo_uart9, "map", 1u, 0u, 1u);
            online_count = serial_servo_scan_positions(&servo_uart6, &servo_uart9, positions, ret_pos);
            g_servo_online_count = online_count;
            serial_servo_log_mapping_feedback(positions, ret_pos);
            serial_servo_publish_joint_payload(positions, ret_pos);
            Referee_SetMappingResetDone(true);
            serial_servo_publish_joint_payload(positions, ret_pos);
            serial_servo_log_session_status(positions, ret_pos, 1u, 0u);
            g_servo_mapping_reset_active = 0u;
            LOGI("[MAP][RESET][SERVO] done ok=%u/%u", restore_ok_count, SERVO_COUNT);
        } else if ((mapping_reset_request == 0u) && (prev_mapping_reset_request != 0u)) {
            Referee_SetMappingResetDone(false);
            g_servo_mapping_reset_active = 0u;
            LOGI("[MAP][RESET][SERVO] clear");
        }
        prev_mapping_reset_request = mapping_reset_request;

        online_count = serial_servo_scan_positions(&servo_uart6, &servo_uart9, positions, ret_pos);
        g_servo_online_count = online_count;
        serial_servo_publish_joint_payload(positions, ret_pos);

        LOGI("[SERVO] pos: 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d",
             positions[0],
             positions[1],
             positions[2],
             positions[3],
             positions[4],
             positions[5],
             positions[6]);

        for (servo_id = SERVO_ID_MIN; servo_id <= SERVO_ID_MAX; ++servo_id) {
            uint8_t index = servo_id_to_index(servo_id);
            if (ret_pos[index] != SERIAL_SERVO_H12H_OK) {
                LOGW("[SERVO][%s] id=%u read pos failed ret=%d(%s)",
                     serial_servo_bus_str(servo_id),
                     servo_id,
                     ret_pos[index],
                     serial_servo_ret_str(ret_pos[index]));
            }
        }

        if ((mapping_reset_request != 0u || g_servo_mapping_reset_active != 0u) &&
            ((HAL_GetTick() - last_mapping_log_ms) >= 500u)) {
            last_mapping_log_ms = HAL_GetTick();
            serial_servo_log_mapping_feedback(positions, ret_pos);
        }

        serial_servo_log_bus_diag_periodic(&servo_uart6, &servo_uart9);

        serial_servo_diag_note_loop(HAL_GetTick() - loop_start_ms);
        if ((xTaskGetTickCount() - loop_start_tick) > poll_ticks) {
            last_wake_tick = xTaskGetTickCount();
            continue;
        }
        vTaskDelayUntil(&last_wake_tick, poll_ticks);
    }
}

void Start_SerialServo_Diag_Task(void *argument)
{
    serial_servo_h12h_t servo_uart6;
    serial_servo_h12h_t servo_uart9;
    int16_t position = 0;
    uint8_t servo_id = 0;
    uint8_t scan_max_id = (SERVO_DIAG_SCAN_MAX_ID < SERVO_ID_MIN) ? SERVO_ID_MIN : SERVO_DIAG_SCAN_MAX_ID;
#if (SERVO_DIAG_MOVE_TEST_ENABLE == 1u)
    uint8_t move_toggle = 0u;
#endif

    (void)argument;

    if (scan_max_id > SERVO_ID_MAX) {
        scan_max_id = SERVO_ID_MAX;
    }

    serial_servo_h12h_init(&servo_uart6, &huart6, 50u);
    serial_servo_h12h_init(&servo_uart9, &huart9, 50u);
    serial_servo_log_bus_transport("uart6", &huart6);
    serial_servo_log_bus_transport("uart9", &huart9);
    vTaskDelay(pdMS_TO_TICKS(300));
    g_servo_boot_state = SERVO_BOOT_DONE;
    LOGI("[SERVO_DIAG] start, scan id=%u..%u", SERVO_ID_MIN, scan_max_id);

    for (;;) {
        int16_t positions[SERVO_COUNT] = {0};
        int ret_pos[SERVO_COUNT] = {0};
        uint8_t online_count = 0u;
        uint8_t first_online_id = 0u;
        uint8_t idx = 0u;
        serial_servo_h12h_t* first_online_bus = NULL;

        for (idx = 0u; idx < SERVO_COUNT; ++idx) {
            ret_pos[idx] = SERIAL_SERVO_H12H_ERR_ARG;
        }

        for (servo_id = SERVO_ID_MIN; servo_id <= scan_max_id; ++servo_id) {
            uint8_t index = servo_id_to_index(servo_id);
            serial_servo_h12h_t* bus = serial_servo_bus_handle(servo_id, &servo_uart6, &servo_uart9);
            if (bus == NULL) {
                ret_pos[index] = SERIAL_SERVO_H12H_ERR_ARG;
                continue;
            }
            ret_pos[index] = serial_servo_h12h_read_position(bus, servo_id, &position);
            if (ret_pos[index] == SERIAL_SERVO_H12H_OK) {
                positions[index] = position;
                online_count++;
                if (first_online_id == 0u) {
                    first_online_id = servo_id;
                    first_online_bus = bus;
                }
            }
        }

        g_servo_online_count = online_count;
        LOGI("[SERVO_DIAG] online=%u/%u pos: 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d",
             online_count,
             (uint8_t)(scan_max_id - SERVO_ID_MIN + 1u),
             positions[0],
             positions[1],
             positions[2],
             positions[3],
             positions[4],
             positions[5],
             positions[6]);

        for (servo_id = SERVO_ID_MIN; servo_id <= scan_max_id; ++servo_id) {
            uint8_t index = servo_id_to_index(servo_id);
            if (ret_pos[index] != SERIAL_SERVO_H12H_OK) {
                LOGW("[SERVO_DIAG][%s] id=%u read pos failed ret=%d(%s)",
                     serial_servo_bus_str(servo_id),
                     servo_id,
                     ret_pos[index],
                     serial_servo_ret_str(ret_pos[index]));
            }
        }

        if (first_online_id != 0u && first_online_bus != NULL) {
            uint16_t vin_mv = 0u;
            uint8_t temp_c = 0u;
            uint8_t load = 0u;
            int ret_vin = serial_servo_h12h_read_vin(first_online_bus, first_online_id, &vin_mv);
            int ret_temp = serial_servo_h12h_read_temp(first_online_bus, first_online_id, &temp_c);
            int ret_load = serial_servo_h12h_read_load(first_online_bus, first_online_id, &load);
            LOGI("[SERVO_DIAG][%s] id=%u vin=%u(ret=%d,%s) temp=%u(ret=%d,%s) load=%u(ret=%d,%s)",
                 serial_servo_bus_str(first_online_id),
                 first_online_id,
                 vin_mv,
                 ret_vin,
                 serial_servo_ret_str(ret_vin),
                 temp_c,
                 ret_temp,
                 serial_servo_ret_str(ret_temp),
                 load,
                 ret_load,
                 serial_servo_ret_str(ret_load));

#if (SERVO_DIAG_MOVE_TEST_ENABLE == 1u)
            if (SERVO_DIAG_TARGET_ID >= SERVO_ID_MIN && SERVO_DIAG_TARGET_ID <= SERVO_ID_MAX) {
                if (first_online_id == SERVO_DIAG_TARGET_ID) {
                    uint16_t target_pos = move_toggle ? 550u : 450u;
                    int ret_move = serial_servo_h12h_set_position(first_online_bus, first_online_id, target_pos, 300u);
                    LOGI("[SERVO_DIAG][%s] move id=%u target=%u ret=%d(%s)",
                         serial_servo_bus_str(first_online_id),
                         first_online_id,
                         target_pos,
                         ret_move,
                         serial_servo_ret_str(ret_move));
                    if (ret_move == SERIAL_SERVO_H12H_OK) {
                        move_toggle = (uint8_t)(1u - move_toggle);
                    }
                }
            }
#endif
        } else {
            LOGW("[SERVO_DIAG] no online servo in id=%u..%u", SERVO_ID_MIN, scan_max_id);
        }

        serial_servo_log_bus_diag_periodic(&servo_uart6, &servo_uart9);

        vTaskDelay(pdMS_TO_TICKS(SERIAL_SERVO_POLL_MS));
    }
}
