#include "../Inc/Referee_Task.h"

#include "SerialServo_Task.h"
#include "FreeRTOS.h"
#include "bsp_buzzer.h"
#include "bsp_srn_log.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include "usart.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" {
volatile referee_uart8_tx_debug_watch_t g_referee_uart8_tx_debug = {};
}

#ifdef OMNIX_FEATURE_MPU_DCACHE
#if __has_include("mem_sections.h")
#include "mem_sections.h"
#define REFEREE_RX_DMA_SEC SEC_DMA_NC_BUF
#define REFEREE_TX_DMA_SEC SEC_DMA_NC_BUF
#else
#define REFEREE_RX_DMA_SEC __attribute__((aligned(32)))
#define REFEREE_TX_DMA_SEC __attribute__((aligned(32)))
#endif
#else
#define REFEREE_RX_DMA_SEC __attribute__((aligned(32)))
#define REFEREE_TX_DMA_SEC __attribute__((aligned(32)))
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
constexpr uint16_t REFEREE_FRAME_LEN = REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_CUSTOM_DATA_LEN + REFEREE_TAIL_LEN;
constexpr uint32_t REFEREE_CTRL_0302_TX_INTERVAL_MS = 40u;
constexpr uint32_t REFEREE_CTRL_0302_MAX_HZ = 25u;
constexpr uint32_t REFEREE_TASK_LOOP_PERIOD_MS = 1u;
constexpr const char* REFEREE_TX_STAGE = "uart8_server";
constexpr const char* REFEREE_RX_STAGE = "uart8";
constexpr uint16_t REFEREE_RX_DMA_BUF_SIZE = 128u;
constexpr uint16_t REFEREE_RX_FIFO_SIZE = 1024u;
constexpr uint16_t REFEREE_RX_PARSE_BUF_SIZE = 512u;
constexpr uint32_t REFEREE_RX_BEEP_FREQ_HZ = 1800u;
constexpr uint32_t REFEREE_RX_BEEP_MS = 120u;
constexpr uint8_t REFEREE_LATENCY_PROBE_ENABLE_MASK = 0x01u;
constexpr size_t REFEREE_LATENCY_WINDOW_SIZE = 128u;
constexpr uintptr_t REFEREE_DMA_RAM_D2_START = 0x30000000u;
constexpr uintptr_t REFEREE_DMA_RAM_D2_END_EXCLUSIVE = 0x30008000u;

#ifndef REFEREE_TX_FORCE_FIXED_FRAME
#define REFEREE_TX_FORCE_FIXED_FRAME 0
#endif

#ifndef REFEREE_TX_REQUIRE_PUBLISH_TICK
#define REFEREE_TX_REQUIRE_PUBLISH_TICK 0
#endif

static_assert((REFEREE_RX_FIFO_SIZE & (REFEREE_RX_FIFO_SIZE - 1u)) == 0u, "referee rx fifo must be power of two");
static_assert((REFEREE_CTRL_0302_TX_INTERVAL_MS * REFEREE_CTRL_0302_MAX_HZ) >= 1000u,
              "0x0302 tx rate must not exceed configured max rate");

static_assert(OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE <= REFEREE_CUSTOM_DATA_LEN, "joint raw payload exceeds 0x0302 data space");
static_assert(OMNIX_ROBOT_JOINT_FEEDBACK_PAYLOAD_SIZE == REFEREE_CUSTOM_DATA_LEN, "0x0309 payload must be 30 bytes");
static_assert(REFEREE_FRAME_LEN == 39u, "fixed referee tx frame expects 39-byte 0x0302 frame");

constexpr uint8_t REFEREE_FIXED_TX_FRAME[REFEREE_FRAME_LEN] = {
    0xA5u, 0x1Eu, 0x00u, 0x06u, 0xA0u, 0x02u, 0x03u,
    0x05u, 0x06u, 0x07u, 0x08u, 0x09u, 0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, 0x0Fu,
    0x10u, 0x11u, 0x12u, 0x13u, 0x14u, 0x15u, 0x16u, 0x17u, 0x18u, 0x19u, 0x1Au,
    0x1Bu, 0x1Cu, 0x1Du, 0x1Eu, 0x1Fu, 0x20u, 0x21u, 0x22u, 0xA6u, 0xFDu
};

volatile uint32_t g_joint_tx_guard = 0u;
omnix_custom_controller_raw_payload_t g_joint_tx_payload = {
    OMNIX_CUSTOM_CONTROLLER_PROTOCOL_VERSION,
    0u,
    0u,
    0u,
    {0u, 0u, 0u, 0u, 0u, 0u, 0u},
    0u,
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
volatile uint32_t g_joint_tx_tick_ms = 0u;
volatile uint32_t g_joint_feedback_rx_guard = 0u;
omnix_robot_joint_feedback_payload_t g_joint_feedback_rx_payload = {
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
volatile uint32_t g_joint_feedback_rx_tick_ms = 0u;
uint8_t g_referee_frame_seq = 0u;
REFEREE_TX_DMA_SEC uint8_t g_referee_tx_frame[REFEREE_FRAME_LEN];
uint8_t g_referee_uart8_rx_ready = 0u;
uint8_t g_referee_rx_beep_active = 0u;
uint32_t g_referee_rx_beep_stop_ms = 0u;
uint8_t g_referee_rx_remote_buzzer_prev_flag = 0u;
uint32_t g_referee_rx_frame_count = 0u;
volatile uint32_t g_referee_rx_fifo_overflow_count = 0u;
volatile uint16_t g_referee_rx_fifo_head = 0u;
volatile uint16_t g_referee_rx_fifo_tail = 0u;
uint16_t g_referee_rx_parse_len = 0u;
REFEREE_RX_DMA_SEC uint8_t g_referee_rx_dma_buf[REFEREE_RX_DMA_BUF_SIZE];
uint8_t g_referee_rx_fifo[REFEREE_RX_FIFO_SIZE] = {0};
uint8_t g_referee_rx_parse_buf[REFEREE_RX_PARSE_BUF_SIZE] = {0};
volatile bool g_mapping_reset_requested = false;
volatile bool g_mapping_reset_done = false;
volatile uint32_t g_referee_uart8_diag_guard = 0u;

struct referee_uart8_diag_t {
    uint32_t rx_event_count;
    uint32_t rx_event_bytes;
    uint32_t rx_sof_hits;
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
    uint32_t fifo_overflow_count;
    uint32_t parse_total_frames;
    uint32_t parse_good_frames;
    uint32_t parse_cmd_0309_frames;
    uint32_t parse_cmd_0309_good_frames;
    uint32_t parse_cmd_0309_crc_fail_frames;
    uint32_t crc8_errors;
    uint32_t crc16_errors;
    uint32_t payload_len_errors;
    uint32_t payload_version_errors;
    uint32_t sof_drop_bytes;
    uint32_t rx_seq_gap_count;
    uint32_t tx_dma_ok;
    uint32_t tx_dma_busy;
    uint32_t tx_dma_err;
    uint32_t tx_not_ready_count;
    uint32_t tx_due_miss;
    uint32_t tx_skip_busy;
    uint32_t tx_skip_not_ready;
    uint32_t tx_loop_overrun;
    uint32_t tx_period_last_ms;
    uint32_t tx_period_max_ms;
    uint8_t last_rx_seq;
    uint8_t last_rx_seq_valid;
};

referee_uart8_diag_t g_referee_uart8_diag = {};

void referee_store_joint_feedback(const omnix_robot_joint_feedback_payload_t* payload, uint32_t now_ms);

struct referee_loop_ctx_t {
    uint32_t now_ms;
    referee_uart8_diag_t diag;
    omnix_custom_controller_raw_payload_t payload;
    omnix_robot_joint_feedback_payload_t feedback;
    uint32_t payload_tick_ms;
    uint32_t feedback_tick_ms;
    bool has_diag;
    bool has_payload;
    bool has_feedback;
};

struct referee_rx_log_state_t {
    uint32_t task_start_ms;
    uint32_t last_diag_log_ms;
    uint32_t last_rx_overflow_log_ms;
    uint32_t last_rx_overflow_count;
    uint16_t last_feedback_seq;
    uint32_t last_feedback_log_ms;
    uint32_t last_map_rx_log_ms;
    uint32_t last_latency_log_ms;
    referee_uart8_diag_t last_diag;
};

struct referee_tx_state_t {
    uint16_t next_tx_seq;
    uint32_t last_log_ms;
    uint32_t last_send_ms;
    uint32_t last_sent_publish_tick_ms;
    uint32_t last_wait_payload_log_ms;
    uint32_t last_uart_not_ready_log_ms;
    uint32_t last_map_tx_log_ms;
    uint32_t next_tx_due_ms;
};

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
    referee_latency_hist_t rtt_ms;
    referee_latency_hist_t one_way_ms;
    uint16_t last_board_rx_to_tx_ms;
    uint32_t bad_echo_samples;
};

struct referee_latency_snapshot_t {
    uint8_t has_sample;
    uint32_t total_samples;
    uint32_t bad_echo_samples;
    uint16_t board_rx_to_tx_last_ms;
    uint16_t rtt_last_ms;
    uint16_t rtt_min_ms;
    uint16_t rtt_p50_ms;
    uint16_t rtt_p95_ms;
    uint16_t rtt_max_ms;
    uint16_t one_way_last_ms;
    uint16_t one_way_min_ms;
    uint16_t one_way_p50_ms;
    uint16_t one_way_p95_ms;
    uint16_t one_way_max_ms;
};

referee_latency_state_t g_referee_latency = {};

template <typename Fn>
void referee_uart8_diag_mutate(Fn&& fn)
{
    __DMB();
    g_referee_uart8_diag_guard++;
    __DMB();
    fn(g_referee_uart8_diag);
    __DMB();
    g_referee_uart8_diag_guard++;
    __DMB();
}

bool referee_uart8_diag_copy(referee_uart8_diag_t* out)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    referee_uart8_diag_t local{};

    do {
        seq_a = g_referee_uart8_diag_guard;
        __DMB();
        local = g_referee_uart8_diag;
        __DMB();
        seq_b = g_referee_uart8_diag_guard;
    } while (seq_a != seq_b);

    *out = local;
    return true;
}

uint8_t referee_crc8(const uint8_t* data, uint16_t len)
{
    static constexpr uint8_t REFEREE_CRC8_TAB[256] = {
        0x00u, 0x5Eu, 0xBCu, 0xE2u, 0x61u, 0x3Fu, 0xDDu, 0x83u, 0xC2u, 0x9Cu, 0x7Eu, 0x20u, 0xA3u, 0xFDu, 0x1Fu, 0x41u,
        0x9Du, 0xC3u, 0x21u, 0x7Fu, 0xFCu, 0xA2u, 0x40u, 0x1Eu, 0x5Fu, 0x01u, 0xE3u, 0xBDu, 0x3Eu, 0x60u, 0x82u, 0xDCu,
        0x23u, 0x7Du, 0x9Fu, 0xC1u, 0x42u, 0x1Cu, 0xFEu, 0xA0u, 0xE1u, 0xBFu, 0x5Du, 0x03u, 0x80u, 0xDEu, 0x3Cu, 0x62u,
        0xBEu, 0xE0u, 0x02u, 0x5Cu, 0xDFu, 0x81u, 0x63u, 0x3Du, 0x7Cu, 0x22u, 0xC0u, 0x9Eu, 0x1Du, 0x43u, 0xA1u, 0xFFu,
        0x46u, 0x18u, 0xFAu, 0xA4u, 0x27u, 0x79u, 0x9Bu, 0xC5u, 0x84u, 0xDAu, 0x38u, 0x66u, 0xE5u, 0xBBu, 0x59u, 0x07u,
        0xDBu, 0x85u, 0x67u, 0x39u, 0xBAu, 0xE4u, 0x06u, 0x58u, 0x19u, 0x47u, 0xA5u, 0xFBu, 0x78u, 0x26u, 0xC4u, 0x9Au,
        0x65u, 0x3Bu, 0xD9u, 0x87u, 0x04u, 0x5Au, 0xB8u, 0xE6u, 0xA7u, 0xF9u, 0x1Bu, 0x45u, 0xC6u, 0x98u, 0x7Au, 0x24u,
        0xF8u, 0xA6u, 0x44u, 0x1Au, 0x99u, 0xC7u, 0x25u, 0x7Bu, 0x3Au, 0x64u, 0x86u, 0xD8u, 0x5Bu, 0x05u, 0xE7u, 0xB9u,
        0x8Cu, 0xD2u, 0x30u, 0x6Eu, 0xEDu, 0xB3u, 0x51u, 0x0Fu, 0x4Eu, 0x10u, 0xF2u, 0xACu, 0x2Fu, 0x71u, 0x93u, 0xCDu,
        0x11u, 0x4Fu, 0xADu, 0xF3u, 0x70u, 0x2Eu, 0xCCu, 0x92u, 0xD3u, 0x8Du, 0x6Fu, 0x31u, 0xB2u, 0xECu, 0x0Eu, 0x50u,
        0xAFu, 0xF1u, 0x13u, 0x4Du, 0xCEu, 0x90u, 0x72u, 0x2Cu, 0x6Du, 0x33u, 0xD1u, 0x8Fu, 0x0Cu, 0x52u, 0xB0u, 0xEEu,
        0x32u, 0x6Cu, 0x8Eu, 0xD0u, 0x53u, 0x0Du, 0xEFu, 0xB1u, 0xF0u, 0xAEu, 0x4Cu, 0x12u, 0x91u, 0xCFu, 0x2Du, 0x73u,
        0xCAu, 0x94u, 0x76u, 0x28u, 0xABu, 0xF5u, 0x17u, 0x49u, 0x08u, 0x56u, 0xB4u, 0xEAu, 0x69u, 0x37u, 0xD5u, 0x8Bu,
        0x57u, 0x09u, 0xEBu, 0xB5u, 0x36u, 0x68u, 0x8Au, 0xD4u, 0x95u, 0xCBu, 0x29u, 0x77u, 0xF4u, 0xAAu, 0x48u, 0x16u,
        0xE9u, 0xB7u, 0x55u, 0x0Bu, 0x88u, 0xD6u, 0x34u, 0x6Au, 0x2Bu, 0x75u, 0x97u, 0xC9u, 0x4Au, 0x14u, 0xF6u, 0xA8u,
        0x74u, 0x2Au, 0xC8u, 0x96u, 0x15u, 0x4Bu, 0xA9u, 0xF7u, 0xB6u, 0xE8u, 0x0Au, 0x54u, 0xD7u, 0x89u, 0x6Bu, 0x35u
    };
    uint8_t crc = 0xFFu;

    if (data == nullptr) {
        return crc;
    }

    while (len-- > 0u) {
        crc = REFEREE_CRC8_TAB[crc ^ (*data++)];
    }

    return crc;
}

uint16_t referee_crc16(const uint8_t* data, uint16_t len)
{
    static constexpr uint16_t REFEREE_CRC16_TAB[256] = {
        0x0000u, 0x1189u, 0x2312u, 0x329Bu, 0x4624u, 0x57ADu, 0x6536u, 0x74BFu,
        0x8C48u, 0x9DC1u, 0xAF5Au, 0xBED3u, 0xCA6Cu, 0xDBE5u, 0xE97Eu, 0xF8F7u,
        0x1081u, 0x0108u, 0x3393u, 0x221Au, 0x56A5u, 0x472Cu, 0x75B7u, 0x643Eu,
        0x9CC9u, 0x8D40u, 0xBFDBu, 0xAE52u, 0xDAEDu, 0xCB64u, 0xF9FFu, 0xE876u,
        0x2102u, 0x308Bu, 0x0210u, 0x1399u, 0x6726u, 0x76AFu, 0x4434u, 0x55BDu,
        0xAD4Au, 0xBCC3u, 0x8E58u, 0x9FD1u, 0xEB6Eu, 0xFAE7u, 0xC87Cu, 0xD9F5u,
        0x3183u, 0x200Au, 0x1291u, 0x0318u, 0x77A7u, 0x662Eu, 0x54B5u, 0x453Cu,
        0xBDCBu, 0xAC42u, 0x9ED9u, 0x8F50u, 0xFBEFu, 0xEA66u, 0xD8FDu, 0xC974u,
        0x4204u, 0x538Du, 0x6116u, 0x709Fu, 0x0420u, 0x15A9u, 0x2732u, 0x36BBu,
        0xCE4Cu, 0xDFC5u, 0xED5Eu, 0xFCD7u, 0x8868u, 0x99E1u, 0xAB7Au, 0xBAF3u,
        0x5285u, 0x430Cu, 0x7197u, 0x601Eu, 0x14A1u, 0x0528u, 0x37B3u, 0x263Au,
        0xDECDu, 0xCF44u, 0xFDDFu, 0xEC56u, 0x98E9u, 0x8960u, 0xBBFBu, 0xAA72u,
        0x6306u, 0x728Fu, 0x4014u, 0x519Du, 0x2522u, 0x34ABu, 0x0630u, 0x17B9u,
        0xEF4Eu, 0xFEC7u, 0xCC5Cu, 0xDDD5u, 0xA96Au, 0xB8E3u, 0x8A78u, 0x9BF1u,
        0x7387u, 0x620Eu, 0x5095u, 0x411Cu, 0x35A3u, 0x242Au, 0x16B1u, 0x0738u,
        0xFFCFu, 0xEE46u, 0xDCDDu, 0xCD54u, 0xB9EBu, 0xA862u, 0x9AF9u, 0x8B70u,
        0x8408u, 0x9581u, 0xA71Au, 0xB693u, 0xC22Cu, 0xD3A5u, 0xE13Eu, 0xF0B7u,
        0x0840u, 0x19C9u, 0x2B52u, 0x3ADBu, 0x4E64u, 0x5FEDu, 0x6D76u, 0x7CFFu,
        0x9489u, 0x8500u, 0xB79Bu, 0xA612u, 0xD2ADu, 0xC324u, 0xF1BFu, 0xE036u,
        0x18C1u, 0x0948u, 0x3BD3u, 0x2A5Au, 0x5EE5u, 0x4F6Cu, 0x7DF7u, 0x6C7Eu,
        0xA50Au, 0xB483u, 0x8618u, 0x9791u, 0xE32Eu, 0xF2A7u, 0xC03Cu, 0xD1B5u,
        0x2942u, 0x38CBu, 0x0A50u, 0x1BD9u, 0x6F66u, 0x7EEFu, 0x4C74u, 0x5DFDu,
        0xB58Bu, 0xA402u, 0x9699u, 0x8710u, 0xF3AFu, 0xE226u, 0xD0BDu, 0xC134u,
        0x39C3u, 0x284Au, 0x1AD1u, 0x0B58u, 0x7FE7u, 0x6E6Eu, 0x5CF5u, 0x4D7Cu,
        0xC60Cu, 0xD785u, 0xE51Eu, 0xF497u, 0x8028u, 0x91A1u, 0xA33Au, 0xB2B3u,
        0x4A44u, 0x5BCDu, 0x6956u, 0x78DFu, 0x0C60u, 0x1DE9u, 0x2F72u, 0x3EFBu,
        0xD68Du, 0xC704u, 0xF59Fu, 0xE416u, 0x90A9u, 0x8120u, 0xB3BBu, 0xA232u,
        0x5AC5u, 0x4B4Cu, 0x79D7u, 0x685Eu, 0x1CE1u, 0x0D68u, 0x3FF3u, 0x2E7Au,
        0xE70Eu, 0xF687u, 0xC41Cu, 0xD595u, 0xA12Au, 0xB0A3u, 0x8238u, 0x93B1u,
        0x6B46u, 0x7ACFu, 0x4854u, 0x59DDu, 0x2D62u, 0x3CEBu, 0x0E70u, 0x1FF9u,
        0xF78Fu, 0xE606u, 0xD49Du, 0xC514u, 0xB1ABu, 0xA022u, 0x92B9u, 0x8330u,
        0x7BC7u, 0x6A4Eu, 0x58D5u, 0x495Cu, 0x3DE3u, 0x2C6Au, 0x1EF1u, 0x0F78u
    };
    uint16_t crc = 0xFFFFu;

    if (data == nullptr) {
        return crc;
    }

    while (len-- > 0u) {
        crc = (uint16_t)(((crc >> 8u) ^ REFEREE_CRC16_TAB[(crc ^ (*data++)) & 0x00FFu]) & 0xFFFFu);
    }

    return crc;
}

void referee_write_u16_le(uint8_t* dst, uint16_t value)
{
    if (dst == nullptr) {
        return;
    }

    dst[0] = (uint8_t)(value & 0x00FFu);
    dst[1] = (uint8_t)((value >> 8) & 0x00FFu);
}

void referee_copy_unaligned_bytes(uint8_t* dst, const uint8_t* src, uint16_t len)
{
    volatile uint8_t* d = nullptr;
    const volatile uint8_t* s = nullptr;

    if (dst == nullptr || src == nullptr || len == 0u) {
        return;
    }

    d = reinterpret_cast<volatile uint8_t*>(dst);
    s = reinterpret_cast<const volatile uint8_t*>(src);
    while (len-- > 0u) {
        *d++ = *s++;
    }
}

void referee_fill_custom_controller_data(const omnix_custom_controller_raw_payload_t& payload, uint8_t out_data[REFEREE_CUSTOM_DATA_LEN])
{
    const uint8_t* payload_bytes = reinterpret_cast<const uint8_t*>(&payload);

    if (out_data == nullptr) {
        return;
    }

    memset(out_data, 0, REFEREE_CUSTOM_DATA_LEN);
    referee_copy_unaligned_bytes(out_data, payload_bytes, OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE);
}

bool referee_build_custom_controller_frame(const omnix_custom_controller_raw_payload_t& payload, uint8_t* out_frame, uint16_t out_frame_len)
{
    uint8_t* data_ptr = nullptr;
    uint16_t crc16 = 0u;

    if (out_frame == nullptr || out_frame_len < REFEREE_FRAME_LEN) {
        return false;
    }

#if REFEREE_TX_FORCE_FIXED_FRAME
    (void)payload;
    memcpy(out_frame, REFEREE_FIXED_TX_FRAME, REFEREE_FRAME_LEN);
    return true;
#else
    memset(out_frame, 0, REFEREE_FRAME_LEN);
    out_frame[0] = REFEREE_SOF;
    referee_write_u16_le(&out_frame[1], REFEREE_CUSTOM_DATA_LEN);
    out_frame[3] = g_referee_frame_seq++;
    out_frame[4] = referee_crc8(out_frame, 4u);

    referee_write_u16_le(&out_frame[5], REFEREE_CMD_ID_CUSTOM_CONTROLLER_TO_ROBOT);
    data_ptr = &out_frame[REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN];
    referee_fill_custom_controller_data(payload, data_ptr);

    crc16 = referee_crc16(out_frame, (uint16_t)(REFEREE_FRAME_LEN - REFEREE_TAIL_LEN));
    referee_write_u16_le(&out_frame[REFEREE_FRAME_LEN - REFEREE_TAIL_LEN], crc16);
    return true;
#endif
}

bool referee_uart8_tx_ready(void)
{
    return (huart8.gState == HAL_UART_STATE_READY) &&
           (huart8.hdmatx != nullptr) &&
           (huart8.hdmatx->State == HAL_DMA_STATE_READY);
}

void referee_uart8_tx_debug_refresh_state(void)
{
    g_referee_uart8_tx_debug.last_uart_gstate = static_cast<uint32_t>(huart8.gState);
    g_referee_uart8_tx_debug.last_dma_state =
        (huart8.hdmatx != nullptr) ? static_cast<uint32_t>(huart8.hdmatx->State) : 0xFFFFFFFFu;
}

uint16_t referee_read_u16_le(const uint8_t* src)
{
    if (src == nullptr) {
        return 0u;
    }

    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

uint32_t referee_read_u32_le(const uint8_t* src)
{
    if (src == nullptr) {
        return 0u;
    }

    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8u) |
           ((uint32_t)src[2] << 16u) |
           ((uint32_t)src[3] << 24u);
}

void referee_write_u32_le(uint8_t* dst, uint32_t value)
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

void referee_latency_record_echo(const omnix_robot_joint_feedback_payload_t& payload, uint32_t now_ms)
{
    const uint32_t echoed_tick_ms = referee_read_u32_le(&payload.reserved[0]);
    const uint16_t board_rx_to_tx_ms = referee_read_u16_le(&payload.reserved[4]);

    if (echoed_tick_ms == 0u) {
        return;
    }

    const uint32_t rtt_u32 = now_ms - echoed_tick_ms;
    if (rtt_u32 > 60000u) {
        g_referee_latency.bad_echo_samples++;
        return;
    }

    const uint16_t rtt_ms = (uint16_t)rtt_u32;
    const uint16_t one_way_ms =
        (rtt_ms > board_rx_to_tx_ms) ? (uint16_t)((rtt_ms - board_rx_to_tx_ms) / 2u) : 0u;

    referee_latency_hist_add(&g_referee_latency.rtt_ms, rtt_ms);
    referee_latency_hist_add(&g_referee_latency.one_way_ms, one_way_ms);
    g_referee_latency.last_board_rx_to_tx_ms = board_rx_to_tx_ms;
}

bool referee_latency_snapshot(referee_latency_snapshot_t* out)
{
    if (out == nullptr || g_referee_latency.rtt_ms.has_sample == 0u || g_referee_latency.one_way_ms.has_sample == 0u) {
        return false;
    }

    out->has_sample = 1u;
    out->total_samples = g_referee_latency.rtt_ms.total_samples;
    out->bad_echo_samples = g_referee_latency.bad_echo_samples;
    out->board_rx_to_tx_last_ms = g_referee_latency.last_board_rx_to_tx_ms;
    out->rtt_last_ms = g_referee_latency.rtt_ms.last_ms;
    out->rtt_min_ms = g_referee_latency.rtt_ms.min_ms;
    out->rtt_p50_ms = referee_latency_hist_percentile(&g_referee_latency.rtt_ms, 50u);
    out->rtt_p95_ms = referee_latency_hist_percentile(&g_referee_latency.rtt_ms, 95u);
    out->rtt_max_ms = g_referee_latency.rtt_ms.max_ms;
    out->one_way_last_ms = g_referee_latency.one_way_ms.last_ms;
    out->one_way_min_ms = g_referee_latency.one_way_ms.min_ms;
    out->one_way_p50_ms = referee_latency_hist_percentile(&g_referee_latency.one_way_ms, 50u);
    out->one_way_p95_ms = referee_latency_hist_percentile(&g_referee_latency.one_way_ms, 95u);
    out->one_way_max_ms = g_referee_latency.one_way_ms.max_ms;
    return true;
}

bool referee_dma_buf_in_ram_d2(const void* ptr, size_t len)
{
    uintptr_t start = 0u;
    uintptr_t end_exclusive = 0u;

    if (ptr == nullptr || len == 0u) {
        return false;
    }

    start = reinterpret_cast<uintptr_t>(ptr);
    end_exclusive = start + len;
    return (start >= REFEREE_DMA_RAM_D2_START) &&
           (end_exclusive <= REFEREE_DMA_RAM_D2_END_EXCLUSIVE) &&
           (end_exclusive > start);
}

bool referee_dma_buf_aligned32(const void* ptr)
{
    if (ptr == nullptr) {
        return false;
    }
    return (reinterpret_cast<uintptr_t>(ptr) & 0x1Fu) == 0u;
}

void referee_uart8_check_dma_buffers_once(void)
{
    static uint8_t s_checked = 0u;

    if (s_checked != 0u) {
        return;
    }
    s_checked = 1u;

    if (!referee_dma_buf_in_ram_d2(g_referee_rx_dma_buf, sizeof(g_referee_rx_dma_buf))) {
        LOGW("[REF][DMA][%s] rx buffer not in RAM_D2 ptr=0x%08lX len=%lu",
             REFEREE_RX_STAGE,
             static_cast<unsigned long>(reinterpret_cast<uintptr_t>(g_referee_rx_dma_buf)),
             static_cast<unsigned long>(sizeof(g_referee_rx_dma_buf)));
    }
    if (!referee_dma_buf_in_ram_d2(g_referee_tx_frame, sizeof(g_referee_tx_frame))) {
        LOGW("[REF][DMA][%s] tx buffer not in RAM_D2 ptr=0x%08lX len=%lu",
             REFEREE_TX_STAGE,
             static_cast<unsigned long>(reinterpret_cast<uintptr_t>(g_referee_tx_frame)),
             static_cast<unsigned long>(sizeof(g_referee_tx_frame)));
    }
    if (!referee_dma_buf_aligned32(g_referee_rx_dma_buf)) {
        LOGW("[REF][DMA][%s] rx buffer not 32-byte aligned ptr=0x%08lX",
             REFEREE_RX_STAGE,
             static_cast<unsigned long>(reinterpret_cast<uintptr_t>(g_referee_rx_dma_buf)));
    }
    if (!referee_dma_buf_aligned32(g_referee_tx_frame)) {
        LOGW("[REF][DMA][%s] tx buffer not 32-byte aligned ptr=0x%08lX",
             REFEREE_TX_STAGE,
             static_cast<unsigned long>(reinterpret_cast<uintptr_t>(g_referee_tx_frame)));
    }
    if (huart8.hdmarx == nullptr) {
        LOGW("[REF][DMA][%s] UART8 RX DMA link missing", REFEREE_RX_STAGE);
    }
    if (huart8.hdmatx == nullptr) {
        LOGW("[REF][DMA][%s] UART8 TX DMA link missing", REFEREE_TX_STAGE);
    }
}

void referee_uart8_rx_start(void)
{
    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart8, g_referee_rx_dma_buf, REFEREE_RX_DMA_BUF_SIZE);

    referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
        diag.rx_arm_last_ret = (uint32_t)ret;
        diag.rx_armed = ((ret == HAL_OK) || (ret == HAL_BUSY)) ? 1u : 0u;
        if (ret == HAL_OK) {
            diag.rx_arm_ok++;
        } else if (ret == HAL_BUSY) {
            diag.rx_arm_busy++;
        } else {
            diag.rx_arm_err++;
        }
    });

    if ((ret == HAL_OK) || (ret == HAL_BUSY)) {
        if (huart8.hdmarx != nullptr) {
            __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
        }
        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
        __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
        g_referee_uart8_rx_ready = 1u;
    } else {
        g_referee_uart8_rx_ready = 0u;
    }
}

uint16_t referee_rx_fifo_free_nolock(void)
{
    uint16_t head = g_referee_rx_fifo_head;
    uint16_t tail = g_referee_rx_fifo_tail;

    return (uint16_t)(REFEREE_RX_FIFO_SIZE - ((uint16_t)(head - tail) & (REFEREE_RX_FIFO_SIZE - 1u)) - 1u);
}

void referee_rx_fifo_push_byte(uint8_t byte)
{
    uint16_t head = g_referee_rx_fifo_head;
    g_referee_rx_fifo[head] = byte;
    g_referee_rx_fifo_head = (uint16_t)((head + 1u) & (REFEREE_RX_FIFO_SIZE - 1u));
}

size_t referee_rx_fifo_pop_bytes(uint8_t* out, size_t max_len)
{
    size_t count = 0u;

    if (out == nullptr || max_len == 0u) {
        return 0u;
    }

    while (count < max_len) {
        uint16_t tail = g_referee_rx_fifo_tail;
        if (tail == g_referee_rx_fifo_head) {
            break;
        }

        out[count++] = g_referee_rx_fifo[tail];
        g_referee_rx_fifo_tail = (uint16_t)((tail + 1u) & (REFEREE_RX_FIFO_SIZE - 1u));
    }

    return count;
}

void referee_uart8_rx_event_callback(UART_HandleTypeDef* huart, uint16_t size)
{
    if (huart != &huart8) {
        return;
    }

    if ((size == 0u) || (size > REFEREE_RX_DMA_BUF_SIZE)) {
        referee_uart8_rx_start();
        return;
    }

    uint16_t free = referee_rx_fifo_free_nolock();
    uint16_t write_len = (size <= free) ? size : free;
    uint16_t sof_hits = 0u;

    for (uint16_t i = 0u; i < write_len; ++i) {
        referee_rx_fifo_push_byte(g_referee_rx_dma_buf[i]);
        if (g_referee_rx_dma_buf[i] == REFEREE_SOF) {
            ++sof_hits;
        }
    }

    referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
        diag.rx_event_count++;
        diag.rx_event_bytes += size;
        diag.rx_sof_hits += sof_hits;
    });

    if (write_len < size) {
        g_referee_rx_fifo_overflow_count++;
        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.fifo_overflow_count++;
        });
    }

    referee_uart8_rx_start();
}

void referee_uart8_error_callback(UART_HandleTypeDef* huart)
{
    if (huart != &huart8) {
        return;
    }

    const uint32_t error_code = (uint32_t)huart->ErrorCode;
    const uint32_t ore = ((error_code & HAL_UART_ERROR_ORE) != 0u) ? 1u : 0u;
    const uint32_t fe = ((error_code & HAL_UART_ERROR_FE) != 0u) ? 1u : 0u;
    const uint32_t ne = ((error_code & HAL_UART_ERROR_NE) != 0u) ? 1u : 0u;

    referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
        diag.rx_armed = 0u;
        diag.rx_hw_error_count++;
        diag.rx_hw_error_last_code = error_code;
        diag.rx_hw_error_ore_count += ore;
        diag.rx_hw_error_fe_count += fe;
        diag.rx_hw_error_ne_count += ne;
    });

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    referee_uart8_rx_start();
}

void referee_uart8_rx_init(void)
{
    static uint8_t s_rx_initialized = 0u;

    if (s_rx_initialized != 0u) {
        return;
    }

    g_referee_rx_fifo_head = 0u;
    g_referee_rx_fifo_tail = 0u;
    g_referee_rx_parse_len = 0u;
    g_referee_rx_fifo_overflow_count = 0u;
    g_referee_uart8_diag = {};
    referee_uart8_check_dma_buffers_once();

    (void)HAL_UART_RegisterRxEventCallback(&huart8, referee_uart8_rx_event_callback);
    (void)HAL_UART_RegisterCallback(&huart8, HAL_UART_ERROR_CB_ID, referee_uart8_error_callback);
    referee_uart8_rx_start();
    s_rx_initialized = 1u;
}

void referee_handle_rx_beep(uint32_t now_ms)
{
    if (g_referee_rx_beep_active != 0u) {
        return;
    }

    if (Buzzer_IsActive() == false) {
        Buzzer_Start(REFEREE_RX_BEEP_FREQ_HZ);
        g_referee_rx_beep_active = 1u;
        g_referee_rx_beep_stop_ms = now_ms + REFEREE_RX_BEEP_MS;
    }
}

void referee_update_rx_beep(uint32_t now_ms)
{
    if ((g_referee_rx_beep_active != 0u) && ((int32_t)(now_ms - g_referee_rx_beep_stop_ms) >= 0)) {
        Buzzer_Stop();
        g_referee_rx_beep_active = 0u;
    }
}

void referee_on_valid_rx_frame(const uint8_t* frame, uint16_t frame_len, uint32_t now_ms)
{
    uint16_t cmd_id = 0u;
    uint16_t data_len = 0u;
    uint8_t seq = 0u;

    if (frame == nullptr || frame_len < REFEREE_MIN_FRAME_LEN) {
        return;
    }

    data_len = referee_read_u16_le(&frame[1]);
    cmd_id = referee_read_u16_le(&frame[5]);
    seq = frame[3];
    g_referee_rx_frame_count++;
    referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
        diag.parse_good_frames++;
        if (cmd_id == REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER) {
            diag.parse_cmd_0309_frames++;
            if (diag.last_rx_seq_valid != 0u) {
                const uint8_t expected_seq = (uint8_t)(diag.last_rx_seq + 1u);
                if (seq != expected_seq) {
                    diag.rx_seq_gap_count++;
                }
            }
            diag.last_rx_seq = seq;
            diag.last_rx_seq_valid = 1u;
        }
    });

    LOGI("[REF][RX][%s] cnt=%lu seq=%u cmd=0x%04X len=%u",
         REFEREE_RX_STAGE,
         static_cast<unsigned long>(g_referee_rx_frame_count),
         seq,
         cmd_id,
         data_len);

    if (cmd_id == REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER && data_len == OMNIX_ROBOT_JOINT_FEEDBACK_PAYLOAD_SIZE) {
        omnix_robot_joint_feedback_payload_t payload{};
        uint8_t* payload_bytes = reinterpret_cast<uint8_t*>(&payload);
        const uint8_t* frame_data = &frame[REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN];
        referee_copy_unaligned_bytes(payload_bytes, frame_data, OMNIX_ROBOT_JOINT_FEEDBACK_PAYLOAD_SIZE);
        if (payload.version == OMNIX_ROBOT_JOINT_FEEDBACK_VERSION) {
            referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
                diag.parse_cmd_0309_good_frames++;
            });
            referee_latency_record_echo(payload, now_ms);
            referee_store_joint_feedback(&payload, now_ms);
            const bool remote_buzzer_req =
                ((payload.diag_flags & OMNIX_DIAG_FLAG_REMOTE_BUZZER_REQ) != 0u);
            if (remote_buzzer_req && (g_referee_rx_remote_buzzer_prev_flag == 0u)) {
                referee_handle_rx_beep(now_ms);
            }
            g_referee_rx_remote_buzzer_prev_flag = remote_buzzer_req ? 1u : 0u;
        } else {
            referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
                diag.payload_version_errors++;
            });
            LOGW("[REF][0309][RX][%s] bad version=%u", REFEREE_RX_STAGE, payload.version);
        }
    } else if (cmd_id == REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER) {
        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.payload_len_errors++;
        });
    }
}

void referee_drop_parse_prefix(uint16_t drop_len)
{
    if (drop_len == 0u) {
        return;
    }

    if (drop_len >= g_referee_rx_parse_len) {
        g_referee_rx_parse_len = 0u;
        return;
    }

    std::memmove(g_referee_rx_parse_buf,
                 &g_referee_rx_parse_buf[drop_len],
                 (size_t)(g_referee_rx_parse_len - drop_len));
    g_referee_rx_parse_len = (uint16_t)(g_referee_rx_parse_len - drop_len);
}

void referee_try_parse_rx_frames(uint32_t now_ms)
{
    while (g_referee_rx_parse_len > 0u) {
        if (g_referee_rx_parse_buf[0] != REFEREE_SOF) {
            referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
                diag.sof_drop_bytes++;
            });
            referee_drop_parse_prefix(1u);
            continue;
        }

        if (g_referee_rx_parse_len < REFEREE_HEADER_LEN) {
            break;
        }

        if (referee_crc8(g_referee_rx_parse_buf, 4u) != g_referee_rx_parse_buf[4]) {
            referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
                diag.crc8_errors++;
                diag.sof_drop_bytes++;
            });
            referee_drop_parse_prefix(1u);
            continue;
        }

        uint16_t data_len = referee_read_u16_le(&g_referee_rx_parse_buf[1]);
        uint16_t frame_len = (uint16_t)(REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_TAIL_LEN);
        if ((frame_len < REFEREE_MIN_FRAME_LEN) || (frame_len > REFEREE_RX_PARSE_BUF_SIZE)) {
            referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
                diag.payload_len_errors++;
                diag.sof_drop_bytes++;
            });
            referee_drop_parse_prefix(1u);
            continue;
        }

        if (g_referee_rx_parse_len < frame_len) {
            break;
        }

        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.parse_total_frames++;
        });

        const uint16_t cmd_id = referee_read_u16_le(&g_referee_rx_parse_buf[5]);
        uint16_t frame_crc = referee_read_u16_le(&g_referee_rx_parse_buf[frame_len - REFEREE_TAIL_LEN]);
        uint16_t calc_crc = referee_crc16(g_referee_rx_parse_buf, (uint16_t)(frame_len - REFEREE_TAIL_LEN));
        if (calc_crc != frame_crc) {
            referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
                diag.crc16_errors++;
                if (cmd_id == REFEREE_CMD_ID_ROBOT_TO_CUSTOM_CONTROLLER) {
                    diag.parse_cmd_0309_crc_fail_frames++;
                }
                diag.sof_drop_bytes++;
            });
            referee_drop_parse_prefix(1u);
            continue;
        }

        referee_on_valid_rx_frame(g_referee_rx_parse_buf, frame_len, now_ms);
        referee_drop_parse_prefix(frame_len);
    }
}

void referee_poll_rx_fifo(uint32_t now_ms)
{
    uint8_t chunk[128] = {0};
    size_t chunk_len = 0u;

    do {
        chunk_len = referee_rx_fifo_pop_bytes(chunk, sizeof(chunk));
        if (chunk_len == 0u) {
            break;
        }

        for (size_t i = 0u; i < chunk_len; ++i) {
            if (g_referee_rx_parse_len < REFEREE_RX_PARSE_BUF_SIZE) {
                g_referee_rx_parse_buf[g_referee_rx_parse_len++] = chunk[i];
            } else {
                std::memmove(g_referee_rx_parse_buf,
                             &g_referee_rx_parse_buf[1],
                             (size_t)(REFEREE_RX_PARSE_BUF_SIZE - 1u));
                g_referee_rx_parse_buf[REFEREE_RX_PARSE_BUF_SIZE - 1u] = chunk[i];
            }
        }

        referee_try_parse_rx_frames(now_ms);
    } while (chunk_len > 0u);
}

const char* referee_servo_boot_state_str(serial_servo_boot_state_t state)
{
    switch (state) {
        case SERVO_BOOT_WAIT_ONLINE:
            return "wait_online";
        case SERVO_BOOT_RESTORING:
            return "restoring";
        case SERVO_BOOT_HOLDING:
            return "holding";
        case SERVO_BOOT_DONE:
            return "done";
        default:
            return "unknown";
    }
}

bool referee_copy_joint_raw_cmd(omnix_custom_controller_raw_payload_t* out, uint32_t* tick_ms)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    uint32_t local_tick = 0u;
    omnix_custom_controller_raw_payload_t local{};

    do {
        seq_a = g_joint_tx_guard;
        __DMB();
        local = g_joint_tx_payload;
        local_tick = g_joint_tx_tick_ms;
        __DMB();
        seq_b = g_joint_tx_guard;
    } while (seq_a != seq_b);

    *out = local;
    if (tick_ms != nullptr) {
        *tick_ms = local_tick;
    }
    return true;
}

bool referee_copy_joint_feedback(omnix_robot_joint_feedback_payload_t* out, uint32_t* tick_ms)
{
    if (out == nullptr) {
        return false;
    }

    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    uint32_t local_tick = 0u;
    omnix_robot_joint_feedback_payload_t local{};

    do {
        seq_a = g_joint_feedback_rx_guard;
        __DMB();
        local = g_joint_feedback_rx_payload;
        local_tick = g_joint_feedback_rx_tick_ms;
        __DMB();
        seq_b = g_joint_feedback_rx_guard;
    } while (seq_a != seq_b);

    if (local_tick == 0u) {
        return false;
    }

    *out = local;
    if (tick_ms != nullptr) {
        *tick_ms = local_tick;
    }
    return true;
}

void referee_store_joint_feedback(const omnix_robot_joint_feedback_payload_t* payload, uint32_t now_ms)
{
    if (payload == nullptr) {
        return;
    }

    __DMB();
    g_joint_feedback_rx_guard++;
    __DMB();
    g_joint_feedback_rx_payload = *payload;
    g_joint_feedback_rx_tick_ms = now_ms;
    __DMB();
    g_joint_feedback_rx_guard++;
    __DMB();

    g_mapping_reset_requested = ((payload->diag_flags & OMNIX_DIAG_FLAG_MAPPING_RESET_REQ) != 0u);
    if (!g_mapping_reset_requested) {
        g_mapping_reset_done = false;
    }
}

bool referee_time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return (int32_t)(now_ms - target_ms) >= 0;
}

void referee_service_rx(referee_loop_ctx_t* ctx)
{
    if (ctx == nullptr) {
        return;
    }

    referee_update_rx_beep(ctx->now_ms);
    referee_poll_rx_fifo(ctx->now_ms);
    if (g_referee_uart8_rx_ready == 0u) {
        referee_uart8_rx_start();
    }

    ctx->has_diag = referee_uart8_diag_copy(&ctx->diag);
    ctx->has_feedback = referee_copy_joint_feedback(&ctx->feedback, &ctx->feedback_tick_ms);
    ctx->has_payload = referee_copy_joint_raw_cmd(&ctx->payload, &ctx->payload_tick_ms);
}

void referee_service_logs(const referee_loop_ctx_t* ctx, referee_rx_log_state_t* rx_log_state, referee_tx_state_t* tx_state)
{
    if (ctx == nullptr || rx_log_state == nullptr || tx_state == nullptr) {
        return;
    }

    if ((g_referee_rx_fifo_overflow_count != rx_log_state->last_rx_overflow_count) &&
        ((ctx->now_ms - rx_log_state->last_rx_overflow_log_ms) >= 1000u)) {
        rx_log_state->last_rx_overflow_log_ms = ctx->now_ms;
        rx_log_state->last_rx_overflow_count = g_referee_rx_fifo_overflow_count;
        LOGW("[REF][RX][%s] fifo overflow cnt=%lu",
             REFEREE_RX_STAGE,
             static_cast<unsigned long>(rx_log_state->last_rx_overflow_count));
    }

    if (ctx->has_diag && ((ctx->now_ms - rx_log_state->last_diag_log_ms) >= 1000u)) {
        const uint32_t rx_event_count_delta = ctx->diag.rx_event_count - rx_log_state->last_diag.rx_event_count;
        const uint32_t rx_event_bytes_delta = ctx->diag.rx_event_bytes - rx_log_state->last_diag.rx_event_bytes;
        const uint32_t rx_sof_delta = ctx->diag.rx_sof_hits - rx_log_state->last_diag.rx_sof_hits;
        const uint32_t rx_arm_ok_delta = ctx->diag.rx_arm_ok - rx_log_state->last_diag.rx_arm_ok;
        const uint32_t rx_arm_busy_delta = ctx->diag.rx_arm_busy - rx_log_state->last_diag.rx_arm_busy;
        const uint32_t rx_arm_err_delta = ctx->diag.rx_arm_err - rx_log_state->last_diag.rx_arm_err;
        const uint32_t rx_hw_error_delta = ctx->diag.rx_hw_error_count - rx_log_state->last_diag.rx_hw_error_count;
        const uint32_t tx_ok_delta = ctx->diag.tx_dma_ok - rx_log_state->last_diag.tx_dma_ok;
        const uint32_t tx_busy_delta = ctx->diag.tx_dma_busy - rx_log_state->last_diag.tx_dma_busy;
        const uint32_t tx_err_delta = ctx->diag.tx_dma_err - rx_log_state->last_diag.tx_dma_err;
        const uint32_t tx_not_ready_delta = ctx->diag.tx_not_ready_count - rx_log_state->last_diag.tx_not_ready_count;
        const uint32_t tx_due_miss_delta = ctx->diag.tx_due_miss - rx_log_state->last_diag.tx_due_miss;
        const uint32_t tx_skip_busy_delta = ctx->diag.tx_skip_busy - rx_log_state->last_diag.tx_skip_busy;
        const uint32_t tx_skip_not_ready_delta = ctx->diag.tx_skip_not_ready - rx_log_state->last_diag.tx_skip_not_ready;
        const uint32_t tx_loop_overrun_delta = ctx->diag.tx_loop_overrun - rx_log_state->last_diag.tx_loop_overrun;
        const uint32_t cmd_0309_good_delta =
            ctx->diag.parse_cmd_0309_good_frames - rx_log_state->last_diag.parse_cmd_0309_good_frames;
        const uint32_t ack_gap_delta = ctx->diag.rx_seq_gap_count - rx_log_state->last_diag.rx_seq_gap_count;
        const uint32_t feedback_silence_ms =
            ctx->has_feedback ? (ctx->now_ms - ctx->feedback_tick_ms) : (ctx->now_ms - rx_log_state->task_start_ms);

        rx_log_state->last_diag = ctx->diag;
        rx_log_state->last_diag_log_ms = ctx->now_ms;
        LOGI("[REF][DMA][%s] tx_ready=%u tx_gstate=%lu tx_dma=%u tx_dma_state=%lu rx_ready=%u rx_dma=%u rx_dma_state=%lu evt_total=%lu evt_1s=%lu bytes_total=%lu bytes_1s=%lu sof_total=%lu sof_1s=%lu arm_ok_total=%lu arm_ok_1s=%lu arm_busy_total=%lu arm_busy_1s=%lu arm_err_total=%lu arm_err_1s=%lu armed=%lu arm_last=%lu hw_err_total=%lu hw_err_1s=%lu hw_last=0x%08lX ore_total=%lu fe_total=%lu ne_total=%lu fifo=%lu frames=%lu good=%lu 0309=%lu c8=%lu c16=%lu len=%lu ver=%lu sof_drop=%lu last_rx_seq=%u seq_gap=%lu tx_ok=%lu tx_ok_1s=%lu tx_busy=%lu tx_busy_1s=%lu tx_err=%lu tx_err_1s=%lu tx_not_ready=%lu tx_not_ready_1s=%lu tx_due_miss=%lu tx_due_miss_1s=%lu tx_skip_busy=%lu tx_skip_busy_1s=%lu tx_skip_not_ready=%lu tx_skip_not_ready_1s=%lu tx_loop_ovr=%lu tx_loop_ovr_1s=%lu tx_period_last=%lu tx_period_max=%lu",
             REFEREE_RX_STAGE,
             referee_uart8_tx_ready() ? 1u : 0u,
             static_cast<unsigned long>(huart8.gState),
             (huart8.hdmatx != nullptr) ? 1u : 0u,
             (huart8.hdmatx != nullptr) ? static_cast<unsigned long>(huart8.hdmatx->State) : 0xFFFFFFFFul,
             g_referee_uart8_rx_ready,
             (huart8.hdmarx != nullptr) ? 1u : 0u,
             (huart8.hdmarx != nullptr) ? static_cast<unsigned long>(huart8.hdmarx->State) : 0xFFFFFFFFul,
             static_cast<unsigned long>(ctx->diag.rx_event_count),
             static_cast<unsigned long>(rx_event_count_delta),
             static_cast<unsigned long>(ctx->diag.rx_event_bytes),
             static_cast<unsigned long>(rx_event_bytes_delta),
             static_cast<unsigned long>(ctx->diag.rx_sof_hits),
             static_cast<unsigned long>(rx_sof_delta),
             static_cast<unsigned long>(ctx->diag.rx_arm_ok),
             static_cast<unsigned long>(rx_arm_ok_delta),
             static_cast<unsigned long>(ctx->diag.rx_arm_busy),
             static_cast<unsigned long>(rx_arm_busy_delta),
             static_cast<unsigned long>(ctx->diag.rx_arm_err),
             static_cast<unsigned long>(rx_arm_err_delta),
             static_cast<unsigned long>(ctx->diag.rx_armed),
             static_cast<unsigned long>(ctx->diag.rx_arm_last_ret),
             static_cast<unsigned long>(ctx->diag.rx_hw_error_count),
             static_cast<unsigned long>(rx_hw_error_delta),
             static_cast<unsigned long>(ctx->diag.rx_hw_error_last_code),
             static_cast<unsigned long>(ctx->diag.rx_hw_error_ore_count),
             static_cast<unsigned long>(ctx->diag.rx_hw_error_fe_count),
             static_cast<unsigned long>(ctx->diag.rx_hw_error_ne_count),
             static_cast<unsigned long>(ctx->diag.fifo_overflow_count),
             static_cast<unsigned long>(ctx->diag.parse_total_frames),
             static_cast<unsigned long>(ctx->diag.parse_good_frames),
             static_cast<unsigned long>(ctx->diag.parse_cmd_0309_frames),
             static_cast<unsigned long>(ctx->diag.crc8_errors),
             static_cast<unsigned long>(ctx->diag.crc16_errors),
             static_cast<unsigned long>(ctx->diag.payload_len_errors),
             static_cast<unsigned long>(ctx->diag.payload_version_errors),
             static_cast<unsigned long>(ctx->diag.sof_drop_bytes),
             ctx->diag.last_rx_seq_valid != 0u ? ctx->diag.last_rx_seq : 0u,
             static_cast<unsigned long>(ctx->diag.rx_seq_gap_count),
             static_cast<unsigned long>(ctx->diag.tx_dma_ok),
             static_cast<unsigned long>(tx_ok_delta),
             static_cast<unsigned long>(ctx->diag.tx_dma_busy),
             static_cast<unsigned long>(tx_busy_delta),
             static_cast<unsigned long>(ctx->diag.tx_dma_err),
             static_cast<unsigned long>(tx_err_delta),
             static_cast<unsigned long>(ctx->diag.tx_not_ready_count),
             static_cast<unsigned long>(tx_not_ready_delta),
             static_cast<unsigned long>(ctx->diag.tx_due_miss),
             static_cast<unsigned long>(tx_due_miss_delta),
             static_cast<unsigned long>(ctx->diag.tx_skip_busy),
             static_cast<unsigned long>(tx_skip_busy_delta),
             static_cast<unsigned long>(ctx->diag.tx_skip_not_ready),
             static_cast<unsigned long>(tx_skip_not_ready_delta),
             static_cast<unsigned long>(ctx->diag.tx_loop_overrun),
             static_cast<unsigned long>(tx_loop_overrun_delta),
             static_cast<unsigned long>(ctx->diag.tx_period_last_ms),
             static_cast<unsigned long>(ctx->diag.tx_period_max_ms));
        LOGI("[REF][HEALTH][%s] 0302_tx_ok_1s=%lu 0302_tx_busy_1s=%lu 0302_tx_not_ready_1s=%lu 0302_due_miss_1s=%lu 0302_tx_int_ms=%lu 0309_good_1s=%lu 0309_silence_ms=%lu ack_gap_1s=%lu",
             REFEREE_RX_STAGE,
             static_cast<unsigned long>(tx_ok_delta),
             static_cast<unsigned long>(tx_busy_delta),
             static_cast<unsigned long>(tx_not_ready_delta),
             static_cast<unsigned long>(tx_due_miss_delta),
             static_cast<unsigned long>(REFEREE_CTRL_0302_TX_INTERVAL_MS),
             static_cast<unsigned long>(cmd_0309_good_delta),
             static_cast<unsigned long>(feedback_silence_ms),
             static_cast<unsigned long>(ack_gap_delta));
    }

    if (ctx->has_feedback &&
        (ctx->feedback.seq != rx_log_state->last_feedback_seq) &&
        ((ctx->now_ms - rx_log_state->last_feedback_log_ms) >= 1000u)) {
        rx_log_state->last_feedback_seq = ctx->feedback.seq;
        rx_log_state->last_feedback_log_ms = ctx->now_ms;
        const uint32_t echo_controller_tick_ms = referee_read_u32_le(&ctx->feedback.reserved[0]);
        const uint16_t board_rx_to_tx_ms = referee_read_u16_le(&ctx->feedback.reserved[4]);
        LOGI("[REF][0309][RX][%s] ver=%u seq=%u age=%lu echo=%lu board_rx_to_tx=%u flags=0x%02X diag=0x%02X valid=0x%02X online=0x%02X pause=%u J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
             REFEREE_RX_STAGE,
             ctx->feedback.version,
             ctx->feedback.seq,
             static_cast<unsigned long>(ctx->now_ms - ctx->feedback_tick_ms),
             static_cast<unsigned long>(echo_controller_tick_ms),
             board_rx_to_tx_ms,
             ctx->feedback.flags,
             ctx->feedback.diag_flags,
             ctx->feedback.valid_mask,
             ctx->feedback.online_mask,
             ctx->feedback.pause_phase,
             static_cast<unsigned>(ctx->feedback.raw_u16[0]),
             static_cast<unsigned>(ctx->feedback.raw_u16[1]),
             static_cast<unsigned>(ctx->feedback.raw_u16[2]),
             static_cast<unsigned>(ctx->feedback.raw_u16[3]),
             static_cast<unsigned>(ctx->feedback.raw_u16[4]),
             static_cast<unsigned>(ctx->feedback.raw_u16[5]),
             static_cast<unsigned>(ctx->feedback.raw_u16[6]));
    }

    if ((Referee_IsMappingResetRequested() || g_mapping_reset_done) &&
        ctx->has_feedback &&
        ((ctx->now_ms - rx_log_state->last_map_rx_log_ms) >= 500u)) {
        rx_log_state->last_map_rx_log_ms = ctx->now_ms;
        LOGI("[MAP][0309][RX][%s] ver=%u seq=%u req=%u age=%lu J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
             REFEREE_RX_STAGE,
             ctx->feedback.version,
             ctx->feedback.seq,
             ((ctx->feedback.diag_flags & OMNIX_DIAG_FLAG_MAPPING_RESET_REQ) != 0u) ? 1u : 0u,
             static_cast<unsigned long>(ctx->now_ms - ctx->feedback_tick_ms),
             static_cast<unsigned>(ctx->feedback.raw_u16[0]),
             static_cast<unsigned>(ctx->feedback.raw_u16[1]),
             static_cast<unsigned>(ctx->feedback.raw_u16[2]),
             static_cast<unsigned>(ctx->feedback.raw_u16[3]),
             static_cast<unsigned>(ctx->feedback.raw_u16[4]),
             static_cast<unsigned>(ctx->feedback.raw_u16[5]),
             static_cast<unsigned>(ctx->feedback.raw_u16[6]));
    }

    if ((ctx->now_ms - rx_log_state->last_latency_log_ms) >= 1000u) {
        referee_latency_snapshot_t latency{};
        rx_log_state->last_latency_log_ms = ctx->now_ms;
        if (referee_latency_snapshot(&latency)) {
            LOGI("[REF][LAT][0302->0309] n=%lu bad=%lu board_rx_to_tx=%u rtt_ms(last/min/p50/p95/max)=%u/%u/%u/%u/%u one_way_ms(last/min/p50/p95/max)=%u/%u/%u/%u/%u",
                 static_cast<unsigned long>(latency.total_samples),
                 static_cast<unsigned long>(latency.bad_echo_samples),
                 latency.board_rx_to_tx_last_ms,
                 latency.rtt_last_ms,
                 latency.rtt_min_ms,
                 latency.rtt_p50_ms,
                 latency.rtt_p95_ms,
                 latency.rtt_max_ms,
                 latency.one_way_last_ms,
                 latency.one_way_min_ms,
                 latency.one_way_p50_ms,
                 latency.one_way_p95_ms,
                 latency.one_way_max_ms);
        }
    }

    if (!ctx->has_payload || ctx->payload_tick_ms == 0u) {
        if ((ctx->now_ms - tx_state->last_wait_payload_log_ms) >= 1000u) {
            tx_state->last_wait_payload_log_ms = ctx->now_ms;
            LOGI("[REF][TX][%s] waiting payload boot=%s online=%u/%u",
                 REFEREE_TX_STAGE,
                 referee_servo_boot_state_str(g_servo_boot_state),
                 g_servo_online_count,
                 SERVO_COUNT);
        }
        return;
    }

    const uint32_t publish_age_ms = ctx->now_ms - ctx->payload_tick_ms;
    const uint32_t send_age_ms = (tx_state->last_send_ms != 0u) ? (ctx->now_ms - tx_state->last_send_ms) : 0u;
    const uint32_t sent_publish_age_ms =
        (tx_state->last_sent_publish_tick_ms != 0u) ? (ctx->now_ms - tx_state->last_sent_publish_tick_ms) : 0u;

    if ((ctx->now_ms - tx_state->last_log_ms) >= 1000u) {
        tx_state->last_log_ms = ctx->now_ms;
        LOGI("[CTRL][0302][TX][%s] ver=%u next_seq=%u mask=0x%02X diag=0x%02X last_publish_age_ms=%lu last_send_age_ms=%lu last_sent_publish_age_ms=%lu J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
             REFEREE_TX_STAGE,
             ctx->payload.version,
             tx_state->next_tx_seq,
             ctx->payload.valid_mask,
             ctx->payload.diag_flags,
             static_cast<unsigned long>(publish_age_ms),
             static_cast<unsigned long>(send_age_ms),
             static_cast<unsigned long>(sent_publish_age_ms),
             static_cast<unsigned>(ctx->payload.raw_u16[0]),
             static_cast<unsigned>(ctx->payload.raw_u16[1]),
             static_cast<unsigned>(ctx->payload.raw_u16[2]),
             static_cast<unsigned>(ctx->payload.raw_u16[3]),
             static_cast<unsigned>(ctx->payload.raw_u16[4]),
             static_cast<unsigned>(ctx->payload.raw_u16[5]),
             static_cast<unsigned>(ctx->payload.raw_u16[6]));
    }

    if ((Referee_IsMappingResetRequested() || g_mapping_reset_done) &&
        ((ctx->now_ms - tx_state->last_map_tx_log_ms) >= 500u)) {
        tx_state->last_map_tx_log_ms = ctx->now_ms;
        LOGI("[MAP][0302][TX][%s] ver=%u next_seq=%u done=%u age=%lu J1=%u J2=%u J3=%u J4=%u J5=%u J6=%u J7=%u",
             REFEREE_TX_STAGE,
             ctx->payload.version,
             tx_state->next_tx_seq,
             g_mapping_reset_done ? 1u : 0u,
             static_cast<unsigned long>(publish_age_ms),
             static_cast<unsigned>(ctx->payload.raw_u16[0]),
             static_cast<unsigned>(ctx->payload.raw_u16[1]),
             static_cast<unsigned>(ctx->payload.raw_u16[2]),
             static_cast<unsigned>(ctx->payload.raw_u16[3]),
             static_cast<unsigned>(ctx->payload.raw_u16[4]),
             static_cast<unsigned>(ctx->payload.raw_u16[5]),
             static_cast<unsigned>(ctx->payload.raw_u16[6]));
    }
}

void referee_service_tx(const referee_loop_ctx_t* ctx, referee_tx_state_t* tx_state)
{
    if (ctx == nullptr || tx_state == nullptr) {
        return;
    }

    g_referee_uart8_tx_debug.call_count++;
    g_referee_uart8_tx_debug.last_call_ms = ctx->now_ms;
    g_referee_uart8_tx_debug.last_payload_tick_ms = ctx->payload_tick_ms;
    referee_uart8_tx_debug_refresh_state();
    if (!ctx->has_payload) {
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_IDLE;
        return;
    }

#if REFEREE_TX_REQUIRE_PUBLISH_TICK
    if (ctx->payload_tick_ms == 0u) {
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_IDLE;
        return;
    }
#endif

    if (tx_state->next_tx_due_ms == 0u) {
        tx_state->next_tx_due_ms = ctx->now_ms;
    }
    g_referee_uart8_tx_debug.last_next_due_ms = tx_state->next_tx_due_ms;
    if (!referee_time_reached(ctx->now_ms, tx_state->next_tx_due_ms)) {
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_WAIT_DUE;
        g_referee_uart8_tx_debug.last_lateness_ms = 0u;
        g_referee_uart8_tx_debug.last_missed_slots = 0u;
        return;
    }

    const uint32_t lateness_ms = ctx->now_ms - tx_state->next_tx_due_ms;
    const uint32_t missed_slots = lateness_ms / REFEREE_CTRL_0302_TX_INTERVAL_MS;
    g_referee_uart8_tx_debug.last_lateness_ms = lateness_ms;
    g_referee_uart8_tx_debug.last_missed_slots = missed_slots;
    if (missed_slots > 0u) {
        referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
            diag.tx_due_miss += missed_slots;
        });
    }
    tx_state->next_tx_due_ms += (missed_slots + 1u) * REFEREE_CTRL_0302_TX_INTERVAL_MS;
    g_referee_uart8_tx_debug.last_next_due_ms = tx_state->next_tx_due_ms;

    if (!referee_uart8_tx_ready()) {
        g_referee_uart8_tx_debug.tx_not_ready_count++;
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_NOT_READY;
        referee_uart8_tx_debug_refresh_state();
        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.tx_not_ready_count++;
            diag.tx_skip_not_ready++;
        });
        if ((ctx->now_ms - tx_state->last_uart_not_ready_log_ms) >= 1000u) {
            tx_state->last_uart_not_ready_log_ms = ctx->now_ms;
            LOGW("[REF][TX][%s] uart8 not ready gState=%lu dma_link=%u dma_state=%lu",
                 REFEREE_TX_STAGE,
                 static_cast<unsigned long>(huart8.gState),
                 (huart8.hdmatx != nullptr) ? 1u : 0u,
                 (huart8.hdmatx != nullptr) ? static_cast<unsigned long>(huart8.hdmatx->State) : 0xFFFFFFFFul);
        }
        return;
    }

    omnix_custom_controller_raw_payload_t tx_payload = ctx->payload;
    tx_payload.seq = tx_state->next_tx_seq;
    memset(tx_payload.reserved, 0, sizeof(tx_payload.reserved));
    referee_write_u32_le(&tx_payload.reserved[0], ctx->now_ms);
    tx_payload.reserved[4] = REFEREE_LATENCY_PROBE_ENABLE_MASK;
    if (!referee_build_custom_controller_frame(tx_payload, g_referee_tx_frame, sizeof(g_referee_tx_frame))) {
        g_referee_uart8_tx_debug.frame_build_fail_count++;
        g_referee_uart8_tx_debug.last_seq = tx_payload.seq;
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_FRAME_BUILD_FAIL;
        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.tx_dma_err++;
        });
        LOGW("[REF][TX][%s] frame build failed seq=%u", REFEREE_TX_STAGE, tx_payload.seq);
        return;
    }

    g_referee_uart8_tx_debug.last_seq = tx_payload.seq;
    g_referee_uart8_tx_debug.last_sof = g_referee_tx_frame[0];
    g_referee_uart8_tx_debug.last_crc8 = g_referee_tx_frame[4];
    g_referee_uart8_tx_debug.last_cmd_id = referee_read_u16_le(&g_referee_tx_frame[5]);
    g_referee_uart8_tx_debug.last_crc16 =
        referee_read_u16_le(&g_referee_tx_frame[REFEREE_FRAME_LEN - REFEREE_TAIL_LEN]);

    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(&huart8, g_referee_tx_frame, REFEREE_FRAME_LEN);
    g_referee_uart8_tx_debug.last_hal_ret = static_cast<uint32_t>(ret);
    referee_uart8_tx_debug_refresh_state();
    if (ret == HAL_OK) {
        g_referee_uart8_tx_debug.tx_ok_count++;
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_DMA_OK;
        referee_uart8_diag_mutate([&](referee_uart8_diag_t& diag) {
            diag.tx_dma_ok++;
            if (tx_state->last_send_ms != 0u) {
                const uint32_t period_ms = ctx->now_ms - tx_state->last_send_ms;
                diag.tx_period_last_ms = period_ms;
                if (period_ms >= diag.tx_period_max_ms) {
                    diag.tx_period_max_ms = period_ms;
                }
            }
        });
        tx_state->last_send_ms = ctx->now_ms;
        tx_state->last_sent_publish_tick_ms = ctx->payload_tick_ms;
        tx_state->next_tx_seq = static_cast<uint16_t>(tx_state->next_tx_seq + 1u);
    } else if (ret == HAL_BUSY) {
        g_referee_uart8_tx_debug.tx_busy_count++;
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_DMA_BUSY;
        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.tx_dma_busy++;
            diag.tx_skip_busy++;
        });
    } else {
        g_referee_uart8_tx_debug.tx_err_count++;
        g_referee_uart8_tx_debug.last_stage = REFEREE_UART8_TX_DBG_STAGE_DMA_ERR;
        referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
            diag.tx_dma_err++;
        });
        LOGW("[REF][TX][%s] uart8 dma send failed ret=%d seq=%u", REFEREE_TX_STAGE, (int)ret, tx_payload.seq);
    }
}

} // namespace

extern "C" void Referee_PublishJointRawCmd(const omnix_custom_controller_raw_payload_t* payload)
{
    if (payload == nullptr) {
        return;
    }

    omnix_custom_controller_raw_payload_t local = *payload;
    local.version = OMNIX_CUSTOM_CONTROLLER_PROTOCOL_VERSION;
    local.seq = 0u;
    if (g_mapping_reset_done) {
        local.diag_flags |= OMNIX_DIAG_FLAG_MAPPING_RESET_DONE;
    }
    memset(local.reserved, 0, sizeof(local.reserved));

    __DMB();
    g_joint_tx_guard++;
    __DMB();
    g_joint_tx_payload = local;
    g_joint_tx_tick_ms = HAL_GetTick();
    __DMB();
    g_joint_tx_guard++;
    __DMB();
}

extern "C" bool Referee_CopyJointRawCmd(omnix_custom_controller_raw_payload_t* out)
{
    return referee_copy_joint_raw_cmd(out, nullptr);
}

extern "C" bool Referee_CopyJointRawCmdBytes(uint8_t* out_buf, uint16_t out_buf_len, uint16_t* out_len)
{
    omnix_custom_controller_raw_payload_t payload{};

    if (out_buf == nullptr || out_len == nullptr || out_buf_len < OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE) {
        return false;
    }
    if (!referee_copy_joint_raw_cmd(&payload, nullptr)) {
        return false;
    }

    referee_copy_unaligned_bytes(out_buf,
                                 reinterpret_cast<const uint8_t*>(&payload),
                                 OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE);
    *out_len = OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE;
    return true;
}

extern "C" bool Referee_CopyCustomControllerData0302(uint8_t out_data[30])
{
    omnix_custom_controller_raw_payload_t payload{};

    if (out_data == nullptr) {
        return false;
    }
    if (!referee_copy_joint_raw_cmd(&payload, nullptr)) {
        return false;
    }

    referee_fill_custom_controller_data(payload, out_data);
    return true;
}

extern "C" bool Referee_ReadRobotJointRawFeedback(omnix_robot_joint_feedback_payload_t* out)
{
    return referee_copy_joint_feedback(out, nullptr);
}

extern "C" bool Referee_IsMappingResetRequested(void)
{
    return g_mapping_reset_requested;
}

extern "C" void Referee_SetMappingResetDone(bool done)
{
    g_mapping_reset_done = done;
}

extern "C" void Start_Referee_Task(void *argument)
{
    (void)argument;

    const TickType_t loop_ticks = (pdMS_TO_TICKS(REFEREE_TASK_LOOP_PERIOD_MS) > 0u) ? pdMS_TO_TICKS(REFEREE_TASK_LOOP_PERIOD_MS) : 1u;
    TickType_t last_wake_tick = 0u;
    referee_rx_log_state_t rx_log_state = {};
    referee_tx_state_t tx_state = {};

    referee_uart8_rx_init();
    (void)referee_uart8_diag_copy(&rx_log_state.last_diag);
    rx_log_state.task_start_ms = HAL_GetTick();
    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        const TickType_t loop_start_tick = xTaskGetTickCount();
        referee_loop_ctx_t ctx{};
        ctx.now_ms = HAL_GetTick();

        referee_service_rx(&ctx);
        referee_service_logs(&ctx, &rx_log_state, &tx_state);
        referee_service_tx(&ctx, &tx_state);

        if ((xTaskGetTickCount() - loop_start_tick) > loop_ticks) {
            referee_uart8_diag_mutate([](referee_uart8_diag_t& diag) {
                diag.tx_loop_overrun++;
            });
            last_wake_tick = xTaskGetTickCount();
            continue;
        }

        vTaskDelayUntil(&last_wake_tick, loop_ticks);
    }
}
