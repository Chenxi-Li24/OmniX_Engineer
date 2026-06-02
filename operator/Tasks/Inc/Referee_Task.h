#ifndef H723VG_V2_FREERTOS_REFEREE_TASK_H
#define H723VG_V2_FREERTOS_REFEREE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "Servo_Joint_Angle_Protocol.h"

// Macro to temporarily disable all serial port inputs
#define DISABLE_SERIAL_INPUT

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool online;
    bool fresh;
    uint16_t seq;
    uint8_t valid_mask;
    uint16_t raw_u16[OMNIX_JOINT_RAW_COUNT];
    uint8_t diag_flags;
    uint32_t rx_tick_ms;
} referee_joint_raw_cmd_t;

typedef enum {
    REF_CUSTOM_CTRL_RX_SOURCE_NONE = 0,
    REF_CUSTOM_CTRL_RX_SOURCE_VTX_RAW = 1,
    REF_CUSTOM_CTRL_RX_SOURCE_SERVER_BRIDGE = 2
} RefCustomCtrlRxSource;

typedef struct {
    uint32_t bytes;
    uint32_t events;
    uint32_t frames_total;
    uint32_t frames_good;
    uint32_t cmd_0302_good;
    uint32_t crc8_errors;
    uint32_t crc16_errors;
    uint32_t payload_len_errors;
    uint32_t payload_version_errors;
    uint32_t unknown_cmd_frames;
    uint32_t seq_gap_count;
    uint32_t last_good_tick_ms;
    uint16_t last_seq;
    uint8_t last_seq_valid;
    uint8_t consecutive_good_frames;
} referee_custom_rx_path_diag_t;

typedef struct {
    uint32_t total_bytes;
    uint32_t total_frames;
    uint32_t good_frames;
    uint32_t crc8_errors;
    uint32_t crc16_errors;
    uint32_t short_frames;
    uint32_t oversize_frames;
    uint32_t unknown_cmd_frames;
    uint32_t cmd_0302_frames;
    uint32_t cmd_0302_good_frames;
    uint32_t payload_len_errors;
    uint32_t payload_version_errors;
    uint32_t cmd_0302_seq_gap_count;
    uint32_t uart_errors;
    uint32_t fifo_overflows;
    uint32_t last_rx_tick_ms;
    uint16_t last_cmd_id;
    uint16_t last_frame_len;
    uint16_t last_0302_seq;
    uint8_t last_0302_seq_valid;
    uint32_t total_tx_frames;
    uint32_t good_tx_frames;
    uint32_t cmd_0309_tx_frames;
    uint32_t tx_busy;
    uint32_t tx_errors;
    uint32_t last_tx_tick_ms;
    uint16_t last_tx_cmd_id;
} referee_rx_stats_t;

typedef struct {
    uint32_t signature;              /* 'RDBG' */
    uint32_t version;

    uint32_t task_loop_count;
    uint32_t task_last_os_tick;
    uint32_t task_last_ms_tick;

    uint32_t rx_chunk_last_bytes;
    uint32_t rx_chunk_nonzero_loops;
    uint32_t rx_fifo_used;
    uint32_t rx_fifo_used_max_observed;
    uint32_t rx_parse_len;
    uint32_t rx_parse_len_max_observed;

    uint32_t uart9_rx_event_count;
    uint32_t uart9_rx_event_bytes;
    uint32_t uart9_rx_event_invalid_size;
    uint32_t uart9_rx_vt03_feed_events;
    uint32_t uart9_rx_vt03_feed_bytes;
    uint32_t uart9_rx_hw_error_count;
    uint32_t uart9_rx_hw_error_ore_count;
    uint32_t uart9_rx_hw_error_fe_count;
    uint32_t uart9_rx_hw_error_ne_count;

    uint32_t referee_total_bytes;
    uint32_t referee_total_frames;
    uint32_t referee_good_frames;
    uint32_t referee_cmd_0302_frames;
    uint32_t referee_cmd_0302_good_frames;
    uint32_t referee_crc8_errors;
    uint32_t referee_crc16_errors;
    uint32_t referee_payload_len_errors;
    uint32_t referee_payload_version_errors;
    uint32_t referee_unknown_cmd_frames;
    uint32_t referee_fifo_overflows;

    uint32_t cmd_online;
    uint32_t cmd_fresh;
    uint32_t cmd_age_ms;
    uint32_t cmd_rx_tick_ms;
    uint16_t cmd_seq;
    uint8_t cmd_valid_mask;
    uint8_t reserved0;

    uint32_t active_source;
    uint32_t active_source_since_ms;
    uint32_t active_source_frames_good;
    uint32_t active_source_cmd_0302_good;
} referee_debug_probe_t;

extern volatile referee_debug_probe_t g_referee_debug_probe;

void Start_Referee_Task(void *argument);

bool Referee_IngestJointRawPayload(const uint8_t* data, uint16_t len);
bool Referee_IngestCustomCtrlFrameFromBridge(const uint8_t* data, uint16_t len);
bool Referee_ReadJointRawCmd(referee_joint_raw_cmd_t* out);
bool Referee_GetJointRawU16(uint8_t joint_id, uint16_t* out_raw_u16);
bool Referee_ReadRxStats(referee_rx_stats_t* out);
bool Referee_ReadRxRaw0302(uint8_t* out_buf, uint16_t out_len);
bool Referee_IsExternalControlEnabled(void);
bool Referee_IsMappingResetActive(void);
void Referee_RequestMappingReset(void);
void Referee_ClearMappingReset(void);
void Referee_RequestRemoteBuzzerPulses(uint8_t count);
bool Referee_ReadTxRobotJointFeedback(omnix_robot_joint_feedback_payload_t* out);

#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_REFEREE_TASK_H
