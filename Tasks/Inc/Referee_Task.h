#ifndef H723VG_V2_FREERTOS_REFEREE_TASK_H
#define H723VG_V2_FREERTOS_REFEREE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "Servo_Joint_Angle_Protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REFEREE_UART8_TX_DBG_STAGE_IDLE             0u
#define REFEREE_UART8_TX_DBG_STAGE_WAIT_DUE         1u
#define REFEREE_UART8_TX_DBG_STAGE_NOT_READY        2u
#define REFEREE_UART8_TX_DBG_STAGE_FRAME_BUILD_FAIL 3u
#define REFEREE_UART8_TX_DBG_STAGE_DMA_OK           4u
#define REFEREE_UART8_TX_DBG_STAGE_DMA_BUSY         5u
#define REFEREE_UART8_TX_DBG_STAGE_DMA_ERR          6u

typedef struct {
    uint32_t call_count;
    uint32_t last_call_ms;
    uint32_t last_payload_tick_ms;
    uint32_t last_next_due_ms;
    uint32_t last_lateness_ms;
    uint32_t last_missed_slots;
    uint32_t last_uart_gstate;
    uint32_t last_dma_state;
    uint32_t last_stage;
    uint32_t last_hal_ret;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_err_count;
    uint32_t tx_not_ready_count;
    uint32_t frame_build_fail_count;
    uint16_t last_seq;
    uint8_t last_sof;
    uint8_t last_crc8;
    uint16_t last_cmd_id;
    uint16_t last_crc16;
} referee_uart8_tx_debug_watch_t;

extern volatile referee_uart8_tx_debug_watch_t g_referee_uart8_tx_debug;

void Start_Referee_Task(void *argument);

void Referee_PublishJointRawCmd(const omnix_custom_controller_raw_payload_t* payload);
bool Referee_CopyJointRawCmd(omnix_custom_controller_raw_payload_t* out);
bool Referee_CopyJointRawCmdBytes(uint8_t* out_buf, uint16_t out_buf_len, uint16_t* out_len);
bool Referee_CopyCustomControllerData0302(uint8_t out_data[30]);
bool Referee_ReadRobotJointRawFeedback(omnix_robot_joint_feedback_payload_t* out);
bool Referee_IsMappingResetRequested(void);
void Referee_SetMappingResetDone(bool done);

#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_REFEREE_TASK_H
