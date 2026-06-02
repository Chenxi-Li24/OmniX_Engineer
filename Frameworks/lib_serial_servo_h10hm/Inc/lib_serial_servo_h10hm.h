#ifndef LIB_SERIAL_SERVO_H10HM_H
#define LIB_SERIAL_SERVO_H10HM_H

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_SERVO_H10HM_OK                          0
#define SERIAL_SERVO_H10HM_ERR_ARG                     -1
#define SERIAL_SERVO_H10HM_ERR_LOCK                    -2
#define SERIAL_SERVO_H10HM_ERR_TX                      -3
#define SERIAL_SERVO_H10HM_ERR_RX_TIMEOUT              -4
#define SERIAL_SERVO_H10HM_ERR_CHECKSUM                -5
#define SERIAL_SERVO_H10HM_ERR_LEN                     -6
#define SERIAL_SERVO_H10HM_ERR_ID                      -7
#define SERIAL_SERVO_H10HM_ERR_CMD                     -8

#define SERIAL_SERVO_H10HM_BROADCAST_ID               0xFEu
#define SERIAL_SERVO_H10HM_FRAME_HEADER               0xFFu
#define SERIAL_SERVO_H10HM_DEFAULT_BAUD               1000000u
#define SERIAL_SERVO_H10HM_POSITION_MAX               4095u

#ifndef SERIAL_SERVO_H10HM_RX_USE_DMA
#define SERIAL_SERVO_H10HM_RX_USE_DMA                 1u
#endif

#ifndef SERIAL_SERVO_H10HM_RX_DMA_BUF_SIZE
#define SERIAL_SERVO_H10HM_RX_DMA_BUF_SIZE            64u
#endif

#if defined(__GNUC__)
#define SERIAL_SERVO_H10HM_DMA_ALIGN32                __attribute__((aligned(32)))
#else
#define SERIAL_SERVO_H10HM_DMA_ALIGN32
#endif

#ifndef SERIAL_SERVO_H10HM_INTER_FRAME_GAP_MS
#define SERIAL_SERVO_H10HM_INTER_FRAME_GAP_MS         1u
#endif

#ifndef SERIAL_SERVO_H10HM_DIAG_ENABLE
#define SERIAL_SERVO_H10HM_DIAG_ENABLE                0u
#endif

#ifndef SERIAL_SERVO_H10HM_DIAG_HEX_DUMP_MAX
#define SERIAL_SERVO_H10HM_DIAG_HEX_DUMP_MAX          32u
#endif

#ifndef SERIAL_SERVO_H10HM_DIAG_STATS_WINDOW
#define SERIAL_SERVO_H10HM_DIAG_STATS_WINDOW          1000u
#endif

#define SERIAL_SERVO_H10HM_INST_PING                  0x01u
#define SERIAL_SERVO_H10HM_INST_READ                  0x02u
#define SERIAL_SERVO_H10HM_INST_WRITE                 0x03u
#define SERIAL_SERVO_H10HM_INST_REG_WRITE             0x04u
#define SERIAL_SERVO_H10HM_INST_ACTION                0x05u
#define SERIAL_SERVO_H10HM_INST_RESET                 0x06u
#define SERIAL_SERVO_H10HM_INST_SYNC_READ             0x82u
#define SERIAL_SERVO_H10HM_INST_SYNC_WRITE            0x83u

/* Registers follow HX-10HM protocol examples and HX-35HM-compatible control table. */
#define SERIAL_SERVO_H10HM_REG_ID                     0x05u
#define SERIAL_SERVO_H10HM_REG_MIN_ANGLE_LIMIT_L      0x09u
#define SERIAL_SERVO_H10HM_REG_MAX_ANGLE_LIMIT_L      0x0Bu
#define SERIAL_SERVO_H10HM_REG_TEMP_LIMIT             0x0Du
#define SERIAL_SERVO_H10HM_REG_MAX_VIN                0x0Eu
#define SERIAL_SERVO_H10HM_REG_MIN_VIN                0x0Fu
#define SERIAL_SERVO_H10HM_REG_LED_ERROR              0x14u
#define SERIAL_SERVO_H10HM_REG_OFFSET_L               0x1Fu
#define SERIAL_SERVO_H10HM_REG_MODE                   0x21u
#define SERIAL_SERVO_H10HM_REG_TORQUE_ENABLE          0x28u
#define SERIAL_SERVO_H10HM_REG_ACC                    0x29u
#define SERIAL_SERVO_H10HM_REG_GOAL_POSITION_L        0x2Au
#define SERIAL_SERVO_H10HM_REG_GOAL_TIME_L            0x2Cu
#define SERIAL_SERVO_H10HM_REG_GOAL_SPEED_L           0x2Eu
#define SERIAL_SERVO_H10HM_REG_LOCK                   0x37u
#define SERIAL_SERVO_H10HM_REG_PRESENT_POSITION_L     0x38u
#define SERIAL_SERVO_H10HM_REG_PRESENT_SPEED_L        0x3Au
#define SERIAL_SERVO_H10HM_REG_PRESENT_LOAD_L         0x3Cu
#define SERIAL_SERVO_H10HM_REG_PRESENT_VIN            0x3Eu
#define SERIAL_SERVO_H10HM_REG_PRESENT_TEMP           0x3Fu
/* LED/current related offsets are kept as inferred defaults until full table is confirmed. */
#define SERIAL_SERVO_H10HM_REG_LED_CTRL               0x41u
#define SERIAL_SERVO_H10HM_REG_MOVING                 0x42u
#define SERIAL_SERVO_H10HM_REG_PRESENT_CURRENT_L      0x45u

typedef enum {
  SERIAL_SERVO_H10HM_MODE_POSITION = 0,
  SERIAL_SERVO_H10HM_MODE_MOTOR = 1
} serial_servo_h10hm_mode_t;

typedef struct {
  UART_HandleTypeDef* huart;
  uint32_t timeout_ms;
  osMutexId_t lock;
  osSemaphoreId_t rx_sem;
  volatile uint16_t rx_size;
  volatile uint8_t rx_ready;
  volatile uint8_t rx_dma_active;
  volatile uint8_t rx_error;
  volatile uint8_t rx_dma_guard_head;
  uint8_t rx_dma_buf[SERIAL_SERVO_H10HM_RX_DMA_BUF_SIZE] SERIAL_SERVO_H10HM_DMA_ALIGN32;
  volatile uint8_t rx_dma_guard_tail;
#if (SERIAL_SERVO_H10HM_DIAG_ENABLE == 1u)
  volatile int16_t diag_last_err;
  volatile uint16_t diag_last_rx_size;
  volatile uint8_t diag_last_rx_ready;
  volatile uint8_t diag_last_rx_error;
  volatile uint8_t diag_last_rx_dma_active;
  volatile uint8_t diag_last_hdr_hits;
  volatile int16_t diag_last_hdr_index;
  volatile uint32_t diag_last_dump_tick;
  volatile uint32_t diag_err_timeout;
  volatile uint32_t diag_err_checksum;
  volatile uint32_t diag_err_len;
  volatile uint32_t diag_err_id;
  volatile uint32_t diag_err_cmd;
#endif
} serial_servo_h10hm_t;

typedef struct {
  uint8_t id;
  int16_t position;
  int8_t deviation;
  uint16_t angle_limit_min;
  uint16_t angle_limit_max;
  uint16_t vin_limit_min;
  uint16_t vin_limit_max;
  uint16_t vin_mv;
  uint8_t temp_limit;
  uint8_t temp_c;
  uint8_t load_unload;
  serial_servo_h10hm_mode_t mode;
  int16_t motor_speed;
  uint16_t move_time_pos;
  uint16_t move_time_ms;
  uint16_t wait_time_pos;
  uint16_t wait_time_ms;
  uint8_t led_ctrl;
  uint8_t led_error;
  uint16_t distance;
} serial_servo_h10hm_info_t;

int serial_servo_h10hm_init(serial_servo_h10hm_t* self, UART_HandleTypeDef* huart, uint32_t timeout_ms);

int serial_servo_h10hm_read_id(serial_servo_h10hm_t* self, uint8_t query_id, uint8_t* out_id);
int serial_servo_h10hm_write_id(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t new_id);
int serial_servo_h10hm_set_position(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t position, uint16_t duration_ms);
int serial_servo_h10hm_move_time_read(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t* position, uint16_t* duration_ms);
int serial_servo_h10hm_move_time_wait_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t position, uint16_t duration_ms);
int serial_servo_h10hm_move_time_wait_read(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t* position, uint16_t* duration_ms);
int serial_servo_h10hm_move_start(serial_servo_h10hm_t* self, uint8_t servo_id);
int serial_servo_h10hm_move_stop(serial_servo_h10hm_t* self, uint8_t servo_id);
int serial_servo_h10hm_read_position(serial_servo_h10hm_t* self, uint8_t servo_id, int16_t* position);
int serial_servo_h10hm_angle_offset_adjust(serial_servo_h10hm_t* self, uint8_t servo_id, int8_t offset);
int serial_servo_h10hm_angle_offset_write(serial_servo_h10hm_t* self, uint8_t servo_id);
int serial_servo_h10hm_read_deviation(serial_servo_h10hm_t* self, uint8_t servo_id, int8_t* deviation);
int serial_servo_h10hm_angle_limit_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t min, uint16_t max);
int serial_servo_h10hm_read_angle_limit(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t* min, uint16_t* max);
int serial_servo_h10hm_vin_limit_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t min_mv, uint16_t max_mv);
int serial_servo_h10hm_read_vin_limit(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t* min_mv, uint16_t* max_mv);
int serial_servo_h10hm_read_vin(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t* vin_mv);
int serial_servo_h10hm_temp_limit_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t temp_limit);
int serial_servo_h10hm_read_temp_limit(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t* temp_limit);
int serial_servo_h10hm_read_temp(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t* temp_c);
int serial_servo_h10hm_load_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t load_unload);
int serial_servo_h10hm_read_load(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t* load_unload);
int serial_servo_h10hm_led_ctrl_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t led_ctrl);
int serial_servo_h10hm_led_ctrl_read(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t* led_ctrl);
int serial_servo_h10hm_led_error_write(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t led_error);
int serial_servo_h10hm_led_error_read(serial_servo_h10hm_t* self, uint8_t servo_id, uint8_t* led_error);
int serial_servo_h10hm_read_distance(serial_servo_h10hm_t* self, uint8_t servo_id, uint16_t* distance);

int serial_servo_h10hm_set_mode_position(serial_servo_h10hm_t* self, uint8_t servo_id);
int serial_servo_h10hm_set_mode_motor(serial_servo_h10hm_t* self, uint8_t servo_id, int16_t speed);
int serial_servo_h10hm_read_mode(serial_servo_h10hm_t* self, uint8_t servo_id, serial_servo_h10hm_mode_t* mode, int16_t* speed);

int serial_servo_h10hm_read_all_info(serial_servo_h10hm_t* self, uint8_t servo_id, serial_servo_h10hm_info_t* info);
float serial_servo_h10hm_position_to_deg(int16_t position);

#ifdef __cplusplus
}
#endif

#endif




