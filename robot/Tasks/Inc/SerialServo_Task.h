#ifndef SERIAL_SERVO_TASK_H
#define SERIAL_SERVO_TASK_H

#include <stdint.h>

#include "Servo_Joint_Angle_Protocol.h"


#define SERIAL_SERVO_POLL_MS 10u

#define SERVO_ID_MIN 1u
#define SERVO_ID_MAX 7u
#define SERVO_COUNT (SERVO_ID_MAX - SERVO_ID_MIN + 1u)

#define SERVO_UART6_ID_MIN 1u
#define SERVO_UART6_ID_MAX 3u

#define SERVO_UART9_ID_MIN 4u
#define SERVO_UART9_ID_MAX 7u

#define SERVO_J1_ZERO_RAW OMNIX_CTRL_J1_RAW_ZERO
#define SERVO_J2_ZERO_RAW OMNIX_CTRL_J2_RAW_ZERO
#define SERVO_J3_ZERO_RAW OMNIX_CTRL_J3_RAW_ZERO
#define SERVO_J4_ZERO_RAW OMNIX_CTRL_J4_RAW_ZERO
#define SERVO_J5_ZERO_RAW OMNIX_CTRL_J5_RAW_ZERO
#define SERVO_J6_ZERO_RAW OMNIX_CTRL_J6_RAW_ZERO
#define SERVO_J7_ZERO_RAW OMNIX_CTRL_J7_RAW_ZERO

#define SERVO_ZERO_RAW_BY_INDEX_INITIALIZER \
  SERVO_J1_ZERO_RAW, \
  SERVO_J2_ZERO_RAW, \
  SERVO_J3_ZERO_RAW, \
  SERVO_J4_ZERO_RAW, \
  SERVO_J5_ZERO_RAW, \
  SERVO_J6_ZERO_RAW, \
  SERVO_J7_ZERO_RAW

#ifndef SERVO_DIAG_ENABLE
#define SERVO_DIAG_ENABLE 0u
#endif

#ifndef SERVO_DIAG_SCAN_MAX_ID
#define SERVO_DIAG_SCAN_MAX_ID SERVO_ID_MAX
#endif

#ifndef SERVO_DIAG_TARGET_ID
#define SERVO_DIAG_TARGET_ID SERVO_ID_MIN
#endif

#ifndef SERVO_DIAG_MOVE_TEST_ENABLE
#define SERVO_DIAG_MOVE_TEST_ENABLE 0u
#endif

typedef enum {
    SERVO_BOOT_WAIT_ONLINE = 0u,
    SERVO_BOOT_RESTORING = 1u,
    SERVO_BOOT_HOLDING = 2u,
    SERVO_BOOT_DONE = 3u
} serial_servo_boot_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void Start_SerialServo_Task(void *argument);
void Start_SerialServo_Diag_Task(void *argument);
extern volatile uint8_t g_servo_online_count;
extern volatile serial_servo_boot_state_t g_servo_boot_state;

#ifdef __cplusplus
}
#endif

#endif
