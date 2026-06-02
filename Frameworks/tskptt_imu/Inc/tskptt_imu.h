//
// Created by sirin on 2026/1/13.
//

#ifndef H723VG_V2_FREERTOS_SENTRY_TSKPTT_IMU_H
#define H723VG_V2_FREERTOS_SENTRY_TSKPTT_IMU_H

#include <stdint.h>
#include <stdbool.h>

#include "lib_imu.h"
#include "lib_adp_omximu.h"

typedef enum
{
    IMU_APP_STATE_BOOT = 0,      /* just booted, not heating yet */
    IMU_APP_STATE_WARMUP,        /* heating up */
    IMU_APP_STATE_CALIB,         /* calibrating (static / turntable) */
    IMU_APP_STATE_CONV,          /* converging */
    IMU_APP_STATE_RUNNING,       /* normal running */
    IMU_APP_STATE_FAULT          /* fault (sensor / overtemp etc.) */
} imu_app_state_t;

/* Global IMU state for other tasks to read */
extern volatile imu_app_state_t g_imu_app_state;

/* Optional: expose current calibration mode if needed */
extern const uint8_t g_imu_calib_mode;

typedef struct {
    imu_app_state_t state;
    uint8_t         calib_mode;
    uint8_t         temp_stable;
    uint8_t         calib_done;
    float           current_temp;
} imu_task_status_t;

typedef struct {
    float    accel[3];
    float    gyro[3];
    float    quat[4];
    float    euler[3];
    uint32_t last_update_ms;
    uint8_t  online;
    uint8_t  valid;
} imu_sensor_state_t;

typedef struct {
    imu_sensor_state_t internal;
    imu_sensor_state_t external;
} imu_task_snapshot_t;

typedef struct {
    float internal_euler_deg[3];
    float external_euler_deg[3];
} imu_euler_debug_t;

extern volatile imu_euler_debug_t g_imu_euler_debug;

void TSKPTT_IMU_Task(void *argument);
bool IMU_GetOmxImuState(OmxImuState* out);
bool IMU_GetSnapshot(imu_task_snapshot_t* out);

#endif //H723VG_V2_FREERTOS_SENTRY_TSKPTT_IMU_H
