//
// lib_ins.h  —— IMU-based INS + Mahony wrapper
//

#ifndef LIB_INS_H
#define LIB_INS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lib_imu.h"        // for imu_sample_t

/* INS 对外状态机：主要是给上层用来判断能不能信任姿态 */
typedef enum
{
    LIB_INS_APP_STATE_BOOT = 0,   /* 刚上电 / 未初始化完 */
    LIB_INS_APP_STATE_CONV,       /* Mahony/INS 正在收敛，输出还不可信 */
    LIB_INS_APP_STATE_RUN         /* 已收敛，输出可用于控制 */
} lib_ins_app_state_t;

/* INS 状态结构，对外只读 */
typedef struct
{
    /* 机体系原始测量（已经是物理单位） */
    float accel_b[3];          /* m/s^2 */
    float gyro_b[3];           /* deg/s（按你现在的 IMU 输出设定） */

    /* 去重力 + 滤波后的加速度 */
    float motion_accel_b[3];   /* body frame, m/s^2 */
    float motion_accel_n[3];   /* earth frame, m/s^2 */

    /* 姿态四元数：q = [w, x, y, z] */
    float q[4];

    /* 原始 Mahony 解出的姿态（rad）——调试用 */
    float roll;                /* rad */
    float pitch;               /* rad */
    float yaw;                 /* rad */

    /* 给上层控制/平衡用的姿态（rad）
     * - 在 CONV 阶段强制为 0，不影响控制
     * - 进入 RUN 的瞬间，将当前 roll/pitch/yaw 记为 bias，并从此开始输出相对姿态
     */
    float roll_out;            /* rad */
    float pitch_out;           /* rad */
    float yaw_out;             /* rad */

    /* 基于 yaw_out 的多圈积分（rad） */
    float yaw_total;

    /* 一维速度/位移（沿 earth-frame Y 轴），目前保留和原项目兼容 */
    float v_n;
    float x_n;

    /* 当前 INS 应用状态 */
    lib_ins_app_state_t app_state;

    /* 调试用 */
    float debug_yaw_gyro;

} lib_ins_state_t;

typedef struct
{
    /* Raw IMU data in body frame */
    float Accel[3];          /* body-frame accelerometer (m/s^2) */
    float Gyro[3];           /* body-frame gyro (rad/s) */

    /* Attitude quaternion (if you want to fill it later) */
    float q[4];              /* q0, q1, q2, q3 */

    /* Linear acceleration (gravity removed) */
    float MotionAccel_b[3];  /* body-frame motion acceleration (m/s^2) */
    float MotionAccel_n[3];  /* earth-frame motion acceleration (m/s^2) */

    /* Euler angles */
    float Roll;              /* roll angle (rad) */
    float Pitch;             /* pitch angle (rad) */
    float Yaw;               /* yaw angle (rad, wrapped -pi~pi) */

    /* Continuous yaw handling */
    float   YawTotalAngle;   /* continuous yaw (rad) */
    float   YawAngleLast;    /* last yaw (rad) */
    int32_t YawRoundCount;   /* number of 2*pi wraps */

    /* Simple 1D navigation along n-frame Y axis */
    float v_n;               /* velocity in n-frame Y (m/s) */
    float x_n;               /* position in n-frame Y (m) */

    /* Accel low-pass time constant */
    float AccelLPF;          /* low-pass time constant for accel */

    /* INS ready flag (1 = RUN, 0 = not ready) */
    uint8_t ins_flag;
} INS_t;

/* Global INS instance exposed to other modules */
extern INS_t INS;


/**
 * @brief  初始化 INS（内部会初始化 Mahony、清零状态等）
 */
void LIB_INS_Init(void);

/**
 * @brief  软复位（等价于重新 Init）
 */
void LIB_INS_Reset(void);

/**
 * @brief  每次有一帧 IMU 数据时调用，更新 INS 状态
 *
 * @param imu   当前一帧 IMU 采样（来自 lib_imu）
 * @param dt_s  本次与上次 Update 的时间间隔（秒）
 */
void LIB_INS_Update(const imu_sample_t *imu, float dt_s);

/**
 * @brief  设置 INS 收敛时间（秒）
 *
 * 在这段时间内，INS 处于 CONV 状态：
 *  - Mahony 会正常收敛
 *  - 但 roll_out/pitch_out/yaw_out 都保持 0
 *  - 不积分 v_n/x_n/yaw_total
 * 到了这个时间点，自动进入 RUN 状态，将当前姿态记为零点。
 */
void LIB_INS_SetWarmupTime(float warmup_s);

/**
 * @brief  强制以当前姿态为新的 0 姿态（重新对齐）
 *
 * 通常在：
 *  - 机器人已经站稳，你想把当前姿态当成新的“直立 = 0” 的时候使用。
 */
void LIB_INS_Recenter(void);

/**
 * @brief  获取 INS 当前状态指针（只读）
 */
const lib_ins_state_t *LIB_INS_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* LIB_INS_H */