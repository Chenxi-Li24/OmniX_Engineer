//
// lib_ins.c  —— IMU-based INS + Mahony wrapper
//

//特殊标记除外 默认使用rad进行计算

#include "lib_ins.h"
#include "mahony_filter.h"
#include <math.h>
#include <string.h>

#ifndef DEG2RAD_LOCAL
#define DEG2RAD_LOCAL   ((float)M_PI / 180.0f)
#endif

INS_t INS = {0};

/* ---------- Local helpers ---------- */

static inline float clampf_local(float x, float lo, float hi)
{
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return x;
}

static inline float wrap_pi(float x)
{
    /* wrap to (-pi, pi] */
    while (x >  (float)M_PI)     x -= 2.0f * (float)M_PI;
    while (x <= -(float)M_PI)    x += 2.0f * (float)M_PI;
    return x;
}

/* ---------- Internal static state ---------- */

/* INS state exposed via getter */
static lib_ins_state_t s_ins;

/* Mahony filter internal state */
static struct MAHONY_FILTER_t s_mahony;

/* Axis3f type is usually defined in mahony_filter.h */
static Axis3f s_gyro_axis;
static Axis3f s_accel_axis;

/* Gravity vector in earth frame (这里简化为 +Z 方向重力) */
static float s_gravity_ef[3] = {0.0f, 0.0f, 9.81f};

/* Accel low-pass parameter (same as your old INS.AccelLPF) */
static float s_accel_lpf = 0.0089f;

/* 收敛时间（秒）：在这段时间内处于 CONV 状态，不输出有效 R/P/Y_out，不积分 v/x */
static float s_warmup_time_s = 2.0f;

/* Internal time accumulator */
static float s_time_acc_s = 0.0f;

/* For yaw multi-turn integration */
static int   s_yaw_round_count = 0;
static float s_yaw_last_raw    = 0.0f;

/* 姿态偏置：从 CONV -> RUN 的瞬间记录，用于输出 “相对姿态” */
static float s_roll_bias  = 0.0f;
static float s_pitch_bias = 0.0f;
static float s_yaw_bias   = 0.0f;

/* ---------- Quaternion frame transforms ---------- */

/**
 * @brief Transform 3D vector from BodyFrame to EarthFrame using quaternion.
 *
 * @param vecBF  Input vector in body frame.
 * @param vecEF  Output vector in earth frame.
 * @param q      Quaternion [w, x, y, z].
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, const float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief Transform 3D vector from EarthFrame to BodyFrame using quaternion.
 *
 * @param vecEF  Input vector in earth frame.
 * @param vecBF  Output vector in body frame.
 * @param q      Quaternion [w, x, y, z].
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, const float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/* ---------- Public APIs ---------- */

void LIB_INS_Init(void)
{
    memset(&s_ins, 0, sizeof(s_ins));

    s_ins.q[0] = 1.0f;
    s_ins.q[1] = 0.0f;
    s_ins.q[2] = 0.0f;
    s_ins.q[3] = 0.0f;

    s_ins.app_state = LIB_INS_APP_STATE_CONV;   /* 一上来就认为在收敛阶段 */

    s_time_acc_s       = 0.0f;
    s_yaw_round_count  = 0;
    s_yaw_last_raw     = 0.0f;

    s_roll_bias  = 0.0f;
    s_pitch_bias = 0.0f;
    s_yaw_bias   = 0.0f;

    /* Mahony init (kp, ki, dt_init) — 保持和之前一致 */
    mahony_init(&s_mahony, 1.0f, 0.0f, 0.001f);

    /* 预设输出姿态为 0，避免上层看到垃圾值 */
    s_ins.roll_out  = 0.0f;
    s_ins.pitch_out = 0.0f;
    s_ins.yaw_out   = 0.0f;
    s_ins.yaw_total = 0.0f;
}

void LIB_INS_Reset(void)
{
    LIB_INS_Init();
}

void LIB_INS_SetWarmupTime(float warmup_s)
{
    if (warmup_s < 0.1f)
        warmup_s = 0.1f;
    if (warmup_s > 30.0f)
        warmup_s = 30.0f;
    s_warmup_time_s = warmup_s;
}

/* 在当前姿态上重新定义零点 */
void LIB_INS_Recenter(void)
{
    /* 将当前原始姿态当作新的 reference */
    s_roll_bias  = s_ins.roll;
    s_pitch_bias = s_ins.pitch;
    s_yaw_bias   = s_ins.yaw;

    /* 同时重置 yaw 多圈、速度、位移 */
    s_yaw_round_count = 0;
    s_yaw_last_raw    = s_ins.yaw;
    s_ins.v_n         = 0.0f;
    s_ins.x_n         = 0.0f;
    s_ins.yaw_total   = 0.0f;
}

void LIB_INS_Update(const imu_sample_t *imu, float dt_s)
{
    if (!imu || dt_s <= 0.0f)
    {
        return;
    }

    /* ===== Debug: 纯陀螺积分 yaw（rad） ===== */
    {
        static float s_yaw_gyro_int = 0.0f;
        const float DEG2RAD = 0.017453292519943295f;

        /* 你现在 samp.gyro[] 是 deg/s，这里转一次 rad/s */
        float gz_rad = imu->gyro[2] * DEG2RAD;
        s_yaw_gyro_int += gz_rad * dt_s;

        s_ins.debug_yaw_gyro = s_yaw_gyro_int;
    }

    s_time_acc_s += dt_s;

    /* 1. Copy IMU data into internal state (body frame) */

    /* accel in m/s^2 */
    /* accel in m/s^2 */
    s_ins.accel_b[0] = imu->accel[0];
    s_ins.accel_b[1] = imu->accel[1];
    s_ins.accel_b[2] = imu->accel[2];

    /* gyro in deg/s (for logging / external use) */
    s_ins.gyro_b[0] = imu->gyro[0];
    s_ins.gyro_b[1] = imu->gyro[1];
    s_ins.gyro_b[2] = imu->gyro[2];

    /* 2. Feed Mahony filter (require gyro in rad/s) */

    s_gyro_axis.x = s_ins.gyro_b[0] * DEG2RAD_LOCAL;
    s_gyro_axis.y = s_ins.gyro_b[1] * DEG2RAD_LOCAL;
    s_gyro_axis.z = s_ins.gyro_b[2] * DEG2RAD_LOCAL;

    s_accel_axis.x = s_ins.accel_b[0];
    s_accel_axis.y = s_ins.accel_b[1];
    s_accel_axis.z = s_ins.accel_b[2];

    s_mahony.dt = dt_s;
    mahony_input(&s_mahony, s_gyro_axis, s_accel_axis);
    mahony_update(&s_mahony);
    mahony_output(&s_mahony);
    RotationMatrix_update(&s_mahony);

    /* 3. Update quaternion & raw euler (rad) */

    s_ins.q[0] = s_mahony.q0;
    s_ins.q[1] = s_mahony.q1;
    s_ins.q[2] = s_mahony.q2;
    s_ins.q[3] = s_mahony.q3;

    s_ins.roll  = s_mahony.roll;
    s_ins.pitch = s_mahony.pitch;
    s_ins.yaw   = s_mahony.yaw;

    /* 4. Gravity compensation and accel low-pass in body frame */

    float gravity_b[3];
    EarthFrameToBodyFrame(s_gravity_ef, gravity_b, s_ins.q);

    for (uint8_t i = 0; i < 3; i++)
    {
        float acc_no_g = s_ins.accel_b[i] - gravity_b[i];
        float alpha    = dt_s / (s_accel_lpf + dt_s);

        s_ins.motion_accel_b[i] =
            acc_no_g * alpha +
            s_ins.motion_accel_b[i] * (1.0f - alpha);
    }

    /* 5. Transform motion accel to earth frame */

    BodyFrameToEarthFrame(s_ins.motion_accel_b, s_ins.motion_accel_n, s_ins.q);

    /* 6. Deadzones */

    if (fabsf(s_ins.motion_accel_n[0]) < 0.02f)
    {
        s_ins.motion_accel_n[0] = 0.0f;
    }
    if (fabsf(s_ins.motion_accel_n[1]) < 0.02f)
    {
        s_ins.motion_accel_n[1] = 0.0f;
    }
    if (fabsf(s_ins.motion_accel_n[2]) < 0.04f)
    {
        s_ins.motion_accel_n[2] = 0.0f;
    }

    /* 7. 状态机：从 CONV -> RUN 的切换逻辑 */

    if (s_ins.app_state == LIB_INS_APP_STATE_CONV)
    {
        /* 简单版：只根据时间判断收敛
         * 如果你想更严谨，可以在这里再加：
         *  - motion_accel_n 的范数很小
         *  - roll/pitch/yaw 变化率很小 等等
         */
        if (s_time_acc_s >= s_warmup_time_s)
        {
            /* 在进入 RUN 的瞬间，将当前姿态视作零点 */
            s_roll_bias  = s_ins.roll;
            s_pitch_bias = s_ins.pitch;
            s_yaw_bias   = s_ins.yaw;

            s_yaw_round_count = 0;
            s_yaw_last_raw    = s_ins.yaw;

            s_ins.v_n       = 0.0f;
            s_ins.x_n       = 0.0f;
            s_ins.yaw_total = 0.0f;

            s_ins.app_state = LIB_INS_APP_STATE_RUN;
        }
    }

    /* 8. 根据当前 app_state 决定对外 R/P/Y_out & v/x/yaw_total 的行为 */

    if (s_ins.app_state == LIB_INS_APP_STATE_RUN)
    {
        /* Roll / pitch outputs in rad (relative to bias) */
        s_ins.roll_out  = s_ins.roll  - s_roll_bias;
        s_ins.pitch_out = s_ins.pitch - s_pitch_bias;

        /* Yaw relative to bias, in rad */
        float yaw_rel = wrap_pi(s_ins.yaw - s_yaw_bias);

        /* Multi-turn tracking in raw yaw space */
        float dyaw_raw = s_ins.yaw - s_yaw_last_raw;
        if (dyaw_raw > (float)M_PI)
        {
            s_yaw_round_count--;
        }
        else if (dyaw_raw < -(float)M_PI)
        {
            s_yaw_round_count++;
        }
        s_yaw_last_raw = s_ins.yaw;

        float yaw_total_phys =
            2.0f * (float)M_PI * (float)s_yaw_round_count + yaw_rel;

        /* Directly export in rad */
        s_ins.yaw_out   = yaw_rel;
        s_ins.yaw_total = yaw_total_phys;

        /* Velocity / position integration (unchanged) */
        s_ins.v_n += s_ins.motion_accel_n[1] * dt_s;
        s_ins.x_n += s_ins.v_n * dt_s;
    }
    else
    {
        s_ins.roll_out  = 0.0f;
        s_ins.pitch_out = 0.0f;
        s_ins.yaw_out   = 0.0f;
    }
}

const lib_ins_state_t *LIB_INS_GetState(void)
{
    return &s_ins;
}