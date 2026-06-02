//
// lib_imu.cpp
//
#include "lib_imu.h"
#include "pid.h"
#include "bsp_bmi088.h"   // 提供 bmi088_data_t / BSP_BMI088_* API
#include <math.h>

// === NEW: orientation adapter ===
#include "lib_imu_orientation.h"

using namespace algo;

/* ---------- 小工具 ---------- */

static inline float clamp01_local(float x)
{
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
}

extern "C" {

/* ============================================================
 *  IMU 基础接口（封装 BMI088）
 * ============================================================ */

static uint8_t s_imu_sampling = 0U;   // 0: 空闲，1: 正在等待这次采样完成

imu_status_t IMU_Init(void)
{
    s_imu_sampling = 0U;
    return IMU_STATUS_OK;
}

/**
 * @brief 阻塞版一次读取（内部直接调用 Blocking BMI088）
 *
 * 注意：
 *  - raw.* 是传感器坐标系 S
 *  - 这里统一转换到机体系 B：v_B = R_SB * v_S
 */
imu_status_t IMU_SampleOnceBlocking(imu_sample_t *out)
{
    if (out == nullptr)
    {
        return IMU_STATUS_INVALID_ARG;
    }

    bmi088_data_t raw;
    BSP_BMI088_ReadBlocking(&raw);

    // ==== ORI BEGIN: sensor(S) -> body(B) ====
    float gyro_S[3]  = { raw.gyro[0],  raw.gyro[1],  raw.gyro[2]  };
    float accel_S[3] = { raw.accel[0], raw.accel[1], raw.accel[2] };
    float gyro_B[3];
    float accel_B[3];

    imu_apply_orientation(&g_imu_R_SB, gyro_S,  gyro_B);
    imu_apply_orientation(&g_imu_R_SB, accel_S, accel_B);

    out->gyro[0]  = gyro_B[0];
    out->gyro[1]  = gyro_B[1];
    out->gyro[2]  = gyro_B[2];

    out->accel[0] = accel_B[0];
    out->accel[1] = accel_B[1];
    out->accel[2] = accel_B[2];
    // ==== ORI END ====

    out->temperature = raw.temperature;

    return IMU_STATUS_OK;
}

/**
 * @brief 低层手动控制：仅发起采样（一般不给 Task 直接用）
 */
imu_status_t IMU_BeginSample(void)
{
    bsp_bmi088_status_t st = BSP_BMI088_BeginSample();
    switch (st)
    {
        case BSP_BMI088_OK:   return IMU_STATUS_OK;
        case BSP_BMI088_BUSY: return IMU_STATUS_BUSY;
        default:              return IMU_STATUS_ERR_BMI;
    }
}

/**
 * @brief 低层手动控制：尝试取样（一般不给 Task 直接用）
 *
 * 同样在这里完成 S->B 转换
 */
imu_status_t IMU_GetSampleIfReady(imu_sample_t *out)
{
    if (out == nullptr)
    {
        return IMU_STATUS_INVALID_ARG;
    }

    bmi088_data_t raw;
    bsp_bmi088_status_t st = BSP_BMI088_GetSampleIfReady(&raw);

    if (st == BSP_BMI088_BUSY)
    {
        return IMU_STATUS_BUSY;
    }
    if (st != BSP_BMI088_OK)
    {
        return IMU_STATUS_ERR_BMI;
    }

    // ==== ORI BEGIN: sensor(S) -> body(B) ====
    float gyro_S[3]  = { raw.gyro[0],  raw.gyro[1],  raw.gyro[2]  };
    float accel_S[3] = { raw.accel[0], raw.accel[1], raw.accel[2] };
    float gyro_B[3];
    float accel_B[3];

    imu_apply_orientation(&g_imu_R_SB, gyro_S,  gyro_B);
    imu_apply_orientation(&g_imu_R_SB, accel_S, accel_B);

    out->gyro[0]  = gyro_B[0];
    out->gyro[1]  = gyro_B[1];
    out->gyro[2]  = gyro_B[2];

    out->accel[0] = accel_B[0];
    out->accel[1] = accel_B[1];
    out->accel[2] = accel_B[2];
    // ==== ORI END ====

    out->temperature = raw.temperature;

    return IMU_STATUS_OK;
}

/**
 * @brief 推荐给 Task 用的“完全非阻塞单次采样接口”
 *
 * 同样统一输出为机体系 B
 */
imu_status_t IMU_SampleOnce(imu_sample_t *out)
{
    if (out == nullptr)
    {
        return IMU_STATUS_INVALID_ARG;
    }

    /* 阶段 1：如果当前没有在等一帧采样，则尝试发起一次 */
    if (s_imu_sampling == 0U)
    {
        bsp_bmi088_status_t st = BSP_BMI088_BeginSample();

        if (st == BSP_BMI088_OK)
        {
            s_imu_sampling = 1U;
            return IMU_STATUS_BUSY;
        }
        else if (st == BSP_BMI088_BUSY)
        {
            return IMU_STATUS_BUSY;
        }
        else
        {
            return IMU_STATUS_ERR_BMI;
        }
    }

    /* 阶段 2：已经有一次在飞的采样了，看看是不是 ready 了 */
    bmi088_data_t raw;
    bsp_bmi088_status_t st = BSP_BMI088_GetSampleIfReady(&raw);

    if (st == BSP_BMI088_BUSY)
    {
        return IMU_STATUS_BUSY;
    }

    s_imu_sampling = 0U;

    if (st != BSP_BMI088_OK)
    {
        return IMU_STATUS_ERR_BMI;
    }

    // ==== ORI BEGIN: sensor(S) -> body(B) ====
    float gyro_S[3]  = { raw.gyro[0],  raw.gyro[1],  raw.gyro[2]  };
    float accel_S[3] = { raw.accel[0], raw.accel[1], raw.accel[2] };
    float gyro_B[3];
    float accel_B[3];

    imu_apply_orientation(&g_imu_R_SB, gyro_S,  gyro_B);
    imu_apply_orientation(&g_imu_R_SB, accel_S, accel_B);

    out->gyro[0]  = gyro_B[0];
    out->gyro[1]  = gyro_B[1];
    out->gyro[2]  = gyro_B[2];

    out->accel[0] = accel_B[0];
    out->accel[1] = accel_B[1];
    out->accel[2] = accel_B[2];
    // ==== ORI END ====

    out->temperature = raw.temperature;

    return IMU_STATUS_OK;
}

/* ============================================================
 *  温度控制内部状态
 * ============================================================ */

static imu_temp_ctrl_hw_t s_temp_hw {};
static uint8_t            s_temp_hw_bound = 0U;

static pid_type_def       s_imu_temp_pid {};

static float              s_target_temp_c   = 40.0f;
static float              s_stable_delta_c  = 2.0f;
static uint32_t           s_stable_time_ms  = 5000U;
static uint32_t           s_in_range_start_ms = 0U;

static imu_temp_state_t   s_temp_state      = IMU_TEMP_STATE_COLD;
static uint8_t            s_heater_enable   = 1U;
static uint8_t            s_fault_latched   = 0U;

static float              s_last_temp_c     = 25.0f;

/* internal helpers */

static void IMU_TempCtrl_SetDuty_Internal(float duty)
{
    if (!s_temp_hw_bound)
    {
        return;
    }

    duty = clamp01_local(duty);

    uint32_t ccr = static_cast<uint32_t>(duty * s_temp_hw.max_ccr + 0.5f);
    __HAL_TIM_SET_COMPARE(s_temp_hw.htim, s_temp_hw.channel, ccr);
}

static void IMU_TempCtrl_UpdateState(float current_temp_c)
{
    s_last_temp_c = current_temp_c;

    float err = fabsf(current_temp_c - s_target_temp_c);
    uint32_t now = HAL_GetTick();

    if (err > s_stable_delta_c)
    {
        s_temp_state = IMU_TEMP_STATE_WARMUP;
        s_in_range_start_ms = 0U;
        return;
    }

    if (s_in_range_start_ms == 0U)
    {
        s_in_range_start_ms = now;
        s_temp_state = IMU_TEMP_STATE_WARMUP;
        return;
    }

    uint32_t in_range_time = now - s_in_range_start_ms;
    if (in_range_time >= s_stable_time_ms)
    {
        s_temp_state = IMU_TEMP_STATE_STABLE;
    }
    else
    {
        s_temp_state = IMU_TEMP_STATE_WARMUP;
    }
}

static void IMU_TempCtrl_ControlWithTemp(float current_temp_c)
{
    /* Over-temperature protection */
    if (current_temp_c > s_target_temp_c + 10.0f)
    {
        IMU_TempCtrl_Fault();
        s_last_temp_c = current_temp_c;
        return;
    }

    IMU_TempCtrl_UpdateState(current_temp_c);

    if (!s_temp_hw_bound || s_fault_latched)
    {
        IMU_TempCtrl_SetDuty_Internal(0.0f);
        return;
    }

    if (!s_heater_enable)
    {
        IMU_TempCtrl_SetDuty_Internal(0.0f);
        return;
    }

    /* If already above target, do not heat */
    if (current_temp_c > s_target_temp_c)
    {
        IMU_TempCtrl_SetDuty_Internal(0.0f);
        return;
    }

    /* Cold-start fast heating when far from target */
    float err = fabsf(current_temp_c - s_target_temp_c);
    if ((s_temp_state != IMU_TEMP_STATE_STABLE) && (err > s_stable_delta_c))
    {
        IMU_TempCtrl_SetDuty_Internal(1.0f);
        return;
    }

    /* Fine control by PID near target */
    float pid_out = PID_calc(&s_imu_temp_pid,
                             current_temp_c,
                             s_target_temp_c);

    if (pid_out < 0.0f) pid_out = 0.0f;
    if (pid_out > s_imu_temp_pid.max_out) pid_out = s_imu_temp_pid.max_out;

    float duty = pid_out / s_imu_temp_pid.max_out;
    IMU_TempCtrl_SetDuty_Internal(duty);
}

/* public temp ctrl APIs */

void IMU_TempCtrl_Init(const imu_temp_ctrl_hw_t *hw,
                       float target_temp_c,
                       const float pid_param[3],
                       float max_out,
                       float max_iout,
                       float stable_delta_c,
                       uint32_t stable_time_ms)
{
    if (hw == nullptr || hw->htim == nullptr || hw->max_ccr <= 0.0f)
    {
        s_temp_hw_bound = 0U;
        return;
    }

    s_temp_hw = *hw;
    s_temp_hw_bound = 1U;

    IMU_TempCtrl_SetDuty_Internal(0.0f);
    HAL_TIM_PWM_Start(s_temp_hw.htim, s_temp_hw.channel);

    PID_init(&s_imu_temp_pid,
             PID_DELTA,
             pid_param,
             max_out,
             max_iout);

    s_target_temp_c   = target_temp_c;
    s_stable_delta_c  = (stable_delta_c > 0.0f) ? stable_delta_c : 2.0f;
    s_stable_time_ms  = (stable_time_ms > 0U) ? stable_time_ms : 5000U;

    s_in_range_start_ms = 0U;
    s_temp_state        = IMU_TEMP_STATE_COLD;
    s_heater_enable     = 1U;
    s_fault_latched     = 0U;
}

void IMU_TempCtrl_Step(void)
{
    imu_sample_t sample;
    imu_status_t st = IMU_SampleOnceBlocking(&sample);

    if (st != IMU_STATUS_OK)
    {
        IMU_TempCtrl_Fault();
        return;
    }

    IMU_TempCtrl_ControlWithTemp(sample.temperature);
}

void IMU_TempCtrl_StepWithTemp(float current_temp_c)
{
    IMU_TempCtrl_ControlWithTemp(current_temp_c);
}

imu_temp_state_t IMU_TempCtrl_GetState(void)
{
    return s_temp_state;
}

uint8_t IMU_TempCtrl_IsStable(void)
{
    return (s_temp_state == IMU_TEMP_STATE_STABLE) ? 1U : 0U;
}

void IMU_TempCtrl_SetTarget(float target_temp_c)
{
    s_target_temp_c = target_temp_c;
    s_in_range_start_ms = 0U;
    if (s_temp_state == IMU_TEMP_STATE_STABLE)
    {
        s_temp_state = IMU_TEMP_STATE_WARMUP;
    }
}

void IMU_TempCtrl_SetEnable(uint8_t enable)
{
    s_heater_enable = enable ? 1U : 0U;

    if (!s_heater_enable)
    {
        IMU_TempCtrl_SetDuty_Internal(0.0f);
    }
}

void IMU_TempCtrl_Fault(void)
{
    s_fault_latched = 1U;
    s_heater_enable = 0U;
    IMU_TempCtrl_SetDuty_Internal(0.0f);
}

void IMU_TempCtrl_ClearFault(void)
{
    s_fault_latched = 0U;
}

float IMU_TempCtrl_GetCurrentTemp(void)
{
    return s_last_temp_c;
}

float IMU_TempCtrl_GetTargetTemp(void)
{
    return s_target_temp_c;
}

/* ============================================================
 *        静态校准实现（任意姿态，保持静止即可）
 *        —— 由上层喂样本，不再内部采样
 * ============================================================ */

static imu_calib_state_t  s_calib_state = IMU_CALIB_IDLE;
static imu_calib_result_t s_calib_result {};

static const uint16_t  IMU_CALIB_SAMPLES    = 2000U;    // 1kHz 下约 2s
static const uint32_t  IMU_CALIB_TIMEOUT_MS = 15000U;   // 超时时间
static const float     IMU_CALIB_GNORM_TARGET     = 9.81f;

static uint32_t s_calib_start_ms = 0U;
static uint16_t s_calib_count    = 0U;

static float s_gyro_sum[3] = {0.0f, 0.0f, 0.0f};
static float s_gnorm_sum   = 0.0f;

void IMU_Calib_Start(void)
{
    s_calib_result.gyro_bias[0] = 0.0f;
    s_calib_result.gyro_bias[1] = 0.0f;
    s_calib_result.gyro_bias[2] = 0.0f;
    s_calib_result.accel_scale  = 1.0f;
    s_calib_result.g_norm       = IMU_CALIB_GNORM_TARGET; // 9.81f
    s_calib_result.temp_c       = 25.0f;
    s_calib_result.success      = 0U;

    s_calib_state    = IMU_CALIB_RUNNING;
    s_calib_start_ms = HAL_GetTick();
    s_calib_count    = 0U;

    s_gyro_sum[0]    = 0.0f;
    s_gyro_sum[1]    = 0.0f;
    s_gyro_sum[2]    = 0.0f;
    s_gnorm_sum      = 0.0f;
}

void IMU_Calib_AddSample(const imu_sample_t *samp)
{
    if (s_calib_state != IMU_CALIB_RUNNING)
        return;
    if (!samp)
        return;

    uint32_t now = HAL_GetTick();
    if (now - s_calib_start_ms > IMU_CALIB_TIMEOUT_MS)
    {
        s_calib_state          = IMU_CALIB_FAILED;
        s_calib_result.success = 0U;
        return;
    }

    // 使用已经转换到 Body frame 的样本
    float ax = samp->accel[0];
    float ay = samp->accel[1];
    float az = samp->accel[2];
    float gnorm = sqrtf(ax * ax + ay * ay + az * az);

    s_gnorm_sum   += gnorm;
    s_gyro_sum[0] += samp->gyro[0];
    s_gyro_sum[1] += samp->gyro[1];
    s_gyro_sum[2] += samp->gyro[2];
    s_calib_count++;

    if (s_calib_count < IMU_CALIB_SAMPLES)
        return;

    // 样本足够，算均值
    float gnorm_avg = s_gnorm_sum / (float)IMU_CALIB_SAMPLES;
    float gyro_bias[3] = {
        s_gyro_sum[0] / (float)IMU_CALIB_SAMPLES,
        s_gyro_sum[1] / (float)IMU_CALIB_SAMPLES,
        s_gyro_sum[2] / (float)IMU_CALIB_SAMPLES,
    };

    float accel_scale = 1.0f;
    if (gnorm_avg > 1e-3f)
    {
        accel_scale = IMU_CALIB_GNORM_TARGET / gnorm_avg; // 自动拉回 9.81
    }

    s_calib_result.gyro_bias[0] = gyro_bias[0];
    s_calib_result.gyro_bias[1] = gyro_bias[1];
    s_calib_result.gyro_bias[2] = gyro_bias[2];
    s_calib_result.g_norm       = gnorm_avg;
    s_calib_result.accel_scale  = accel_scale;
    s_calib_result.temp_c       = samp->temperature;
    s_calib_result.success      = 1U;

    s_calib_state = IMU_CALIB_DONE;
}

imu_calib_state_t IMU_Calib_GetState(void)
{
    return s_calib_state;
}

void IMU_Calib_GetResult(imu_calib_result_t *out)
{
    if (!out) return;

    out->gyro_bias[0] = 0.0f;
    out->gyro_bias[1] = 0.0f;
    out->gyro_bias[2] = 0.0f;
    out->accel_scale  = 1.0f;
    out->g_norm       = IMU_CALIB_GNORM_TARGET;
    out->temp_c       = 25.0f;
    out->success      = 0U;

    if (s_calib_state == IMU_CALIB_DONE && s_calib_result.success)
    {
        *out = s_calib_result;
    }
}

} // extern "C"