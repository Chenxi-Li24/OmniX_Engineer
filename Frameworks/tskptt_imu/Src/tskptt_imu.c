//
// Created by sirin on 2026/1/13.
//
//
// IMU_Task.c —— 完全非阻塞 IMU 采样 + 温控 + 校准 + EEPROM 持久化
//

#include "tskptt_imu.h"
#include "lib_imu.h"
#include "lib_ins.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bsp_srn_log.h"
#include "NTFDCAN_Router.h"
#include "lib_adp_omximu.h"

#include "lib_24c02_bl.h"
#include "bsp_i2c.h"
#include "bsp_bmi088.h"
#include "main.h"

#include <string.h>
#include <math.h>   /* for M_PI */

/* ======================= 配置宏 ======================= */

/* 任务周期：1 kHz */
#define IMU_TASK_PERIOD_MS      1u

#define IMU_LOG_DIV             250u

/* OMX IMU CAN */
#ifndef OMXIMU_CAN_BUS
#define OMXIMU_CAN_BUS           3u
#endif
#ifndef OMXIMU_OFFLINE_MS
#define OMXIMU_OFFLINE_MS        100u
#endif

/* 温控相关参数 */
#define IMU_TARGET_TEMP_C       55.0f
#define IMU_STABLE_DELTA_C      5.0f
#define IMU_STABLE_TIME_MS      2500u

/* 校准模式：
 * 0 = 不重新校准，升温完成后直接从 EEPROM 读校准数据；
 *     若 EEPROM 没有有效数据，则使用默认值。
 * 1 = 静态校准（任意姿态但保持静止），完成后写入 EEPROM。
 * 2 = 转台校准（TODO：此处留空，先用默认值），完成后也可以写 EEPROM。
 */
#define IMU_CALIBRATION_MODE    0u

/* EEPROM 参数 */
#define EEPROM_I2C_ADDR_7BIT      0x50u   /* 典型 24C02 地址，A2~A0 全拉低 */
#define EEPROM_IMU_CALIB_ADDR     0x20u   /* 校准数据起始地址（32 字节空间足够） */

#define IMU_EEPROM_MAGIC          0x434D5549u  /* "IMUC" 反过来写也无所谓，只要唯一 */
#define IMU_EEPROM_VERSION        0x0001u

#ifndef M_PI_F
#define M_PI_F 3.1415926f
#endif

/* ======================= 对外可见状态 ======================= */

/* 其他模块可用 extern 获取 */
volatile imu_app_state_t g_imu_app_state = IMU_APP_STATE_BOOT;

/* 若你需要对外暴露当前校准模式，也可以这样： */
const uint8_t g_imu_calib_mode = IMU_CALIBRATION_MODE;

/* ======================= 句柄与全局变量 ======================= */

QueueHandle_t g_imuQueue = NULL;     /* 之后给 EKF / 控制任务用 */

extern TIM_HandleTypeDef htim3;      /* PWM 加热：TIM3_CH4 */
extern I2C_HandleTypeDef hi2c4;      /* EEPROM：I2C4 */

/* 外置 CAN IMU（OMX） */
static OmxImu s_omx_imu;
static OmxImuState s_omx_cache;
static volatile uint32_t s_omx_cache_seq = 0;

/* 统一接口快照（internal/external） */
static imu_task_snapshot_t s_imu_snapshot = {0};
static volatile uint32_t s_imu_snapshot_seq = 0;

/* 简化调试：只暴露欧拉角（角度制） */
volatile imu_euler_debug_t g_imu_euler_debug = {0};

/* ====== 兼容老 INS 结构：对外暴露姿态 / 加速度等 ====== */
/* NOTE: INS_t 与你之前发的 ins_task.c 中保持一致 */
extern INS_t INS;

/* Blocking 版 24C02 句柄 + 最近状态 */
static lib_24c02_bl_t        s_eeprom;
static lib_24c02_bl_status_t s_eeprom_status = LIB_24C02_BL_ERROR;

/* 校准数据在 EEPROM 中的持久化格式 */
typedef struct
{
    uint32_t magic;          /* IMU_EEPROM_MAGIC */
    uint16_t version;        /* IMU_EEPROM_VERSION */
    uint16_t reserved;

    float gyro_bias[3];
    float accel_scale;
    float g_norm;
    float temp_c;
} imu_calib_persist_t;

/* 当前生效的校准结果（来自 EEPROM 或本次校准） */
static imu_calib_result_t s_calib_result;

/* 是否已经完成“校准处理”阶段（不论是否真的跑过校准） */
static uint8_t s_calib_done    = 0u;
static uint8_t s_calib_started = 0u;

/* ======================= EEPROM 工具函数 ======================= */

static lib_24c02_bl_status_t imu_eeprom_write_calib(const imu_calib_persist_t *p)
{
    if (!p)
        return LIB_24C02_BL_INVALID_PARAM;

    if (s_eeprom_status != LIB_24C02_BL_OK)
        return LIB_24C02_BL_ERROR;

    lib_24c02_bl_status_t st =
        lib_24c02_bl_write(&s_eeprom,
                           EEPROM_IMU_CALIB_ADDR,
                           (const uint8_t *)p,
                           sizeof(*p),
                           100u,   /* HAL I2C timeout */
                           5u);    /* tWR, ms */

    if (st != LIB_24C02_BL_OK)
    {
        LOGE("EEPROM write calib failed, st=%d", (int)st);
    }

    return st;
}

static lib_24c02_bl_status_t imu_eeprom_read_calib(imu_calib_persist_t *p)
{
    if (!p)
        return LIB_24C02_BL_INVALID_PARAM;

    if (s_eeprom_status != LIB_24C02_BL_OK)
        return LIB_24C02_BL_ERROR;

    lib_24c02_bl_status_t st =
        lib_24c02_bl_read(&s_eeprom,
                          EEPROM_IMU_CALIB_ADDR,
                          (uint8_t *)p,
                          sizeof(*p),
                          100u);

    if (st != LIB_24C02_BL_OK)
    {
        LOGE("EEPROM read calib failed, st=%d", (int)st);
    }

    return st;
}

/* 默认校准值：等价于“没有校准” */
static void imu_calib_result_set_default(imu_calib_result_t *r)
{
    if (!r) return;

    r->gyro_bias[0] = 0.0f;
    r->gyro_bias[1] = 0.0f;
    r->gyro_bias[2] = 0.0f;
    r->accel_scale  = 1.0f;
    r->g_norm       = 9.81f;
    r->temp_c       = IMU_TARGET_TEMP_C;
    r->success      = 0u;
}

/* 把 IMU_Calib_Result 写入持久化结构 */
static void imu_calib_persist_from_result(imu_calib_persist_t *dst,
                                          const imu_calib_result_t *src)
{
    if (!dst || !src) return;

    dst->magic    = IMU_EEPROM_MAGIC;
    dst->version  = IMU_EEPROM_VERSION;
    dst->reserved = 0u;

    for (int i = 0; i < 3; ++i)
    {
        dst->gyro_bias[i] = src->gyro_bias[i];
    }

    dst->accel_scale = src->accel_scale;
    dst->g_norm      = src->g_norm;
    dst->temp_c      = src->temp_c;
}

/* 从 EEPROM 持久化结构恢复到校准结果 */
static void imu_calib_result_from_persist(imu_calib_result_t *dst,
                                          const imu_calib_persist_t *src)
{
    if (!dst || !src) return;

    for (int i = 0; i < 3; ++i)
    {
        dst->gyro_bias[i] = src->gyro_bias[i];
    }

    dst->accel_scale = src->accel_scale;
    dst->g_norm      = src->g_norm;
    dst->temp_c      = src->temp_c;
    dst->success     = 1u;
}

/* ======================= OMX IMU 快照缓存 ======================= */
static void omx_imu_cache_publish(const OmxImuState *in)
{
    if (!in) return;

    __atomic_fetch_add(&s_omx_cache_seq, 1u, __ATOMIC_ACQ_REL);
    s_omx_cache = *in;
    __atomic_fetch_add(&s_omx_cache_seq, 1u, __ATOMIC_RELEASE);
}

bool IMU_GetOmxImuState(OmxImuState* out)
{
    if (!out) return false;

    for (int attempt = 0; attempt < 3; ++attempt)
    {
        uint32_t s1 = __atomic_load_n(&s_omx_cache_seq, __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue;

        OmxImuState tmp = s_omx_cache;

        uint32_t s2 = __atomic_load_n(&s_omx_cache_seq, __ATOMIC_ACQUIRE);
        if (s1 == s2)
        {
            *out = tmp;
            return true;
        }
    }

    return false;
}

static void imu_snapshot_update_internal(const imu_sample_t *samp,
                                         const lib_ins_state_t *ins,
                                         uint32_t now_ms,
                                         uint8_t online)
{
    if (!samp || !ins) {
        return;
    }

    __atomic_fetch_add(&s_imu_snapshot_seq, 1u, __ATOMIC_ACQ_REL);

    imu_task_snapshot_t s = s_imu_snapshot;
    for (int i = 0; i < 3; ++i) {
        s.internal.accel[i] = samp->accel[i];
        s.internal.gyro[i] = samp->gyro[i];
        s.internal.euler[i] = (i == 0) ? ins->roll_out :
                              (i == 1) ? ins->pitch_out : ins->yaw_out;
    }
    for (int i = 0; i < 4; ++i) {
        s.internal.quat[i] = ins->q[i];
    }
    s.internal.last_update_ms = now_ms;
    s.internal.online = online;
    s.internal.valid = 1u;
    s_imu_snapshot = s;

    const float rad2deg = 180.0f / M_PI_F;
    g_imu_euler_debug.internal_euler_deg[0] = ins->roll_out * rad2deg;
    g_imu_euler_debug.internal_euler_deg[1] = ins->pitch_out * rad2deg;
    g_imu_euler_debug.internal_euler_deg[2] = ins->yaw_out * rad2deg;

    __atomic_fetch_add(&s_imu_snapshot_seq, 1u, __ATOMIC_RELEASE);
}

static void imu_snapshot_set_internal_offline(uint32_t now_ms)
{
    __atomic_fetch_add(&s_imu_snapshot_seq, 1u, __ATOMIC_ACQ_REL);

    imu_task_snapshot_t s = s_imu_snapshot;
    s.internal.last_update_ms = now_ms;
    s.internal.online = 0u;
    s.internal.valid = 0u;
    s_imu_snapshot = s;

    __atomic_fetch_add(&s_imu_snapshot_seq, 1u, __ATOMIC_RELEASE);
}

static void imu_snapshot_update_external(const OmxImuState *ext, uint32_t now_ms)
{
    if (!ext) {
        return;
    }

    __atomic_fetch_add(&s_imu_snapshot_seq, 1u, __ATOMIC_ACQ_REL);

    imu_task_snapshot_t s = s_imu_snapshot;
    for (int i = 0; i < 3; ++i) {
        s.external.accel[i] = ext->accel[i];
        s.external.gyro[i] = ext->gyro[i];
        s.external.euler[i] = ext->euler[i];
    }
    for (int i = 0; i < 4; ++i) {
        s.external.quat[i] = ext->quat[i];
    }
    s.external.last_update_ms = ext->last_rx_ms ? ext->last_rx_ms : now_ms;
    s.external.online = ext->is_online ? 1u : 0u;
    s.external.valid = 1u;
    s_imu_snapshot = s;

    const float rad2deg = 180.0f / M_PI_F;
    g_imu_euler_debug.external_euler_deg[0] = ext->euler[0] * rad2deg;
    g_imu_euler_debug.external_euler_deg[1] = ext->euler[1] * rad2deg;
    g_imu_euler_debug.external_euler_deg[2] = ext->euler[2] * rad2deg;

    __atomic_fetch_add(&s_imu_snapshot_seq, 1u, __ATOMIC_RELEASE);
}

bool IMU_GetSnapshot(imu_task_snapshot_t* out)
{
    if (!out) return false;

    for (int attempt = 0; attempt < 3; ++attempt)
    {
        uint32_t s1 = __atomic_load_n(&s_imu_snapshot_seq, __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue;

        imu_task_snapshot_t tmp = s_imu_snapshot;

        uint32_t s2 = __atomic_load_n(&s_imu_snapshot_seq, __ATOMIC_ACQUIRE);
        if (s1 == s2)
        {
            *out = tmp;
            return true;
        }
    }

    return false;
}

/* ======================= INS 对外同步工具函数 ======================= */

/**
 * @brief  Sync internal lib_ins state into legacy INS_t struct.
 *         This keeps field names compatible with old INS usage.
 *
 * @param  samp  Pointer to calibrated IMU sample (after bias/scale).
 * @param  ins   Pointer to lib_ins_state_t returned by LIB_INS_GetState().
 */
static void IMU_Sync_INS_Public(const imu_sample_t *samp,
                                const lib_ins_state_t *ins)
{
    if (!samp || !ins)
    {
        return;
    }

    /* 1. Raw (but calibrated) IMU data in body frame */
    INS.Accel[0] = samp->accel[0];
    INS.Accel[1] = samp->accel[1];
    INS.Accel[2] = samp->accel[2];

    INS.Gyro[0]  = samp->gyro[0];
    INS.Gyro[1]  = samp->gyro[1];
    INS.Gyro[2]  = samp->gyro[2];

    /* 2. Attitude in Euler angles
     *    Use *_out so that:
     *      - during CONV they stay 0
     *      - after RUN they are bias-removed relative attitude
     */
    INS.Roll  = ins->roll_out;
    INS.Pitch = ins->pitch_out;
    INS.Yaw   = ins->yaw_out;

    /* 3. Continuous yaw angle (equivalent to historical YawTotalAngle) */
    INS.YawTotalAngle = ins->yaw_total;

    /* If you still use YawRoundCount / YawAngleLast somewhere,
     * we reconstruct them approximately. YawRoundCount is not
     * strictly needed for most applications; YawTotalAngle is enough.
     */
    const float two_pi = 2.0f * M_PI_F;
    INS.YawRoundCount  = (int32_t)(INS.YawTotalAngle / two_pi);
    INS.YawAngleLast   = INS.Yaw;

    /* 4. Linear acceleration in body / earth frame */
    INS.MotionAccel_b[0] = ins->motion_accel_b[0];
    INS.MotionAccel_b[1] = ins->motion_accel_b[1];
    INS.MotionAccel_b[2] = ins->motion_accel_b[2];

    INS.MotionAccel_n[0] = ins->motion_accel_n[0];
    INS.MotionAccel_n[1] = ins->motion_accel_n[1];
    INS.MotionAccel_n[2] = ins->motion_accel_n[2];

    /* 5. Simple 1D navigation result (v_n / x_n) */
    INS.v_n = ins->v_n;
    INS.x_n = ins->x_n;

    /* 6. INS ready flag: 1 only when INS is in RUN state */
    INS.ins_flag = (ins->app_state == LIB_INS_APP_STATE_RUN) ? 1u : 0u;

    /* 7. Optionally export quaternion if lib_ins provides it.
     *    If lib_ins_state_t has q[4], you can uncomment this part.
     *
     *    INS.q[0] = ins->q[0];
     *    INS.q[1] = ins->q[1];
     *    INS.q[2] = ins->q[2];
     *    INS.q[3] = ins->q[3];
     */
}

/* ======================= 主任务入口 ======================= */

void TSKPTT_IMU_Task(void *argument)
{
    (void)argument;

    bsp_bmi088_status_t bmi_st = BSP_BMI088_Init(
        BSP_SPI_BUS_4,
        BMI_ACC_CS_GPIO_Port, BMI_ACC_CS_Pin,
        BMI_CS_GYRO_GPIO_Port, BMI_CS_GYRO_Pin,
        0u);
    if (bmi_st != BSP_BMI088_OK)
    {
        LOGE("BMI088 init failed, st=%d", (int)bmi_st);
    }

    (void)IMU_Init();
    LIB_INS_Init();

    OmxImu_Init(&s_omx_imu, OMXIMU_QUAT_STD_ID);
    OmxImu_SetOfflineTimeout(&s_omx_imu, OMXIMU_OFFLINE_MS);
    if (canrx_register_omximu(OMXIMU_CAN_BUS, &s_omx_imu) != 0)
    {
        LOGW("OMX IMU register failed (bus=%u)", (unsigned)OMXIMU_CAN_BUS);
    }

    /* ------------ 温控初始化：绑定 TIM3_CH4 ------------ */
    imu_temp_ctrl_hw_t hw_cfg = {
        .htim    = &htim3,
        .channel = TIM_CHANNEL_4,
        .max_ccr = (float)__HAL_TIM_GET_AUTORELOAD(&htim3),
    };

    const float pid_param[3] = { 100.0f, 50.0f, 10.0f };

    IMU_TempCtrl_Init(&hw_cfg,
                      IMU_TARGET_TEMP_C,
                      pid_param,
                      500.0f,          /* max_out */
                      300.0f,          /* max_iout */
                      IMU_STABLE_DELTA_C,
                      IMU_STABLE_TIME_MS);

    /* ------------ EEPROM 初始化 ------------ */
    s_eeprom_status = lib_24c02_bl_init(&s_eeprom,
                                        &hi2c4,
                                        EEPROM_I2C_ADDR_7BIT);

    if (s_eeprom_status != LIB_24C02_BL_OK)
    {
        LOGE("EEPROM BL init failed, st=%d", (int)s_eeprom_status);
    }
    else
    {
        LOGI("EEPROM BL init OK (addr=0x%02X).", EEPROM_I2C_ADDR_7BIT);
    }

    /* 校准结果初始化为默认值（即“无校准”） */
    imu_calib_result_set_default(&s_calib_result);
    s_calib_done    = 0u;
    s_calib_started = 0u;

    /* 对外状态：进入升温阶段 */
    g_imu_app_state = IMU_APP_STATE_WARMUP;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   counter   = 0u;

    LOGI("IMU Task Start (non-blocking, temp control + calib)");

    for (;;)
    {
        const uint32_t now_ms = HAL_GetTick();
        OmxImu_Tick(&s_omx_imu, now_ms);
        OmxImuState omx_state;
        if (OmxImu_Snapshot(&s_omx_imu, &omx_state))
        {
            omx_imu_cache_publish(&omx_state);
            imu_snapshot_update_external(&omx_state, now_ms);
        }

        imu_sample_t samp;
        imu_status_t st = IMU_SampleOnce(&samp);

        if (st == IMU_STATUS_OK)
        {
            /* 温控：用当前温度驱动控制环 */
            IMU_TempCtrl_StepWithTemp(samp.temperature);

            uint8_t temp_stable = IMU_TempCtrl_IsStable();

            if (!temp_stable)
            {
                g_imu_app_state = IMU_APP_STATE_WARMUP;

                if ((counter++ % IMU_LOG_DIV) == 0u)
                {
                    LOGI("AWAIT  T[C]=%6.2f  target=%6.2f",
                         samp.temperature,
                         IMU_TempCtrl_GetTargetTemp());
                }
            }
            else
            {
                /* 温度已稳定：根据模式决定是否/如何校准 */
                if (!s_calib_done)
                {
                    /* 只进一次“选择校准策略”的分支 */
#if (IMU_CALIBRATION_MODE == 0)
                    /* 模式 0：不做新校准 → 直接从 EEPROM 读 */
                    if (!s_calib_started)
                    {
                        s_calib_started = 1u;
                        LOGI("Temp stable, trying to load calib from EEPROM...");

                        imu_calib_persist_t persist;
                        memset(&persist, 0, sizeof(persist));

                        if (imu_eeprom_read_calib(&persist) == LIB_24C02_BL_OK &&
                            persist.magic   == IMU_EEPROM_MAGIC &&
                            persist.version == IMU_EEPROM_VERSION)
                        {
                            imu_calib_result_from_persist(&s_calib_result, &persist);
                            LOGI("Loaded IMU calib from EEPROM (bias=%.4f,%.4f,%.4f scale=%.6f)",
                                 (double)s_calib_result.gyro_bias[0],
                                 (double)s_calib_result.gyro_bias[1],
                                 (double)s_calib_result.gyro_bias[2],
                                 (double)s_calib_result.accel_scale);
                        }
                        else
                        {
                            LOGW("No valid calib in EEPROM, using default calib.");
                            imu_calib_result_set_default(&s_calib_result);
                        }

                        s_calib_done    = 1u;
                        g_imu_app_state = IMU_APP_STATE_RUNNING;
                        LOGI("IMU calib stage finished (mode=0), start publishing data.");
                    }

#elif (IMU_CALIBRATION_MODE == 1)
                    /* 模式 1：静态校准（任意姿态静止） */
                    if (!s_calib_started)
                    {
                        s_calib_started = 1u;
                        LOGI("Temp stable, starting static calibration...");
                        IMU_Calib_Start();
                    }

                    g_imu_app_state = IMU_APP_STATE_CALIB;

                    /* 非阻塞校准步骤：每圈调用一下 */
                    IMU_Calib_AddSample(&samp);

                    imu_calib_state_t cst = IMU_Calib_GetState();
                    if (cst == IMU_CALIB_DONE)
                    {
                        IMU_Calib_GetResult(&s_calib_result);

                        if (s_calib_result.success)
                        {
                            LOGI("Static calibration OK: bias=%.4f,%.4f,%.4f scale=%.6f g=%.4f T=%.2f",
                                 (double)s_calib_result.gyro_bias[0],
                                 (double)s_calib_result.gyro_bias[1],
                                 (double)s_calib_result.gyro_bias[2],
                                 (double)s_calib_result.accel_scale,
                                 (double)s_calib_result.g_norm,
                                 (double)s_calib_result.temp_c);

                            /* 成功则写入 EEPROM（如果 EEPROM 可用） */
                            if (s_eeprom_status == LIB_24C02_BL_OK)
                            {
                                imu_calib_persist_t persist;
                                imu_calib_persist_from_result(&persist, &s_calib_result);
                                imu_eeprom_write_calib(&persist);
                            }
                        }
                        else
                        {
                            LOGW("Static calibration result not marked success, using default calib.");
                            imu_calib_result_set_default(&s_calib_result);
                        }

                        s_calib_done    = 1u;
                        g_imu_app_state = IMU_APP_STATE_RUNNING;
                        LOGI("IMU calib stage finished (mode=1), start publishing data.");
                    }
                    else if (cst == IMU_CALIB_FAILED)
                    {
                        LOGE("Static calibration FAILED (timeout or motion), using default calib.");
                        imu_calib_result_set_default(&s_calib_result);
                        s_calib_done    = 1u;
                        g_imu_app_state = IMU_APP_STATE_RUNNING;
                    }

#elif (IMU_CALIBRATION_MODE == 2)
                    /* 模式 2：转台校准占位（TODO） */
                    if (!s_calib_started)
                    {
                        s_calib_started = 1u;
                        LOGW("Temp stable, turntable calibration mode (2) NOT implemented, using default calib.");
                        imu_calib_result_set_default(&s_calib_result);
                        s_calib_done    = 1u;
                        g_imu_app_state = IMU_APP_STATE_RUNNING;
                        LOGI("IMU calib stage finished (mode=2 placeholder), start publishing data.");
                    }
#else
#error "Unsupported IMU_CALIBRATION_MODE"
#endif
                }
                else
                {
                    /* ======= 校准阶段已结束：进入 INS 驱动阶段 ======= */

                    /* 先应用校准再送进 INS */
                    samp.gyro[0] -= s_calib_result.gyro_bias[0];
                    samp.gyro[1] -= s_calib_result.gyro_bias[1];
                    samp.gyro[2] -= s_calib_result.gyro_bias[2];

                    samp.accel[0] *= s_calib_result.accel_scale;
                    samp.accel[1] *= s_calib_result.accel_scale;
                    samp.accel[2] *= s_calib_result.accel_scale;

                    static uint32_t s_last_ins_tick = 0;   // 放在函数顶部的 static 区域也行

                    uint32_t now = HAL_GetTick();
                    float dt_s;

                    if (s_last_ins_tick == 0)
                    {
                        /* First run: use nominal dt */
                        dt_s = IMU_TASK_PERIOD_MS / 1000.0f;
                    }
                    else
                    {
                        uint32_t delta_ms = now - s_last_ins_tick;
                        if (delta_ms == 0)
                        {
                            /* Safety guard in case of weird tick */
                            delta_ms = IMU_TASK_PERIOD_MS;
                        }
                        dt_s = delta_ms / 1000.0f;
                    }

                    s_last_ins_tick = now;

                    /* 先更新 INS，再拿状态 */
                    LIB_INS_Update(&samp, dt_s);
                    const lib_ins_state_t *ins = LIB_INS_GetState();

                    /* 根据 INS 的状态机映射到 IMU 的 app_state */
                    switch (ins->app_state)
                    {
                        case LIB_INS_APP_STATE_CONV:
                            g_imu_app_state = IMU_APP_STATE_CONV;      /* INS 收敛中 */
                            break;

                        case LIB_INS_APP_STATE_RUN:
                            g_imu_app_state = IMU_APP_STATE_RUNNING;   /* INS 已收敛，可控 */
                            break;

                        default:
                            g_imu_app_state = IMU_APP_STATE_BOOT;      /* 兜底 */
                            break;
                    }

                    /* === 同步到传统 INS_t 结构，对外暴露统一接口 === */
                    IMU_Sync_INS_Public(&samp, ins);
                    imu_snapshot_update_internal(&samp, ins, now_ms, 1u);

                    /* 如果你之后想用队列推给别的 Task，可以在这里加：
                     *
                     * typedef struct {
                     *     INS_t snapshot;
                     * } imu_ins_msg_t;
                     *
                     * imu_ins_msg_t msg;
                     * msg.snapshot = INS;
                     * if (g_imuQueue) {
                     *     xQueueOverwrite(g_imuQueue, &msg);
                     * }
                     */

                    if ((counter++ % IMU_LOG_DIV) == 0u)
                    {
                        if (ins->app_state == LIB_INS_APP_STATE_CONV)
                        {
                            LOGD("INS R/P/Y[deg]: %7.2f %7.2f %7.2f  "
                                 "YawTotal[deg]: %8.2f  "
                                 "Acc_b[m/s2]: %8.3f %8.3f %8.3f  "
                                 "Acc_n[m/s2]: %8.3f %8.3f %8.3f [CONV]",
                                 ins->roll_out  * 57.2958f,
                                 ins->pitch_out * 57.2958f,
                                 ins->yaw_out   * 57.2958f,
                                 ins->yaw_total * 57.2958f,
                                 ins->motion_accel_b[0],
                                 ins->motion_accel_b[1],
                                 ins->motion_accel_b[2],
                                 ins->motion_accel_n[0],
                                 ins->motion_accel_n[1],
                                 ins->motion_accel_n[2]);
                        }
                        else if (ins->app_state == LIB_INS_APP_STATE_RUN)
                        {
                            /* You can enable this if you need runtime log */
                            // LOGD("INS R/P/Y[deg]: %7.2f %7.2f %7.2f  "
                            //      "YawTotal[deg]: %8.2f  "
                            //      "Acc_b[m/s2]: %8.3f %8.3f %8.3f  "
                            //      "Acc_n[m/s2]: %8.3f %8.3f %8.3f [RUN]",
                            //      ins->roll_out  * 57.2958f,
                            //      ins->pitch_out * 57.2958f,
                            //      ins->yaw_out   * 57.2958f,
                            //      ins->yaw_total * 57.2958f,
                            //      ins->motion_accel_b[0],
                            //      ins->motion_accel_b[1],
                            //      ins->motion_accel_b[2],
                            //      ins->motion_accel_n[0],
                            //      ins->motion_accel_n[1],
                            //      ins->motion_accel_n[2]);
                        }
                    }
                }
            }

        }
        else if (st == IMU_STATUS_ERR_BMI)
        {
            /* BMI088 硬错误：直接关加热并进入故障态 */
            IMU_TempCtrl_Fault();
            g_imu_app_state = IMU_APP_STATE_FAULT;
            LOGE("IMU_SampleOnce ERR_BMI, heater OFF, enter FAULT state");
            imu_snapshot_set_internal_offline(now_ms);
        }
        else
        {
            /* BUSY / INVALID_ARG：本轮无事发生，忽略 */
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}
