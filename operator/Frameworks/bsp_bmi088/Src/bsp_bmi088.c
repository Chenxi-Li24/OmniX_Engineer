#include "bsp_bmi088.h"
#include "bsp_srn_log.h"
#include <string.h>
#include <math.h>

/* ==== 灵敏度常量兜底：优先用你原来头文件里的 ===== */
#ifndef SRN_BMI088_ACCEL_SEN
  #if defined(BMI088_ACCEL_3G_SEN)
    #define SRN_BMI088_ACCEL_SEN  BMI088_ACCEL_3G_SEN
  #elif defined(BMI088_ACCEL_SEN)
    #define SRN_BMI088_ACCEL_SEN  BMI088_ACCEL_SEN
  #else
    /* fallback：每个 LSB ≈ 0.0006 g，自行校准 */
    #define SRN_BMI088_ACCEL_SEN  (0.0006f)
    #warning "bsp_bmi088: no accel sensitivity macro found, using dummy 0.0006f."
  #endif
#endif

#ifndef SRN_BMI088_GYRO_SEN
  #if defined(BMI088_GYRO_2000_SEN)
    #define SRN_BMI088_GYRO_SEN   BMI088_GYRO_2000_SEN
  #else
    /* fallback：假设 1LSB≈1/16.4 deg/s */
    #define SRN_BMI088_GYRO_SEN   (1.0f / 16.4f)
    #warning "bsp_bmi088: no gyro sensitivity macro found, using dummy 1/16.4."
  #endif
#endif

/* 延时兜底 */
#ifndef BMI088_LONG_DELAY_TIME
#define BMI088_LONG_DELAY_TIME   200u
#endif

/* 温度公式兜底：T = 23 + raw * 0.125 */
#ifndef BMI088_TEMP_FACTOR
#define BMI088_TEMP_FACTOR   (0.125f)
#endif
#ifndef BMI088_TEMP_OFFSET
#define BMI088_TEMP_OFFSET   (23.0f)
#endif

#define BMI088_SPI_TIMEOUT_MS   (5u)
#define BMI088_CALIB_SAMPLES    (1024u)

/* ==== SPI 设备 & DMA buffer ===================================== */

static bsp_spi_device_t s_acc_dev;
static bsp_spi_device_t s_gyro_dev;

#define BMI088_SPI_MAX_XFER   (64u)

/* non-cacheable DMA buffers if available */
BSP_SPI_DMA_SEC __attribute__((aligned(32)))
static uint8_t s_tx[BMI088_SPI_MAX_XFER];

BSP_SPI_DMA_SEC __attribute__((aligned(32)))
static uint8_t s_rx[BMI088_SPI_MAX_XFER];

/* ==== 零偏 / scale ============================================== */

static float s_gyro_offset[3]  = {0.0f, 0.0f, 0.0f};
static float s_accel_offset[3] = {0.0f, 0.0f, 0.0f};
static float s_accel_scale     = 1.0f;

/* ==== 帮助函数 =================================================== */

static inline int16_t bmi088_make_s16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((((uint16_t)hi) << 8) | (uint16_t)lo);
}

static void bmi088_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ==== SPI helpers: blocking 版本（用于 Init/Calib/Blocking Read） = */

static uint8_t acc_write(uint8_t reg, uint8_t data)
{
    s_tx[0] = reg & 0x7Fu;
    s_tx[1] = data;

    return (bsp_spi_transfer_blocking(&s_acc_dev,
                                      s_tx,
                                      s_rx,
                                      2u,
                                      BMI088_SPI_TIMEOUT_MS) == BSP_SPI_OK) ? 0u : 1u;
}

static uint8_t acc_read1(uint8_t reg, uint8_t *data)
{
    s_tx[0] = reg | 0x80u;
    s_tx[1] = 0xFFu;
    s_tx[2] = 0xFFu;

    if (bsp_spi_transfer_blocking(&s_acc_dev,
                                  s_tx,
                                  s_rx,
                                  3u,
                                  BMI088_SPI_TIMEOUT_MS) != BSP_SPI_OK)
    {
        return 1u;
    }

    *data = s_rx[2];
    return 0u;
}

static uint8_t acc_readN(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if ((size_t)(len + 2u) > sizeof(s_tx))
        return 1u;

    s_tx[0] = reg | 0x80u;
    s_tx[1] = 0xFFu;
    for (uint8_t i = 0; i < len; ++i)
    {
        s_tx[2 + i] = 0xFFu;
    }

    if (bsp_spi_transfer_blocking(&s_acc_dev,
                                  s_tx,
                                  s_rx,
                                  (size_t)(len + 2u),
                                  BMI088_SPI_TIMEOUT_MS) != BSP_SPI_OK)
    {
        return 1u;
    }

    for (uint8_t i = 0; i < len; ++i)
    {
        buf[i] = s_rx[2u + i];
    }
    return 0u;
}

static uint8_t gyro_write(uint8_t reg, uint8_t data)
{
    s_tx[0] = reg & 0x7Fu;
    s_tx[1] = data;

    return (bsp_spi_transfer_blocking(&s_gyro_dev,
                                      s_tx,
                                      s_rx,
                                      2u,
                                      BMI088_SPI_TIMEOUT_MS) == BSP_SPI_OK) ? 0u : 1u;
}

static uint8_t gyro_readN(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if ((size_t)(len + 1u) > sizeof(s_tx))
        return 1u;

    s_tx[0] = reg | 0x80u;
    for (uint8_t i = 0; i < len; ++i)
    {
        s_tx[1 + i] = 0xFFu;
    }

    if (bsp_spi_transfer_blocking(&s_gyro_dev,
                                  s_tx,
                                  s_rx,
                                  (size_t)(len + 1u),
                                  BMI088_SPI_TIMEOUT_MS) != BSP_SPI_OK)
    {
        return 1u;
    }

    for (uint8_t i = 0; i < len; ++i)
    {
        buf[i] = s_rx[1u + i];
    }
    return 0u;
}

/* ==== 加速度计 / 陀螺仪 Init（blocking） ======================= */

static uint8_t bmi088_accel_init(void)
{
    uint8_t id = 0;

    /* check chip id */
    (void)acc_read1(BMI088_ACC_CHIP_ID, &id);
    bmi088_delay_ms(1);
    (void)acc_read1(BMI088_ACC_CHIP_ID, &id);
    bmi088_delay_ms(1);

    /* soft reset */
    (void)acc_write(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    /* re-check chip id */
    (void)acc_read1(BMI088_ACC_CHIP_ID, &id);
    bmi088_delay_ms(1);
    (void)acc_read1(BMI088_ACC_CHIP_ID, &id);
    bmi088_delay_ms(1);

    if (id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BSP_BMI088_ERR_NO_SENSOR;
    }

    /* power config */
    (void)acc_write(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
    bmi088_delay_ms(1);

    (void)acc_write(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON);
    bmi088_delay_ms(1);

    /* ODR / mode / filter */
    (void)acc_write(
        BMI088_ACC_CONF,
        (uint8_t)(BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set)
    );
    bmi088_delay_ms(1);

#if defined(BMI088_ACC_RANGE_3G)
    (void)acc_write(BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G);
#elif defined(BMI088_ACC_RANGE_6G)
    (void)acc_write(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
#else
    (void)acc_write(BMI088_ACC_RANGE, 0x00u);
#endif
    bmi088_delay_ms(1);

#if defined(BMI088_INT1_IO_CTRL)
    (void)acc_write(
        BMI088_INT1_IO_CTRL,
        (uint8_t)(BMI088_ACC_INT1_IO_ENABLE |
                  BMI088_ACC_INT1_GPIO_PP   |
                  BMI088_ACC_INT1_GPIO_LOW)
    );
    bmi088_delay_ms(1);
#endif

#if defined(BMI088_INT_MAP_DATA)
    (void)acc_write(
        BMI088_INT_MAP_DATA,
        BMI088_ACC_INT1_DRDY_INTERRUPT
    );
    bmi088_delay_ms(1);
#endif

    return BSP_BMI088_OK;
}

static uint8_t bmi088_gyro_init(void)
{
    uint8_t id = 0;

    /* check chip id */
    (void)gyro_readN(BMI088_GYRO_CHIP_ID, &id, 1);
    bmi088_delay_ms(1);
    (void)gyro_readN(BMI088_GYRO_CHIP_ID, &id, 1);
    bmi088_delay_ms(1);

    /* soft reset */
    (void)gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    /* re-check chip id */
    (void)gyro_readN(BMI088_GYRO_CHIP_ID, &id, 1);
    bmi088_delay_ms(1);
    (void)gyro_readN(BMI088_GYRO_CHIP_ID, &id, 1);
    bmi088_delay_ms(1);

    if (id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BSP_BMI088_ERR_NO_SENSOR;
    }

    (void)gyro_write(BMI088_GYRO_RANGE, BMI088_GYRO_2000);
    bmi088_delay_ms(1);

#if defined(BMI088_GYRO_2000_230_HZ)
    (void)gyro_write(
        BMI088_GYRO_BANDWIDTH,
        (uint8_t)(BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set)
    );
#else
    (void)gyro_write(
        BMI088_GYRO_BANDWIDTH,
        (uint8_t)(BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set)
    );
#endif
    bmi088_delay_ms(1);

    (void)gyro_write(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
    bmi088_delay_ms(1);

    (void)gyro_write(BMI088_GYRO_CTRL, BMI088_DRDY_ON);
    bmi088_delay_ms(1);

#if defined(BMI088_GYRO_INT3_INT4_IO_CONF)
    (void)gyro_write(
        BMI088_GYRO_INT3_INT4_IO_CONF,
        (uint8_t)(BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW)
    );
    bmi088_delay_ms(1);
#endif

#if defined(BMI088_GYRO_INT3_INT4_IO_MAP)
    (void)gyro_write(
        BMI088_GYRO_INT3_INT4_IO_MAP,
        BMI088_GYRO_DRDY_IO_INT3
    );
    bmi088_delay_ms(1);
#endif

    return BSP_BMI088_OK;
}

/* ==== 简单零偏校准（blocking，Init 阶段用） =================== */

static void bmi088_simple_calib(void)
{
    uint8_t buf[8];
    int16_t raw;
    float   accel_sum[3] = {0.f, 0.f, 0.f};
    float   gyro_sum[3]  = {0.f, 0.f, 0.f};

    for (uint16_t i = 0; i < BMI088_CALIB_SAMPLES; ++i)
    {
        (void)acc_readN(BMI088_ACCEL_XOUT_L, buf, 6);

        raw = bmi088_make_s16(buf[0], buf[1]);
        float ax = raw * SRN_BMI088_ACCEL_SEN;
        raw = bmi088_make_s16(buf[2], buf[3]);
        float ay = raw * SRN_BMI088_ACCEL_SEN;
        raw = bmi088_make_s16(buf[4], buf[5]);
        float az = raw * SRN_BMI088_ACCEL_SEN;

        accel_sum[0] += ax;
        accel_sum[1] += ay;
        accel_sum[2] += az;

        (void)gyro_readN(BMI088_GYRO_CHIP_ID, buf, 8);
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {
            raw = bmi088_make_s16(buf[2], buf[3]);
            float gx = raw * SRN_BMI088_GYRO_SEN;
            raw = bmi088_make_s16(buf[4], buf[5]);
            float gy = raw * SRN_BMI088_GYRO_SEN;
            raw = bmi088_make_s16(buf[6], buf[7]);
            float gz = raw * SRN_BMI088_GYRO_SEN;

            gyro_sum[0] += gx;
            gyro_sum[1] += gy;
            gyro_sum[2] += gz;
        }

        bmi088_delay_ms(1);
    }

    float invN = 1.0f / (float)BMI088_CALIB_SAMPLES;

    float ax_avg = accel_sum[0] * invN;
    float ay_avg = accel_sum[1] * invN;
    float az_avg = accel_sum[2] * invN;

    float g_norm = sqrtf(ax_avg * ax_avg + ay_avg * ay_avg + az_avg * az_avg);
    if (g_norm < 1e-3f)
        g_norm = 1.0f;

    s_accel_scale = 9.81f / g_norm;

    s_accel_offset[0] = ax_avg * s_accel_scale;
    s_accel_offset[1] = ay_avg * s_accel_scale;
    s_accel_offset[2] = 0.0f;

    s_gyro_offset[0] = gyro_sum[0] * invN;
    s_gyro_offset[1] = gyro_sum[1] * invN;
    s_gyro_offset[2] = gyro_sum[2] * invN;
}

/* ==== 异步采样状态机 ============================================ */

/* raw 数据由 SPI ISR 填写，task 只做 float 计算 */
static volatile int16_t s_raw_acc[3]  = {0};
static volatile int16_t s_raw_gyro[3] = {0};
static volatile int16_t s_raw_temp    = 0;

typedef enum
{
    BMI088_SM_IDLE = 0,
    BMI088_SM_ACC_READ,
    BMI088_SM_GYRO_READ,
    BMI088_SM_TEMP_READ
} bmi088_sm_state_t;

static volatile bmi088_sm_state_t s_sm_state = BMI088_SM_IDLE;
static volatile uint8_t           s_sample_ready = 0;
static volatile uint8_t           s_kick_pending = 0;

static void bmi088_spi_cb(bsp_spi_device_t *dev, void *user_ctx, bsp_spi_status_t status);
static bsp_bmi088_status_t bmi088_kick_next(void);

/* ==== 公共接口：Init ============================================ */

bsp_bmi088_status_t BSP_BMI088_Init(bsp_spi_bus_id_t bus_id,
                                    GPIO_TypeDef *acc_cs_port,  uint16_t acc_cs_pin,
                                    GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin,
                                    uint8_t do_calib)
{
    if (bsp_spi_device_init(&s_acc_dev, bus_id, acc_cs_port, acc_cs_pin) != BSP_SPI_OK)
        return BSP_BMI088_ERR_SPI;
    if (bsp_spi_device_init(&s_gyro_dev, bus_id, gyro_cs_port, gyro_cs_pin) != BSP_SPI_OK)
        return BSP_BMI088_ERR_SPI;

    uint8_t err;

    err = bmi088_accel_init();
    if (err != BSP_BMI088_OK)
        return err;

    err = bmi088_gyro_init();
    if (err != BSP_BMI088_OK)
        return err;

    if (do_calib)
    {
        bmi088_simple_calib();
    }
    else
    {
        s_gyro_offset[0]  = 0.0f;
        s_gyro_offset[1]  = 0.0f;
        s_gyro_offset[2]  = 0.0f;
        s_accel_offset[0] = 0.0f;
        s_accel_offset[1] = 0.0f;
        s_accel_offset[2] = 0.0f;
        s_accel_scale     = 1.0f;
    }

    s_sm_state     = BMI088_SM_IDLE;
    s_sample_ready = 0;

    return BSP_BMI088_OK;
}

/* ==== 公共接口：同步阻塞读取 ==================================== */

void BSP_BMI088_ReadBlocking(bmi088_data_t *out)
{
    if (out == NULL)
        return;

    uint8_t buf[8];
    int16_t raw;

    /* accel */
    (void)acc_readN(BMI088_ACCEL_XOUT_L, buf, 6);
    raw = bmi088_make_s16(buf[0], buf[1]);
    float ax = raw * SRN_BMI088_ACCEL_SEN * s_accel_scale - s_accel_offset[0];

    raw = bmi088_make_s16(buf[2], buf[3]);
    float ay = raw * SRN_BMI088_ACCEL_SEN * s_accel_scale - s_accel_offset[1];

    raw = bmi088_make_s16(buf[4], buf[5]);
    float az = raw * SRN_BMI088_ACCEL_SEN * s_accel_scale - s_accel_offset[2];

    out->accel[0] = ax;
    out->accel[1] = ay;
    out->accel[2] = az;

    /* gyro */
    (void)gyro_readN(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        raw = bmi088_make_s16(buf[2], buf[3]);
        float gx = raw * SRN_BMI088_GYRO_SEN - s_gyro_offset[0];

        raw = bmi088_make_s16(buf[4], buf[5]);
        float gy = raw * SRN_BMI088_GYRO_SEN - s_gyro_offset[1];

        raw = bmi088_make_s16(buf[6], buf[7]);
        float gz = raw * SRN_BMI088_GYRO_SEN - s_gyro_offset[2];

        out->gyro[0] = gx;
        out->gyro[1] = gy;
        out->gyro[2] = gz;
    }

    /* temp */
    (void)acc_readN(BMI088_TEMP_M, buf, 2);
    raw = (int16_t)(((int16_t)buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023)
        raw -= 2048;
    out->temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* ==== 公共接口：异步采样 ======================================== */

bsp_bmi088_status_t BSP_BMI088_BeginSample(void)
{
    if (s_sm_state != BMI088_SM_IDLE)
    {
        // 说明上一帧还没跑完
        return BSP_BMI088_BUSY;
    }

    if ((size_t)(6 + 2u) > sizeof(s_tx))
    {
        LOGE("BMI088", "BeginSample: ACC buf too small");
        return BSP_BMI088_ERR_CONFIG;
    }

    s_tx[0] = BMI088_ACCEL_XOUT_L | 0x80u;
    s_tx[1] = 0xFFu;
    for (uint8_t i = 0; i < 6u; ++i)
    {
        s_tx[2u + i] = 0xFFu;
    }

    s_sample_ready = 0;
    s_kick_pending = 0;
    s_sm_state     = BMI088_SM_ACC_READ;

    bsp_spi_status_t ret = bsp_spi_transfer_async(&s_acc_dev,
                                                  s_tx,
                                                  s_rx,
                                                  8u,
                                                  bmi088_spi_cb,
                                                  NULL);
    if (ret != BSP_SPI_OK)
    {
        LOGE("BMI088", "BeginSample: SPI async start fail=%d", (int)ret);
        s_sm_state = BMI088_SM_IDLE;
        if (ret == BSP_SPI_BUSY)
        {
            return BSP_BMI088_BUSY;
        }
        return BSP_BMI088_ERR_SPI;
    }

    return BSP_BMI088_OK;
}

bsp_bmi088_status_t BSP_BMI088_GetSampleIfReady(bmi088_data_t *out)
{
    if (out == NULL)
        return BSP_BMI088_ERR_CONFIG;

    bsp_bmi088_status_t kick_st = bmi088_kick_next();
    if (kick_st == BSP_BMI088_ERR_SPI || kick_st == BSP_BMI088_ERR_CONFIG)
    {
        return kick_st;
    }

    if (s_sample_ready == 0u)
    {
        return BSP_BMI088_BUSY;
    }

    /* raw -> float at task context */
    float ax = (float)s_raw_acc[0] * SRN_BMI088_ACCEL_SEN * s_accel_scale - s_accel_offset[0];
    float ay = (float)s_raw_acc[1] * SRN_BMI088_ACCEL_SEN * s_accel_scale - s_accel_offset[1];
    float az = (float)s_raw_acc[2] * SRN_BMI088_ACCEL_SEN * s_accel_scale - s_accel_offset[2];

    float gx = (float)s_raw_gyro[0] * SRN_BMI088_GYRO_SEN - s_gyro_offset[0];
    float gy = (float)s_raw_gyro[1] * SRN_BMI088_GYRO_SEN - s_gyro_offset[1];
    float gz = (float)s_raw_gyro[2] * SRN_BMI088_GYRO_SEN - s_gyro_offset[2];

    float t;
    {
        int16_t raw = s_raw_temp;
        t = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    out->accel[0] = ax;
    out->accel[1] = ay;
    out->accel[2] = az;

    out->gyro[0]  = gx;
    out->gyro[1]  = gy;
    out->gyro[2]  = gz;

    out->temperature = t;

    s_sample_ready = 0;

    return BSP_BMI088_OK;
}

static bsp_bmi088_status_t bmi088_kick_next(void)
{
    if (s_kick_pending == 0u)
    {
        return BSP_BMI088_OK;
    }

    bsp_spi_status_t ret = BSP_SPI_OK;

    if (s_sm_state == BMI088_SM_GYRO_READ)
    {
        if ((size_t)(8 + 1u) > sizeof(s_tx))
        {
            LOGE("BMI088", "kick: gyro buf too small");
            s_sm_state = BMI088_SM_IDLE;
            s_kick_pending = 0u;
            return BSP_BMI088_ERR_CONFIG;
        }

        s_tx[0] = BMI088_GYRO_CHIP_ID | 0x80u;
        for (uint8_t i = 0; i < 8u; ++i)
        {
            s_tx[1u + i] = 0xFFu;
        }

        ret = bsp_spi_transfer_async(&s_gyro_dev,
                                     s_tx,
                                     s_rx,
                                     9u,
                                     bmi088_spi_cb,
                                     NULL);
    }
    else if (s_sm_state == BMI088_SM_TEMP_READ)
    {
        if ((size_t)(2 + 2u) > sizeof(s_tx))
        {
            LOGE("BMI088", "kick: temp buf too small");
            s_sm_state = BMI088_SM_IDLE;
            s_kick_pending = 0u;
            return BSP_BMI088_ERR_CONFIG;
        }

        s_tx[0] = BMI088_TEMP_M | 0x80u;
        s_tx[1] = 0xFFu;
        s_tx[2] = 0xFFu;
        s_tx[3] = 0xFFu;

        ret = bsp_spi_transfer_async(&s_acc_dev,
                                     s_tx,
                                     s_rx,
                                     4u,
                                     bmi088_spi_cb,
                                     NULL);
    }
    else
    {
        s_kick_pending = 0u;
        return BSP_BMI088_OK;
    }

    if (ret == BSP_SPI_OK)
    {
        s_kick_pending = 0u;
        return BSP_BMI088_OK;
    }
    if (ret == BSP_SPI_BUSY)
    {
        return BSP_BMI088_BUSY;
    }

    LOGE("BMI088", "kick: SPI async start fail=%d", (int)ret);
    s_sm_state = BMI088_SM_IDLE;
    s_kick_pending = 0u;
    return BSP_BMI088_ERR_SPI;
}

/* ==== SPI callback：在中断里推进 state machine =================== */

static void bmi088_spi_cb(bsp_spi_device_t *dev, void *user_ctx, bsp_spi_status_t status)
{
    (void)user_ctx;

    if (status != BSP_SPI_OK)
    {
        LOGE("SPI cb error status=%d, sm=%d", (int)status, (int)s_sm_state);
        s_sm_state     = BMI088_SM_IDLE;
        s_sample_ready = 0;
        s_kick_pending = 0;
        return;
    }

    switch (s_sm_state)
    {
    case BMI088_SM_ACC_READ:
    {
        // LOGD("cb: ACC_READ");

        uint8_t *p = &s_rx[2];
        s_raw_acc[0] = bmi088_make_s16(p[0], p[1]);
        s_raw_acc[1] = bmi088_make_s16(p[2], p[3]);
        s_raw_acc[2] = bmi088_make_s16(p[4], p[5]);

        s_sm_state = BMI088_SM_GYRO_READ;
        s_kick_pending = 1u;
    }
    break;

    case BMI088_SM_GYRO_READ:
    {
        // LOGD("cb: GYRO_READ");

        // s_rx[0] = dummy（发命令时回读的）
        // s_rx[1..8] = 等价于老 blocking 里的 buf[0..7]
        uint8_t *p = &s_rx[1];  // p[0] = CHIP_ID, p[1] = STATUS, p[2] = Gx_L, ...

        // 更稳一点：兼容一下 offset 不同的实现
        uint8_t *d = NULL;
        if (p[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {
            d = p;          // d[0] = CHIP_ID
        }
        else if (p[1] == BMI088_GYRO_CHIP_ID_VALUE)
        {
            d = &p[1];      // 某些 SPI 实现会再多丢一个 dummy
        }

        if (d == NULL)
        {
            LOGW("GYRO_READ: no valid CHIP_ID in rx: %02X %02X %02X",
                 s_rx[0], s_rx[1], s_rx[2]);
            // 不更新 s_raw_gyro，保持上次值
        }
        else
        {
            // 对齐老代码：Gx = make_s16(buf[2], buf[3]) 等价于 d[2], d[3]
            s_raw_gyro[0] = bmi088_make_s16(d[2], d[3]);
            s_raw_gyro[1] = bmi088_make_s16(d[4], d[5]);
            s_raw_gyro[2] = bmi088_make_s16(d[6], d[7]);
        }

        s_sm_state = BMI088_SM_TEMP_READ;
        s_kick_pending = 1u;
    }
    break;

    case BMI088_SM_TEMP_READ:
    {
        // LOGD("cb: TEMP_READ -> sample_ready=1");

        // 和 blocking 完全对齐：buf[0]=s_rx[2], buf[1]=s_rx[3]
        uint8_t hi = s_rx[2];
        uint8_t lo = s_rx[3];

        int16_t raw = (int16_t)(((int16_t)hi << 3) | (lo >> 5));
        if (raw > 1023)
            raw -= 2048;

        s_raw_temp     = raw;
        s_sample_ready = 1u;
        s_sm_state     = BMI088_SM_IDLE;
        s_kick_pending = 0u;
    }
    break;

    default:
        LOGE("cb: unexpected sm=%d", (int)s_sm_state);
        s_sm_state = BMI088_SM_IDLE;
        s_kick_pending = 0u;
        break;
    }
}

/* ==== 老接口兼容 ================================================ */

void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    bmi088_data_t d;
    BSP_BMI088_ReadBlocking(&d);

    if (gyro)
    {
        gyro[0] = d.gyro[0];
        gyro[1] = d.gyro[1];
        gyro[2] = d.gyro[2];
    }
    if (accel)
    {
        accel[0] = d.accel[0];
        accel[1] = d.accel[1];
        accel[2] = d.accel[2];
    }
    if (temperate)
    {
        *temperate = d.temperature;
    }
}
