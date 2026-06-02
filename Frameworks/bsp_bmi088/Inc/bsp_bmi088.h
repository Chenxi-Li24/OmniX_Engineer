//
// bsp_bmi088.h  —— 适配你当前的 bsp_bmi088.c（支持异步采样）
//
#ifndef H723VG_V2_FREERTOS_BSP_BMI088_H
#define H723VG_V2_FREERTOS_BSP_BMI088_H

#include "bsp_spi.h"
#include "bsp_bmi088_reg.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct
    {
        float gyro[3];        /* deg/s */
        float accel[3];       /* m/s^2 */
        float temperature;    /* deg C */
    } bmi088_data_t;

    /* 和 .c 里用到的错误码保持一致 */
    typedef enum
    {
        BSP_BMI088_OK = 0,
        BSP_BMI088_ERR_NO_SENSOR = 1,
        BSP_BMI088_ERR_CONFIG    = 2,
        BSP_BMI088_ERR_SPI       = 3,
        BSP_BMI088_BUSY          = 4
    } bsp_bmi088_status_t;

    /**
     * @brief  初始化 BMI088（阻塞，内部用 SPI blocking + 可选简易校准）
     */
    bsp_bmi088_status_t BSP_BMI088_Init(bsp_spi_bus_id_t bus_id,
                                        GPIO_TypeDef *acc_cs_port,  uint16_t acc_cs_pin,
                                        GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin,
                                        uint8_t do_calib);

    /**
     * @brief 同步阻塞读一帧数据（供调试、blocking 场景使用）
     */
    void BSP_BMI088_ReadBlocking(bmi088_data_t *out);

    /**
     * @brief 异步开始一次完整采样（acc+gyro+temp）
     *
     * - 内部会启动 ACC 读 DMA，之后在 SPI 回调里按 state machine 继续
     * - 若上一次还没完成，会返回 BSP_BMI088_BUSY
     */
    bsp_bmi088_status_t BSP_BMI088_BeginSample(void);

    /**
     * @brief 如果采样已经完成，则取出最新一帧数据
     *
     * - 若数据尚未 ready，返回 BSP_BMI088_BUSY
     * - 若 OK，则填充 out 并清除 ready 标志
     */
    bsp_bmi088_status_t BSP_BMI088_GetSampleIfReady(bmi088_data_t *out);

    /**
     * @brief 老接口兼容：阻塞读一帧
     */
    void BMI088_read(float gyro[3], float accel[3], float *temperate);

#ifdef __cplusplus
}
#endif

#endif // H723VG_V2_FREERTOS_BSP_BMI088_H