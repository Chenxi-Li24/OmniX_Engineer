//
// EXFDCAN_Router.h - MCP2518FD EXTI-driven TX/RX test routers
//
#ifndef H723VG_V2_FREERTOS_EXFDCAN_ROUTER_H
#define H723VG_V2_FREERTOS_EXFDCAN_ROUTER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXFDCAN_MAX_DATA_BYTES   (8u)

typedef struct
{
    uint16_t sid;
    uint8_t  dlc;
    uint8_t  data[EXFDCAN_MAX_DATA_BYTES];
    uint32_t timestamp_ms;
} exfdcan_rx_frame_t;

typedef struct
{
    uint32_t rx_irq;
    uint32_t tx_irq;
    uint32_t int_irq;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_overflow;
    uint32_t rx_errors;
    uint32_t tx_errors;
    uint32_t tx_full;
    uint32_t module_events;
    uint32_t isr_int0;
    uint32_t isr_int1;
    uint32_t rx_irq_no_rxif;
    uint32_t tx_irq_no_txif;
    uint32_t last_int_flags;
    uint32_t last_rx_status;
    uint32_t last_tx_status;
} exfdcan_stats_t;

// EXFDCAN motor/router compatibility types (mirrors NTFDCAN usage).
typedef void (*excanrx_handler_t)(uint16_t std_id, const uint8_t* data, uint8_t dlc, void* user);

typedef struct {
    uint32_t pops;
    uint32_t routed_ok;
    uint32_t no_subscriber;
    uint32_t wake_by_isr;
    uint32_t wake_by_to;
} excanrx_stats_t;

#ifdef __cplusplus
    class DJI_GM6020;
    class DJI_C620;
    class DM4310;
    class DM4340;
    class LK8016E;
    class Navi;
    class Vision;
    class OmxImu;
#else
    typedef struct DJI_GM6020 DJI_GM6020;
    typedef struct DJI_C620 DJI_C620;
    typedef struct DM4310 DM4310;
    typedef struct DM4340 DM4340;
    typedef struct LK8016E LK8016E;
    typedef struct Navi Navi;
    typedef struct Vision Vision;
    typedef struct OmxImu OmxImu;
#endif

// Debug stats array for debugger watch (index = device id).
extern volatile exfdcan_stats_t g_exfdcan_stats[];

// Called from HAL_GPIO_EXTI_Callback; only sets flags.
void exfdcan_on_exti(uint16_t gpio_pin);

// Optional readout helpers for debugging.
const volatile exfdcan_stats_t* exfdcan_get_stats(uint8_t dev);
size_t exfdcan_rx_read(uint8_t dev, exfdcan_rx_frame_t *out, size_t max);

// EXFDCAN RX router API (signatures mirror NTFDCAN).
const excanrx_stats_t* excanrx_get_stats(uint8_t dev);
int excanrx_subscribe_raw(uint8_t dev, uint16_t std_id, excanrx_handler_t cb, void* user);
int excanrx_unsubscribe_raw(uint8_t dev, uint16_t std_id, excanrx_handler_t cb, void* user);
int excanrx_register_gm6020(uint8_t dev, DJI_GM6020* obj);
int excanrx_register_c620(uint8_t dev, DJI_C620* obj);
int excanrx_register_dm4310(uint8_t dev, DM4310* obj);
int excanrx_register_dm4340(uint8_t dev, DM4340* obj);
int excanrx_register_lk8016e(uint8_t dev, LK8016E* obj);
int excanrx_register_navi(uint8_t dev, Navi* obj);
int excanrx_register_vision(uint8_t dev, Vision* obj);
int excanrx_register_navision(uint8_t dev, Navi* obj);
int excanrx_register_omximu(uint8_t dev, OmxImu* obj);
int excanrx_unregister_adapter(uint8_t dev, void* obj);

// EXFDCAN TX router API (signatures mirror NTFDCAN).
int excantx_send_raw(uint8_t dev, uint16_t sid, const uint8_t* data, uint8_t dlc, uint8_t prio);
int excantx_register_c620(uint8_t dev, DJI_C620* obj, uint32_t period_ms);
int excantx_register_gm6020(uint8_t dev, DJI_GM6020* obj, uint32_t period_ms);
int excantx_register_dm4310(uint8_t dev, DM4310* obj, uint32_t period_ms);
int excantx_register_dm4340(uint8_t dev, DM4340* obj, uint32_t period_ms);
int excantx_register_lk8016e(uint8_t dev, LK8016E* obj, uint32_t period_ms);
bool excantx_send_now(uint8_t dev, uint16_t sid, const uint8_t* data, uint8_t len, uint32_t timeout_us);

// Router task entries (argument: device index).
void Start_EXFDCAN_RxRouter(void *argument);
void Start_EXFDCAN_TxRouter(void *argument);

#ifdef __cplusplus
}
#endif

#endif // H723VG_V2_FREERTOS_EXFDCAN_ROUTER_H
