
#ifndef H723VG_V2_FREERTOS_NTFDCAN_ROUTER_H
#define H723VG_V2_FREERTOS_NTFDCAN_ROUTER_H

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 公共：类型与前置声明 ========================= */

// 原始 RX 回调（标准 ID）
typedef void (*canrx_handler_t)(uint16_t std_id, const uint8_t* data, uint8_t dlc, void* user);

// RX 统计（每总线一份）
typedef struct {
    uint32_t pops;          // 本轮从 RX 环取出的帧数累加
    uint32_t routed_ok;     // 已成功分发（回调/对象）计数
    uint32_t no_subscriber; // 无订阅者计数
    uint32_t wake_by_isr;   // 被 ISR 唤醒次数
    uint32_t wake_by_to;    // 超时唤醒次数
} canrx_stats_t;

// FDCAN register debug snapshot (for watch only)
#ifndef NTFDCAN_DEBUG_BUS_COUNT
#define NTFDCAN_DEBUG_BUS_COUNT 3u
#endif

typedef struct {
    uint32_t last_update_ms;
    uint32_t psr;
    uint32_t ecr;
    uint32_t ir;
    uint32_t txfqs;
    uint32_t rxf0s;
    uint32_t rxf1s;
    uint8_t  tx_free_level;
    uint8_t  tx_full;
    uint8_t  tx_put_idx;
    uint8_t  tx_get_idx;
    uint8_t  rx0_fill_level;
    uint8_t  rx0_full;
    uint8_t  rx0_lost;
    uint8_t  rx0_put_idx;
    uint8_t  rx0_get_idx;
    uint8_t  rx1_fill_level;
    uint8_t  rx1_full;
    uint8_t  rx1_lost;
    uint8_t  rx1_put_idx;
    uint8_t  rx1_get_idx;
    uint8_t  tec;
    uint8_t  rec;
    uint8_t  lec;
} fdcan_bus_debug_t;

typedef struct {
    fdcan_bus_debug_t bus[NTFDCAN_DEBUG_BUS_COUNT];
} fdcan_debug_t;

extern volatile fdcan_debug_t g_fdcan_debug;

// TX router debug snapshot (for watch only)
typedef struct {
    uint32_t send_ok_cnt;
    uint32_t send_busy_cnt;
    uint32_t send_fail_cnt;
} cantx_type_stats_t;

typedef struct {
    cantx_type_stats_t c620;
    cantx_type_stats_t gm6020;
    cantx_type_stats_t dm4310;
    cantx_type_stats_t dm4340;
    cantx_type_stats_t lk8016e;
    cantx_type_stats_t raw;
    uint32_t raw_enqueue_ok_cnt;
    uint32_t raw_enqueue_full_cnt;
} cantx_bus_stats_t;

typedef struct {
    cantx_bus_stats_t bus[NTFDCAN_DEBUG_BUS_COUNT];
} cantx_debug_t;

extern volatile cantx_debug_t g_cantx_debug;

// C++ 适配器前置声明（避免含入重型头）
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

/* ========================= RX 路由 API ========================= */

// 查询 RX 统计指针（只读，按总线 1..N）
const canrx_stats_t* canrx_get_stats(uint8_t bus);

// 原始 ID 订阅/退订（C 风格；同一 (id,cb,user) 去重）
int canrx_subscribe_raw   (uint8_t bus, uint16_t std_id, canrx_handler_t cb, void* user);
int canrx_unsubscribe_raw (uint8_t bus, uint16_t std_id, canrx_handler_t cb, void* user);

// 对象适配器注册（GM6020 / C620）；反注册传回原 obj 指针
int canrx_register_gm6020 (uint8_t bus, DJI_GM6020* obj);
int canrx_register_c620   (uint8_t bus, DJI_C620*   obj);
int canrx_register_dm4310(uint8_t bus, DM4310* obj);
int canrx_register_dm4340(uint8_t bus, DM4340* obj);
int canrx_register_lk8016e(uint8_t bus, LK8016E* obj);
int canrx_register_navi   (uint8_t bus, Navi* obj);
int canrx_register_vision (uint8_t bus, Vision* obj);
int canrx_register_navision(uint8_t bus, Navi* obj); // compatibility
int canrx_register_omximu (uint8_t bus, OmxImu* obj);
int canrx_unregister_adapter(uint8_t bus, void* obj);

// 由 BSP 在 ISR 里调用，用于唤醒对应 bus 的 RX 任务
void bsp_fdcan_on_rx_isr(uint8_t bus);

// RX 路由任务入口（argument 传总线号：1..N）
void Start_CAN_RxRouter(void* argument);

/* ========================= TX 路由 API ========================= */

// 原始帧非阻塞发送（任务层队列；由 TX Router 在周期内送入 BSP）
int cantx_send_raw(uint8_t bus, uint16_t sid, const uint8_t* data, uint8_t dlc, uint8_t prio);

// TX 路由任务入口（argument 传总线号：1..N）
void Start_CAN_TxRouter(void* argument);

int  cantx_send_raw(uint8_t bus, uint16_t sid, const uint8_t* data, uint8_t dlc, uint8_t prio);

// —— 对象注册：默认 1ms；扩展接口可声明 period_ms（1..1000ms）——
int  cantx_register_c620       (uint8_t bus, DJI_C620*   obj, uint32_t period_ms);
int  cantx_register_gm6020     (uint8_t bus, DJI_GM6020* obj, uint32_t period_ms);
int  cantx_register_dm4310     (uint8_t bus, DM4310* obj, uint32_t period_ms);
int  cantx_register_dm4340     (uint8_t bus, DM4340* obj, uint32_t period_ms);
int  cantx_register_lk8016e    (uint8_t bus, LK8016E* obj, uint32_t period_ms);

// —— 立即发送一帧（旁路 Router 周期）——
bool cantx_send_now    (uint8_t bus, uint16_t sid, const uint8_t* data, uint8_t len, uint32_t timeout_us);

// TX 路由任务（每总线一个）
void Start_CAN_TxRouter(void* argument);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // H723VG_V2_FREERTOS_NTFDCAN_ROUTER_H
