// === CAN_TxRouter.cpp ===
// 功能：
//  - 每对象可声明发送周期（period_ms：1..1000 ms；默认=1ms 兼容旧接口）
//  - 周期性聚合对象（C620/GM6020 等）16-bit 指令 -> 8B 帧（group_id + slot0..3）
//  - 原始帧队列 cantx_send_raw()
//  - 立即单帧发送 cantx_send_now()/cantx_send_now_ext()（旁路周期）
//
// 依赖：cmsis_os2.h, stm32h7xx_hal.h, bsp_fdcan.h
//       lib_adp_dji_c620.h, lib_adp_dji_gm6020.h, NTFDCAN_Router.h

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

#include "NTFDCAN_Router.h"
#include "bsp_fdcan.h"

// ---- Optional adapter headers (allow per-project availability) ----
#ifndef NTFDCAN_HAS_ADP_DJI_C620
  #if defined(__has_include)
    #if __has_include("lib_adp_dji_c620.h")
      #define NTFDCAN_HAS_ADP_DJI_C620 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_DJI_C620
  #define NTFDCAN_HAS_ADP_DJI_C620 0
#endif

#ifndef NTFDCAN_HAS_ADP_DJI_GM6020
  #if defined(__has_include)
    #if __has_include("lib_adp_dji_gm6020.h")
      #define NTFDCAN_HAS_ADP_DJI_GM6020 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_DJI_GM6020
  #define NTFDCAN_HAS_ADP_DJI_GM6020 0
#endif

#ifndef NTFDCAN_HAS_ADP_DM_4310
  #if defined(__has_include)
    #if __has_include("lib_adp_dm_4310.h")
      #define NTFDCAN_HAS_ADP_DM_4310 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_DM_4310
  #define NTFDCAN_HAS_ADP_DM_4310 0
#endif

#ifndef NTFDCAN_HAS_ADP_DM_4340
  #if defined(__has_include)
    #if __has_include("lib_adp_dm_4340.h")
      #define NTFDCAN_HAS_ADP_DM_4340 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_DM_4340
  #define NTFDCAN_HAS_ADP_DM_4340 0
#endif

#ifndef NTFDCAN_HAS_ADP_LK_MG8016E
  #if defined(__has_include)
    #if __has_include("lib_adp_lk_mg8016e.h")
      #define NTFDCAN_HAS_ADP_LK_MG8016E 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_LK_MG8016E
  #define NTFDCAN_HAS_ADP_LK_MG8016E 0
#endif

#if NTFDCAN_HAS_ADP_DJI_C620
#include "lib_adp_dji_c620.h"
#endif
#if NTFDCAN_HAS_ADP_DJI_GM6020
#include "lib_adp_dji_gm6020.h"
#endif
#if NTFDCAN_HAS_ADP_DM_4310
#include "lib_adp_dm_4310.h"
#endif
#if NTFDCAN_HAS_ADP_DM_4340
#include "lib_adp_dm_4340.h"
#endif
#if NTFDCAN_HAS_ADP_LK_MG8016E
#include "lib_adp_lk_mg8016e.h"
#endif

volatile cantx_debug_t g_cantx_debug = {};


// ======== 配置 ========
#ifndef CANTX_MAX_BUS
#define CANTX_MAX_BUS            3
#endif
#ifndef CANTX_MAX_OBJ
#define CANTX_MAX_OBJ            24     // 每总线最大对象数（C620+GM6020 总和）
#endif
#ifndef CANTX_MAX_GROUPS
#define CANTX_MAX_GROUPS         24     // 每总线最大 group 数
#endif
#ifndef CANTX_RAW_Q_LEN
#define CANTX_RAW_Q_LEN          96     // 原始队列长度
#endif
#ifndef CANTX_PERIOD_US
#define CANTX_PERIOD_US          1000   // Router 基础节拍（默认 1kHz）
#endif
#ifndef CANTX_TASK_WAIT_ON_BUSY
#define CANTX_TASK_WAIT_ON_BUSY   0     // 控制帧推荐开：轻阻塞等位更稳
#endif

// ---- debug type mask ----
#define CANTX_TYPE_C620        0x01u
#define CANTX_TYPE_GM6020      0x02u
#define CANTX_TYPE_DM4310      0x04u
#define CANTX_TYPE_LK8016E     0x08u
#define CANTX_TYPE_RAW         0x10u
#define CANTX_TYPE_DM4340      0x20u

// —— 发送周期界限（ms）——
#define CANTX_PERIOD_MS_MIN       1u      // 最快 1ms
#define CANTX_PERIOD_MS_MAX       1000u   // 最慢 1000ms = 1s

// ======== 适配器统一查询接口（16-bit 指令） ========
typedef bool (*adapter_query_tx16_fn)(void* obj, uint16_t* group_id, uint8_t* slot, int16_t* value);
typedef bool (*adapter_query_raw8_fn)(void* obj, uint16_t* sid, uint8_t out[8]);

typedef struct {
    void*                   obj;
    adapter_query_raw8_fn   query;
    uint32_t                period_ms;
    uint32_t                next_due_tick;
    uint8_t                 type_mask;
} tx_obj_raw8_t;

// ======== 每对象条目：含独立 period 与下一次到期 tick ========
typedef struct {
    void*                  obj;
    adapter_query_tx16_fn  query;
    uint32_t               period_ms;     // 1..1000
    uint32_t               next_due_tick; // osKernel tick（按本 bus 的 tick_hz）
    int16_t                last_sent_val;
} tx_obj_t;

// ======== 分组聚合结构 ========
typedef struct {
    uint16_t group_id;       // 标准 ID
    bool     valid;
    uint8_t  type_mask;
    bool     slot_used[4];
    int16_t  slot_val [4];   // 16-bit 值
} tx_group_acc_t;

// ======== 原始队列项 ========
typedef struct {
    uint16_t sid;            // standard id
    uint8_t  dlc;            // 0..8
    uint8_t  data[8];
    uint8_t  prio;           // 0 高 1 中 2 低（目前先到先发）
} raw_item_t;

// ======== 每总线上下文 ========
#ifndef CANTX_MAX_DM_RAW8
#define CANTX_MAX_DM_RAW8  16
#endif

typedef struct {
    osEventFlagsId_t  evt;
    uint32_t          tick_hz;
    tx_obj_t          objs[CANTX_MAX_OBJ];       // (C620/GM6020 的 16-bit 通道)
    raw_item_t        raw_q[CANTX_RAW_Q_LEN];    // 原始队列
    volatile uint16_t raw_head, raw_tail;
    tx_group_acc_t    groups[CANTX_MAX_GROUPS];  // 分组聚合

    tx_obj_raw8_t     dm_raw8[CANTX_MAX_DM_RAW8]; // 8B 通道（DM4310 / LK8016E）
} bus_ctx_t;


static bus_ctx_t* ctx_of(uint8_t bus) {
    static bus_ctx_t s_ctx[CANTX_MAX_BUS + 1]; // 1..MAX 有效
    return (bus >= 1 && bus <= CANTX_MAX_BUS) ? &s_ctx[bus] : nullptr;
}

static inline volatile cantx_bus_stats_t* cantx_debug_bus(uint8_t bus);

static inline uint32_t ms_to_ticks(uint32_t ms, uint32_t tick_hz) {
    if (ms == 0) return 1; // 不允许 0，最少 1tick
    // 近似四舍五入： (ms * tick_hz + 999) / 1000
    uint64_t num = (uint64_t)ms * (uint64_t)tick_hz + 999ull;
    uint32_t t   = (uint32_t)(num / 1000ull);
    return (t == 0) ? 1u : t;
}

// ======== 内部：查找/分配 group ========
static tx_group_acc_t* find_or_alloc_group(bus_ctx_t* c, uint16_t gid) {
    for (int i = 0; i < CANTX_MAX_GROUPS; ++i) {
        if (c->groups[i].valid && c->groups[i].group_id == gid) return &c->groups[i];
    }
    for (int i = 0; i < CANTX_MAX_GROUPS; ++i) {
        if (!c->groups[i].valid) {
            c->groups[i].valid    = true;
            c->groups[i].group_id = gid;
            c->groups[i].type_mask = 0;
            for (int k = 0; k < 4; ++k) { c->groups[i].slot_used[k] = false; c->groups[i].slot_val[k] = 0; }
            return &c->groups[i];
        }
    }
    return nullptr; // 满了
}

/* ========================= 立即单帧发送（旁路周期） ========================= */

extern "C" bool cantx_send_now(uint8_t bus, uint16_t sid, const uint8_t* data,
                               uint8_t len, uint32_t timeout_us)
{
    if (!data) return false;
    if (len > 8) len = 8;

    const bool in_isr = (__get_IPSR() != 0);
    if (in_isr || timeout_us == 0) {
        return bsp_fdcan_tx_push(bus, sid, data, len);
    }

    const uint32_t tick_hz = osKernelGetTickFreq();
    const uint32_t start   = osKernelGetTickCount();
    const uint32_t budget  = (timeout_us * tick_hz + 999999u) / 1000000u;
    const uint32_t min_wait = (budget ? budget : 1u);

    for (;;) {
        if (bsp_fdcan_tx_push(bus, sid, data, len)) return true;
        if ((osKernelGetTickCount() - start) >= min_wait) return false;
        osDelay(0);
    }
}

/* ========================= 原始帧入队 ========================= */

extern "C" int cantx_send_raw(uint8_t bus, uint16_t sid, const uint8_t* data, uint8_t dlc, uint8_t prio)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !data || dlc > 8) return -1;
    volatile cantx_bus_stats_t* dbg = cantx_debug_bus(bus);

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    uint16_t next = (uint16_t)((c->raw_tail + 1) % CANTX_RAW_Q_LEN);
    if (next == c->raw_head) {
        if (dbg) dbg->raw_enqueue_full_cnt++;
        if (!ps) __enable_irq();
        return -2; // 队满
    }

    raw_item_t* it = &c->raw_q[c->raw_tail];
    it->sid  = sid;
    it->dlc  = dlc;
    it->prio = prio;
    for (uint8_t i = 0; i < dlc; ++i) it->data[i] = data[i];
    for (uint8_t i = dlc; i < 8;  ++i) it->data[i] = 0;

    c->raw_tail = next;
    if (dbg) dbg->raw_enqueue_ok_cnt++;
    if (!ps) __enable_irq();

    if (c->evt) (void)osEventFlagsSet(c->evt, 0x2);
    return 0;
}

/* ========================= 适配器薄查询 & 注册 ========================= */

#if NTFDCAN_HAS_ADP_DJI_C620
static bool query_c620   (void* o, uint16_t* gid, uint8_t* slot, int16_t* v) {
    auto* m = reinterpret_cast<DJI_C620*>(o);
    return m->exportTx16(gid, slot, v);
}
#endif
#if NTFDCAN_HAS_ADP_DJI_GM6020
static bool query_gm6020 (void* o, uint16_t* gid, uint8_t* slot, int16_t* v) {
    auto* m = reinterpret_cast<DJI_GM6020*>(o);
    return m->exportTx16(gid, slot, v);
}
#endif
#if NTFDCAN_HAS_ADP_DM_4310
static bool query_dm4310_raw8(void* o, uint16_t* sid, uint8_t out[8]) {
    auto* m = reinterpret_cast<DM4310*>(o);
    return m->exportTxRaw8(sid, out); // 返回 true 表示本轮有帧可发
}
#endif
#if NTFDCAN_HAS_ADP_DM_4340
static bool query_dm4340_raw8(void* o, uint16_t* sid, uint8_t out[8]) {
    auto* m = reinterpret_cast<DM4340*>(o);
    return m->exportTxRaw8(sid, out);
}
#endif
#if NTFDCAN_HAS_ADP_LK_MG8016E
static bool query_lk8016e_raw8(void* o, uint16_t* sid, uint8_t out[8]) {
    auto* m = reinterpret_cast<LK8016E*>(o);
    return m->exportTxRaw8(sid, out);
}
#endif

static inline uint8_t cantx_type_mask_from_query(adapter_query_tx16_fn fn)
{
    uint8_t mask = 0;
#if NTFDCAN_HAS_ADP_DJI_C620
    if (fn == query_c620) mask |= CANTX_TYPE_C620;
#endif
#if NTFDCAN_HAS_ADP_DJI_GM6020
    if (fn == query_gm6020) mask |= CANTX_TYPE_GM6020;
#endif
    return mask;
}

static inline volatile cantx_bus_stats_t* cantx_debug_bus(uint8_t bus)
{
    if (bus < 1 || bus > NTFDCAN_DEBUG_BUS_COUNT) return nullptr;
    return &g_cantx_debug.bus[bus - 1];
}

static inline void cantx_debug_inc_ok(volatile cantx_bus_stats_t* d, uint8_t mask)
{
    if (!d) return;
    if (mask & CANTX_TYPE_C620)   d->c620.send_ok_cnt++;
    if (mask & CANTX_TYPE_GM6020) d->gm6020.send_ok_cnt++;
    if (mask & CANTX_TYPE_DM4310) d->dm4310.send_ok_cnt++;
    if (mask & CANTX_TYPE_DM4340) d->dm4340.send_ok_cnt++;
    if (mask & CANTX_TYPE_LK8016E) d->lk8016e.send_ok_cnt++;
    if (mask & CANTX_TYPE_RAW)    d->raw.send_ok_cnt++;
}

static inline void cantx_debug_inc_busy(volatile cantx_bus_stats_t* d, uint8_t mask)
{
    if (!d) return;
    if (mask & CANTX_TYPE_C620)   d->c620.send_busy_cnt++;
    if (mask & CANTX_TYPE_GM6020) d->gm6020.send_busy_cnt++;
    if (mask & CANTX_TYPE_DM4310) d->dm4310.send_busy_cnt++;
    if (mask & CANTX_TYPE_DM4340) d->dm4340.send_busy_cnt++;
    if (mask & CANTX_TYPE_LK8016E) d->lk8016e.send_busy_cnt++;
    if (mask & CANTX_TYPE_RAW)    d->raw.send_busy_cnt++;
}

static inline void cantx_debug_inc_fail(volatile cantx_bus_stats_t* d, uint8_t mask)
{
    if (!d) return;
    if (mask & CANTX_TYPE_C620)   d->c620.send_fail_cnt++;
    if (mask & CANTX_TYPE_GM6020) d->gm6020.send_fail_cnt++;
    if (mask & CANTX_TYPE_DM4310) d->dm4310.send_fail_cnt++;
    if (mask & CANTX_TYPE_DM4340) d->dm4340.send_fail_cnt++;
    if (mask & CANTX_TYPE_LK8016E) d->lk8016e.send_fail_cnt++;
    if (mask & CANTX_TYPE_RAW)    d->raw.send_fail_cnt++;
}

static bool cantx_tx_push_dbg(uint8_t bus, uint16_t sid, const uint8_t* data, uint8_t dlc,
                              volatile cantx_bus_stats_t* dbg, uint8_t type_mask)
{
#if CANTX_TASK_WAIT_ON_BUSY
    const uint32_t timeout_us = 200;
    const uint32_t t_hz = osKernelGetTickFreq();
    const uint32_t start = osKernelGetTickCount();
    const uint32_t budget = (timeout_us * t_hz + 999999u) / 1000000u;
    const uint32_t min_wait = (budget ? budget : 1u);
    for (;;) {
        if (bsp_fdcan_tx_push(bus, sid, data, dlc)) {
            cantx_debug_inc_ok(dbg, type_mask);
            return true;
        }
        cantx_debug_inc_busy(dbg, type_mask);
        if ((osKernelGetTickCount() - start) >= min_wait) {
            cantx_debug_inc_fail(dbg, type_mask);
            return false;
        }
        osDelay(0);
    }
#else
    if (bsp_fdcan_tx_push(bus, sid, data, dlc)) {
        cantx_debug_inc_ok(dbg, type_mask);
        return true;
    }
    cantx_debug_inc_busy(dbg, type_mask);
    cantx_debug_inc_fail(dbg, type_mask);
    return false;
#endif
}

static uint32_t clamp_period_ms(uint32_t ms) {
    if (ms < CANTX_PERIOD_MS_MIN)   ms = CANTX_PERIOD_MS_MIN;
    if (ms > CANTX_PERIOD_MS_MAX)   ms = CANTX_PERIOD_MS_MAX;
    return ms;
}

static int register_obj_ex(uint8_t bus, void* obj, adapter_query_tx16_fn fn, uint32_t period_ms)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj || !fn) return -1;

    period_ms = clamp_period_ms(period_ms);

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重：同一对象+函数不允许重复
    for (int i = 0; i < CANTX_MAX_OBJ; ++i) {
        if (c->objs[i].obj == obj && c->objs[i].query == fn) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    // 填空
    for (int i = 0; i < CANTX_MAX_OBJ; ++i) {
        if (!c->objs[i].query) {
            c->objs[i].obj   = obj;
            c->objs[i].query = fn;
            c->objs[i].period_ms = period_ms;
            // 将 next_due 设为“当前 tick”，实现注册后**立刻**有资格在下次 flush 被采样
            uint32_t now = osKernelGetTickCount();
            c->objs[i].next_due_tick = now;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3; // 表满
}

// 扩展注册（可指定 1..1000ms）
#if NTFDCAN_HAS_ADP_DJI_C620
extern "C" int cantx_register_c620(uint8_t bus, DJI_C620* obj, uint32_t period_ms) {
    return register_obj_ex(bus, obj, query_c620,   period_ms);
}
#else
extern "C" int cantx_register_c620(uint8_t bus, DJI_C620* obj, uint32_t period_ms) {
    (void)bus; (void)obj; (void)period_ms;
    return -1;
}
#endif
#if NTFDCAN_HAS_ADP_DJI_GM6020
extern "C" int cantx_register_gm6020(uint8_t bus, DJI_GM6020* obj, uint32_t period_ms) {
    return register_obj_ex(bus, obj, query_gm6020, period_ms);
}
#else
extern "C" int cantx_register_gm6020(uint8_t bus, DJI_GM6020* obj, uint32_t period_ms) {
    (void)bus; (void)obj; (void)period_ms;
    return -1;
}
#endif
#if NTFDCAN_HAS_ADP_DM_4310
extern "C" int cantx_register_dm4310(uint8_t bus, DM4310* obj, uint32_t period_ms)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    period_ms = (period_ms < 1 ? 1 : (period_ms > 1000 ? 1000 : period_ms)); // 与现有限幅一致

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
        if (c->dm_raw8[i].obj == obj && c->dm_raw8[i].query == query_dm4310_raw8) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    // 填空
    for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
        if (!c->dm_raw8[i].query) {
            c->dm_raw8[i].obj           = obj;
            c->dm_raw8[i].query         = query_dm4310_raw8;
            c->dm_raw8[i].period_ms     = period_ms;
            c->dm_raw8[i].next_due_tick = osKernelGetTickCount();
            c->dm_raw8[i].type_mask     = CANTX_TYPE_DM4310;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3; // 表满
}
#else
extern "C" int cantx_register_dm4310(uint8_t bus, DM4310* obj, uint32_t period_ms)
{
    (void)bus; (void)obj; (void)period_ms;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_DM_4340
extern "C" int cantx_register_dm4340(uint8_t bus, DM4340* obj, uint32_t period_ms)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    period_ms = (period_ms < 1 ? 1 : (period_ms > 1000 ? 1000 : period_ms));

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
        if (c->dm_raw8[i].obj == obj && c->dm_raw8[i].query == query_dm4340_raw8) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    // 填空
    for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
        if (!c->dm_raw8[i].query) {
            c->dm_raw8[i].obj           = obj;
            c->dm_raw8[i].query         = query_dm4340_raw8;
            c->dm_raw8[i].period_ms     = period_ms;
            c->dm_raw8[i].next_due_tick = osKernelGetTickCount();
            c->dm_raw8[i].type_mask     = CANTX_TYPE_DM4340;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3;
}
#else
extern "C" int cantx_register_dm4340(uint8_t bus, DM4340* obj, uint32_t period_ms)
{
    (void)bus; (void)obj; (void)period_ms;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_LK_MG8016E
extern "C" int cantx_register_lk8016e(uint8_t bus, LK8016E* obj, uint32_t period_ms)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    period_ms = (period_ms < 1 ? 1 : (period_ms > 1000 ? 1000 : period_ms));

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
        if (c->dm_raw8[i].obj == obj && c->dm_raw8[i].query == query_lk8016e_raw8) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    // 填空
    for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
        if (!c->dm_raw8[i].query) {
            c->dm_raw8[i].obj           = obj;
            c->dm_raw8[i].query         = query_lk8016e_raw8;
            c->dm_raw8[i].period_ms     = period_ms;
            c->dm_raw8[i].next_due_tick = osKernelGetTickCount();
            c->dm_raw8[i].type_mask     = CANTX_TYPE_LK8016E;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3; // 表满
}
#else
extern "C" int cantx_register_lk8016e(uint8_t bus, LK8016E* obj, uint32_t period_ms)
{
    (void)bus; (void)obj; (void)period_ms;
    return -1;
}
#endif

/* ========================= 一次聚合 + 发送（按对象周期） ========================= */

static void txrouter_flush_once(uint8_t bus, bus_ctx_t* c)
{
    // 0) 清空分组缓存
    for (int i = 0; i < CANTX_MAX_GROUPS; ++i) {
        c->groups[i].valid = false;
        c->groups[i].type_mask = 0;
        for (int k = 0; k < 4; ++k) {
            c->groups[i].slot_used[k] = false;
            c->groups[i].slot_val [k] = 0;
        }
    }

    volatile cantx_bus_stats_t* dbg = cantx_debug_bus(bus);
    const uint32_t now_tick = osKernelGetTickCount();

    // 1) 第一轮：仅处理“到期”的对象
    //    - 采样当前值，写入所属 group 的槽位
    //    - 更新对象的 last_sent_val
    //    - 推进 next_due_tick
    for (int i = 0; i < CANTX_MAX_OBJ; ++i) {
        tx_obj_t& o = c->objs[i];
        if (!o.query) continue;

        // 到期判断（now >= next_due）
        if ((int32_t)(now_tick - o.next_due_tick) < 0) continue;

        uint16_t gid; uint8_t slot; int16_t val;
        if (!o.query(o.obj, &gid, &slot, &val) || slot > 3) {
            // 即使 query 失败，也推进下一次到期，避免“卡死等值”
            o.next_due_tick = now_tick + ms_to_ticks(o.period_ms, c->tick_hz);
            continue;
        }

        tx_group_acc_t* g = find_or_alloc_group(c, gid);
        if (g) {
            g->slot_used[slot] = true;   // 标记本槽位“本轮到期”
            g->slot_val [slot] = val;    // 用当前值
            g->type_mask      |= cantx_type_mask_from_query(o.query);
            o.last_sent_val    = val;    // 记忆已发送值（供影子补齐用）
        }

        // 安排下一次到期
        o.next_due_tick = now_tick + ms_to_ticks(o.period_ms, c->tick_hz);
    }

    // 2) 第二轮：对“本轮至少有一个槽位到期”的 group，用 last_sent_val 填充其余未到期槽位
    //    - 不更新这些对象的 next_due_tick（保持它们自己的节拍）
    for (int gi = 0; gi < CANTX_MAX_GROUPS; ++gi) {
        tx_group_acc_t* g = &c->groups[gi];
        if (!g->valid) continue;

        bool any_due = false;
        for (int k = 0; k < 4; ++k) { if (g->slot_used[k]) { any_due = true; break; } }
        if (!any_due) continue; // 没有到期槽的 group 不发、也不需要补齐

        // 扫描所有对象：找到同组但本轮未到期的槽位，填 last_sent_val
        for (int i = 0; i < CANTX_MAX_OBJ; ++i) {
            tx_obj_t& o = c->objs[i];
            if (!o.query) continue;

            uint16_t gid; uint8_t slot; int16_t dummy;
            if (!o.query(o.obj, &gid, &slot, &dummy) || slot > 3) continue;
            if (gid != g->group_id) continue;
            if (g->slot_used[slot]) continue; // 本轮已到期的不需要影子

            g->slot_used[slot] = true;              // 作为“影子”参与组帧
            g->slot_val [slot] = o.last_sent_val;   // 用上次发送值，避免被清零
            g->type_mask      |= cantx_type_mask_from_query(o.query);
            // 注意：不动 o.next_due_tick —— 让它按原计划到期
        }
    }

    // 3) 先处理原始队列（先到先发；如需优先级可扩展）
    while (c->raw_head != c->raw_tail) {
        raw_item_t it;
        {
            uint32_t ps = __get_PRIMASK(); __disable_irq();
            raw_item_t* src = &c->raw_q[c->raw_head];
            it = *src;
            c->raw_head = (uint16_t)((c->raw_head + 1) % CANTX_RAW_Q_LEN);
            if (!ps) __enable_irq();
        }
        (void)cantx_tx_push_dbg(bus, it.sid, it.data, it.dlc, dbg, CANTX_TYPE_RAW);
    }

    // 4) DM4310 / LK8016E 原始 8B 通道：独立按周期发送（不依赖 group 是否到期）
    {
        const uint32_t now_tick = osKernelGetTickCount();
        for (int i = 0; i < CANTX_MAX_DM_RAW8; ++i) {
            tx_obj_raw8_t& o = c->dm_raw8[i];
            if (!o.query) continue;
            if ((int32_t)(now_tick - o.next_due_tick) < 0) continue;

            uint16_t sid; uint8_t buf[8];
            if (o.query(o.obj, &sid, buf)) {
                (void)cantx_tx_push_dbg(bus, sid, buf, 8, dbg, o.type_mask);
            }
            o.next_due_tick = now_tick + ms_to_ticks(o.period_ms, c->tick_hz);
        }
    }

    // 5) 发送每个“本轮至少有一个槽位到期”的 group（一帧 4×int16，**大端**）
    for (int i = 0; i < CANTX_MAX_GROUPS; ++i) {
        tx_group_acc_t* g = &c->groups[i];
        if (!g->valid) continue;

        bool any_due = false;
        for (int k = 0; k < 4; ++k) { if (g->slot_used[k]) { any_due = true; break; } }
        if (!any_due) continue;

        const uint16_t sid = g->group_id;

        uint8_t buf[8];
        for (int k = 0; k < 4; ++k) {
            int16_t v = g->slot_val[k];
            buf[2*k+0] = (uint8_t)((v >> 8) & 0xFF); // BE 高字节在前
            buf[2*k+1] = (uint8_t)((v >> 0) & 0xFF);
        }
        (void)cantx_tx_push_dbg(bus, sid, buf, 8, dbg, g->type_mask);
    }
}

/* ========================= 任务入口 ========================= */

extern "C" void Start_CAN_TxRouter(void* argument)
{
    uint8_t bus = (uint8_t)(uintptr_t)argument;
    bus_ctx_t* c = ctx_of(bus);
    if (!c) return;

    c->evt = osEventFlagsNew(nullptr);
    c->raw_head = c->raw_tail = 0;
    c->tick_hz = osKernelGetTickFreq();

    (void)bsp_fdcan_start(bus);

    // 基础节拍（用于“扫描是否到期”，不等于每对象的真实 period）
    const uint32_t tick_hz = c->tick_hz; (void)tick_hz;
    const uint32_t period_ticks = ms_to_ticks((CANTX_PERIOD_US + 999u) / 1000u, c->tick_hz);
    uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(last_wake + (period_ticks ? period_ticks : 1));
        last_wake = osKernelGetTickCount();

        (void)osEventFlagsClear(c->evt, 0x2); // 清“原始入队”标志
        txrouter_flush_once(bus, c);
    }
}
