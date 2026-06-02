//
// tskptt_exfdcan_tx_router.cpp - MCP2518FD TX task (INT0 -> TX status)
//
#include "EXFDCAN_Router_Internal.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_defines.h"
#include "bsp_srn_log.h"
#include "stm32h7xx_hal.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

#include "lib_adp_dji_c620.h"
#include "lib_adp_dji_gm6020.h"
#include "lib_adp_dm_4310.h"
#include "lib_adp_dm_4340.h"

// ---- Optional adapter headers (allow per-project availability) ----
#ifndef EXFDCAN_HAS_ADP_LK_MG8016E
  #if defined(__has_include)
    #if __has_include("lib_adp_lk_mg8016e.h")
      #define EXFDCAN_HAS_ADP_LK_MG8016E 1
    #endif
  #endif
#endif
#ifndef EXFDCAN_HAS_ADP_LK_MG8016E
  #define EXFDCAN_HAS_ADP_LK_MG8016E 0
#endif

#if EXFDCAN_HAS_ADP_LK_MG8016E
#include "lib_adp_lk_mg8016e.h"
#endif

#ifndef EXCANTX_MAX_DEV
#define EXCANTX_MAX_DEV          EXFDCAN_DEV_COUNT
#endif
#ifndef EXCANTX_MAX_OBJ
#define EXCANTX_MAX_OBJ          24
#endif
#ifndef EXCANTX_MAX_GROUPS
#define EXCANTX_MAX_GROUPS       24
#endif
#ifndef EXCANTX_RAW_Q_LEN
#define EXCANTX_RAW_Q_LEN         96
#endif
#ifndef EXCANTX_PERIOD_US
#define EXCANTX_PERIOD_US         1000
#endif
#ifndef EXCANTX_TASK_WAIT_ON_BUSY
#define EXCANTX_TASK_WAIT_ON_BUSY 0
#endif
#ifndef EXCANTX_MAX_DM_RAW8
#define EXCANTX_MAX_DM_RAW8       16
#endif

#define EXCANTX_PERIOD_MS_MIN     1u
#define EXCANTX_PERIOD_MS_MAX     1000u

// ======== Adapter query interface (16-bit slots) ========
typedef bool (*adapter_query_tx16_fn)(void* obj, uint16_t* group_id, uint8_t* slot, int16_t* value);
typedef bool (*adapter_query_raw8_fn)(void* obj, uint16_t* sid, uint8_t out[8]);

typedef struct {
    void*                   obj;
    adapter_query_raw8_fn   query;
    uint32_t                period_ms;
    uint32_t                next_due_tick;
} tx_obj_raw8_t;

typedef struct {
    void*                  obj;
    adapter_query_tx16_fn  query;
    uint32_t               period_ms;
    uint32_t               next_due_tick;
    int16_t                last_sent_val;
} tx_obj_t;

typedef struct {
    uint16_t group_id;
    bool     valid;
    bool     slot_used[4];
    int16_t  slot_val[4];
} tx_group_acc_t;

typedef struct {
    uint16_t sid;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  prio;
} raw_item_t;

typedef struct {
    osEventFlagsId_t  evt;
    uint32_t          tick_hz;
    tx_obj_t          objs[EXCANTX_MAX_OBJ];
    raw_item_t        raw_q[EXCANTX_RAW_Q_LEN];
    volatile uint16_t raw_head, raw_tail;
    tx_group_acc_t    groups[EXCANTX_MAX_GROUPS];
    tx_obj_raw8_t     dm_raw8[EXCANTX_MAX_DM_RAW8];
} excantx_bus_ctx_t;

extern "C" int excantx_send_raw(uint8_t dev, uint16_t sid, const uint8_t* data, uint8_t dlc, uint8_t prio);

static excantx_bus_ctx_t* ctx_of(uint8_t dev)
{
    static excantx_bus_ctx_t s_ctx[EXCANTX_MAX_DEV];
    return (dev < EXCANTX_MAX_DEV) ? &s_ctx[dev] : nullptr;
}

static inline uint32_t ms_to_ticks(uint32_t ms, uint32_t tick_hz)
{
    if (ms == 0) return 1;
    uint64_t num = (uint64_t)ms * (uint64_t)tick_hz + 999ull;
    uint32_t t   = (uint32_t)(num / 1000ull);
    return (t == 0) ? 1u : t;
}

static tx_group_acc_t* find_or_alloc_group(excantx_bus_ctx_t* c, uint16_t gid)
{
    for (int i = 0; i < EXCANTX_MAX_GROUPS; ++i) {
        if (c->groups[i].valid && c->groups[i].group_id == gid) return &c->groups[i];
    }
    for (int i = 0; i < EXCANTX_MAX_GROUPS; ++i) {
        if (!c->groups[i].valid) {
            c->groups[i].valid    = true;
            c->groups[i].group_id = gid;
            for (int k = 0; k < 4; ++k) { c->groups[i].slot_used[k] = false; c->groups[i].slot_val[k] = 0; }
            return &c->groups[i];
        }
    }
    return nullptr;
}

static bool exfdcan_tx_push(uint8_t dev, exfdcan_ctx_t *ctx, uint16_t sid, const uint8_t* data, uint8_t len)
{
    if (!ctx || !data)
        return false;

    if (len > EXFDCAN_MAX_DATA_BYTES)
        len = EXFDCAN_MAX_DATA_BYTES;

    CAN_TX_FIFO_STATUS status = CAN_TX_FIFO_FULL;
    CAN_TX_MSGOBJ txObj;

    exfdcan_bus_lock();
    if (DRV_CANFDSPI_TransmitChannelStatusGet(dev, EXFDCAN_TX_FIFO, &status) != 0)
    {
        ctx->stats->tx_errors++;
        exfdcan_bus_unlock();
        return false;
    }

    if (!(status & CAN_TX_FIFO_NOT_FULL))
    {
        ctx->stats->tx_full++;
        ctx->stats->tx_errors++;
        exfdcan_bus_unlock();
        return false;
    }

    memset(&txObj, 0, sizeof(txObj));
    txObj.bF.id.SID = sid;
    txObj.bF.ctrl.DLC = (CAN_DLC)len;
    txObj.bF.ctrl.IDE = 0;
    txObj.bF.ctrl.RTR = 0;
    txObj.bF.ctrl.FDF = 0;
    txObj.bF.ctrl.BRS = 0;

    uint8_t buf[EXFDCAN_MAX_DATA_BYTES];
    for (uint8_t i = 0; i < EXFDCAN_MAX_DATA_BYTES; ++i)
    {
        buf[i] = (i < len) ? data[i] : 0u;
    }

    if (DRV_CANFDSPI_TransmitChannelLoad(dev,
                                         EXFDCAN_TX_FIFO,
                                         &txObj,
                                         buf,
                                         len,
                                         true) == 0)
    {
        ctx->stats->tx_frames++;
        exfdcan_bus_unlock();
        return true;
    }

    ctx->stats->tx_errors++;
    exfdcan_bus_unlock();
    return false;
}

/* ========================= Immediate single-frame send (bypass period) ========================= */

extern "C" bool excantx_send_now(uint8_t dev, uint16_t sid, const uint8_t* data,
                                 uint8_t len, uint32_t timeout_us)
{
    if (!data) return false;
    if (len > 8) len = 8;

    const bool in_isr = (__get_IPSR() != 0);
    if (in_isr || timeout_us == 0)
    {
        return (excantx_send_raw(dev, sid, data, len, 0) == 0);
    }

    exfdcan_ctx_t* ex_ctx = exfdcan_ctx_of(dev);
    if (!ex_ctx) return false;

    const uint32_t tick_hz = osKernelGetTickFreq();
    const uint32_t start   = osKernelGetTickCount();
    const uint32_t budget  = (timeout_us * tick_hz + 999999u) / 1000000u;
    const uint32_t min_wait = (budget ? budget : 1u);

    for (;;) {
        if (exfdcan_tx_push(dev, ex_ctx, sid, data, len)) return true;
        if ((osKernelGetTickCount() - start) >= min_wait) return false;
        osDelay(0);
    }
}

/* ========================= Raw frame enqueue ========================= */

extern "C" int excantx_send_raw(uint8_t dev, uint16_t sid, const uint8_t* data, uint8_t dlc, uint8_t prio)
{
    excantx_bus_ctx_t* c = ctx_of(dev);
    if (!c || !data || dlc > 8) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    uint16_t next = (uint16_t)((c->raw_tail + 1) % EXCANTX_RAW_Q_LEN);
    if (next == c->raw_head) { if (!ps) __enable_irq(); return -2; }

    raw_item_t* it = &c->raw_q[c->raw_tail];
    it->sid  = sid;
    it->dlc  = dlc;
    it->prio = prio;
    for (uint8_t i = 0; i < dlc; ++i) it->data[i] = data[i];
    for (uint8_t i = dlc; i < 8;  ++i) it->data[i] = 0;

    c->raw_tail = next;
    if (!ps) __enable_irq();

    if (c->evt) (void)osEventFlagsSet(c->evt, 0x2);
    return 0;
}

/* ========================= Adapter query + registration ========================= */

static bool query_c620   (void* o, uint16_t* gid, uint8_t* slot, int16_t* v) {
    auto* m = reinterpret_cast<DJI_C620*>(o);
    return m->exportTx16(gid, slot, v);
}
static bool query_gm6020 (void* o, uint16_t* gid, uint8_t* slot, int16_t* v) {
    auto* m = reinterpret_cast<DJI_GM6020*>(o);
    return m->exportTx16(gid, slot, v);
}
static bool query_dm4310_raw8(void* o, uint16_t* sid, uint8_t out[8]) {
    auto* m = reinterpret_cast<DM4310*>(o);
    return m->exportTxRaw8(sid, out);
}
static bool query_dm4340_raw8(void* o, uint16_t* sid, uint8_t out[8]) {
    auto* m = reinterpret_cast<DM4340*>(o);
    return m->exportTxRaw8(sid, out);
}
#if EXFDCAN_HAS_ADP_LK_MG8016E
static bool query_lk8016e_raw8(void* o, uint16_t* sid, uint8_t out[8]) {
    auto* m = reinterpret_cast<LK8016E*>(o);
    return m->exportTxRaw8(sid, out);
}
#endif

static uint32_t clamp_period_ms(uint32_t ms) {
    if (ms < EXCANTX_PERIOD_MS_MIN)   ms = EXCANTX_PERIOD_MS_MIN;
    if (ms > EXCANTX_PERIOD_MS_MAX)   ms = EXCANTX_PERIOD_MS_MAX;
    return ms;
}

static int register_obj_ex(uint8_t dev, void* obj, adapter_query_tx16_fn fn, uint32_t period_ms)
{
    excantx_bus_ctx_t* c = ctx_of(dev);
    if (!c || !obj || !fn) return -1;

    period_ms = clamp_period_ms(period_ms);

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < EXCANTX_MAX_OBJ; ++i) {
        if (c->objs[i].obj == obj && c->objs[i].query == fn) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    for (int i = 0; i < EXCANTX_MAX_OBJ; ++i) {
        if (!c->objs[i].query) {
            c->objs[i].obj   = obj;
            c->objs[i].query = fn;
            c->objs[i].period_ms = period_ms;
            uint32_t now = osKernelGetTickCount();
            c->objs[i].next_due_tick = now;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3;
}

extern "C" int excantx_register_c620(uint8_t dev, DJI_C620* obj, uint32_t period_ms) {
    return register_obj_ex(dev, obj, query_c620, period_ms);
}
extern "C" int excantx_register_gm6020(uint8_t dev, DJI_GM6020* obj, uint32_t period_ms) {
    return register_obj_ex(dev, obj, query_gm6020, period_ms);
}
extern "C" int excantx_register_dm4310(uint8_t dev, DM4310* obj, uint32_t period_ms)
{
    excantx_bus_ctx_t* c = ctx_of(dev);
    if (!c || !obj) return -1;
    period_ms = (period_ms < 1 ? 1 : (period_ms > 1000 ? 1000 : period_ms));

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
        if (c->dm_raw8[i].obj == obj && c->dm_raw8[i].query == query_dm4310_raw8) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
        if (!c->dm_raw8[i].query) {
            c->dm_raw8[i].obj           = obj;
            c->dm_raw8[i].query         = query_dm4310_raw8;
            c->dm_raw8[i].period_ms     = period_ms;
            c->dm_raw8[i].next_due_tick = osKernelGetTickCount();
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3;
}
extern "C" int excantx_register_dm4340(uint8_t dev, DM4340* obj, uint32_t period_ms)
{
    excantx_bus_ctx_t* c = ctx_of(dev);
    if (!c || !obj) return -1;
    period_ms = (period_ms < 1 ? 1 : (period_ms > 1000 ? 1000 : period_ms));

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
        if (c->dm_raw8[i].obj == obj && c->dm_raw8[i].query == query_dm4340_raw8) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
        if (!c->dm_raw8[i].query) {
            c->dm_raw8[i].obj           = obj;
            c->dm_raw8[i].query         = query_dm4340_raw8;
            c->dm_raw8[i].period_ms     = period_ms;
            c->dm_raw8[i].next_due_tick = osKernelGetTickCount();
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3;
}
#if EXFDCAN_HAS_ADP_LK_MG8016E
extern "C" int excantx_register_lk8016e(uint8_t dev, LK8016E* obj, uint32_t period_ms)
{
    excantx_bus_ctx_t* c = ctx_of(dev);
    if (!c || !obj) return -1;
    period_ms = (period_ms < 1 ? 1 : (period_ms > 1000 ? 1000 : period_ms));

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
        if (c->dm_raw8[i].obj == obj && c->dm_raw8[i].query == query_lk8016e_raw8) {
            if (!ps) __enable_irq();
            return -2;
        }
    }
    for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
        if (!c->dm_raw8[i].query) {
            c->dm_raw8[i].obj           = obj;
            c->dm_raw8[i].query         = query_lk8016e_raw8;
            c->dm_raw8[i].period_ms     = period_ms;
            c->dm_raw8[i].next_due_tick = osKernelGetTickCount();
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -3;
}
#else
extern "C" int excantx_register_lk8016e(uint8_t dev, LK8016E* obj, uint32_t period_ms)
{
    (void)dev; (void)obj; (void)period_ms;
    return -1;
}
#endif

/* ========================= Aggregate + send once (per-object period) ========================= */

static void txrouter_flush_once(uint8_t dev, excantx_bus_ctx_t* c, exfdcan_ctx_t* ex_ctx)
{
    for (int i = 0; i < EXCANTX_MAX_GROUPS; ++i) {
        c->groups[i].valid = false;
        for (int k = 0; k < 4; ++k) {
            c->groups[i].slot_used[k] = false;
            c->groups[i].slot_val[k] = 0;
        }
    }

    const uint32_t now_tick = osKernelGetTickCount();

    for (int i = 0; i < EXCANTX_MAX_OBJ; ++i) {
        tx_obj_t& o = c->objs[i];
        if (!o.query) continue;
        if ((int32_t)(now_tick - o.next_due_tick) < 0) continue;

        uint16_t gid; uint8_t slot; int16_t val;
        if (!o.query(o.obj, &gid, &slot, &val) || slot > 3) {
            o.next_due_tick = now_tick + ms_to_ticks(o.period_ms, c->tick_hz);
            continue;
        }

        tx_group_acc_t* g = find_or_alloc_group(c, gid);
        if (g) {
            g->slot_used[slot] = true;
            g->slot_val[slot] = val;
            o.last_sent_val = val;
        }

        o.next_due_tick = now_tick + ms_to_ticks(o.period_ms, c->tick_hz);
    }

    for (int gi = 0; gi < EXCANTX_MAX_GROUPS; ++gi) {
        tx_group_acc_t* g = &c->groups[gi];
        if (!g->valid) continue;

        bool any_due = false;
        for (int k = 0; k < 4; ++k) { if (g->slot_used[k]) { any_due = true; break; } }
        if (!any_due) continue;

        for (int i = 0; i < EXCANTX_MAX_OBJ; ++i) {
            tx_obj_t& o = c->objs[i];
            if (!o.query) continue;

            uint16_t gid; uint8_t slot; int16_t dummy;
            if (!o.query(o.obj, &gid, &slot, &dummy) || slot > 3) continue;
            if (gid != g->group_id) continue;
            if (g->slot_used[slot]) continue;

            g->slot_used[slot] = true;
            g->slot_val[slot] = o.last_sent_val;
        }
    }

    while (c->raw_head != c->raw_tail) {
        raw_item_t it;
        {
            uint32_t ps = __get_PRIMASK(); __disable_irq();
            raw_item_t* src = &c->raw_q[c->raw_head];
            it = *src;
            c->raw_head = (uint16_t)((c->raw_head + 1) % EXCANTX_RAW_Q_LEN);
            if (!ps) __enable_irq();
        }
#if EXCANTX_TASK_WAIT_ON_BUSY
        const uint32_t timeout_us = 200;
        const uint32_t t_hz = osKernelGetTickFreq();
        const uint32_t start = osKernelGetTickCount();
        const uint32_t budget = (timeout_us * t_hz + 999999u) / 1000000u;
        const uint32_t min_wait = (budget ? budget : 1u);
        for (;;) {
            if (exfdcan_tx_push(dev, ex_ctx, it.sid, it.data, it.dlc)) break;
            if ((osKernelGetTickCount() - start) >= min_wait) break;
            osDelay(0);
        }
#else
        (void)exfdcan_tx_push(dev, ex_ctx, it.sid, it.data, it.dlc);
#endif
    }

    {
        const uint32_t now_tick = osKernelGetTickCount();
        for (int i = 0; i < EXCANTX_MAX_DM_RAW8; ++i) {
            tx_obj_raw8_t& o = c->dm_raw8[i];
            if (!o.query) continue;
            if ((int32_t)(now_tick - o.next_due_tick) < 0) continue;

            uint16_t sid;
            uint8_t buf[8];
            if (o.query(o.obj, &sid, buf)) {

#if EXCANTX_TASK_WAIT_ON_BUSY
                const uint32_t timeout_us = 200;
                const uint32_t t_hz   = osKernelGetTickFreq();
                const uint32_t start  = osKernelGetTickCount();
                const uint32_t budget = (timeout_us * t_hz + 999999u) / 1000000u;
                const uint32_t min_wait = (budget ? budget : 1u);

                for (;;) {
                    if (exfdcan_tx_push(dev, ex_ctx, sid, buf, 8)) {
                        break;
                    }
                    if ((osKernelGetTickCount() - start) >= min_wait) {
                        break;
                    }
                    osDelay(0);
                }
#else
                (void)exfdcan_tx_push(dev, ex_ctx, sid, buf, 8);
#endif
            }

            o.next_due_tick = now_tick + ms_to_ticks(o.period_ms, c->tick_hz);
        }
    }

    for (int i = 0; i < EXCANTX_MAX_GROUPS; ++i) {
        tx_group_acc_t* g = &c->groups[i];
        if (!g->valid) continue;

        bool any_due = false;
        for (int k = 0; k < 4; ++k) { if (g->slot_used[k]) { any_due = true; break; } }
        if (!any_due) continue;

        const uint16_t sid = g->group_id;

        uint8_t buf[8];
        for (int k = 0; k < 4; ++k) {
            int16_t v = g->slot_val[k];
            buf[2*k+0] = (uint8_t)((v >> 8) & 0xFF);
            buf[2*k+1] = (uint8_t)((v >> 0) & 0xFF);
        }
#if EXCANTX_TASK_WAIT_ON_BUSY
        const uint32_t timeout_us = 200;
        const uint32_t t_hz = osKernelGetTickFreq();
        const uint32_t start = osKernelGetTickCount();
        const uint32_t budget = (timeout_us * t_hz + 999999u) / 1000000u;
        const uint32_t min_wait = (budget ? budget : 1u);
        for (;;) {
            if (exfdcan_tx_push(dev, ex_ctx, sid, buf, 8)) break;
            if ((osKernelGetTickCount() - start) >= min_wait) break;
            osDelay(0);
        }
#else
        (void)exfdcan_tx_push(dev, ex_ctx, sid, buf, 8);
#endif
    }
}

static void exfdcan_tx_handle_event(uint8_t dev, exfdcan_ctx_t *ctx)
{
    CAN_MODULE_EVENT events = CAN_NO_EVENT;
    CAN_TX_FIFO_STATUS status = CAN_TX_FIFO_FULL;

    exfdcan_bus_lock();
    if (DRV_CANFDSPI_ModuleEventGet(dev, &events) == 0)
    {
        ctx->stats->last_int_flags = (uint32_t)events;
        ctx->stats->module_events |= (uint32_t)events;
        if (!(events & CAN_TX_EVENT))
        {
            ctx->stats->tx_irq_no_txif++;
            goto exfdcan_tx_clear_and_exit;
        }
    }
    else
    {
        ctx->stats->tx_errors++;
        goto exfdcan_tx_clear_and_exit;
    }

    if (DRV_CANFDSPI_TransmitChannelStatusGet(dev, EXFDCAN_TX_FIFO, &status) == 0)
    {
        ctx->stats->last_tx_status = (uint32_t)status;
        if (!(status & CAN_TX_FIFO_NOT_FULL))
        {
            ctx->stats->tx_full++;
            ctx->stats->tx_errors++;
        }
    }
    else
    {
        ctx->stats->tx_errors++;
    }
exfdcan_tx_clear_and_exit:
    (void)DRV_CANFDSPI_ModuleEventClear(dev,
                                        (CAN_MODULE_EVENT)(CAN_TX_EVENT |
                                                           CAN_TX_ATTEMPTS_EVENT));
    (void)DRV_CANFDSPI_TransmitChannelEventAttemptClear(dev, EXFDCAN_TX_FIFO);
    exfdcan_bus_unlock();
}

/* ========================= Task entry ========================= */

extern "C" void Start_EXFDCAN_TxRouter(void *argument)
{
    uint8_t dev = (uint8_t)(uintptr_t)argument;
    exfdcan_ctx_t *ctx = exfdcan_ctx_of(dev);
    excantx_bus_ctx_t *tx_ctx = ctx_of(dev);
    if (!ctx || !tx_ctx)
        return;

    if (!ctx->evt)
    {
        uint32_t ps = __get_PRIMASK(); __disable_irq();
        if (!ctx->evt)
        {
            ctx->evt = osEventFlagsNew(NULL);
        }
        if (!ps) __enable_irq();
    }
    if (ctx->evt && ctx->pending_isr)
    {
        (void)osEventFlagsSet(ctx->evt, ctx->pending_isr);
        ctx->pending_isr = 0u;
    }

    if (!tx_ctx->evt)
    {
        tx_ctx->evt = osEventFlagsNew(NULL);
    }
    tx_ctx->raw_head = tx_ctx->raw_tail = 0;
    tx_ctx->tick_hz = osKernelGetTickFreq();

    (void)exfdcan_hw_init(dev);

    const uint32_t period_ticks = ms_to_ticks((EXCANTX_PERIOD_US + 999u) / 1000u, tx_ctx->tick_hz);
    uint32_t last_wake = osKernelGetTickCount();

    for (;;)
    {
        osDelayUntil(last_wake + (period_ticks ? period_ticks : 1));
        last_wake = osKernelGetTickCount();

        if (tx_ctx->evt) (void)osEventFlagsClear(tx_ctx->evt, 0x2);
        txrouter_flush_once(dev, tx_ctx, ctx);

        uint32_t flags = osEventFlagsWait(ctx->evt, EXFDCAN_EVT_TX, osFlagsWaitAny, 0);
        if (flags & EXFDCAN_EVT_TX)
        {
            ctx->stats->tx_irq++;
            exfdcan_tx_handle_event(dev, ctx);
        }
    }
}
