//
// tskptt_exfdcan_rx_router.cpp - MCP2518FD RX task (INT1 -> RX FIFO)
//
#include "EXFDCAN_Router_Internal.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_defines.h"
#include "bsp_srn_log.h"
#include "stm32h7xx_hal.h"

#include "lib_adp_dji_gm6020.h"
#include "lib_adp_dji_c620.h"
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

#ifndef EXFDCAN_HAS_ADP_NAVISION
  #if defined(__has_include)
    #if __has_include("lib_adp_navision.h")
      #define EXFDCAN_HAS_ADP_NAVISION 1
    #endif
  #endif
#endif
#ifndef EXFDCAN_HAS_ADP_NAVISION
  #define EXFDCAN_HAS_ADP_NAVISION 0
#endif

#ifndef EXFDCAN_HAS_ADP_OMXIMU
  #if defined(__has_include)
    #if __has_include("lib_adp_omximu.h")
      #define EXFDCAN_HAS_ADP_OMXIMU 1
    #endif
  #endif
#endif
#ifndef EXFDCAN_HAS_ADP_OMXIMU
  #define EXFDCAN_HAS_ADP_OMXIMU 0
#endif

#if EXFDCAN_HAS_ADP_LK_MG8016E
#include "lib_adp_lk_mg8016e.h"
#endif
#if EXFDCAN_HAS_ADP_NAVISION
#include "lib_adp_navision.h"
#endif
#if EXFDCAN_HAS_ADP_OMXIMU
#include "lib_adp_omximu.h"
#endif

#define EXFDCAN_RX_WAIT_MS     (5u)
#ifndef EXFDCAN_RX_LOG_EVERY
#define EXFDCAN_RX_LOG_EVERY   (1u)
#endif
#ifndef EXFDCAN_RX_DRAIN_MAX
#define EXFDCAN_RX_DRAIN_MAX   (16u)
#endif
#ifndef EXFDCAN_RX_DRAIN_YIELD_MS
#define EXFDCAN_RX_DRAIN_YIELD_MS (1u)
#endif

#ifndef EXCANRX_MAX_SUBS_RAW
#define EXCANRX_MAX_SUBS_RAW   32
#endif
#ifndef EXCANRX_MAX_SUBS_OBJ
#define EXCANRX_MAX_SUBS_OBJ   16
#endif

// Raw subscription table
typedef struct {
    uint16_t           id;
    excanrx_handler_t  cb;
    void*              user;
} sub_raw_t;

// Adapter subscription table
typedef void (*adapter_dispatch_fn)(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc);

typedef struct {
    uint16_t             id;
    void*                obj;
    adapter_dispatch_fn  dispatch;
} sub_obj_t;

// Per-device context
typedef struct {
    sub_raw_t        raw[EXCANRX_MAX_SUBS_RAW];
    sub_obj_t        obj[EXCANRX_MAX_SUBS_OBJ];
    excanrx_stats_t  stats;
} excanrx_bus_ctx_t;

static excanrx_bus_ctx_t* excanrx_ctx_of(uint8_t dev)
{
    static excanrx_bus_ctx_t s_ctx[EXFDCAN_DEV_COUNT];
    return (dev < EXFDCAN_DEV_COUNT) ? &s_ctx[dev] : nullptr;
}

static inline void exfdcan_rx_yield(uint32_t delay_ms)
{
    if (__get_IPSR() != 0U)
    {
        __NOP();
        return;
    }
    if (osKernelGetState() != osKernelRunning)
    {
        __NOP();
        return;
    }
    if (delay_ms == 0u)
    {
        osDelay(0);
    }
    else
    {
        osDelay(delay_ms);
    }
}

// Adapter dispatch (pack to 8B)
static void dispatch_gm6020(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    DJI_GM6020* m = reinterpret_cast<DJI_GM6020*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    m->onRxFeedback(f, HAL_GetTick());
}

static void dispatch_c620(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    DJI_C620* m = reinterpret_cast<DJI_C620*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    m->onRxFeedback(f, HAL_GetTick());
}

static void dispatch_dm4310(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    DM4310* m = reinterpret_cast<DM4310*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    m->onRxFeedback(f, HAL_GetTick());
}
static void dispatch_dm4340(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    DM4340* m = reinterpret_cast<DM4340*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    m->onRxFeedback(f, HAL_GetTick());
}
#if EXFDCAN_HAS_ADP_LK_MG8016E
static void dispatch_lk8016e(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    LK8016E* m = reinterpret_cast<LK8016E*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    m->onRxFeedback(f, HAL_GetTick());
}
#endif

#if EXFDCAN_HAS_ADP_NAVISION
static void dispatch_navi(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    Navi* n = reinterpret_cast<Navi*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    n->onRxFeedback(f, HAL_GetTick());
}

static void dispatch_vision(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    Vision* v = reinterpret_cast<Vision*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    v->onRxFeedback(f, HAL_GetTick());
}
#endif

#if EXFDCAN_HAS_ADP_OMXIMU
static void dispatch_omximu(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc)
{
    if (!obj) return;
    OmxImu* imu = reinterpret_cast<OmxImu*>(obj);
    CanFrame f{};
    f.id     = sid;
    f.is_ext = false;
    f.dlc    = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i < dlc) ? data[i] : 0;
    OmxImu_OnRxFeedback(imu, &f, HAL_GetTick());
}
#endif

// Public API: stats
extern "C" const excanrx_stats_t* excanrx_get_stats(uint8_t dev)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    return c ? &c->stats : nullptr;
}

// Public API: raw subscription
extern "C" int excanrx_subscribe_raw(uint8_t dev, uint16_t std_id, excanrx_handler_t cb, void* user)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !cb) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_RAW; ++i) {
        if (c->raw[i].cb && c->raw[i].id == std_id &&
            c->raw[i].cb == cb && c->raw[i].user == user) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_RAW; ++i) {
        if (!c->raw[i].cb) {
            c->raw[i].id   = std_id;
            c->raw[i].cb   = cb;
            c->raw[i].user = user;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

extern "C" int excanrx_unsubscribe_raw(uint8_t dev, uint16_t std_id, excanrx_handler_t cb, void* user)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !cb) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < EXCANRX_MAX_SUBS_RAW; ++i) {
        if (c->raw[i].cb && c->raw[i].id == std_id &&
            c->raw[i].cb == cb && c->raw[i].user == user) {
            c->raw[i].id = 0; c->raw[i].cb = nullptr; c->raw[i].user = nullptr;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

// Public API: adapter registration (GM6020 / C620 / DM4310)
extern "C" int excanrx_register_gm6020(uint8_t dev, DJI_GM6020* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_gm6020) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_gm6020;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

extern "C" int excanrx_register_c620(uint8_t dev, DJI_C620* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_c620) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_c620;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

extern "C" int excanrx_register_dm4310(uint8_t dev, DM4310* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->fbMasterSid();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_dm4310) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_dm4310;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

extern "C" int excanrx_register_dm4340(uint8_t dev, DM4340* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->fbMasterSid();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_dm4340) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_dm4340;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

#if EXFDCAN_HAS_ADP_LK_MG8016E
extern "C" int excanrx_register_lk8016e(uint8_t dev, LK8016E* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t id = obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_lk8016e) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_lk8016e;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}
#else
extern "C" int excanrx_register_lk8016e(uint8_t dev, LK8016E* obj)
{
    (void)dev; (void)obj;
    return -1;
}
#endif

#if EXFDCAN_HAS_ADP_NAVISION
extern "C" int excanrx_register_navi(uint8_t dev, Navi* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t id = obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Deduplicate
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_navi) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // Fill slot
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_navi;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

extern "C" int excanrx_register_vision(uint8_t dev, Vision* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;
    const uint16_t base = obj->base_id();
    bool present[VISION_ITEM_COUNT] = {0};
    uint16_t free_slots = 0;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Scan: existing entries + free slots
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            free_slots++;
            continue;
        }
        if (c->obj[i].obj == obj && c->obj[i].dispatch == dispatch_vision) {
            const uint16_t id = c->obj[i].id;
            if (id >= base && id < (uint16_t)(base + VISION_ITEM_COUNT)) {
                present[id - base] = true;
            }
        }
    }

    uint16_t missing = 0;
    for (uint16_t idx = 0; idx < VISION_ITEM_COUNT; ++idx) {
        if (!present[idx]) missing++;
    }
    if (missing == 0) {
        if (!ps) __enable_irq();
        return 0;
    }
    if (free_slots < missing) {
        if (!ps) __enable_irq();
        return -1;
    }

    for (uint16_t idx = 0; idx < VISION_ITEM_COUNT; ++idx) {
        if (present[idx]) continue;
        for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
            if (!c->obj[i].dispatch) {
                c->obj[i].id       = (uint16_t)(base + idx);
                c->obj[i].obj      = obj;
                c->obj[i].dispatch = dispatch_vision;
                break;
            }
        }
    }
    if (!ps) __enable_irq();
    return 0;
}

extern "C" int excanrx_register_navision(uint8_t dev, Navi* obj)
{
    return excanrx_register_navi(dev, obj);
}
#else
extern "C" int excanrx_register_navi(uint8_t dev, Navi* obj)
{
    (void)dev; (void)obj;
    return -1;
}

extern "C" int excanrx_register_vision(uint8_t dev, Vision* obj)
{
    (void)dev; (void)obj;
    return -1;
}

extern "C" int excanrx_register_navision(uint8_t dev, Navi* obj)
{
    (void)dev; (void)obj;
    return -1;
}
#endif

#if EXFDCAN_HAS_ADP_OMXIMU
extern "C" int excanrx_register_omximu(uint8_t dev, OmxImu* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;

    const uint16_t ids[4] = {
        OmxImu_QuatId(obj),
        OmxImu_EulerId(obj),
        OmxImu_GyroId(obj),
        OmxImu_AccelId(obj),
    };
    bool present[4] = {0};
    uint16_t free_slots = 0;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // Scan: existing entries + free slots
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            free_slots++;
            continue;
        }
        if (c->obj[i].obj == obj && c->obj[i].dispatch == dispatch_omximu) {
            for (int k = 0; k < 4; ++k) {
                if (c->obj[i].id == ids[k]) {
                    present[k] = true;
                }
            }
        }
    }

    uint16_t missing = 0;
    for (int k = 0; k < 4; ++k) {
        if (!present[k]) missing++;
    }
    if (missing == 0) {
        if (!ps) __enable_irq();
        return 0;
    }
    if (free_slots < missing) {
        if (!ps) __enable_irq();
        return -1;
    }

    for (int k = 0; k < 4; ++k) {
        if (present[k]) continue;
        for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
            if (!c->obj[i].dispatch) {
                c->obj[i].id       = ids[k];
                c->obj[i].obj      = obj;
                c->obj[i].dispatch = dispatch_omximu;
                break;
            }
        }
    }
    if (!ps) __enable_irq();
    return 0;
}
#else
extern "C" int excanrx_register_omximu(uint8_t dev, OmxImu* obj)
{
    (void)dev; (void)obj;
    return -1;
}
#endif

extern "C" int excanrx_unregister_adapter(uint8_t dev, void* obj)
{
    excanrx_bus_ctx_t* c = excanrx_ctx_of(dev);
    if (!c || !obj) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < EXCANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].dispatch) {
            c->obj[i].id = 0; c->obj[i].obj = nullptr; c->obj[i].dispatch = nullptr;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

static void excanrx_dispatch_frame(excanrx_bus_ctx_t* c,
                                  uint16_t sid,
                                  const uint8_t* data,
                                  uint8_t dlc)
{
    if (!c || !data) return;

    c->stats.pops++;
    int dispatched = 0;

    // 1) Raw callbacks (by ID)
    {
        uint32_t ps = __get_PRIMASK(); __disable_irq();
        for (int k = 0; k < EXCANRX_MAX_SUBS_RAW; ++k) {
            if (c->raw[k].cb && c->raw[k].id == sid) {
                auto cb = c->raw[k].cb;
                void* usr = c->raw[k].user;
                if (!ps) __enable_irq();
                cb(sid, data, dlc, usr);
                dispatched++;
                ps = __get_PRIMASK(); __disable_irq();
            }
        }
        if (!ps) __enable_irq();
    }

    // 2) Object adapters
    {
        uint32_t ps = __get_PRIMASK(); __disable_irq();
        for (int k = 0; k < EXCANRX_MAX_SUBS_OBJ; ++k) {
            if (c->obj[k].dispatch && c->obj[k].id == sid) {
                auto fn  = c->obj[k].dispatch;
                void* obj = c->obj[k].obj;
                if (!ps) __enable_irq();
                fn(obj, sid, data, dlc);
                dispatched++;
                ps = __get_PRIMASK(); __disable_irq();
            }
        }
        if (!ps) __enable_irq();
    }

    if (dispatched) c->stats.routed_ok += (uint32_t)dispatched;
    else            c->stats.no_subscriber++;
}

int receive_cnt = 0u;

static void exfdcan_rx_push(exfdcan_ctx_t *ctx,
                            uint16_t sid,
                            uint8_t dlc,
                            const uint8_t *data)
{
    if (!ctx || !data)
        return;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    uint16_t next = (uint16_t)((ctx->rx_head + 1u) % EXFDCAN_RX_LOG_DEPTH);
    if (next == ctx->rx_tail)
    {
        ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1u) % EXFDCAN_RX_LOG_DEPTH);
        ctx->stats->rx_overflow++;
    }

    exfdcan_rx_frame_t *slot = &ctx->rx_log[ctx->rx_head];
    slot->sid = sid;
    slot->dlc = dlc;
    for (uint8_t i = 0; i < EXFDCAN_MAX_DATA_BYTES; ++i)
    {
        slot->data[i] = (i < dlc) ? data[i] : 0u;
    }
    slot->timestamp_ms = HAL_GetTick();

    ctx->rx_head = next;
    ctx->stats->rx_frames++;
    if (!ps) __enable_irq();

#if EXFDCAN_RX_LOG_EVERY
    if ((ctx->stats->rx_frames % EXFDCAN_RX_LOG_EVERY) == 0u)
    {
        receive_cnt++;
        LOGD("[EXFDCAN RX] sid=0x%03X dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X [%d]",
             sid,
             dlc,
             slot->data[0], slot->data[1], slot->data[2], slot->data[3],
             slot->data[4], slot->data[5], slot->data[6], slot->data[7], receive_cnt);
    }
#endif
}

static bool exfdcan_rx_drain_locked(uint8_t dev,
                                    exfdcan_ctx_t *ctx,
                                    excanrx_bus_ctx_t *rx_ctx)
{
    CAN_RX_FIFO_STATUS status = CAN_RX_FIFO_EMPTY;
    uint32_t drained = 0u;
    bool hit_limit = false;

    for (;;)
    {
        if (DRV_CANFDSPI_ReceiveChannelStatusGet(dev, EXFDCAN_RX_FIFO, &status) != 0)
        {
            ctx->stats->rx_errors++;
            break;
        }

        ctx->stats->last_rx_status = (uint32_t)status;
        if (!(status & CAN_RX_FIFO_NOT_EMPTY))
        {
            break;
        }

        CAN_RX_MSGOBJ rxObj;
        uint8_t data[EXFDCAN_MAX_DATA_BYTES];
        uint8_t dlc_bytes = 0u;

        if (DRV_CANFDSPI_ReceiveMessageGet(dev, EXFDCAN_RX_FIFO, &rxObj, data, EXFDCAN_MAX_DATA_BYTES) != 0)
        {
            ctx->stats->rx_errors++;
            break;
        }

        drained++;
        dlc_bytes = (uint8_t)DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
        if (dlc_bytes > EXFDCAN_MAX_DATA_BYTES)
            dlc_bytes = EXFDCAN_MAX_DATA_BYTES;

        if (rxObj.bF.ctrl.IDE)
        {
            if (drained >= EXFDCAN_RX_DRAIN_MAX)
            {
                hit_limit = true;
                break;
            }
            continue;
        }

        uint16_t sid = (uint16_t)rxObj.bF.id.SID;
        exfdcan_bus_unlock();
        exfdcan_rx_push(ctx, sid, dlc_bytes, data);
        if (rx_ctx)
        {
            excanrx_dispatch_frame(rx_ctx, sid, data, dlc_bytes);
        }
        exfdcan_bus_lock();
        if (drained >= EXFDCAN_RX_DRAIN_MAX)
        {
            hit_limit = true;
            break;
        }
    }

    if (status & CAN_RX_FIFO_OVERFLOW)
    {
        (void)DRV_CANFDSPI_ReceiveChannelEventOverflowClear(dev, EXFDCAN_RX_FIFO);
        ctx->stats->rx_errors++;
    }

    (void)DRV_CANFDSPI_ModuleEventClear(dev, CAN_RX_EVENT);
    return hit_limit;
}

static void exfdcan_rx_handle_event(uint8_t dev,
                                   exfdcan_ctx_t *ctx,
                                   excanrx_bus_ctx_t *rx_ctx)
{
    CAN_MODULE_EVENT events = CAN_NO_EVENT;
    bool hit_limit = false;

    exfdcan_bus_lock();
    if (DRV_CANFDSPI_ModuleEventGet(dev, &events) != 0)
    {
        ctx->stats->rx_errors++;
        goto exfdcan_rx_clear_and_exit;
    }

    ctx->stats->last_int_flags = (uint32_t)events;
    ctx->stats->module_events |= (uint32_t)events;
    if (!(events & CAN_RX_EVENT))
    {
        ctx->stats->rx_irq_no_rxif++;
        goto exfdcan_rx_clear_and_exit;
    }

    hit_limit = exfdcan_rx_drain_locked(dev, ctx, rx_ctx);
exfdcan_rx_clear_and_exit:
    if (events & CAN_RX_OVERFLOW_EVENT)
    {
        (void)DRV_CANFDSPI_ReceiveChannelEventOverflowClear(dev, EXFDCAN_RX_FIFO);
    }
    (void)DRV_CANFDSPI_ModuleEventClear(dev,
                                        (CAN_MODULE_EVENT)(CAN_RX_EVENT |
                                                           CAN_RX_OVERFLOW_EVENT));
    exfdcan_bus_unlock();
    if (hit_limit)
    {
        exfdcan_rx_yield(EXFDCAN_RX_DRAIN_YIELD_MS);
    }
}

static void exfdcan_rx_poll(uint8_t dev,
                            exfdcan_ctx_t *ctx,
                            excanrx_bus_ctx_t *rx_ctx)
{
    bool hit_limit = false;
    exfdcan_bus_lock();
    hit_limit = exfdcan_rx_drain_locked(dev, ctx, rx_ctx);
    exfdcan_bus_unlock();
    if (hit_limit)
    {
        exfdcan_rx_yield(EXFDCAN_RX_DRAIN_YIELD_MS);
    }
}

extern "C" void Start_EXFDCAN_RxRouter(void *argument)
{
    uint8_t dev = (uint8_t)(uintptr_t)argument;
    exfdcan_ctx_t *ctx = exfdcan_ctx_of(dev);
    excanrx_bus_ctx_t *rx_ctx = excanrx_ctx_of(dev);
    if (!ctx || !rx_ctx)
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

    (void)exfdcan_hw_init(dev);

    const uint32_t tick_hz = osKernelGetTickFreq();
    const uint32_t wait_ticks = (EXFDCAN_RX_WAIT_MS * tick_hz + 999u) / 1000u;

    for (;;)
    {
        uint32_t flags = osEventFlagsWait(ctx->evt,
                                          EXFDCAN_EVT_RX,
                                          osFlagsWaitAny,
                                          (wait_ticks ? wait_ticks : 1u));
        if (flags & EXFDCAN_EVT_RX)
        {
            ctx->stats->rx_irq++;
            rx_ctx->stats.wake_by_isr++;
            exfdcan_rx_handle_event(dev, ctx, rx_ctx);
        }
        else
        {
            rx_ctx->stats.wake_by_to++;
            exfdcan_rx_poll(dev, ctx, rx_ctx);
        }
    }
}
