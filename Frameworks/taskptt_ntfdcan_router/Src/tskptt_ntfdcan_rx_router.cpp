//
// Created by sirin on 2025/10/5.
//
// === CAN_RxRouter.cpp ===
// 一个文件搞定：多总线 RX 路由、原始回调订阅、对象适配器订阅（GM6020 / C620 / …）
// 依赖：cmsis_os2.h, stm32h7xx_hal.h, bsp_fdcan.h, 适配器头（lib_adp_dji_*）
//
// 用法（示例）：
//   osThreadNew(Start_CAN_RxRouter, (void*)(uintptr_t)1, &attr);   // 启动 CAN1 Router 任务
//   canrx_register_gm6020(1, &motor1);
//   canrx_register_c620 (1, &c620_5);
//   // 或者原始 ID 订阅：
//   canrx_subscribe_raw(1, 0x209, my_cb, my_user);
//
// 设计：
//   - 固定容量数组，无动态分配；每个 bus 独立一份订阅表与统计
//   - 适配器槽位存 “void* 对象指针 + 调度函数指针”，扩展新的电机只需新增一个注册函数 + 一个静态 dispatch_...()

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "bsp_fdcan.h"
#include "NTFDCAN_Router.h"

#include <cstdint>
#include <cstddef>
#include <cstring>

// ------------ 适配器头（按需引入） ------------
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

#ifndef NTFDCAN_HAS_ADP_NAVISION
  #if defined(__has_include)
    #if __has_include("lib_adp_navision.h")
      #define NTFDCAN_HAS_ADP_NAVISION 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_NAVISION
  #define NTFDCAN_HAS_ADP_NAVISION 0
#endif

#ifndef NTFDCAN_HAS_ADP_OMXIMU
  #if defined(__has_include)
    #if __has_include("lib_adp_omximu.h")
      #define NTFDCAN_HAS_ADP_OMXIMU 1
    #endif
  #endif
#endif
#ifndef NTFDCAN_HAS_ADP_OMXIMU
  #define NTFDCAN_HAS_ADP_OMXIMU 0
#endif

#if NTFDCAN_HAS_ADP_DJI_GM6020
#include "lib_adp_dji_gm6020.h"
#endif
#if NTFDCAN_HAS_ADP_DJI_C620
#include "lib_adp_dji_c620.h"
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
#if NTFDCAN_HAS_ADP_NAVISION
#include "lib_adp_navision.h"
#endif
#if NTFDCAN_HAS_ADP_OMXIMU
#include "lib_adp_omximu.h"
#endif

volatile fdcan_debug_t g_fdcan_debug = {};

// ------------ 配置 ------------
#ifndef CANRX_MAX_BUS
#define CANRX_MAX_BUS          3     // 支持的总线编号范围 1..CANRX_MAX_BUS
#endif

#define CANRX_MAX_SUBS_RAW     32    // 每总线：原始 ID 回调订阅数
#define CANRX_MAX_SUBS_OBJ     16    // 每总线：对象订阅数
#define CANRX_RX_BATCH         32    // 一次最多弹出帧数

// ------------ 原始回调签名（C 风格） ------------
typedef void (*canrx_handler_t)(uint16_t std_id, const uint8_t* data, uint8_t dlc, void* user);

// ------------ 原始订阅表 ------------
typedef struct {
    uint16_t         id;
    canrx_handler_t  cb;
    void*            user;
} sub_raw_t;

// ------------ 对象订阅表（无类型化 + 调度函数） ------------
typedef void (*adapter_dispatch_fn)(void* obj, uint16_t sid, const uint8_t* data, uint8_t dlc);

typedef struct {
    uint16_t             id;       // 标准 ID
    void*                obj;      // 适配器对象指针
    adapter_dispatch_fn  dispatch; // 如何把帧喂给对象
} sub_obj_t;

// ------------ 每个总线的上下文 ------------
typedef struct {
    osEventFlagsId_t  evt;
    sub_raw_t         raw[CANRX_MAX_SUBS_RAW];
    sub_obj_t         obj[CANRX_MAX_SUBS_OBJ];
    canrx_stats_t     stats;
} bus_ctx_t;

static bus_ctx_t* ctx_of(uint8_t bus)
{
    static bus_ctx_t s_ctx[CANRX_MAX_BUS + 1]; // 下标 1..MAX 有效
    return (bus >= 1 && bus <= CANRX_MAX_BUS) ? &s_ctx[bus] : nullptr;
}

static inline FDCAN_GlobalTypeDef* fdcan_inst_of(uint8_t bus)
{
    switch (bus) {
        case 1: return FDCAN1;
        case 2: return FDCAN2;
        case 3: return FDCAN3;
        default: return nullptr;
    }
}

static void fdcan_debug_update(uint8_t bus)
{
    if (bus < 1 || bus > NTFDCAN_DEBUG_BUS_COUNT) return;

    volatile fdcan_bus_debug_t* d = &g_fdcan_debug.bus[bus - 1];
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - d->last_update_ms) < 1000u) return;

    FDCAN_GlobalTypeDef* inst = fdcan_inst_of(bus);
    if (!inst) return;

    const uint32_t psr = inst->PSR;
    const uint32_t ecr = inst->ECR;
    const uint32_t ir = inst->IR;
    const uint32_t txfqs = inst->TXFQS;
    const uint32_t rxf0s = inst->RXF0S;
    const uint32_t rxf1s = inst->RXF1S;

    d->last_update_ms = now;
    d->psr = psr;
    d->ecr = ecr;
    d->ir = ir;
    d->txfqs = txfqs;
    d->rxf0s = rxf0s;
    d->rxf1s = rxf1s;

    d->tx_free_level = (uint8_t)((txfqs & FDCAN_TXFQS_TFFL_Msk) >> FDCAN_TXFQS_TFFL_Pos);
    d->tx_get_idx = (uint8_t)((txfqs & FDCAN_TXFQS_TFGI_Msk) >> FDCAN_TXFQS_TFGI_Pos);
    d->tx_put_idx = (uint8_t)((txfqs & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos);
    d->tx_full = (uint8_t)((txfqs & FDCAN_TXFQS_TFQF_Msk) ? 1u : 0u);

    d->rx0_fill_level = (uint8_t)((rxf0s & FDCAN_RXF0S_F0FL_Msk) >> FDCAN_RXF0S_F0FL_Pos);
    d->rx0_get_idx = (uint8_t)((rxf0s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos);
    d->rx0_put_idx = (uint8_t)((rxf0s & FDCAN_RXF0S_F0PI_Msk) >> FDCAN_RXF0S_F0PI_Pos);
    d->rx0_full = (uint8_t)((rxf0s & FDCAN_RXF0S_F0F_Msk) ? 1u : 0u);
    d->rx0_lost = (uint8_t)((rxf0s & FDCAN_RXF0S_RF0L_Msk) ? 1u : 0u);

    d->rx1_fill_level = (uint8_t)((rxf1s & FDCAN_RXF1S_F1FL_Msk) >> FDCAN_RXF1S_F1FL_Pos);
    d->rx1_get_idx = (uint8_t)((rxf1s & FDCAN_RXF1S_F1GI_Msk) >> FDCAN_RXF1S_F1GI_Pos);
    d->rx1_put_idx = (uint8_t)((rxf1s & FDCAN_RXF1S_F1PI_Msk) >> FDCAN_RXF1S_F1PI_Pos);
    d->rx1_full = (uint8_t)((rxf1s & FDCAN_RXF1S_F1F_Msk) ? 1u : 0u);
    d->rx1_lost = (uint8_t)((rxf1s & FDCAN_RXF1S_RF1L_Msk) ? 1u : 0u);

    d->tec = (uint8_t)((ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos);
    d->rec = (uint8_t)((ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos);
    d->lec = (uint8_t)((psr & FDCAN_PSR_LEC_Msk) >> FDCAN_PSR_LEC_Pos);
}

// ------------ DLC 解析（兼容不同 HAL 表示） ------------
static inline uint8_t decode_dlc_bytes(uint32_t v)
{
    // ① 典型 HAL: DLC 码在 [19:16]
    uint8_t code_hi = (uint8_t)((v >> 16) & 0x0F);
    // ② 有些 BSP 把字节数或码塞在低 4bit
    uint8_t code_lo = (uint8_t)(v & 0x0F);

    // DLC LUT
    static const uint8_t lut[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};

    // 优先用高位（标准 HAL 宏 FDCAN_DLC_BYTES_xx）
    if (code_hi && code_hi < 16) return lut[code_hi];
    // 再尝试低位 nibble
    if (code_lo && code_lo < 16) return lut[code_lo];
    // 兜底：一些自写驱动直接给“字节数”
    if (v <= 64) return (uint8_t)v;

    // 最后兜底成 8B（经典 CAN）
    return 8;
}

// ------------ 适配器调度函数：统一构造 8B 反馈并补零 ------------
// GM6020
#if NTFDCAN_HAS_ADP_DJI_GM6020
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
#endif
// C620（3508/2006 同类反馈）
#if NTFDCAN_HAS_ADP_DJI_C620
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
#endif

#if NTFDCAN_HAS_ADP_DM_4310
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
#endif

#if NTFDCAN_HAS_ADP_DM_4340
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
#endif

#if NTFDCAN_HAS_ADP_LK_MG8016E
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

#if NTFDCAN_HAS_ADP_NAVISION
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

#if NTFDCAN_HAS_ADP_OMXIMU
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

// ------------ 对外 API：统计 ------------
extern "C" const canrx_stats_t* canrx_get_stats(uint8_t bus)
{
    bus_ctx_t* c = ctx_of(bus);
    return c ? &c->stats : nullptr;
}

// ------------ 对外 API：原始回调订阅 ------------
extern "C" int canrx_subscribe_raw(uint8_t bus, uint16_t std_id, canrx_handler_t cb, void* user)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !cb) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_RAW; ++i) {
        if (c->raw[i].cb && c->raw[i].id == std_id &&
            c->raw[i].cb == cb && c->raw[i].user == user) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_RAW; ++i) {
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

extern "C" int canrx_unsubscribe_raw(uint8_t bus, uint16_t std_id, canrx_handler_t cb, void* user)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !cb) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < CANRX_MAX_SUBS_RAW; ++i) {
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

// ------------ 对外 API：对象适配器注册（GM6020 / C620） ------------
#if NTFDCAN_HAS_ADP_DJI_GM6020
extern "C" int canrx_register_gm6020(uint8_t bus, DJI_GM6020* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_gm6020) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
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
#else
extern "C" int canrx_register_gm6020(uint8_t bus, DJI_GM6020* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_DJI_C620
extern "C" int canrx_register_c620(uint8_t bus, DJI_C620* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_c620) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
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
#else
extern "C" int canrx_register_c620(uint8_t bus, DJI_C620* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_DM_4310
extern "C" int canrx_register_dm4310(uint8_t bus, DM4310* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->fbMasterSid(); // 统一反馈 SID

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_dm4310) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_dm4310;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1; // 表满
}
#else
extern "C" int canrx_register_dm4310(uint8_t bus, DM4310* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_DM_4340
extern "C" int canrx_register_dm4340(uint8_t bus, DM4340* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t id = (uint16_t)obj->fbMasterSid();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_dm4340) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_dm4340;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1; // 表满
}
#else
extern "C" int canrx_register_dm4340(uint8_t bus, DM4340* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_LK_MG8016E
extern "C" int canrx_register_lk8016e(uint8_t bus, LK8016E* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t id = obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_lk8016e) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_lk8016e;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1; // 表满
}
#else
extern "C" int canrx_register_lk8016e(uint8_t bus, LK8016E* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_NAVISION
extern "C" int canrx_register_navi(uint8_t bus, Navi* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t id = obj->rxId();

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 去重
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].id == id && c->obj[i].dispatch == dispatch_navi) {
            if (!ps) __enable_irq();
            return 0;
        }
    }
    // 填空
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (!c->obj[i].dispatch) {
            c->obj[i].id       = id;
            c->obj[i].obj      = obj;
            c->obj[i].dispatch = dispatch_navi;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1; // 表满
}

extern "C" int canrx_register_vision(uint8_t bus, Vision* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;
    const uint16_t base = obj->base_id();
    bool present[VISION_ITEM_COUNT] = {0};
    uint16_t free_slots = 0;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    // 扫描：已有条目 + 空位
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
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
        for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
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

extern "C" int canrx_register_navision(uint8_t bus, Navi* obj)
{
    return canrx_register_navi(bus, obj);
}
#else
extern "C" int canrx_register_navi(uint8_t bus, Navi* obj)
{
    (void)bus; (void)obj;
    return -1;
}

extern "C" int canrx_register_vision(uint8_t bus, Vision* obj)
{
    (void)bus; (void)obj;
    return -1;
}

extern "C" int canrx_register_navision(uint8_t bus, Navi* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

#if NTFDCAN_HAS_ADP_OMXIMU
extern "C" int canrx_register_omximu(uint8_t bus, OmxImu* obj)
{
    bus_ctx_t* c = ctx_of(bus);
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
    // 扫描：已有条目 + 空位
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
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
        for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
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
extern "C" int canrx_register_omximu(uint8_t bus, OmxImu* obj)
{
    (void)bus; (void)obj;
    return -1;
}
#endif

extern "C" int canrx_unregister_adapter(uint8_t bus, void* obj)
{
    bus_ctx_t* c = ctx_of(bus);
    if (!c || !obj) return -1;

    uint32_t ps = __get_PRIMASK(); __disable_irq();
    for (int i = 0; i < CANRX_MAX_SUBS_OBJ; ++i) {
        if (c->obj[i].obj == obj && c->obj[i].dispatch) {
            c->obj[i].id = 0; c->obj[i].obj = nullptr; c->obj[i].dispatch = nullptr;
            if (!ps) __enable_irq();
            return 0;
        }
    }
    if (!ps) __enable_irq();
    return -1;
}

// ------------ 覆盖 bsp 的弱钩子：ISR 中唤醒对应 bus 任务 ------------
extern "C" void bsp_fdcan_on_rx_isr(uint8_t bus)
{
    bus_ctx_t* c = ctx_of(bus);
    if (c && c->evt) {
        (void)osEventFlagsSet(c->evt, 0x1);
    }
}

// ------------ 任务入口：任意 bus 的 RX 路由 ------------
extern "C" void Start_CAN_RxRouter(void* argument)
{
    uint8_t bus = (uint8_t)(uintptr_t)argument;
    bus_ctx_t* c = ctx_of(bus);
    if (!c) return;

    c->evt = osEventFlagsNew(nullptr);
    (void)bsp_fdcan_start(bus);

    for (;;) {
        uint32_t f = osEventFlagsWait(c->evt, 0x1, osFlagsWaitAny, 2);
        if (f & 0x1) c->stats.wake_by_isr++; else c->stats.wake_by_to++;

        fdcan_debug_update(bus);

        bsp_fdcan_rx_frame_t frames[CANRX_RX_BATCH];
        for (;;) {
            size_t n = bsp_fdcan_rx_pop(bus, frames, CANRX_RX_BATCH);
            if (!n) break;

            c->stats.pops += (uint32_t)n;

            for (size_t i = 0; i < n; ++i) {
                FDCAN_RxHeaderTypeDef* h = &frames[i].hdr;
                if (h->IdType != FDCAN_STANDARD_ID) continue;

                const uint16_t sid = (uint16_t)h->Identifier;
                const uint8_t*  data = frames[i].data;
                const uint8_t   dlc   = decode_dlc_bytes(h->DataLength);

                int dispatched = 0;

                // 1) 原始回调（按 ID）
                {
                    uint32_t ps = __get_PRIMASK(); __disable_irq();
                    for (int k = 0; k < CANRX_MAX_SUBS_RAW; ++k) {
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

                // 2) 对象适配器
                {
                    uint32_t ps = __get_PRIMASK(); __disable_irq();
                    for (int k = 0; k < CANRX_MAX_SUBS_OBJ; ++k) {
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
        }
    }
}
