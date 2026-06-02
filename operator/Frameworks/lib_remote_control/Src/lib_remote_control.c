/*
 * lib_remote_control.c - Unified Remote Control Input Manager (DR16 + VT03)
 */

#include "lib_remote_control.h"
#include <string.h>

/* 使用原始解码结构 */
#include "bsp_vt03.h"
#include "bsp_dr16.h"

/* ========== 内部记录各源最近帧 ========== */
typedef struct {
    RC_State last;        /* 最近一次该源的统一帧 */
    uint32_t last_ok_ms;  /* 最近一次有效帧时间戳 */
    uint8_t  seen;        /* 是否曾收到过有效帧 */
} _src_slot_t;

/* 槽：NONE/DR16/VT03 -> 0..2 */
static _src_slot_t g_src[3];

static uint16_t g_stick_deadzone = 10u;
static int16_t g_ch4_center = 0;
static uint8_t g_ch4_center_valid = 0;

/* 优先级队列：primary/secondary */
static rc_source_t g_pri[2] = { RC_PRI_DEFAULT_PRIMARY, RC_PRI_DEFAULT_SECONDARY };

/* 超时（ms） */
static uint16_t g_idle_ms    = 50u;
static uint16_t g_offline_ms = 300u;

/* 公共快照与状态 */
volatile RC_State g_rc_unified;
static RC_Status  g_status;

/* 可选：后处理 Hook（用于键位/模式映射等扩展） */
static rc_post_map_hook_t g_post_map_hook = NULL;

/* ========== 工具函数 ========== */
static inline uint8_t _online(uint32_t dt_ms, uint16_t offline_ms) { return (dt_ms <= offline_ms); }
static inline uint8_t _idle  (uint32_t dt_ms, uint16_t idle_ms)    { return (dt_ms >  idle_ms);   }

static rc_source_t _select_active(uint32_t now_ms)
{
    for (int i = 0; i < 2; ++i) {
        rc_source_t s = g_pri[i];
        if (s == RC_SRC_NONE) continue;
        uint32_t dt = now_ms - g_src[s].last_ok_ms;
        if (g_src[s].seen && _online(dt, g_offline_ms)) return s;
    }
    return RC_SRC_NONE;
}

static inline uint8_t _sticks_idle_with(const RC_State* s, uint16_t dz)
{
    const int16_t c0 = s->rc.ch[0];
    if (c0 >  (int16_t)dz || c0 < -(int16_t)dz) return 0;

    const int16_t c1 = s->rc.ch[1];
    if (c1 >  (int16_t)dz || c1 < -(int16_t)dz) return 0;

    const int16_t c2 = s->rc.ch[2];
    if (c2 >  (int16_t)dz || c2 < -(int16_t)dz) return 0;

    const int16_t c3 = s->rc.ch[3];
    if (c3 >  (int16_t)dz || c3 < -(int16_t)dz) return 0;

    const int16_t c4 = s->rc.ch[4];
    if (c4 >  (int16_t)dz || c4 < -(int16_t)dz) return 0;

    return 1;
}

/* ========== API 实现 ========== */
void RC_InputInit(void)
{
    memset((void*)g_src, 0, sizeof(g_src));
    memset((void*)&g_rc_unified, 0, sizeof(g_rc_unified));
    memset((void*)&g_status, 0, sizeof(g_status));
    g_ch4_center = 0;
    g_ch4_center_valid = 0;

    g_pri[0] = RC_PRI_DEFAULT_PRIMARY;
    g_pri[1] = RC_PRI_DEFAULT_SECONDARY;

    g_idle_ms    = 50u;
    g_offline_ms = 300u;

    g_status.active_src = RC_SRC_NONE;
    g_status.online  = 0;
    g_status.idle    = 1;
    g_status.blocked = 1;
    g_status.last_ok_ms = 0;
    g_status.sticks_idle = 1;

    g_status.dr16_online = 0;
    g_status.vt03_online = 0;

    g_post_map_hook = NULL;
}

void RC_SetSourcePriority(rc_source_t primary, rc_source_t secondary)
{
    g_pri[0] = primary;
    g_pri[1] = secondary;
}

void RC_SetTimeouts(uint16_t idle_ms, uint16_t offline_ms)
{
    if (idle_ms == 0) idle_ms = 1;
    if (offline_ms < (uint16_t)(idle_ms + 1)) offline_ms = (uint16_t)(idle_ms + 1);
    g_idle_ms    = idle_ms;
    g_offline_ms = offline_ms;
}

void RC_UpdateFromSource(rc_source_t src, const RC_State* s, uint32_t now_ms)
{
    if (src == RC_SRC_NONE || src > RC_SRC_VT03 || s == NULL) return;

    /* 可选后处理（统一位置，便于全局自定义映射） */
    RC_State tmp = *s;
    if (g_post_map_hook) {
        g_post_map_hook(&tmp);
    }

    /* 记录来源帧 */
    g_src[src].last = tmp;
    g_src[src].last_ok_ms = now_ms;
    g_src[src].seen = 1;

    /* 各源在线状态更新 */
    uint32_t dt_dr16 = now_ms - g_src[RC_SRC_DR16].last_ok_ms;
    uint32_t dt_vt03 = now_ms - g_src[RC_SRC_VT03].last_ok_ms;
    g_status.dr16_online = (g_src[RC_SRC_DR16].seen && _online(dt_dr16, g_offline_ms)) ? 1 : 0;
    g_status.vt03_online = (g_src[RC_SRC_VT03].seen && _online(dt_vt03, g_offline_ms)) ? 1 : 0;

    /* 根据优先级与在线性重新选择活动源 */
    rc_source_t sel = _select_active(now_ms);
    g_status.active_src = sel;

    if (sel != RC_SRC_NONE) {
        RC_INPUT_ENTER_CRITICAL();
        memcpy((void*)&g_rc_unified, (const void*)&g_src[sel].last, sizeof(RC_State));
        RC_INPUT_EXIT_CRITICAL();

        g_status.last_ok_ms = g_src[sel].last_ok_ms;

        uint32_t dt = now_ms - g_src[sel].last_ok_ms;
        g_status.online  = _online(dt, g_offline_ms);
        g_status.idle    = _idle(dt, g_idle_ms);
        g_status.blocked = 0;
        g_status.sticks_idle = _sticks_idle_with(&g_src[sel].last, g_stick_deadzone);
    } else {
        g_status.online  = 0;
        g_status.idle    = 1;
        g_status.blocked = 1;
        g_status.sticks_idle = 1;
    }
}

void RC_Tick(uint32_t now_ms)
{
    rc_source_t sel = _select_active(now_ms);
    g_status.active_src = sel;

    /* 刷各源在线标志 */
    uint32_t dt_dr16 = now_ms - g_src[RC_SRC_DR16].last_ok_ms;
    uint32_t dt_vt03 = now_ms - g_src[RC_SRC_VT03].last_ok_ms;
    g_status.dr16_online = (g_src[RC_SRC_DR16].seen && _online(dt_dr16, g_offline_ms)) ? 1 : 0;
    g_status.vt03_online = (g_src[RC_SRC_VT03].seen && _online(dt_vt03, g_offline_ms)) ? 1 : 0;

    if (sel == RC_SRC_NONE) {
        g_status.online  = 0;
        g_status.idle    = 1;
        g_status.blocked = 1;
        g_status.sticks_idle = 1;
        return;
    }

    uint32_t dt = now_ms - g_src[sel].last_ok_ms;
    g_status.online  = _online(dt, g_offline_ms);
    g_status.idle    = _idle(dt, g_idle_ms);
    g_status.blocked = !_online(dt, g_offline_ms);
    g_status.sticks_idle = _sticks_idle_with(&g_src[sel].last, g_stick_deadzone);

    if (g_status.online) {
        g_status.last_ok_ms = g_src[sel].last_ok_ms;
    }
}

void RC_GetSnapshot(RC_State* out)
{
    if (!out) return;
    RC_INPUT_ENTER_CRITICAL();
    memcpy(out, (const void*)&g_rc_unified, sizeof(RC_State));
    RC_INPUT_EXIT_CRITICAL();
}

RC_Status RC_GetStatus(void)
{
    return g_status;
}

int RC_GetSourceInfo(rc_source_t src, RC_SourceInfo* out)
{
    if (!out) return 0;
    if (src == RC_SRC_NONE || src > RC_SRC_VT03) return 0;
    out->seen = g_src[src].seen;
    out->last_ok_ms = g_src[src].last_ok_ms;
    return 1;
}

void RC_SetStickDeadzone(uint16_t dz)
{
    if (dz > 2000u) dz = 2000u;
    g_stick_deadzone = dz;
}

int RC_IsSticksIdle(void)
{
    return g_status.sticks_idle ? 1 : 0;
}

void RC_RegisterPostMapHook(rc_post_map_hook_t hook)
{
    g_post_map_hook = hook;
}

/* ========== 内部：VT03 三档映射 0/1/2 -> 1/3/2 ========== */
static inline uint8_t _vt03_gear_to_rc_sw(uint8_t g)
{
    g &= 0x03;
    switch (g) {
        case 0: return RC_SW_UP;   /* 1 */
        case 1: return RC_SW_MID;  /* 3 */
        case 2: return RC_SW_DOWN; /* 2 */
        default: return RC_SW_MID;
    }
}

/* ========== 适配器：VT03 dec -> RC_State（统一中心=0） ========== */
/* VT03：ch0..ch3(11bit, center=1024), gear(2bit only one switch), mouse xyz, ml/mr/mm 可选
 * 统一顺序（与你工程中使用一致）：LH, LV, RH, RV
 *   LH = v->ch[3], LV = v->ch[2], RH = v->ch[0], RV = v->ch[1]
 */
void RC_UpdateFromVT03(const vt03_dec_t* v, uint32_t now_ms)
{
    if (!v) return;

    RC_State s;
    memset(&s, 0, sizeof(s));

    s.rc.ch[0] = (int16_t)(v->ch[0] - 1024);
    s.rc.ch[1] = (int16_t)(v->ch[1] - 1024);
    s.rc.ch[2] = (int16_t)(v->ch[3] - 1024);
    s.rc.ch[3] = (int16_t)(v->ch[2] - 1024);
    s.rc.ch[4] = (int16_t)(v->wheel - 1024);

    /* 单拨 -> 对齐 DR16 的右拨 s[0]；s[1] 为空置 0 */
    s.rc.s[0] = _vt03_gear_to_rc_sw(v->gear);
    s.rc.s[1] = 0;

    /* 鼠标/键盘：保留轴，键盘沿用 DR16 的低 16 位布局 */
    s.mouse.x = v->mx;
    s.mouse.y = v->my;
    s.mouse.z = v->mz;
    s.mouse.press_l = 0;
    s.mouse.press_r = 0;
    s.mouse._reserved0 = 0;

    s.key.v     = v->keys;
    s.key.v_ext = 0;
    const bool capslock_proxy_pressed = (v->keys & KEY_PROXY_VT03_CAPSLOCK) != 0u;
    const bool space_proxy_pressed = (v->keys & KEY_PROXY_VT03_SPACE) != 0u;
    if (v->pause || capslock_proxy_pressed) s.key.v_ext |= KEY_EXT_VT03_PAUSE;   /* bit 62 */
    if (v->fn1)   s.key.v_ext |= KEY_EXT_VT03_BTN_L;   /* bit 63 */
    if (v->fn2)   s.key.v_ext |= KEY_EXT_VT03_BTN_R;   /* bit 64 */
    if (v->dial || space_proxy_pressed)  s.key.v_ext |= KEY_EXT_VT03_TRIGGER; /* bit 76: trigger */

    RC_UpdateFromSource(RC_SRC_VT03, &s, now_ms);
}

/* ========== 适配器：DR16 dec -> RC_State（统一中心=0） ========== */
/* DR16：按照统一顺序 LH, LV, RH, RV；拨杆 s[0]=右拨，s[1]=左拨；鼠标与键盘透传 */
void RC_UpdateFromDR16_Adapter(const dr16_dec_t* d, uint32_t now_ms)
{
    if (!d) return;

    RC_State s;
    memset(&s, 0, sizeof(s));

    s.rc.ch[0] = (int16_t)(d->ch[0] - 1024);
    s.rc.ch[1] = (int16_t)(d->ch[1] - 1024);
    s.rc.ch[2] = (int16_t)(d->ch[2] - 1024);
    s.rc.ch[3] = (int16_t)(d->ch[3] - 1024);
    s.rc.ch[4] = (int16_t)(d->ch[4] - 1024);

    s.rc.s[0] = (uint8_t)(d->s1 & 0x03);      /* 1/3/2 */
    s.rc.s[1] = (uint8_t)(d->s2 & 0x03);

    s.mouse.x = d->mx;
    s.mouse.y = d->my;
    s.mouse.z = d->mz;
    s.mouse.press_l = (uint8_t)(d->ml ? 1 : 0);
    s.mouse.press_r = (uint8_t)(d->mr ? 1 : 0);
    s.mouse._reserved0 = 0;

    s.key.v     = d->keys;
    s.key.v_ext = 0;

    RC_UpdateFromSource(RC_SRC_DR16, &s, now_ms);
}
