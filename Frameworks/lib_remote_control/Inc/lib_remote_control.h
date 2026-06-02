#pragma once
/*
 * lib_remote_control - Unified Remote Control Input Manager (DR16 + VT03)
 *
 * 设计要点：
 *  - 不依赖具体 BSP，任务层只需调用 RC_UpdateFromDR16_Adapter / RC_UpdateFromVT03。
 *  - 统一的 0 居中通道定义：rc.ch[0..4]，rc.s[0..1] 为三档（1=UP/3=MID/2=DOWN）。
 *  - 可配置来源优先级与超时阈值；提供在线/空闲/阻塞状态与快照 API。
 *  - 预留键盘映射扩展位，后续可加入更多 key/功能映射。
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct vt03_dec_s;
struct dr16_dec_s;
typedef struct vt03_dec_s vt03_dec_t;
typedef struct dr16_dec_s dr16_dec_t;

/* ===== 可选的临界区钩子（默认空宏） ===== */
#ifndef RC_INPUT_ENTER_CRITICAL
#define RC_INPUT_ENTER_CRITICAL()  do {} while (0)
#endif
#ifndef RC_INPUT_EXIT_CRITICAL
#define RC_INPUT_EXIT_CRITICAL()   do {} while (0)
#endif

/* ===== 三档常量（与 DR16 语义一致） ===== */
#define RC_SW_UP                ((uint8_t)1)
#define RC_SW_MID               ((uint8_t)3)
#define RC_SW_DOWN              ((uint8_t)2)
#define switch_is_up(s)         ((s) == RC_SW_UP)
#define switch_is_mid(s)        ((s) == RC_SW_MID)
#define switch_is_down(s)       ((s) == RC_SW_DOWN)

/* ===== 通道范围（DJI 习惯值；仅供参考/归一化时使用） ===== */
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ===== Optional CH4 auto-center (useful when CH4 drifts on boot) ===== */
#ifndef RC_CH4_AUTO_CENTER
#define RC_CH4_AUTO_CENTER      (1)
#endif

/* ===== 键盘位（PC-style） =====
 * v 的低 16 位沿用常用 WASD/Shift/Ctrl 等；高位预留扩展
 */
#define KEY_PRESSED_OFFSET_W      ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S      ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A      ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D      ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT  ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL   ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q      ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E      ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R      ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F      ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G      ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z      ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X      ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C      ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V      ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B      ((uint16_t)1 << 15)

/* VT03 扩展键位：放在 key.v_ext */
#define KEY_EXT_VT03_PAUSE        ((uint16_t)1 << 0)   /* bit 62 */
#define KEY_EXT_VT03_BTN_L        ((uint16_t)1 << 1)   /* bit 63: customizable left */
#define KEY_EXT_VT03_BTN_R        ((uint16_t)1 << 2)   /* bit 64: customizable right */
#define KEY_EXT_VT03_TRIGGER      ((uint16_t)1 << 3)   /* bit 76: trigger button */

/* ===== 统一 RC 状态（以 0 为中心） ===== */
typedef struct __attribute__((packed)) {
    struct __attribute__((packed)) {
        int16_t ch[5];    /* ch0..ch3 主摇杆；ch4 备用；全部以 0 为中心 */
        uint8_t s[2];     /* 两个 3 档：1=UP / 3=MID / 2=DOWN。VT03 仅用 s[0] */
    } rc;
    struct __attribute__((packed)) {
        int16_t x, y, z;        /* 鼠标轴（可选） */
        uint8_t press_l, press_r; /* 鼠标键（可选） */
        uint8_t _reserved0;     /* 预留给中键/滚轮按下等 */
    } mouse;
    struct __attribute__((packed)) {
        uint16_t v;             /* 低 16 位键盘位 */
        uint16_t v_ext;         /* 预留扩展键位（默认 0） */
    } key;
} RC_State;

/* ===== 来源与状态 ===== */
typedef enum {
    RC_SRC_NONE = 0,
    RC_SRC_DR16 = 1,
    RC_SRC_VT03 = 2,
} rc_source_t;

/* 默认优先级（可在运行时修改） */
#define RC_PRI_DEFAULT_PRIMARY    RC_SRC_DR16
#define RC_PRI_DEFAULT_SECONDARY  RC_SRC_VT03

typedef struct {
    uint8_t    online;        /* 当前活动源在线（dt<=offline_ms） */
    uint8_t    idle;          /* 活动源空闲（dt>idle_ms） */
    uint8_t    blocked;       /* 无可用源或活动源离线 */
    rc_source_t active_src;   /* 当前驱动统一态的来源 */
    uint32_t   last_ok_ms;    /* 活动源最近一次有效帧时间 */

    uint8_t    sticks_idle;   /* 摇杆是否在死区内 */

    /* 每个来源的在线状态（便于上层指示灯/容错逻辑） */
    uint8_t    dr16_online;
    uint8_t    vt03_online;
} RC_Status;

typedef struct {
    uint8_t  seen;        /* 曾收到有效帧 */
    uint32_t last_ok_ms;  /* 最近一次有效帧时间 */
} RC_SourceInfo;

/* 供其他任务直接读的快照（若需一致性，调用 RC_GetSnapshot） */
extern volatile RC_State g_rc_unified;

/* ===== API ===== */
void RC_InputInit(void);
void RC_SetSourcePriority(rc_source_t primary, rc_source_t secondary);
void RC_SetTimeouts(uint16_t idle_ms, uint16_t offline_ms);
void RC_UpdateFromSource(rc_source_t src, const RC_State* s, uint32_t now_ms);
void RC_Tick(uint32_t now_ms);
void RC_GetSnapshot(RC_State* out);
RC_Status RC_GetStatus(void);
int  RC_GetSourceInfo(rc_source_t src, RC_SourceInfo* out);
void RC_SetStickDeadzone(uint16_t dz);
int  RC_IsSticksIdle(void);

/* ==== 适配入口（由任务层调用，把原始解码结构交给 lib） ==== */
void RC_UpdateFromVT03(const vt03_dec_t* v, uint32_t now_ms);
void RC_UpdateFromDR16_Adapter(const dr16_dec_t* d, uint32_t now_ms);

/* ==== 未来扩展预留：自定义映射/键位转换 Hook ==== */
/* 若想在不改库代码的情况下做额外键位映射，可在应用层实现并在 .c 中调用 */
typedef void (*rc_post_map_hook_t)(RC_State* s);
void RC_RegisterPostMapHook(rc_post_map_hook_t hook);   /* 可选 */

#ifdef __cplusplus
}
#endif
