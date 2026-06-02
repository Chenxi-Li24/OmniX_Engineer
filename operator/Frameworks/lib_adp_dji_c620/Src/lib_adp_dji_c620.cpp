// === lib_adp_dji_c620.cpp ===

#include "lib_adp_dji_c620.h"
#include <cstring>
#include <cstddef>
#include <atomic>

// ---- 公用 BE 读写 ----
static inline uint16_t be16(const uint8_t* p) {
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}
static inline void wr_be16(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 0) & 0xFF);
}

// ---- 映射规则（型号绑定单一事实来源）----
static inline uint32_t c620_map_rx_id(uint8_t motor_id) {
    return 0x200u + motor_id; // 反馈 0x201..0x208
}
static inline bool c620_map_group_slot(uint8_t motor_id, uint16_t& gid, uint8_t& slot) {
    if (motor_id >= 1 && motor_id <= 4)  { gid = 0x200; slot = static_cast<uint8_t>(motor_id - 1); return true; }
    if (motor_id >= 5 && motor_id <= 8)  { gid = 0x1FF; slot = static_cast<uint8_t>(motor_id - 5); return true; }
    return false;
}

static inline uint8_t clamp_id_1_8(uint8_t id) { return (id < 1) ? 1 : (id > 8 ? 8 : id); }

// ---- 构造 ----
DJI_C620::DJI_C620(uint8_t motor_id, uint32_t rx_id)
    : motor_id_(clamp_id_1_8(motor_id)),
      group_id_([&](){ uint16_t g; uint8_t s; c620_map_group_slot(clamp_id_1_8(motor_id), g, s); return g; }()),
      rx_id_(rx_id ? rx_id : c620_map_rx_id(clamp_id_1_8(motor_id)))
{}

// ---- 控制值写入 ----
void DJI_C620::set_give_cmd(int16_t cmd_raw) {
    if (cmd_raw >  16384) cmd_raw =  16384;
    if (cmd_raw < -16384) cmd_raw = -16384;
    give_cmd_raw_.store(cmd_raw, std::memory_order_relaxed);
    dbg_cmd_shadow_ = cmd_raw;
}

// ---- 反馈解析（仅处理本电机的反馈帧）----
void DJI_C620::onRxFeedback(const CanFrame& f, uint32_t now_ms) {
    if (f.is_ext || f.dlc < 8 || f.id != rx_id_) return;

    // seqlock 写开始：置奇数
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);
    C620State s = state_;

    s.prev_angle_raw = s.mech_angle_raw;
    s.prev_speed_raw = s.speed_raw;
    s.prev_iq_raw    = s.iq_raw;

    s.mech_angle_raw = static_cast<uint16_t>(be16(&f.data[0])) & 0x1FFFu; // 常见 13 位有效
    s.speed_raw      = static_cast<int16_t>(be16(&f.data[2]));
    s.iq_raw         = static_cast<int16_t>(be16(&f.data[4]));
    s.temp_C         = f.data[6];
    s.mech_angle_deg = (static_cast<float>(s.mech_angle_raw) * 360.0f) / 8192.0f;

    s.last_rx_ms = now_ms;
    s.online     = true;

    state_ = s;
    // seqlock 写结束：置偶数
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
}

// ---- 离线判定 ----
void DJI_C620::tick(uint32_t now_ms) {
    // 轻读：原子读取 last_rx_ms / online
    const uint32_t last = __atomic_load_n(&state_.last_rx_ms, __ATOMIC_RELAXED);
        const bool was_online = __atomic_load_n(&state_.online, __ATOMIC_RELAXED);
        if (was_online && (now_ms - last > offline_ms_)) {
            __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);
            state_.online = false;
            __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
        }
}

bool DJI_C620::snapshot(C620State& out) const {
    for (int attempt = 0; attempt < 3; ++attempt) {
        uint32_t s1 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue; // 写入中，重试

        C620State tmp;
        std::memcpy(&tmp, &state_, sizeof(tmp));

        uint32_t s2 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 == s2) { out = tmp; return true; }
    }
    return false;
}


// ---- 导出给 TX 路由器：一个对象 -> (group_id, slot, val) ----
bool DJI_C620::exportTx16(uint16_t* group_id, uint8_t* slot, int16_t* val) const {
    if (!group_id || !slot || !val) return false;
    if (!c620_map_group_slot(motor_id_, *group_id, *slot)) return false;
    *val = give_cmd_raw_.load(std::memory_order_relaxed);
    return true;
}

// ---- 清帧工具 ----
static void clear_frame(CanFrame& f, uint32_t id) {
    f.id = id;
    f.dlc = 8;
    f.is_ext = false;
    std::memset(f.data, 0, sizeof(f.data));
}

// ---- 批量组帧（最小系统/Bring-up 用）----
void C620_ComposeTxFrames(DJI_C620* const motors[],
                          std::size_t n,
                          CanFrame out[],
                          std::size_t& out_count)
{
    int16_t cur_1_4[4] = {0,0,0,0};
    int16_t cur_5_8[4] = {0,0,0,0};
    bool have_200 = false, have_1FF = false;

    for (std::size_t i = 0; i < n; ++i) {
        DJI_C620* m = motors[i];
        if (!m) continue;

        uint16_t gid; uint8_t slot; int16_t v;
        if (!m->exportTx16(&gid, &slot, &v)) continue;

        if (gid == 0x200) { cur_1_4[slot] = v; have_200 = true; }
        else              { cur_5_8[slot] = v; have_1FF = true; }
    }

    std::size_t idx = 0;

    if (have_200) {
        clear_frame(out[idx], 0x200u);
        wr_be16(&out[idx].data[0], cur_1_4[0]); // id1
        wr_be16(&out[idx].data[2], cur_1_4[1]); // id2
        wr_be16(&out[idx].data[4], cur_1_4[2]); // id3
        wr_be16(&out[idx].data[6], cur_1_4[3]); // id4
        idx++;
    }
    if (have_1FF) {
        clear_frame(out[idx], 0x1FFu);
        wr_be16(&out[idx].data[0], cur_5_8[0]); // id5
        wr_be16(&out[idx].data[2], cur_5_8[1]); // id6
        wr_be16(&out[idx].data[4], cur_5_8[2]); // id7
        wr_be16(&out[idx].data[6], cur_5_8[3]); // id8
        idx++;
    }

    out_count = idx;
}
