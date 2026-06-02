// === lib_adp_dji_c620.h ===
// C620/C610/C602x 一类电机的最小适配：
// - 与型号绑定：1..4 -> TX 0x200，5..8 -> TX 0x1FF；反馈 RX = 0x200 + motor_id
// - 16-bit 指令（大端上行），单位含义由项目约定（常见为电流）
// - 提供 exportTx16() 供 TX 路由聚合；保留 ComposeTxFrames() 供简单场景直发两帧

#ifndef H723VG_V2_FREERTOS_LIB_ADP_DJI_C620_H
#define H723VG_V2_FREERTOS_LIB_ADP_DJI_C620_H

#pragma once
#include <cstdint>
#include <cstddef>  // std::size_t
#include <atomic>

// ---- 公共 CAN 帧描述（最小集合）----
#ifndef ADP_CANFRAME_DEFINED
#define ADP_CANFRAME_DEFINED
struct CanFrame {
    uint32_t id;
    uint8_t  dlc;
    bool     is_ext;
    uint8_t  data[8];
};
#endif

// ---- 反馈状态 ----
struct C620State {
    // feedback
    uint16_t mech_angle_raw = 0;   // 0..8191 (one rev = 8192)
    float    mech_angle_deg = 0.0f;
    int16_t  speed_raw = 0;        // datasheet unit
    int16_t  iq_raw = 0;           // actual torque current (raw)
    uint8_t  temp_C = 0;

    // meta
    uint32_t last_rx_ms = 0;
    bool     online = false;

    // previous snapshot
    uint16_t prev_angle_raw = 0;
    int16_t  prev_speed_raw = 0;
    int16_t  prev_iq_raw = 0;
};

class DJI_C620 {
public:
    /**
     * motor_id: 1..8
     * rx_id:    0 = auto (0x200 + motor_id) ; 非 0 则使用传入值（保留向后兼容的灵活口）
     * group_id: auto by motor_id (1..4 -> 0x200, 5..8 -> 0x1FF)
     */
    explicit DJI_C620(uint8_t motor_id, uint32_t rx_id = 0);

    // identification（构造后固定）
    uint8_t  motorId() const { return motor_id_; }
    uint32_t groupId() const { return group_id_; }
    uint32_t rxId()    const { return rx_id_;   }

    // === control (protocol raw) ===
    // range: [-16384, 16384] ≈ 常见 20A 档（按项目定义）
    void     set_give_cmd(int16_t cmd_raw);
    int16_t  give_cmd() const { return give_cmd_raw_.load(std::memory_order_relaxed); }
    // debug helper: plain shadow for debuggers that cannot introspect atomic internals
    int16_t  dbg_give_cmd_shadow() const { return dbg_cmd_shadow_; }

    // === feedback ===
    void onRxFeedback(const CanFrame& f, uint32_t now_ms);
    // 与 GM6020 一致：提供按值返回的“快照”访问器（内部用 seqlock 保障一致性）
    C620State state() const noexcept {
        C620State s{};
        (void)snapshot(s);
        return s;
    }
        // 无锁多读快照；最多尝试 3 次
    bool snapshot(C620State& out) const;

    void set_offline_timeout(uint32_t ms) { offline_ms_ = ms; }
    void tick(uint32_t now_ms);

    // === TX 路由导出接口：一个对象 -> (group_id, slot, 16-bit) ===
    // 注意：16-bit 上行按 BE 打包（高字节在前）
    bool exportTx16(uint16_t* group_id, uint8_t* slot, int16_t* val) const;

private:
    const uint8_t  motor_id_;       // 1..8
    const uint32_t group_id_;       // 0x200 or 0x1FF (auto by motor_id)
    const uint32_t rx_id_;          // 0x200 + motor_id (当构造传入 rx_id==0 时)

    std::atomic<int16_t> give_cmd_raw_{0};  // [-16384, 16384] 原子读写
    volatile int16_t dbg_cmd_shadow_{0};    // 调试影子，供 Ozone 直接采样
    // seqlock 计数器：偶数=稳定快照，奇数=写入中
    mutable uint32_t state_seq_{0};
    mutable C620State state_{};
    uint32_t  offline_ms_ = 100;
};

/**
 * 批量拼两帧控制：
 *   - 0x200 for ids 1..4  (bytes: id1,id2,id3,id4 each int16 **BE**)
 *   - 0x1FF for ids 5..8  (bytes: id5,id6,id7,id8 each int16 **BE**)
 * 缺失的槽位补 0。
 *
 * motors: 指针数组（可含 nullptr）
 * n:      数组元素数
 * out:    至少容纳 2 帧
 * out_count: 返回有效帧数(0..2)
 */
void C620_ComposeTxFrames(DJI_C620* const motors[],
                          std::size_t n,
                          CanFrame out[],
                          std::size_t& out_count);

#endif // H723VG_V2_FREERTOS_LIB_ADP_DJI_C620_H
