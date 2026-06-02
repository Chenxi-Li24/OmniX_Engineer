#include "lib_adp_dm_4310.h"
#include <cstring>
#include <cstddef>

// uint32_t rx_1 = 0;
// uint32_t rx_2 = 0;
// uint32_t rx_3 = 0;
// uint32_t rx_4 = 0;

// ===== MIT-mode ranges and float→fixed mapping =====
namespace {
    // Typical MIT / mini-cheetah ranges.
    // If you have official motor ranges, change here.
    constexpr float P_MIN  = -3.2f;
    constexpr float P_MAX  =  3.2f;
    constexpr float V_MIN  = -45.0f;
    constexpr float V_MAX  =  45.0f;
    constexpr float KP_MIN =   0.0f;
    constexpr float KP_MAX = 500.0f;
    constexpr float KD_MIN =   0.0f;
    constexpr float KD_MAX =   5.0f;
    constexpr float T_MIN  = -18.0f;
    constexpr float T_MAX  =  18.0f;

    // Q16 raw → rad
    constexpr float POS_Q16_TO_RAD = 2.0f * 3.14159265f / 65535.0f;

    // Map float to unsigned fixed-point, same idea as C mit_ctrl.
    static inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
    {
        const float span   = x_max - x_min;
        float       offset = x - x_min;
        if (offset < 0.0f) offset = 0.0f;
        if (offset > span) offset = span;

        const uint32_t levels = (1u << bits) - 1u;
        uint32_t ret = static_cast<uint32_t>(offset * static_cast<float>(levels) / span + 0.5f);
        if (ret > levels) ret = levels;
        return static_cast<uint16_t>(ret);
    }

    // Little-endian float32 writer (PosVel / Velocity modes)
    static inline void wr_le_f32(uint8_t* p, float v) {
        static_assert(sizeof(float) == 4, "float must be 32-bit IEEE754");
        uint32_t u;
        std::memcpy(&u, &v, 4);
        p[0] = static_cast<uint8_t>(u >> 0);
        p[1] = static_cast<uint8_t>(u >> 8);
        p[2] = static_cast<uint8_t>(u >> 16);
        p[3] = static_cast<uint8_t>(u >> 24);
    }
} // namespace

DM4310::DM4310(uint8_t can_id, uint16_t fb_master_sid)
: can_id_(can_id), fb_master_sid_(fb_master_sid) {}

void DM4310::set_mit_cmd(fp32 pos_rad, fp32 vel_rad_s, fp32 kp, fp32 kd, fp32 torque_nm)
{
    cmd_mit_pos_u32_.store(f2u(static_cast<float>(pos_rad)), std::memory_order_relaxed);
    cmd_mit_vel_u32_.store(f2u(static_cast<float>(vel_rad_s)), std::memory_order_relaxed);
    cmd_mit_kp_u32_.store(f2u(static_cast<float>(kp)), std::memory_order_relaxed);
    cmd_mit_kd_u32_.store(f2u(static_cast<float>(kd)), std::memory_order_relaxed);
    cmd_mit_torque_u32_.store(f2u(static_cast<float>(torque_nm)), std::memory_order_relaxed);
    mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4310::set_mit_pos_rad(fp32 pos_rad)
{
    cmd_mit_pos_u32_.store(f2u(static_cast<float>(pos_rad)), std::memory_order_relaxed);
    mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4310::set_mit_vel_rad_s(fp32 vel_rad_s)
{
    cmd_mit_vel_u32_.store(f2u(static_cast<float>(vel_rad_s)), std::memory_order_relaxed);
    mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4310::set_mit_kp(fp32 kp)
{
    cmd_mit_kp_u32_.store(f2u(static_cast<float>(kp)), std::memory_order_relaxed);
    mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4310::set_mit_kd(fp32 kd)
{
    cmd_mit_kd_u32_.store(f2u(static_cast<float>(kd)), std::memory_order_relaxed);
    mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4310::set_mit_torque_nm(fp32 torque_nm)
{
    cmd_mit_torque_u32_.store(f2u(static_cast<float>(torque_nm)), std::memory_order_relaxed);
    mit_cmd_valid_.store(1u, std::memory_order_relaxed);
}

void DM4310::onRxFeedback(const CanFrame& f, uint32_t now_ms) {
    if (f.is_ext || f.dlc < 8 || f.id != fb_master_sid_) {
        return;
    }

    const uint8_t* d = f.data;
    const uint8_t id_in = dm4310_payload_id(d[0]);
    const uint8_t err   = dm4310_payload_err(d[0]);

    // If you use 8-bit ID in protocol, change this to: if (d[0] == can_id_) ...
    if (id_in != (can_id_ & 0x0F)) {
        return;
    }

    // begin write: make seq odd
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);

    DM4310State s = state_;

    // keep previous values
    s.prev_pos_raw    = s.pos_raw;
    s.prev_speed_raw  = s.speed_rpm;
    s.prev_torque_raw = s.torque_raw;
    const bool was_online = s.online;

    // POS 16b; VEL/TQ 12b (high bits assembled + sign extension)
    const int16_t pos_q16 = static_cast<int16_t>(
        (static_cast<uint16_t>(d[1]) << 8) | d[2]
    );
    const int16_t vel_q12 = dm4310_signext12(
        (static_cast<uint16_t>(d[3]) << 4) | (d[4] >> 4)
    );
    const int16_t tq_q12  = dm4310_signext12(
        (static_cast<uint16_t>(d[4] & 0x0F) << 8) | d[5]
    );

    // ===== multi-turn accumulator =====
    if (!was_online) {
        // First valid frame (or re-online after timeout):
        // initialize accumulator from current single-turn value.
        s.pos_raw_total = static_cast<int32_t>(pos_q16);
    } else {
        // Compute step with wrap-around compensation.
        int32_t delta = static_cast<int32_t>(pos_q16) -
                        static_cast<int32_t>(s.prev_pos_raw);

        // When 16-bit wraps through ±32768, the raw difference will be ~±65536.
        if (delta > 32767) {
            delta -= 65536;
        } else if (delta < -32768) {
            delta += 65536;
        }

        s.pos_raw_total += delta;
    }

    // fill feedback (single-turn + multi-turn)
    s.motor_id     = id_in;
    s.error_code   = err;
    s.pos_raw      = pos_q16;
    s.pos_rad      = static_cast<fp32>(pos_q16) * POS_Q16_TO_RAD;
    s.pos_rad_total= static_cast<fp32>(s.pos_raw_total) * POS_Q16_TO_RAD;
    s.speed_rpm    = vel_q12;
    s.speed_rad_s  = static_cast<fp32>(vel_q12) * (2.0f * 3.14159265f / 60.0f);
    s.torque_raw   = tq_q12;
    s.temp_mos_C   = d[6];
    s.temp_rotor_C = d[7];
    s.last_rx_ms   = now_ms;
    s.online       = true;

    state_ = s;

    // end write: make seq even
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
}

void DM4310::tick(uint32_t now_ms) {
    const uint32_t last = __atomic_load_n(&state_.last_rx_ms, __ATOMIC_RELAXED);
    const bool was_online = __atomic_load_n(&state_.online, __ATOMIC_RELAXED);

    if (was_online && (now_ms - last > offline_ms_)) {
        __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);
        state_.online = false;
        __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
    }
}

bool DM4310::snapshot(DM4310State& out) const {
    for (int i = 0; i < 3; ++i) {
        uint32_t s1 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 & 1u) {
            // writer in progress
            continue;
        }

        DM4310State tmp;
        std::memcpy(&tmp, &state_, sizeof(tmp));

        uint32_t s2 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 == s2) {
            out = tmp;
            return true;
        }
    }
    return false;
}

bool DM4310::exportTxRaw8(uint16_t* sid, uint8_t out[8]) const {
    if (!sid || !out) {
        return false;
    }

    const DM4310Mode m = mode();
    switch (m) {
        case DM4310Mode::MIT: {
            // ===== 1. Read command registers as "physical" values =====
            const bool mit_valid = (mit_cmd_valid_.load(std::memory_order_relaxed) != 0);
            float pos;
            float vel;
            float kp;
            float kd;
            float torq;
            if (mit_valid) {
                pos  = u2f(cmd_mit_pos_u32_.load(std::memory_order_relaxed));
                vel  = u2f(cmd_mit_vel_u32_.load(std::memory_order_relaxed));
                kp   = u2f(cmd_mit_kp_u32_.load(std::memory_order_relaxed));
                kd   = u2f(cmd_mit_kd_u32_.load(std::memory_order_relaxed));
                torq = u2f(cmd_mit_torque_u32_.load(std::memory_order_relaxed));
            } else {
                pos  = get_pos_cmd_rad();
                vel  = get_vel_cmd_lim();
                torq = static_cast<float>(cmd_torque_raw_.load(std::memory_order_relaxed));
                kp   = 0.0f;
                kd   = 0.0f;
            }

            // ===== 2. Float → fixed-point mapping =====
            const uint16_t pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
            const uint16_t vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
            const uint16_t kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
            const uint16_t kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
            const uint16_t tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

            // ===== 3. Pack 8B according to MIT protocol =====
            out[0] = static_cast<uint8_t>(pos_tmp >> 8);
            out[1] = static_cast<uint8_t>(pos_tmp & 0xFFu);
            out[2] = static_cast<uint8_t>(vel_tmp >> 4);
            out[3] = static_cast<uint8_t>(((vel_tmp & 0x0Fu) << 4) | (kp_tmp >> 8));
            out[4] = static_cast<uint8_t>(kp_tmp & 0xFFu);
            out[5] = static_cast<uint8_t>(kd_tmp >> 4);
            out[6] = static_cast<uint8_t>(((kd_tmp & 0x0Fu) << 4) | (tor_tmp >> 8));
            out[7] = static_cast<uint8_t>(tor_tmp & 0xFFu);

            *sid = static_cast<uint16_t>(0x000u + can_id_);
            return true;
        }

        case DM4310Mode::PosVel: {
            // position + max velocity (both LE float)
            // Modified to use get_pos_cmd_rad() / get_vel_cmd_lim() as requested
            float p_rad = get_pos_cmd_rad();
            float v_lim = get_vel_cmd_lim();

            wr_le_f32(&out[0], p_rad);
            wr_le_f32(&out[4], v_lim);
            *sid = static_cast<uint16_t>(0x100u + can_id_);
            return true;
        }

        case DM4310Mode::Velocity: {
            // Velocity mode usually takes velocity in D[0]-D[3]
            float v = get_vel_cmd_lim();
            wr_le_f32(&out[0], v);
            out[4] = out[5] = out[6] = out[7] = 0;
            *sid = static_cast<uint16_t>(0x200u + can_id_);
            return true;
        }

        default:
            break;
    }

    return false;
}

void DM4310_CollectTxFrames(DM4310* const motors[],
                            std::size_t n,
                            CanFrame out[],
                            std::size_t& out_count)
{
    std::size_t idx = 0;
    for (std::size_t i = 0; i < n; ++i) {
        DM4310* m = motors[i];
        if (!m) {
            continue;
        }

        uint16_t sid;
        uint8_t payload[8];
        if (!m->exportTxRaw8(&sid, payload)) {
            continue;
        }

        CanFrame f{};
        f.id     = sid;
        f.is_ext = false;
        f.dlc    = 8;
        std::memcpy(f.data, payload, 8);

        out[idx++] = f;
    }
    out_count = idx;
}
